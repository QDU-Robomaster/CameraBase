# CameraBase

`CameraBase` 是当前 Webots/Linux 自瞄链路里的相机 ABI 基线。

它本身不实现具体驱动，只负责定义：

- 编译期静态相机信息
- 原始图像和同步后 IMU 的数据结构
- 图像 sink 的注册与提交边界

## 当前职责

- `CameraTypes::CameraInfo`
  - 编译期静态相机描述
  - 包含分辨率、步长、编码、内参、畸变、校正矩阵、投影矩阵
- `CameraBase<Info>::ImageFrame`
  - 固定尺寸图像载荷
  - 供具体相机模块直接写入，再交给后续共享图像发布环节
- `CameraBase<Info>::ImuStamped`
  - 最终对下游发布的同步后 IMU 数据
- 图像 sink API
  - `RegisterImageSink(...)`
  - `ImageSinkReady()`
  - `GetWritableImage()`
  - `CommitImage()`
- 生产者侧内录
  - 在 `CommitImage()` 进入同步模块前记录原始图像字节
  - 不通过订阅队列，因此不会因为 detector / preview 消费慢而丢帧

## 模块边界

- `CameraBase<Info>`
  - 拥有类型定义、sink 边界、生产者侧图像内录和同步后 IMU 发布 helper
- 具体相机模块，例如 `WebotsCamera<Info>`
  - 发布原始传感器数据
  - 把图像写进 `ImageFrame`
  - 调用 `CommitImage()`
- `CameraFrameSync<Info>`
  - 承接图像 lease
  - 处理原始 `gyro / accl / quat`，并负责对齐
  - 发布同步后的 `ImuStamped`
  - 记录图像时间戳与 IMU 时间戳的同步映射和 IMU 数据

## 图像内录

具体相机模块可以把 `CameraBase::RecordingParam` 透传到基类。开启后，
`CameraBase` 在生产者提交点同步写出：

- `<时间>_<相机名>_frames.bin`：所有图像原始字节顺序追加，像素格式与
  `CameraInfo.encoding` 一致。
- `<时间>_<相机名>_frames.csv`：每帧的
  `frame_index,camera_timestamp_us,offset_bytes,size_bytes`。
- `<时间>_<相机名>_camera_info.yaml`：宽、高、step、encoding、单帧字节数、相机名和 stem。

默认 `output_dir` 为空时，最终目录为 `runs/camera_record/<时间>_<相机名>/`。
运行中实际写入同级 `<时间>_<相机名>.tmp/`，并先写出
`<时间>_<相机名>.recording` 恢复标记。正常退出时会整理临时目录并改名成最终目录；
如果比赛中直接断电，下次进程启动时会先扫描恢复标记，截掉 CSV/RAW 尾部不完整记录，
再把临时目录整理成可回放的最终目录。写盘不经过共享 topic 订阅者；磁盘慢会反压采集
线程，但不会静默丢帧。

`CameraFrameSync` 开启同步记录并复用同一目录时，会额外写出同 stem 的
`<时间>_<相机名>_imu.csv`。这三份文件可以直接交给 `CaptureFileCamera` 的
raw package 模式回放。

## 同步相关约定

- 原始 `gyro / accl / quat` 的采样时刻由发布端通过 Topic timestamp 携带；
  `CameraBase` 不再为三路原始传感器定义额外 wrapper，也不保留 `seq/id`。
- `ImuStamped / ImageFrame` 的 `timestamp_us` 统一使用
  `LibXR::MicrosecondTimestamp` 表达，ABI 仍保持 64 位微秒时间戳。
- `ImageFrame::timestamp_us` 是图像传感器侧时间戳；同步后的
  `ImuStamped::timestamp_us` 使用对应图像的传感器侧时间戳。
- 原始 IMU 时间戳只保证在 IMU 数据域内部可比较，不能直接把图像域时间戳和
  IMU 域时间戳拿来做跨域绝对值匹配。
- 跨域同步关系应先通过专门的同步策略锁定，再在 IMU 域内使用 `offset`
  推导最终样本。
- 相机触发同步命令不属于 `CameraBase`。实机链路应使用 `CameraSync::SyncCommand`
  作为跨端 payload，并通过专门的同步 topic 下发。

## 在线标定命令

`CameraBase` 内置一个面向实机调试的 GShang ChArUco 在线标定入口。启动后，
`CommitImage()` 会先把图像交给标定器；标定器接管的帧不会继续发布到
`CameraFrameSync`，避免下游算法在标定期间继续消费旧相机参数下的图像。

命令在相机自己的 RamFS 文件下执行：

```text
cali <标记尺寸mm> <列数> <行数>
cali status
cali save
cali stop
```

示例：

```text
cali 25mm 8 6
```

对应 GShang 在线工具里的 `ChArUcoBoard`，`charucoDictionary=aruco`，
`markerSize=25`，列数 `8`，行数 `6`。当前实现按 GShang 的实际几何处理：
ArUco-original 标记只放在 `(row + col) % 2 == 1` 的棋盘格中，完整方格宽度由
`square_mm = marker_mm * 9 / 7` 推导，不需要额外输入 `gridWidth`。

采样策略：

- 只接受能检测到足够标记、且单应性重投影误差合理的视角。
- 对检测到的 marker 区域计算清晰度分数，运动模糊或明显低于本轮最佳清晰度的帧不会入库。
- 对画面中心、标定板尺度和旋转角度做近重复判断，只拦截非常接近的重复姿态，正常移动采样不会被过度限制。
- 不再 50 帧自动结束；建议采满约 `120` 个有效视角，之后仍可继续采样。
- 每接受一个有效视角都会打印当前已接受视角数、已截断帧数、已处理帧数和已检测帧数。
- `cali status` 查看已处理帧、已检测帧、模糊拒绝数、重复拒绝数、有效视角数和输出目录。
- `cali save` 会结束本轮标定、写出结果、直接打印可粘贴到 `xrobot.yaml` 的
  `constexprs.MainCameraInfo` 片段；打印前也会输出本次标定的重投影误差、
  样本覆盖和内参合理性报告，然后主动退出当前进程。
- `cali stop` 只停止并恢复图像发布，不写出新的标定结果。

输出目录格式为 `runs/camera_calib/<时间>_<相机名>_<标记尺寸>mm_<列数>x<行数>/`，
主要文件包括：

- `calibration.yml`：OpenCV 相机矩阵、畸变参数、RMS 和标定板几何。
- `views.csv`：每个保留视角的帧号、时间戳、标记数、清晰度、画面位置、尺度、角度和误差。
- `quality_report.txt`：保存成功时同步打印的重投影误差、样本覆盖和内参合理性报告。
- `camera_info_snippet.txt`：可拷回 `xrobot.yaml` 的 `MainCameraInfo` 片段。
- `debug/*.jpg`：每个接受视角的检测可视化图。

采集时尽量覆盖画面中心、四角、近距离、远距离和不同倾角；避免屏幕反光、运动模糊和
整块标定板长期停在同一姿态。

## 备注

- 图像字节数在编译期由 `CameraInfo.step * CameraInfo.height` 推导。
- 图像 sink 切槽回调现在使用 `LibXR::Callback<ImageFrame*&>`，
  不再单独保留裸函数指针 + context。
- `name / image_topic_name / imu_topic_name` 构造入口使用 `std::string_view`，基类内部
  自有保存一份，避免派生模块或装配代码传入临时字符串后出现悬空引用。
- 这个模块刻意把 ABI 保持成固定尺寸 / 标准布局，便于共享内存与 topic 搬运。
- `CameraTypes::BuildPnPDistCoeffs(...)` 是编译期静态转换 helper。
  推荐在静态相机信息定义旁直接生成并缓存结果，而不是在运行时反复构造。
