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

## 模块边界

- `CameraBase<Info>`
  - 只拥有类型定义、sink 边界和同步后 IMU 发布 helper
- 具体相机模块，例如 `WebotsCamera<Info>`
  - 发布原始传感器数据
  - 把图像写进 `ImageFrame`
  - 调用 `CommitImage()`
- `CameraFrameSync<Info>`
  - 承接图像 lease
  - 处理原始 `gyro / accl / quat`，并负责对齐
  - 发布同步后的 `ImuStamped`

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
- 不再 50 帧自动结束；建议采满约 `120` 个有效视角，最多缓存 `300` 个视角。
- `cali status` 查看已处理帧、已检测帧、有效视角数和输出目录。
- `cali save` 会结束本轮标定并写出结果；至少需要 `8` 个有效视角。
- `cali stop` 只停止并恢复图像发布，不写出新的标定结果。

输出目录格式为 `runs/camera_calib/<时间>_<相机名>_<标记尺寸>mm_<列数>x<行数>/`，
主要文件包括：

- `calibration.yml`：OpenCV 相机矩阵、畸变参数、RMS 和标定板几何。
- `views.csv`：每个保留视角的帧号、时间戳、标记数和误差。
- `camera_info_snippet.txt`：可拷回静态 `CameraInfo` 的矩阵片段。
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
