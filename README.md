# CameraBase

`CameraBase` 是当前这条 Webots/Linux 自瞄链路里的“相机前半段 ABI”。

它本身不实现具体驱动，只负责定义：

- 编译期静态相机信息
- 原始图像 / 原始 IMU / 图像事件的数据结构
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
- 原始传感器 ABI
  - `GyroStamped`
  - `AcclStamped`
  - `QuatStamped`
  - `ImageEvent`
  - `SensorSyncCmd`
- 图像 sink API
  - `RegisterImageSink(...)`
  - `ImageSinkReady()`
  - `GetWritableImage()`
  - `CommitImage()`

## 模块边界

- `CameraBase<Info>`
  - 只拥有类型定义、sink 边界和同步后 IMU 发布 helper
- 具体相机模块，例如 `WebotsCamera<Info>`
  - 填写原始 IMU
  - 填写 `ImageEvent`
  - 把图像写进 `ImageFrame`
  - 调用 `CommitImage()`
- `CameraFrameSync<Info>`
  - 承接图像 lease
  - 处理原始 `gyro / accl / quat / image_event`
  - 发布同步后的 `ImuStamped`

## 同步相关约定

- `GyroStamped / AcclStamped / QuatStamped / ImageEvent` 里的 `sensor_timestamp_us`
  都是“各自传感器侧时间基”下的时间戳。
- 这些时间戳与同步后 `ImuStamped / ImageFrame` 的 `timestamp_us`
  统一使用 `LibXR::MicrosecondTimestamp` 表达，ABI 仍保持 64 位微秒时间戳。
- 这些时间戳只保证在**各自数据域内部**可比较，不能直接把图像域时间戳和 IMU
  域时间戳拿来做跨域最近邻匹配。
- 跨域同步关系应先通过专门的同步策略锁定，再在 IMU 域内使用 `offset`
  推导最终样本。
- `SensorSyncCmd` 是一次性同步探针命令。
- 当前约定下，采集端只需要临时改变一次图像发布节拍；主机侧通过 `image_event`
  的到达时间差变化识别探针帧，不依赖显式 `seq/id` 回填。

## 备注

- 图像字节数在编译期由 `CameraInfo.step * CameraInfo.height` 推导。
- 图像 sink 切槽回调现在使用 `LibXR::Callback<ImageFrame*&>`，
  不再单独保留裸函数指针 + context。
- `name / image_topic_name / imu_topic_name` 内部保存为 `std::string_view`，
  仍保留 `const char*` getter 便于现有模块继续接 C 风格接口。
- 这个模块刻意把 ABI 保持成固定尺寸 / 标准布局，便于共享内存与 topic 搬运。
- `CameraTypes::BuildPnPDistCoeffs(...)` 是编译期静态转换 helper。
  推荐在静态相机信息定义旁直接生成并缓存结果，而不是在运行时反复构造。
