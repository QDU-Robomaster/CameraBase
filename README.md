# CameraBase

`CameraBase` 是当前 Webots / Linux 自瞄链路里的相机前半段边界模块。

它本身不实现具体相机驱动，而是定义整条视觉链共享的编译期相机描述，
以及具体相机模块会用到的生产者侧原始图像帧 / IMU ABI。

## 运行时职责

- 编译期唯一相机静态真值源：
  - `CameraTypes::CameraInfo`
- 生产者侧载荷定义：
  - `CameraBase<Info>::ImageFrame`
  - `CameraBase<Info>::ImuStamped`
- 生产者侧图像可写槽位注册与提交边界
- 为具体相机实现提供 IMU topic 发布辅助

## 当前边界

- `CameraBase<Info>` 持有原始生产者侧类型，以及强类型图像 lease 边界
- `WebotsCamera<Info>` 或其他具体相机模块负责：
  - 填充 `ImuStamped`
  - 把图像字节写入已注册的 `ImageFrame`
  - 调用 `CommitImage()`
- `CameraFrameSync<Info>` 持有共享图像发布与图像/IMU 同步桥
- 下游模块应消费这个 sync 边界，而不是重新拼装旧式混合大载荷

## 对外约定

- `CameraTypes::CameraInfo`
  - 固定尺寸的编译期相机静态信息
  - `width / height / step / encoding`
  - 相机内参、畸变模型、校正矩阵、投影矩阵
- `CameraBase<Info>::ImageFrame`
  - 由编译期 `CameraInfo` 推导出的固定尺寸图像载荷
  - 按共享内存安全传输要求对齐
- `CameraBase<Info>::ImuStamped`
  - 带时间戳的姿态、平移、角速度、线加速度
- 图像 sink API：
  - `RegisterImageSink(ImageLeaseSink&)`
  - `ImageSinkReady()`
  - `GetWritableImage()`
  - `CommitImage()`

## 说明

- 图像载荷字节数在编译期由 `CameraInfo.step * CameraInfo.height` 推导
- 这个模块刻意把前半段 ABI 保持为 trivial / standard-layout，
  让共享内存边界尽量稳定、可预测
- 图像 lease 注册接口现在是强类型且收窄后的边界：
  - 一个 sink 对象提供当前可写图像槽位
  - 每次 `CommitImage()` 返回下一块可写图像槽位
  - 不再保留 `void* + function pointer` 这种伪通用回调接口
- `CameraTypes::BuildPnPDistCoeffs(...)` 提供面向 PnP 的编译期畸变描述辅助
