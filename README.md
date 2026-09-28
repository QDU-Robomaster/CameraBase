# CameraBase

CameraBase 提供相机模块共用的 C++ 类型，以及单进程内的共享图像所有权协议。它拥有
两个固定图像槽，但不打开相机、不保存消息队列，也不创建工作线程。

## 类型

`CameraTypes::FrameLayout` 描述编译期固定的图像存储布局：

- `width`：图像宽度，单位像素
- `height`：图像高度，单位像素
- `step`：每行字节数
- `encoding`：像素格式（`CameraTypes::Encoding`，如 `BGR8`、`MONO8`、`BAYER_RGGB8`）

`CameraTypes::CameraCalibration` 描述原生传感器坐标系下的标定：

- `native_width / native_height`：标定对应的原生传感器尺寸
- `camera_matrix`：3x3 内参矩阵，按行存储
- `distortion_model`：畸变模型
- `distortion_coefficients`：畸变参数（14 项，布局跟随 ROS CameraInfo）
- `rectification_matrix`：3x3 校正矩阵，按行存储
- `projection_matrix`：3x4 投影矩阵，按行存储

构造期 sanity 只检查原生尺寸、内参矩阵和畸变系数；`rectification_matrix` 与
`projection_matrix` 仅按值携带，不在本模块内验证。`CameraTypes::BuildPnPDistCoeffs()`
把 `NONE`、`PLUMB_BOB`（5 项）和 `RATIONAL_POLYNOMIAL`（8 项）转换为 PnP 用畸变系数，
其他模型标记为需要先去畸变。

`CameraTypes::FrameGeometry` 描述一帧图像到原生传感器坐标的映射。它携带
当前帧尺寸和步长、原生 ROI 偏移、横纵下采样倍率、翻转标志和采样相位。
坐标映射为（`FrameToNative()` / `NativeToFrame()`）：

```text
oriented = reverse ? extent - 1 - frame : frame
native = roi_offset + sample_phase + decimation * oriented
```

同一镜头只保存一份 `CameraCalibration`；ROI、下采样和翻转不修改内参，而是由每帧
`FrameGeometry` 表达。

`CameraTypes::CameraProfile` 描述相机支持的固定采样档位，包括唯一的
`ProfileId::{WIDE,NARROW}`、逐帧几何和非零触发周期。`AppliedProfile` 是切档成功后
驱动返回的实际档位与几何快照。

`CameraBase<FrameLayoutV>::ImageFrame` 表示一帧图像：

- `timestamp_us`：源传感器或录制文件的采样时间，单位微秒
- `geometry`：当前帧按值携带的 `FrameGeometry`
- `data`：固定大小图像数据，大小为 `FrameLayoutV.step * FrameLayoutV.height`

`CameraBase<FrameLayoutV>::ImuStamped` 表示一帧同步 IMU 数据。阶段包装可以把相机本地
时间的 `ImageFrame` 与 MCU 时间的 `ImuStamped` 关联起来，两者时间戳不要求数值相等：

- `timestamp_us`：生成同步结果的模块定义的权威时间，单位微秒
- `rotation_wxyz`：四元数，顺序为 `w, x, y, z`
- `translation_xyz`：平移，单位米
- `angular_velocity_xyz`：角速度，单位 `rad/s`
- `linear_acceleration_xyz`：线加速度，单位 `m/s^2`

## 配置检查

`CameraBase<FrameLayoutV>` 在编译期检查帧布局，并在构造时检查按值传入的
`CameraCalibration`（不合理时 `REQUIRE` 失败）。`CameraTypes::ValidateFrameGeometry(...)`
用于检查每帧采样几何。

检查内容包括：

- 布局宽高和 `step` 非零，且 `step` 能容纳一行像素；`YUV422` 宽度须为偶数
- 内参矩阵为 pinhole 形状，焦距和主点落在合理范围内
- `fx / fy` 比例不能明显异常，畸变系数有限且不过大
- `FrameGeometry` 的下采样倍率和采样相位有效
- 当前帧尺寸和 `step` 与 `FrameLayoutV` 一致，映射结果不超出原生标定范围

这些检查只拒绝错误配置，不会缩放或改写标定。

## 发布契约

CameraBase 使用 `LibXR::MPMCObjectPool<ImageFrame>` 管理两个进程内图像槽。相机
生产和发布顺序是：

1. 相机调用 `GetWritableImage()`
2. 相机写入 `timestamp_us`、`geometry` 和 `data`
3. 相机调用 `CommitImage()`
4. CameraBase 在同一采集线程内发布临时 `const SharedFrame*`
5. 所有权订阅者在 `Topic::Callback` 内复制 `SharedFrame`
6. 回调把句柄移动到稳定的异步工作槽位后立即返回
7. 最后一个 `SharedFrame` 析构时，图像槽自动返回 CameraBase

`const SharedFrame*` 只在当前 `Publish()` 的同步回调期间有效。回调不得保存这个
裸指针。`SharedFrame` 的复制只增加槽位引用计数，不复制 `ImageFrame::data`，因此
多个模块可以在不同线程保留同一帧。

```cpp
using Camera = CameraBase<layout>;

void OnImage(bool, Worker* worker, const Camera::SharedFrame* borrowed) {
  if (borrowed == nullptr) {
    return;
  }
  Camera::SharedFrame owned = *borrowed;
  worker->Enqueue(std::move(owned));
}
```

所有副本指向同一个 `ImageFrame`，并且只能通过 `SharedFrame` 只读访问。可写指针只由
CameraBase 私有图像池授予当前唯一生产者；生产者调用 `CommitImage()` 后不得再访问旧
写指针，写权限不会随 topic 中的句柄传播到下游。

`CommitImage()` 返回 true 表示当前帧已经完成同步发布，返回值不描述下一槽位。
下一次 `GetWritableImage()` 会按需获取槽位；两个槽都仍被下游持有时返回 nullptr，
任一持有者释放后即可恢复。没有订阅者时，发布结束即归还当前槽位。

派生驱动在采集线程确认停流后、切档前可调用受保护的 `DiscardWritableImage()`，立即
释放尚未提交的生产者槽位。该操作不发布图像、不等待下游仍持有的其他槽位；后续
`GetWritableImage()` 会按需重新取槽。

不能用 `Topic::SyncSubscriber` 或 `Topic::QueuedSubscriber` 接管图像所有权：前者
可能错过发布，后者队列满时会静默丢弃，而普通 `Topic::Publish()` 不返回逐订阅者的
接收结果。所有权必须在同步 `Topic::Callback` 内通过复制句柄建立。

## 时间戳

`ImageFrame::timestamp_us` 是同步所用的源采样时间，不是主机到达、解码、预处理或
发布时间。实时相机应使用设备时钟换算值并保留其采样语义。CameraBase 不比较时间戳，
不重映射时钟域，也不处理复位、回绕、回退或跨源排序；同步模块按自己的时间线策略
接受、复位或丢帧。确定性回放必须保持输入记录的原始顺序和时间，包括原始重复值，
不能为了制造单调时间线而改写数据。

## 预处理与背压

必须发生在原始图像发布前的采集侧预处理，应在 `CommitImage()` 前完成。CameraBase
不执行算法预处理，也不启动额外线程。

订阅回调只复制句柄并把它交给稳定的工作槽位，推理、输出整理和解码等重处理在工作
线程中异步执行。工作槽位拒绝接收时，回调中的临时副本自然析构。CameraBase 的
固定两槽池形成有界背压：一槽可由采集线程写入，另一槽可由下游异步链路持有；具体等待、
丢帧和计数策略由相机或订阅模块定义。

## 线程与生命周期

`GetWritableImage()` 与 `CommitImage()` 必须始终由同一个采集线程调用。普通 topic
callback 在该采集线程内同步执行，不得递归发布同一个图像 topic。图像句柄可以复制后
跨线程移动和析构，但同一个句柄对象不能被多个线程无同步地同时修改。

CameraBase、callback 目标、槽位池和工作线程按进程生命周期存在。所有 `SharedFrame`
都不得超过所属 CameraBase 的生命周期。CameraBase 析构不注销 callback、不等待异步
持有者，也不停止派生模块的工作线程；本模块不定义 stop token 或 join 流程。

## RamFS 命令

CameraBase 会创建一个 RamFS 命令文件，名字和相机实例名（构造参数 `name`）相同。

```text
set_exposure <值>
set_gain <值>
```

不带参数时打印用法。单位和范围由具体相机实现决定（HikCamera 的曝光单位为微秒）。

## 依赖

本仓库是库（`standalone: false`），不会被单独实例化；相机与视觉模块在自己的
`depends` 中声明 `QDU-Robomaster/CameraBase` 并包含 `CameraBase.hpp`。无其他模块
依赖，仅使用 LibXR。

## 公共接口

- `CameraTypes`：`Encoding`、`DistortionModel`、`FrameLayout`、`CameraCalibration`、
  `FrameGeometry`、`CameraProfile`、`AppliedProfile`，以及 `BytesPerPixel()`、
  `ValidateFrameLayout()`、`ValidateFrameGeometry()`、`FrameToNative()`、
  `NativeToFrame()`、`BuildPnPDistCoeffs()` 等 constexpr 工具函数。
- `CameraBaseIntrinsicSanity`：构造期内参检查规则，以及在线标定用的质量指标
  （`Evaluate()`）。
- `CameraBase<FrameLayoutV>`：相机生产者基类，派生相机驱动通过下面的构造函数把
  RamFS、原生标定和 topic 名称交给基类：

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class CameraBase;

CameraBase(LibXR::RamFS& ramfs, CameraCalibration calibration,
           std::string_view name = "camera",
           std::string_view image_topic_name = "camera_image",
           std::string_view imu_topic_name = "camera_imu");
```

  - `ramfs`：注册相机命令文件的 RamFS。
  - `calibration`：原生标定，按值持有，`Calibration()` 返回其引用。
  - `name`：相机实例名，也是 RamFS 命令文件名。
  - `image_topic_name`：图像 topic，载荷类型为 `const SharedFrame*`。
  - `imu_topic_name`：`PublishImu()` 发布 `ImuStamped` 的 topic。

  主要成员：`GetWritableImage()`、`CommitImage()`、`AvailableImageSlots()`、
  `PublishImu()`、`Calibration()`、`Name()`、`ImageTopicName()`、`ImuTopicName()`，
  受保护的 `DiscardWritableImage()`，以及派生类必须实现的纯虚函数（见下节）。

## 如何实现相机

继承 `CameraBase<FrameLayoutV>`，构造时把原生标定传给基类，并实现：

```cpp
void SetExposure(double exposure) override;
void SetGain(double gain) override;
std::span<const CameraProfile> Profiles() const noexcept override;
LibXR::ErrorCode SwitchProfile(ProfileId id, AppliedProfile& applied) override;
```

`Profiles()` 返回的表必须非空且在相机生命周期内保持地址和顺序稳定；首项是构造完成
时的当前档位，ID 唯一，触发周期非零。`SwitchProfile()` 是阻塞事务：同档请求也返回
`OK` 并填写 `applied`，不支持返回 `NOT_SUPPORT`，其他失败同样不得修改 `applied`。
失败后的实际相机状态由派生驱动和上层状态机处理，本接口不承诺自动回滚或恢复采集。

采集时只写当前可写槽位：

```cpp
auto* image = GetWritableImage();
if (image == nullptr) {
  return;
}

image->timestamp_us = timestamp;
image->geometry = frame_geometry;
// 写入 image->data

if (!CommitImage()) {
  // 当前没有可提交的写槽位
  return;
}
```

## 使用

依赖本库的模块会通过 `depends` 自动引入它，一般无需手动添加；需要单独引入时：

```sh
xrobot module add QDU-Robomaster/CameraBase
xrobot setup
```

本库不创建实例。模板化的视觉模块以 `CameraTypes::FrameLayout` 作为模板参数，BSP 在
`User/xrobot.yaml` 中用 constexpr 定义布局，再在各实例的 `template_args` 中引用它：

```yaml
constexpr_includes:
  - CameraBase.hpp
constexprs:
  FrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 640, .height = 480, .step = 1920, .encoding = CameraTypes::Encoding::BGR8}'
```

整条视觉链（相机、同步、检测、跟踪、瞄准）必须使用同一个布局，且它必须与相机实际
输出一致。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/CameraBase`
（在 BSP 中）打印清单。

## 测试

在打开 `BUILD_TESTING` 的 BSP 构建中，本模块会加入 `tests/` 下的
`camera_geometry_test`、`camera_image_sink_test` 和 `camera_intrinsic_sanity_test`，
用 `ctest` 运行。

## 说明

- `step` 是字节数，不是像素数
- `YUV422` 只保证每像素两字节且宽度为偶数，打包顺序仍由具体源约定；当前主链不用它
- `FrameGeometry` 固定为 36 字节，按值进入 `ImageFrame`
- `ImageFrame` 和 `ImuStamped` 均为标准布局、可平凡复制类型
- `ImageFrame::geometry` 位于偏移 8，图像数据从 64 字节对齐的偏移 64 开始；
  `ImuStamped` 固定为 64 字节
- 上述载荷布局属于 ABI；所有参与模块必须使用同一 CameraBase 版本重编译，并随同一
  进程一起重启，不能和旧布局混用
- CameraBase 保存两个固定图像槽和一个当前可写句柄，不保存消息队列
- `SharedFrame` 只用于单进程共享所有权，不可序列化或跨进程传递
- 原生内参放在不可变 `CameraCalibration`，相机外参不放在这里
