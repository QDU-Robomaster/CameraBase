# CameraBase

相机共用类型与单进程内共享图像所有权的基类 / Common camera types and a base class for in-process shared image ownership

## 1. 模块作用 / Purpose

CameraBase 是库型模块（`standalone: false`），由其他模块包含和派生使用。相机驱动、同步、检测、跟踪和瞄准等模块在各自的 `depends` 中声明 `QDU-Robomaster/CameraBase` 并包含 `CameraBase.hpp`。

CameraBase 提供三部分内容：

- `CameraTypes`：像素编码、帧存储布局、原生传感器标定、逐帧采样几何、固定采样档位，以及 PnP 畸变系数换算等 `constexpr` 工具函数。
- `CameraBaseIntrinsicSanity`：原生内参的合理性检查规则，以及在线标定使用的质量指标与文本报告。
- `CameraBase<FrameLayoutV>`：相机生产者基类。派生的相机驱动在采集线程中向基类的两个图像槽写入图像并提交，基类通过 Topic 发布图像，并提供 RamFS 命令文件 `set_exposure` / `set_gain`。

CameraBase 在构造时创建一个 RamFS 命令文件。图像槽、Topic 订阅回调和采集线程的约定见第 3 节。

CameraBase is a library Module (`standalone: false`) that other Modules include and derive from. Camera driver, synchronization, detection, tracking and aiming Modules declare `QDU-Robomaster/CameraBase` in their `depends` and include `CameraBase.hpp`.

CameraBase provides three parts:

- `CameraTypes`: pixel encodings, frame storage layout, native sensor calibration, per-frame sampling geometry, fixed sampling profiles, and `constexpr` helpers such as the PnP distortion coefficient conversion.
- `CameraBaseIntrinsicSanity`: sanity rules for the native intrinsics, plus quality metrics and a text report for online calibration.
- `CameraBase<FrameLayoutV>`: the camera producer base class. A derived camera driver writes images into the two image slots of the base class on its capture thread and commits them; the base class publishes the images on a Topic and provides the RamFS command file `set_exposure` / `set_gain`.

CameraBase creates one RamFS command file at construction. Section 3 describes the conventions for the image slots, Topic subscription callbacks and the capture thread.

## 2. 类型与几何 / Types and Geometry

`CameraTypes::FrameLayout` 是编译期固定的图像存储布局，作为 `CameraBase` 的模板参数：

- `width`、`height`：图像宽高，单位像素。
- `step`：每行字节数。
- `encoding`：像素编码 `CameraTypes::Encoding`，例如 `BGR8`、`MONO8`、`BAYER_RGGB8`。

编译期检查要求宽高和 `step` 非零、`step` 能容纳一行像素，`YUV422` 的宽度为偶数。`YUV422` 每像素两字节，字节顺序由图像源约定。

`CameraTypes::CameraCalibration` 是原生传感器坐标系下的标定，数组均按行存储：

- `native_width`、`native_height`：标定对应的原生传感器尺寸，单位像素。
- `camera_matrix`：3x3 内参矩阵。
- `distortion_model`：畸变模型 `CameraTypes::DistortionModel`。
- `distortion_coefficients`：14 项畸变系数，布局跟随 ROS CameraInfo。
- `rectification_matrix`：3x3 校正矩阵，按值携带。
- `projection_matrix`：3x4 投影矩阵，按值携带。

`CameraBase` 构造时对 `CameraCalibration` 做合理性检查，检查失败触发 `REQUIRE`。检查项为：原生尺寸非零；`camera_matrix` 为 pinhole 形状（`k[1]`、`k[3]`、`k[6]`、`k[7]` 为 0，`k[8]` 为 1）；`fx`、`fy` 为正且落在图像长边的 0.15 到 10 倍之间；`fx / fy` 在 0.5 到 2 之间；主点落在图像范围外扩 25% 之内；畸变系数有限且绝对值不超过 10。

`CameraTypes::FrameGeometry` 描述一帧图像到原生传感器坐标的映射，携带当前帧尺寸和步长、原生 ROI 偏移、横纵下采样倍率 `decimation_x` / `decimation_y`、翻转标志 `FrameGeometryFlags` 和采样相位。`FrameToNative()` 与 `NativeToFrame()` 实现坐标映射：

```text
oriented = reverse ? extent - 1 - frame : frame
native = roi_offset + sample_phase + decimation * oriented
```

`ValidateFrameGeometry()` 检查一份 `FrameGeometry` 与 `FrameLayout`、`CameraCalibration` 是否一致：尺寸和步长与布局相同，下采样倍率非零，采样相位有限且小于下采样倍率，标志位有效，映射结果落在原生标定范围内。同一镜头保存一份 `CameraCalibration`；ROI、下采样和翻转由每帧的 `FrameGeometry` 表达。

`CameraTypes::CameraProfile` 描述相机支持的一个固定采样档位：唯一的 `ProfileId`（`WIDE` 或 `NARROW`）、逐帧几何 `geometry` 和非零的触发周期 `trigger_period_us`。`AppliedProfile` 是切档成功后驱动返回的实际档位与几何。

`CameraTypes::BuildPnPDistCoeffs()` 把 `NONE`（0 项）、`PLUMB_BOB`（5 项）和 `RATIONAL_POLYNOMIAL`（8 项）换算为 PnP 使用的畸变系数；其他畸变模型在返回值中标记 `requires_undistort_first`。

`CameraBase<FrameLayoutV>::ImageFrame` 表示一帧图像：

- `timestamp_us`：源传感器或录制文件的采样时间，单位微秒。
- `geometry`：该帧的 `FrameGeometry`。
- `data`：图像数据，大小为 `FrameLayoutV.step * FrameLayoutV.height`，含每行 padding。

`CameraBase<FrameLayoutV>::ImuStamped` 表示一帧同步 IMU 数据：

- `timestamp_us`：生成同步结果的模块定义的时间，单位微秒。
- `rotation_wxyz`：姿态四元数，顺序为 `w, x, y, z`。
- `translation_xyz`：平移，单位 m。
- `angular_velocity_xyz`：角速度，单位 rad/s。
- `linear_acceleration_xyz`：线加速度，单位 m/s²。

`FrameGeometry`、`ImageFrame` 和 `ImuStamped` 是标准布局、可平凡复制的类型。`FrameGeometry` 为 36 字节；`ImageFrame::geometry` 位于偏移 8，`ImageFrame::data` 位于 64 字节对齐的偏移 64；`ImuStamped` 为 64 字节。这些布局是共享的二进制约定，参与的模块使用同一版本的 CameraBase 编译。`step` 的单位是字节。

`CameraTypes::FrameLayout` is the compile-time image storage layout and the template parameter of `CameraBase`:

- `width`, `height`: image width and height in pixels.
- `step`: bytes per row.
- `encoding`: pixel encoding `CameraTypes::Encoding`, for example `BGR8`, `MONO8` or `BAYER_RGGB8`.

The compile-time checks require non-zero width, height and `step`, a `step` that holds one row of pixels, and an even width for `YUV422`. `YUV422` uses two bytes per pixel with the byte order defined by the image source.

`CameraTypes::CameraCalibration` is the calibration in native sensor coordinates; all arrays are stored row by row:

- `native_width`, `native_height`: native sensor size the calibration refers to, in pixels.
- `camera_matrix`: 3x3 intrinsic matrix.
- `distortion_model`: distortion model `CameraTypes::DistortionModel`.
- `distortion_coefficients`: 14 distortion coefficients, laid out as in ROS CameraInfo.
- `rectification_matrix`: 3x3 rectification matrix, carried by value.
- `projection_matrix`: 3x4 projection matrix, carried by value.

`CameraBase` checks the `CameraCalibration` for plausibility at construction, and a failed check triggers `REQUIRE`. The checks are: non-zero native size; a pinhole-shaped `camera_matrix` (`k[1]`, `k[3]`, `k[6]`, `k[7]` equal to 0 and `k[8]` equal to 1); positive `fx` and `fy` between 0.15 and 10 times the longer image side; `fx / fy` between 0.5 and 2; a principal point within the image extended by 25%; finite distortion coefficients with absolute values of at most 10.

`CameraTypes::FrameGeometry` describes the mapping from one image frame to native sensor coordinates. It carries the frame size and step, the native ROI offset, the horizontal and vertical decimation factors `decimation_x` / `decimation_y`, the flip flags `FrameGeometryFlags` and the sampling phase. `FrameToNative()` and `NativeToFrame()` implement the coordinate mapping given in the code block above.

`ValidateFrameGeometry()` checks that a `FrameGeometry` is consistent with the `FrameLayout` and the `CameraCalibration`: size and step equal to the layout, non-zero decimation factors, a finite sampling phase smaller than the decimation factor, valid flag bits, and a mapped extent inside the native calibration range. One `CameraCalibration` is kept per lens; ROI, decimation and flipping are expressed by the per-frame `FrameGeometry`.

`CameraTypes::CameraProfile` describes one fixed sampling profile supported by the camera: a unique `ProfileId` (`WIDE` or `NARROW`), the per-frame geometry `geometry` and a non-zero trigger period `trigger_period_us`. `AppliedProfile` is the profile and geometry the driver reports after a successful profile switch.

`CameraTypes::BuildPnPDistCoeffs()` converts `NONE` (0 coefficients), `PLUMB_BOB` (5) and `RATIONAL_POLYNOMIAL` (8) into the distortion coefficients used by PnP; other distortion models are flagged `requires_undistort_first` in the return value.

`CameraBase<FrameLayoutV>::ImageFrame` represents one image frame:

- `timestamp_us`: sampling time of the source sensor or recorded file, in microseconds.
- `geometry`: the `FrameGeometry` of this frame.
- `data`: image data of `FrameLayoutV.step * FrameLayoutV.height` bytes, including row padding.

`CameraBase<FrameLayoutV>::ImuStamped` represents one synchronized IMU sample:

- `timestamp_us`: time defined by the Module that produces the synchronized result, in microseconds.
- `rotation_wxyz`: attitude quaternion in the order `w, x, y, z`.
- `translation_xyz`: translation in m.
- `angular_velocity_xyz`: angular velocity in rad/s.
- `linear_acceleration_xyz`: linear acceleration in m/s².

`FrameGeometry`, `ImageFrame` and `ImuStamped` are standard-layout, trivially copyable types. `FrameGeometry` is 36 bytes; `ImageFrame::geometry` is at offset 8 and `ImageFrame::data` at the 64-byte-aligned offset 64; `ImuStamped` is 64 bytes. These layouts are a shared binary convention, and the participating Modules are built with the same CameraBase version. `step` is measured in bytes.

## 3. 图像发布与所有权 / Image Publication and Ownership

CameraBase 有两个进程内图像槽。相机生产与发布图像的顺序为：

1. 采集线程调用 `GetWritableImage()`，取得当前可写的 `ImageFrame*`。
2. 相机写入 `timestamp_us`、`geometry` 和 `data`。
3. 相机调用 `CommitImage()`。
4. CameraBase 在同一线程内向图像 Topic 发布临时的 `const SharedFrame*`。
5. 需要保留图像的订阅者在 `Topic::Callback` 内复制 `SharedFrame`，把副本移动到稳定的异步工作槽位后返回。
6. 最后一个 `SharedFrame` 析构时，图像槽回到 CameraBase。

`const SharedFrame*` 在当前 `Publish()` 的同步回调期间有效。`SharedFrame` 的复制只增加槽位的引用计数，`ImageFrame::data` 保持原处，因此多个模块可以在不同线程持有同一帧。所有副本只读访问同一个 `ImageFrame`；可写指针只授予当前的生产者，经 Topic 传递的句柄为只读。`CommitImage()` 之后，生产者使用下一次 `GetWritableImage()` 返回的指针。

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

`CommitImage()` 返回 `true` 表示当前帧已完成同步发布，返回 `false` 表示没有可写帧。`GetWritableImage()` 在需要时从池中取槽；两个槽都被下游持有时返回 `nullptr`，下游释放任一槽位后再次调用即可取得。发布时没有订阅者的图像槽在发布结束后立即回到池中。派生驱动停流并等待采集线程退出后，可调用受保护的 `DiscardWritableImage()` 释放尚未提交的槽位，该调用不发布图像，也不等待下游持有的槽位。

订阅者在同步 `Topic::Callback` 内复制 `SharedFrame` 以取得图像所有权。订阅回调只复制句柄并交给稳定的工作槽位；推理、输出整理和解码等处理在工作线程中异步执行。必须在原始图像发布前完成的采集侧预处理，在 `CommitImage()` 之前完成。

`GetWritableImage()` 与 `CommitImage()` 由同一个采集线程调用。图像 Topic 的回调在该线程内同步执行，该 Topic 只由这个采集线程发布。`SharedFrame` 可以复制后跨线程移动和析构；同一个句柄对象被多个线程访问时，由调用方负责同步。

`SharedFrame` 用于单进程内的共享所有权，其生命周期不超过所属的 CameraBase；CameraBase 销毁前，所有 `SharedFrame` 已释放。

对象池、背压和析构的实现细节见 [docs/internals.md](docs/internals.md)。

CameraBase has two in-process image slots. A camera produces and publishes an image in this order:

1. The capture thread calls `GetWritableImage()` and obtains the current writable `ImageFrame*`.
2. The camera writes `timestamp_us`, `geometry` and `data`.
3. The camera calls `CommitImage()`.
4. CameraBase publishes a temporary `const SharedFrame*` on the image Topic on the same thread.
5. A subscriber that keeps the image copies the `SharedFrame` inside `Topic::Callback`, moves the copy to a stable asynchronous worker slot and returns.
6. When the last `SharedFrame` is destroyed, the image slot returns to CameraBase.

The `const SharedFrame*` is valid during the synchronous callback of the current `Publish()`. Copying a `SharedFrame` only increments the slot reference count and `ImageFrame::data` stays in place, so several Modules can hold the same frame on different threads. All copies read the same `ImageFrame` read-only; the writable pointer is granted only to the current producer, and the handle passed through the Topic is read-only. After `CommitImage()`, the producer uses the pointer returned by the next `GetWritableImage()`.

`CommitImage()` returns `true` when the current frame has been published synchronously and `false` when there is no writable frame. `GetWritableImage()` takes a slot from the pool when needed; it returns `nullptr` while both slots are held downstream, and succeeds again after downstream releases either slot. An image slot published without subscribers returns to the pool when the publication ends. After the derived driver stops the stream and waits for the capture thread to exit, it can call the protected `DiscardWritableImage()` to release an uncommitted slot; the call publishes no image and does not wait for slots held downstream.

A subscriber takes image ownership by copying the `SharedFrame` inside the synchronous `Topic::Callback`. The subscription callback only copies the handle and hands it to a stable worker slot; inference, output assembly and decoding run asynchronously on the worker thread. Capture-side preprocessing that must happen before the raw image is published completes before `CommitImage()`.

`GetWritableImage()` and `CommitImage()` are called from the same capture thread. Callbacks of the image Topic run synchronously on that thread, and only this capture thread publishes that Topic. A `SharedFrame` can be copied, moved across threads and destroyed there; access to one handle object from several threads must be synchronized by the caller.

`SharedFrame` provides shared ownership within one process and does not outlive its CameraBase; all `SharedFrame` handles are released before the CameraBase is destroyed.

The implementation details of the object pool, the back pressure and the destruction are in [docs/internals.md](docs/internals.md).

## 4. 时间戳 / Timestamps

`ImageFrame::timestamp_us` 是同步所用的源采样时间，对应传感器的采样时刻。实时相机使用设备时钟换算值，并保留其采样语义。

CameraBase 保存和发布时间戳的原值。重复、回退、回绕、复位和跨源排序由同步模块按各自的时间线策略接受、复位或丢帧。确定性回放保持输入记录的原始顺序和时间，包括原始重复值。

`ImageFrame::timestamp_us` is the source sampling time used for synchronization and corresponds to the sampling instant of the sensor. A real-time camera uses the converted device clock and keeps its sampling semantics.

CameraBase stores and publishes the timestamp value as given. Duplicates, regressions, wrap-around, resets and cross-source ordering are accepted, reset or dropped by the synchronization Module according to its own timeline policy. Deterministic replay keeps the original order and times of the input recording, including original duplicates.

## 5. 构造接口 / Constructor

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class CameraBase;

CameraBase(LibXR::RamFS& ramfs, CameraCalibration calibration,
           std::string_view name = "camera",
           std::string_view image_topic_name = "camera_image",
           std::string_view imu_topic_name = "camera_imu");
```

模板参数：

- `FrameLayoutV`：`CameraTypes::FrameLayout`，整条视觉链共享的编译期帧存储布局，需与相机实际输出一致。

依赖：

- `ramfs`：`LibXR::RamFS`，注册相机命令文件的 RamFS。

配置参数：

- `calibration`：`CameraCalibration`，原生传感器标定，按值持有，`Calibration()` 返回其引用；无默认值。
- `name`：相机实例名，也是 RamFS 命令文件名，默认 `"camera"`。
- `image_topic_name`：图像 Topic 名称，默认 `"camera_image"`。
- `imu_topic_name`：`PublishImu()` 发布 `ImuStamped` 的 Topic 名称，默认 `"camera_imu"`。

RamFS 命令文件 `<name>`：

```text
set_exposure <value>
set_gain <value>
```

不带参数时打印用法。数值的单位和范围由具体相机实现定义，HikCamera 的曝光单位为微秒。

派生的相机驱动实现四个纯虚函数：

```cpp
void SetExposure(double exposure) override;
void SetGain(double gain) override;
std::span<const CameraProfile> Profiles() const noexcept override;
LibXR::ErrorCode SwitchProfile(ProfileId id, AppliedProfile& applied) override;
```

`Profiles()` 返回的表非空，地址和顺序在相机生命周期内保持稳定；首项是构造完成时的当前档位，`id` 唯一，触发周期非零。`SwitchProfile()` 是阻塞调用：请求当前档位时返回 `OK` 并填写 `applied`，不支持的档位返回 `NOT_SUPPORT`，失败时 `applied` 保持原值。切档失败后的相机状态由派生驱动和上层状态机处理。

采集代码只写当前可写槽位：

```cpp
auto* image = GetWritableImage();
if (image == nullptr) {
  return;
}

image->timestamp_us = timestamp;
image->geometry = frame_geometry;
// 写入 / write image->data

if (!CommitImage()) {
  return;
}
```

其他成员：`AvailableImageSlots()` 返回当前空闲图像槽数，用于监控；`PublishImu()` 发布 `ImuStamped`；`Calibration()`、`Name()`、`ImageTopicName()`、`ImuTopicName()` 及对应的 `*View()` 返回构造时保存的值。

Template parameter:

- `FrameLayoutV`: `CameraTypes::FrameLayout`, the compile-time frame storage layout shared by the whole vision chain; it matches the actual camera output.

Dependencies:

- `ramfs`: `LibXR::RamFS`, the RamFS that receives the camera command file.

Configuration parameters:

- `calibration`: `CameraCalibration`, the native sensor calibration, held by value and returned by reference from `Calibration()`; no default.
- `name`: camera instance name, also the RamFS command file name, default `"camera"`.
- `image_topic_name`: name of the image Topic, default `"camera_image"`.
- `imu_topic_name`: name of the Topic on which `PublishImu()` publishes `ImuStamped`, default `"camera_imu"`.

RamFS command file `<name>`, with the commands in the code block above. Without arguments the file prints its usage. The unit and range of the values are defined by the concrete camera; the HikCamera exposure unit is microseconds.

A derived camera driver implements the four pure virtual functions shown above.

The table returned by `Profiles()` is non-empty, and its address and order stay stable for the camera lifetime; the first entry is the profile in effect after construction, every `id` is unique and every trigger period is non-zero. `SwitchProfile()` is a blocking call: requesting the current profile returns `OK` and fills `applied`, an unsupported profile returns `NOT_SUPPORT`, and `applied` keeps its previous value on failure. The camera state after a failed switch is handled by the derived driver and the upper-level state machine.

Capture code writes only the current writable slot, as in the code block above.

Other members: `AvailableImageSlots()` returns the number of free image slots for monitoring; `PublishImu()` publishes `ImuStamped`; `Calibration()`, `Name()`, `ImageTopicName()`, `ImuTopicName()` and the matching `*View()` functions return the values stored at construction.

## 6. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `image_topic_name`（默认 `camera_image`） | 发布 | `const SharedFrame*` | `CommitImage()` 发布的图像句柄，仅在同步回调期间有效 |
| `imu_topic_name`（默认 `camera_imu`） | 发布 | `ImuStamped` | `PublishImu()` 发布的同步 IMU 数据 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `image_topic_name` (default `camera_image`) | Publish | `const SharedFrame*` | Image handle published by `CommitImage()`, valid only during the synchronous callback |
| `imu_topic_name` (default `camera_imu`) | Publish | `ImuStamped` | Synchronized IMU data published by `PublishImu()` |

## 7. 配置示例 / Configuration Example

BSP 在 `User/xrobot.yaml` 的 `constexprs` 中定义 `CameraTypes::FrameLayout` 与 `CameraTypes::CameraCalibration`，相机和视觉模块的实例通过 `template_args` 与 `args` 引用它们。整条视觉链（相机、同步、检测、跟踪、瞄准）使用同一个 `FrameLayout`，并与相机实际输出一致。

The BSP defines `CameraTypes::FrameLayout` and `CameraTypes::CameraCalibration` in the `constexprs` of `User/xrobot.yaml`, and the camera and vision Module instances reference them through `template_args` and `args`. The whole vision chain (camera, synchronization, detection, tracking, aiming) uses the same `FrameLayout`, and it matches the actual camera output.

```yaml
constexpr_namespace: AutoAimRunConfig
constexpr_includes:
  - CameraBase.hpp
constexprs:
  MainCameraCalibration:
    type: CameraTypes::CameraCalibration
    value: '{.native_width = 1440, .native_height = 1080, .camera_matrix = {2328.685719898089, 0.0, 733.3564625092474, 0.0, 2328.670107789996, 540.6187286922773, 0.0, 0.0, 1.0}, .distortion_model = CameraTypes::DistortionModel::PLUMB_BOB, .distortion_coefficients = {-0.09182103918709904, 0.4639907346830205, 0.002609878642637282, 0.0009819586010405485, -0.4751278850310457}, .rectification_matrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, .projection_matrix = {2328.685719898089, 0.0, 733.3564625092474, 0.0, 0.0, 2328.670107789996, 540.6187286922773, 0.0, 0.0, 0.0, 1.0, 0.0}}'
  HikFrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 720, .height = 540, .step = 2160, .encoding = CameraTypes::Encoding::BGR8}'
```

## 8. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：由派生自 `CameraBase` 的相机驱动模块接入具体的相机硬件。

Dependencies: LibXR.

Hardware: the camera driver Modules derived from `CameraBase` connect the concrete camera hardware.
