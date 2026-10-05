# CameraBase

相机基类：两档视角、图像池、采集线程与图像 Topic / Camera base class with two views, an image pool, the capture thread and the image Topic

## 1. 模块作用 / Purpose

CameraBase 是库型模块（`standalone: false`），相机驱动从它派生。驱动只做两件事：取一帧（`GrabFrame`）和把硬件改到指定几何（`ApplyView`）。采集线程、图像池、几何与标定的填写、图像 Topic 的发布都由 CameraBase 完成。下游模块只按名字订阅图像 Topic，不持有相机对象。

CameraBase is a library Module (`standalone: false`) that camera drivers derive from. A driver does two things: grab one frame (`GrabFrame`) and reconfigure the hardware to a given geometry (`ApplyView`). CameraBase runs the capture thread, owns the image pool, stamps the geometry and calibration, and publishes the image Topic. Downstream Modules subscribe to the image Topic by name and hold no camera object.

## 2. 帧与几何 / Frame and Geometry

全系统只有一种帧：640×512 BayerRG8，`(0, 0)` 为 R，每行 640 字节。`ImageFrame` 包含采样时间 `timestamp_us`、几何 `geometry`、相机帧计数 `frame_counter`、标定指针 `calibration` 和像素 `data`。

The whole system uses one frame: 640×512 BayerRG8 with R at `(0, 0)` and 640 bytes per row. `ImageFrame` holds the sampling time `timestamp_us`, the geometry `geometry`, the camera frame counter `frame_counter`, the calibration pointer `calibration` and the pixels `data`.

标定 `CameraCalibration` 在原生传感器坐标下给出：原生宽高、`fx`、`fy`、`cx`、`cy` 与 plumb_bob 的 5 个畸变系数。几何 `FrameGeometry` 说明帧覆盖的原生窗口：起点 `roi_x`、`roi_y` 与跳采倍率 `decimation`（1 或 2）。`FrameToNative` 与 `NativeToFrame` 在两种坐标间换算；跳采 2 时相机按 Bayer 单元取每个 4×4 原生块左上角的 2×2，连续坐标的映射为 `x_n = roi_x + 2x − 0.5`。

`CameraCalibration` is given in native sensor coordinates: native width and height, `fx`, `fy`, `cx`, `cy` and the five plumb_bob distortion coefficients. `FrameGeometry` describes the native window the frame covers: the origin `roi_x`, `roi_y` and the decimation `decimation` (1 or 2). `FrameToNative` and `NativeToFrame` convert between the two; with decimation 2 the camera keeps the top-left 2×2 cell of every 4×4 native block, and the continuous mapping is `x_n = roi_x + 2x − 0.5`.

## 3. 两档视角 / Views

| 视角 / View | 几何 / Geometry |
| --- | --- |
| `View::WIDE` | 原生 (80, 24) 起 1280×1024，2×2 跳采 / 1280×1024 native window at (80, 24), 2×2 skip |
| `View::NARROW` | 640×512，1:1 裁剪，位置由 `NarrowPosition{u, v}` 给出 / 640×512 1:1 crop placed by `NarrowPosition{u, v}` |

`u`、`v` 取 0–1，(0.5, 0.5) 为居中，(0, 0) 与 (1, 1) 为对角；窗口起点对齐到 4 像素。`SwitchView` 阻塞执行：停止采集线程、丢弃写了一半的帧、调用驱动的 `ApplyView`、恢复采集。之后发布的帧带新档位的几何。相机启动在 WIDE。

`u` and `v` range over 0–1; (0.5, 0.5) is centred and (0, 0) and (1, 1) are opposite corners; the window origin is aligned to 4 px. `SwitchView` blocks: it stops the capture thread, drops a half-written frame, calls the driver's `ApplyView` and resumes capture. Frames published afterwards carry the new geometry. The camera starts in WIDE.

## 4. 编写驱动 / Writing a Driver

```cpp
class MyCamera : public CameraBase
{
 public:
  MyCamera(const CameraTypes::CameraCalibration& calibration, NarrowPosition narrow,
           std::string_view name)
      : CameraBase(calibration, narrow, name, SlotPolicy::DROP)
  {
    OpenHardware(CurrentGeometry());
    StartCapture();
  }
  ~MyCamera() override { StopCapture(); }

 protected:
  bool GrabFrame(ImageFrame& frame) override;  // 写 data、timestamp_us、frame_counter
  LibXR::ErrorCode ApplyView(const CameraTypes::FrameGeometry& geometry) override;
};
```

- `GrabFrame` 在采集线程调用，只写像素、时间戳和帧计数；超时或出错返回 false。调用前 `geometry` 已填为当前视角，回放驱动按录像改写它。
- `ApplyView` 调用时采集线程已停止。
- 没有空槽时，`SlotPolicy::DROP`（实时相机）把这一帧取到临时缓冲后丢弃，帧计数随之跳号；`SlotPolicy::WAIT`（回放）等待空槽。
- 每帧发布之后在采集线程调用 `OnPublished(frame)`，默认为空；回放驱动在这里发布同步帧。
- 驱动析构时先调用 `StopCapture`。

- `GrabFrame` runs on the capture thread and writes only the pixels, timestamp and frame counter; it returns false on a timeout or an error. `geometry` already holds the current view when it is called; a replay driver overwrites it with the recorded geometry.
- The capture thread is stopped while `ApplyView` runs.
- With no free slot, `SlotPolicy::DROP` (live cameras) grabs the frame into a scratch buffer and drops it, so the frame counter skips; `SlotPolicy::WAIT` (replay) waits for a slot.
- `OnPublished(frame)` runs on the capture thread after each publication and is empty by default; the replay driver publishes its synced frame there.
- A driver calls `StopCapture` first in its destructor.

## 5. 图像发布与所有权 / Image Publication and Ownership

图像池有 2 个槽。发布时 CameraBase 把可写句柄转成只读的 `SharedFrame`，以 `const SharedFrame*` 同步发布；指针只在回调期间有效。需要保留图像的订阅者在回调里复制句柄，最后一个句柄释放时槽位回到池中。发布的帧一定持有图像。

The image pool has 2 slots. On publication CameraBase turns the writable handle into a read-only `SharedFrame` and publishes `const SharedFrame*` synchronously; the pointer is valid only during the callback. A subscriber that keeps the image copies the handle in the callback; the slot returns to the pool when the last handle is released. A published frame always holds an image.

订阅者长期持有句柄会占住槽位，使相机丢帧。只需要像素内容的订阅者（预览、录制）应拷贝字节而不是持有句柄。相机析构前，所有句柄必须已经释放。

A subscriber that keeps handles for long occupies slots and makes the camera drop frames. Subscribers that only need the pixels (preview, recording) copy the bytes instead of keeping the handle. All handles are released before the camera is destroyed.

## 6. Topic

| Topic | 载荷 / Payload | 创建者 / Created by |
| --- | --- | --- |
| `<name>_image` | `const SharedFrame*` | 相机 / Camera |

`StageTopicName(camera, stage)` 生成 `<camera>_<stage>` 形式的名字，供整条链使用。

`StageTopicName(camera, stage)` produces names of the form `<camera>_<stage>` for the whole chain.

## 7. 依赖 / Dependencies

LibXR（`ObjectPool` 写独占 / 读共享句柄、Topic、Thread）。无硬件依赖。

LibXR (`ObjectPool` with one-writer / shared-reader handles, Topic, Thread). No hardware dependency.
