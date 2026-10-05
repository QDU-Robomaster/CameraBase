#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机基类：两档视角、图像池、采集线程与图像 Topic / Camera base class with two views, an image pool, the capture thread and the image Topic
depends: []
standalone: false
=== END MANIFEST === */
// clang-format on

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <thread>

#include "CameraCalibrationCheck.hpp"
#include "CameraTypes.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "object_pool.hpp"
#include "thread.hpp"

/**
 * @brief 一帧图像。几何、标定指针由 CameraBase 填写，像素、时间戳、帧计数由驱动填写。
 *        One image frame. CameraBase fills the geometry and calibration pointer; the
 *        driver fills the pixels, timestamp and frame counter.
 */
struct alignas(64) ImageFrame
{
  LibXR::MicrosecondTimestamp timestamp_us;  ///< 传感器采样时间 / Sensor sampling time
  CameraTypes::FrameGeometry geometry;       ///< 相对原生传感器 / Relative to the sensor
  uint32_t frame_counter;  ///< 相机帧计数，每次开始取流从 0 起 / Restarts per stream
  const CameraTypes::CameraCalibration* calibration;  ///< 相机存活期间有效 / Camera-owned
  alignas(64) std::array<uint8_t, CameraTypes::FRAME_BYTES> data;  ///< BayerRG8 像素
};

using ImagePool = LibXR::ObjectPool<ImageFrame>;
/// 已发布的图像：只读、可复制，一定持有图像 / Published image: read-only, copyable,
/// always holds a frame.
using SharedFrame = ImagePool::ConstHandle;
/// 图像 Topic 载荷：只在同步回调期间有效，需要保留的订阅者复制句柄。
/// Image Topic payload, valid only during the synchronous callback; subscribers that
/// keep the image copy the handle.
using ImageTopicPayload = const SharedFrame*;

/// 两档视角 / The two views.
enum class View : uint8_t
{
  WIDE,    ///< 全视角，相机侧 2×2 跳采 / Full view, 2×2 skip in the camera
  NARROW,  ///< 远方视角，相机侧 1:1 裁剪 / Far view, 1:1 crop in the camera
};

/// NARROW 窗口位置：0–1 比例，(0.5, 0.5) 为居中 / NARROW window position as 0–1 ratios.
struct NarrowPosition
{
  double u;  ///< 水平 / Horizontal
  double v;  ///< 垂直 / Vertical
};

/// 某台相机某一层的 Topic 名：`<camera>_<stage>` / Topic name of one stage of a camera.
inline std::string StageTopicName(std::string_view camera, std::string_view stage)
{
  return std::string(camera) + "_" + std::string(stage);
}

/**
 * @brief 相机基类。驱动只负责取一帧（GrabFrame）和改硬件（ApplyView）；采集线程、图像池、
 *        几何与标定的填写、图像 Topic 的发布都在这里。
 *        Camera base class. Drivers only grab one frame (GrabFrame) and reconfigure
 *        the hardware (ApplyView); the capture thread, the image pool, geometry and
 *        calibration stamping and the image Topic live here.
 *
 * 图像 Topic 名为 `<name>_image`，由相机创建。池有 2 个槽；没有空槽时实时相机把这一帧
 * 取到临时缓冲后丢弃，回放相机等待。
 * The image Topic `<name>_image` is created by the camera. The pool has 2 slots; with
 * no free slot a live camera grabs the frame into a scratch buffer and drops it, while
 * a replay camera waits.
 */
class CameraBase
{
 public:
  /// WIDE 档几何：原生 (80, 24) 起 1280×1024 窗口，2×2 跳采（v4 训练几何）。
  /// WIDE geometry: 1280×1024 native window at (80, 24) with 2×2 skip (v4 training).
  static constexpr CameraTypes::FrameGeometry WIDE_GEOMETRY{80, 24, 2};
  static constexpr std::size_t IMAGE_SLOT_COUNT = 2;

  /// 没有空槽时的做法 / What to do when no slot is free.
  enum class SlotPolicy : uint8_t
  {
    DROP,  ///< 取到临时缓冲后丢弃（实时相机）/ Grab into scratch and drop (live)
    WAIT,  ///< 等待空槽（回放）/ Wait for a free slot (replay)
  };

  CameraBase(const CameraBase&) = delete;
  CameraBase& operator=(const CameraBase&) = delete;

  /// 驱动须在自己的析构里先调用 StopCapture / Drivers call StopCapture in their own
  /// destructor first.
  virtual ~CameraBase() { ASSERT(!thread_.joinable()); }

  /**
   * @brief 阻塞切换视角；之后发布的帧带新档位的几何。
   *        Switch the view, blocking; frames published afterwards carry its geometry.
   * @return 驱动 ApplyView 的结果；失败时采集保持停止，等开发者处理。
   *         The driver's ApplyView result; on failure capture stays stopped.
   */
  LibXR::ErrorCode SwitchView(View view)
  {
    if (view == view_)
    {
      return LibXR::ErrorCode::OK;
    }
    const CameraTypes::FrameGeometry& geometry =
        view == View::WIDE ? WIDE_GEOMETRY : narrow_geometry_;
    StopCapture();
    writing_.Reset();  // 写了一半的帧属于旧档位 / A half-written frame has the old view
    const LibXR::ErrorCode result = ApplyView(geometry);
    if (result != LibXR::ErrorCode::OK)
    {
      XR_LOG_ERROR("%s: switching view failed (%d); capture stopped", name_.c_str(),
                   static_cast<int>(result));
      return result;
    }
    view_ = view;
    geometry_ = geometry;
    StartCapture();
    return LibXR::ErrorCode::OK;
  }

  /// 原生标定，相机存活期间地址不变 / Native calibration, stable for the camera lifetime.
  const CameraTypes::CameraCalibration& Calibration() const { return calibration_; }
  const std::string& Name() const { return name_; }
  View CurrentView() const { return view_; }

  /// 打印周期摘要 / Print the periodic summary.
  void OnMonitor()
  {
    XR_LOG_INFO("%s: published=%u dropped_no_slot=%u grab_failed=%u", name_.c_str(),
                published_.exchange(0), dropped_.exchange(0), grab_failed_.exchange(0));
  }

 protected:
  /**
   * @param calibration 原生标定 / Native calibration
   * @param narrow NARROW 窗口位置 / NARROW window position
   * @param name 相机名，图像 Topic 为 `<name>_image` / Camera name
   * @param policy 没有空槽时的做法 / Policy when no slot is free
   */
  CameraBase(const CameraTypes::CameraCalibration& calibration, NarrowPosition narrow,
             std::string_view name, SlotPolicy policy)
      : calibration_(calibration),
        narrow_geometry_(NarrowGeometry(calibration, narrow)),
        name_(name),
        topic_name_(StageTopicName(name, "image")),
        policy_(policy),
        topic_(LibXR::Topic::CreateTopic<ImageTopicPayload>(topic_name_.c_str()))
  {
    REQUIRE(CameraTypes::CalibrationReasonable(calibration_));
    REQUIRE(CameraTypes::GeometryInsideSensor(WIDE_GEOMETRY, calibration_));
    REQUIRE(narrow.u >= 0.0 && narrow.u <= 1.0 && narrow.v >= 0.0 && narrow.v <= 1.0);
    REQUIRE(CameraTypes::GeometryInsideSensor(narrow_geometry_, calibration_));
  }

  /**
   * @brief 取一帧：写 `data`、`timestamp_us`、`frame_counter`。在采集线程调用。
   *        Grab one frame: write `data`, `timestamp_us` and `frame_counter`. Called on
   *        the capture thread.
   * @return 取到一帧返回 true；超时或出错返回 false。
   */
  virtual bool GrabFrame(ImageFrame& frame) = 0;

  /**
   * @brief 把硬件改到给定几何。调用时采集线程已停止。
   *        Reconfigure the hardware to the geometry. The capture thread is stopped.
   */
  virtual LibXR::ErrorCode ApplyView(const CameraTypes::FrameGeometry& geometry) = 0;

  /// 驱动构造完成、硬件就绪后调用 / Called by the driver once the hardware is ready.
  void StartCapture()
  {
    running_.store(true);
    thread_ = std::thread([this]() { CaptureLoop(); });
  }

  /// 驱动析构前调用 / Called by the driver before destruction.
  void StopCapture()
  {
    running_.store(false);
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  /// 驱动用来判断是否应继续阻塞等待 / Lets a driver stop blocking waits.
  bool CaptureRunning() const { return running_.load(); }

  /// 驱动初始启动时的几何（WIDE）/ Geometry the driver starts with (WIDE).
  const CameraTypes::FrameGeometry& CurrentGeometry() const { return geometry_; }

 private:
  static CameraTypes::FrameGeometry NarrowGeometry(
      const CameraTypes::CameraCalibration& calibration, NarrowPosition narrow)
  {
    // 对齐到 4 像素：保持 Bayer 相位，且是 13T 上验证过的取值网格。
    // Align to 4 px: keeps the Bayer phase and matches the grid verified on 13T.
    const auto align = [](double ratio, uint32_t range)
    { return static_cast<uint32_t>(std::lround(ratio * range / 4.0) * 4); };
    return {align(narrow.u, calibration.native_width - CameraTypes::FRAME_WIDTH),
            align(narrow.v, calibration.native_height - CameraTypes::FRAME_HEIGHT), 1};
  }

  void CaptureLoop()
  {
    while (running_.load())
    {
      ImageFrame* frame = Acquire();
      if (frame == nullptr)
      {
        LibXR::Thread::Sleep(1);  // WAIT 策略 / WAIT policy
        continue;
      }
      if (!GrabFrame(*frame))
      {
        grab_failed_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      if (frame == &scratch_)
      {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      Publish();
    }
  }

  ImageFrame* Acquire()
  {
    if (!writing_.Valid() && pool_.Acquire(writing_) != LibXR::ErrorCode::OK)
    {
      return policy_ == SlotPolicy::DROP ? &scratch_ : nullptr;
    }
    return &writing_.Get();
  }

  void Publish()
  {
    writing_->geometry = geometry_;
    writing_->calibration = &calibration_;
    SharedFrame frame = std::move(writing_);
    ImageTopicPayload payload = &frame;
    topic_.Publish(payload);
    published_.fetch_add(1, std::memory_order_relaxed);
  }

  const CameraTypes::CameraCalibration calibration_;
  const CameraTypes::FrameGeometry narrow_geometry_;
  const std::string name_;
  const std::string topic_name_;
  const SlotPolicy policy_;
  LibXR::Topic topic_;
  ImagePool pool_{IMAGE_SLOT_COUNT};
  ImagePool::Handle writing_;
  ImageFrame scratch_{};  // DROP 策略下没有空槽时的落脚处 / Landing buffer for DROP
  View view_ = View::WIDE;
  CameraTypes::FrameGeometry geometry_ = WIDE_GEOMETRY;
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::atomic<uint32_t> published_{0};
  std::atomic<uint32_t> dropped_{0};
  std::atomic<uint32_t> grab_failed_{0};
};
