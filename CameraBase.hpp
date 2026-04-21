#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机基础类型与像素编码定义 / Base camera types & encodings
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "logger.hpp"
#include "ramfs.hpp"

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <type_traits>
#include <vector>

/**
 * @class CameraTypes
 * @brief 相机静态类型定义容器。
 */
class CameraTypes
{
 public:
  enum Encoding : uint8_t
  {
    INVALID = 0,
    RGB8,
    BGR8,
    RGBA8,
    BGRA8,
    RGB16,
    BGR16,
    RGBA16,
    BGRA16,
    MONO8,
    MONO16,
    BAYER_RGGB8,
    BAYER_GRBG8,
    BAYER_GBRG8,
    BAYER_BGGR8,
    BAYER_RGGB16,
    BAYER_GRBG16,
    BAYER_GBRG16,
    BAYER_BGGR16,
    YUV422
  };

  enum class DistortionModel : uint8_t
  {
    NONE = 0,
    PLUMB_BOB,
    RATIONAL_POLYNOMIAL,
    EQUIDISTANT,
    FOV,
    OMNI,
    EXTENDED_UNIFIED,
    DOUBLE_SPHERE,
    THIN_PRISM,
    UNKNOWN
  };

  struct CameraInfo
  {
    // Image geometry and row stride in bytes.
    uint32_t width{};
    uint32_t height{};
    uint32_t step{};
    Encoding encoding{};

    // Standard pinhole camera intrinsics.
    std::array<double, 9> camera_matrix;
    DistortionModel distortion_model{};

    // Distortion/rectification/projection layout follows ROS CameraInfo.
    std::array<double, 14> distortion_coefficients;
    std::array<double, 9> rectification_matrix;
    std::array<double, 12> projection_matrix;

    static inline std::vector<double> ToPnPDistCoeffs(
        DistortionModel model, const std::array<double, 14>& distortion_coeffs)
    {
      std::vector<double> dc;
      switch (model)
      {
        case DistortionModel::NONE:
          break;

        case DistortionModel::PLUMB_BOB:
          dc = {distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2],
                distortion_coeffs[3], distortion_coeffs[4]};
          break;

        case DistortionModel::RATIONAL_POLYNOMIAL:
          dc = {distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2],
                distortion_coeffs[3], distortion_coeffs[4], distortion_coeffs[5],
                distortion_coeffs[6], distortion_coeffs[7]};
          XR_LOG_WARN(
              "PnPSolver: using 8-term rational; extend to 14 if backend supports.");
          break;

        case DistortionModel::EQUIDISTANT:
        case DistortionModel::FOV:
        case DistortionModel::OMNI:
        case DistortionModel::EXTENDED_UNIFIED:
        case DistortionModel::DOUBLE_SPHERE:
        case DistortionModel::THIN_PRISM:
        case DistortionModel::UNKNOWN:
        default:
          XR_LOG_WARN(
              "PnPSolver: distortion model not natively supported (%d). "
              "TODO: undistort to pinhole first, then call PnP with NONE.",
              int(model));
          break;
      }
      return dc;
    }
  };
};

/**
 * @class CameraBase
 * @brief 编译期绑定相机静态信息的基类。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CameraBase
{
 public:
  using Encoding = CameraTypes::Encoding;
  using DistortionModel = CameraTypes::DistortionModel;
  using CameraInfo = CameraTypes::CameraInfo;

  // 这一份静态相机描述会被整条视觉链共享，后续模块可直接把分辨率、
  // 内参、畸变模型当作编译期常量使用。
  static inline constexpr CameraInfo camera_info = CameraInfoV;

  // 共享图像帧按固定对齐存放，后续 shared topic / SIMD 用户可以直接消费。
  static constexpr std::size_t frame_data_alignment = 64;
  static constexpr std::size_t frame_bytes =
      static_cast<std::size_t>(camera_info.step) * static_cast<std::size_t>(camera_info.height);

  // 一帧完整图像：时间戳、序号，以及一块连续像素缓冲区。
  struct alignas(frame_data_alignment) Frame
  {
    // 采集时间戳（us）与单调递增帧号。
    uint64_t timestamp_us;
    uint64_t sequence;

    // 像素数据按 step * height 连续存放。
    alignas(frame_data_alignment) std::array<uint8_t, frame_bytes> data;
  };

  // 保留一层显式别名，方便生产者/消费者直接表达“共享图像帧”。
  using SharedImageFrame = Frame;

  // 这些元数据类型需要保持 trivial，不能依赖默认成员初始化。
  struct Pose
  {
    std::array<float, 4> rotation_wxyz;
    std::array<float, 3> translation_xyz;
  };

  struct Motion
  {
    std::array<float, 3> angular_velocity_xyz;
    std::array<float, 3> linear_acceleration_xyz;
  };

  struct FrameContext
  {
    uint64_t timestamp_us;
    uint64_t sequence;
    Pose pose;
    Motion motion;
  };

  struct FrameLease
  {
    uint8_t* image_data;
    std::size_t image_step;
    void* private_data;
  };

  class Sink
  {
   public:
    virtual ~Sink() = default;

    // 为当前帧借出一块可写目标缓冲区。
    virtual bool AcquireFrame(const FrameContext& context, FrameLease& lease) = 0;

    // 相机写完 lease.image_data 后，调用该函数发布整帧。
    virtual void CommitFrame(FrameLease& lease) = 0;

    // 放弃当前借出的缓冲区，不发布。
    virtual void AbortFrame(FrameLease& lease) = 0;
  };

  // shared-memory 传输会直接把这些载荷当裸字节块处理，所以这里强约束
  // trivial / standard-layout。
  static_assert(camera_info.width > 0, "CameraBase requires non-zero width");
  static_assert(camera_info.height > 0, "CameraBase requires non-zero height");
  static_assert(camera_info.step > 0, "CameraBase requires non-zero step");
  static_assert(frame_bytes > 0, "CameraBase requires non-zero frame bytes");
  static_assert(std::is_trivial_v<Frame>, "Frame must be trivial");
  static_assert(std::is_trivially_copyable_v<Frame>, "Frame must be trivially copyable");
  static_assert(std::is_standard_layout_v<Frame>, "Frame must be standard layout");
  static_assert(std::is_trivial_v<Pose>, "Pose must be trivial");
  static_assert(std::is_trivially_copyable_v<Pose>, "Pose must be trivially copyable");
  static_assert(std::is_trivial_v<Motion>, "Motion must be trivial");
  static_assert(std::is_trivially_copyable_v<Motion>, "Motion must be trivially copyable");
  static_assert(std::is_trivial_v<FrameContext>, "FrameContext must be trivial");
  static_assert(std::is_trivially_copyable_v<FrameContext>,
                "FrameContext must be trivially copyable");
  static_assert(std::is_trivial_v<FrameLease>, "FrameLease must be trivial");
  static_assert(std::is_trivially_copyable_v<FrameLease>,
                "FrameLease must be trivially copyable");
  static_assert(alignof(Frame) >= frame_data_alignment, "Frame alignment is too small");
  static_assert(offsetof(Frame, data) % frame_data_alignment == 0,
                "Frame data must be aligned");

  CameraBase(LibXR::HardwareContainer& hw, const char* name = "camera")
      : name_(name), cmd_file_(LibXR::RamFS::CreateFile(name, CommandFun, this))
  {
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
  }

  virtual ~CameraBase() = default;

  virtual void SetExposure(double exposure) = 0;
  virtual void SetGain(double gain) = 0;

  bool RegisterSink(Sink& sink)
  {
    Sink* expected = nullptr;
    if (!sink_.compare_exchange_strong(expected, &sink, std::memory_order_acq_rel,
                                       std::memory_order_acquire))
    {
      XR_LOG_ERROR("CameraBase: sink already registered on %s", name_);
      return false;
    }
    return true;
  }

  void UnregisterSink(Sink& sink)
  {
    Sink* expected = &sink;
    if (!sink_.compare_exchange_strong(expected, nullptr, std::memory_order_acq_rel,
                                       std::memory_order_acquire) &&
        expected != nullptr)
    {
      XR_LOG_WARN("CameraBase: unregister ignored because another sink is active on %s",
                  name_);
    }
  }

  // bring-up 阶段的临时命令入口。
  static int CommandFun(CameraBase* self, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf("Camera: %s\n\n", self->name_);
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf("  set_exposure <exposure>\r\n");
      LibXR::STDIO::Printf("  set_gain <gain>\r\n");
      return 0;
    }
    else if (argc == 3)
    {
      if (strcmp(argv[1], "set_exposure") == 0)
      {
        self->SetExposure(atof(argv[2]));
        return 0;
      }
      if (strcmp(argv[1], "set_gain") == 0)
      {
        self->SetGain(atof(argv[2]));
        return 0;
      }
    }

    LibXR::STDIO::Printf("Unknown command: %s\n", argv[1]);
    return -1;
  }

 protected:
  Sink* GetSink() const { return sink_.load(std::memory_order_acquire); }

  const char* name_;
  LibXR::RamFS::File cmd_file_;
  std::atomic<Sink*> sink_{nullptr};
};
