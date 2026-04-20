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

#include <array>
#include <cstddef>
#include <cstdint>
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

  // Camera model is fixed by the template argument so downstream modules can
  // use width/height/intrinsics as compile-time constants.
  static inline constexpr CameraInfo camera_info = CameraInfoV;

  // Shared image buffers are aligned so later SIMD/shared-memory users do not
  // need an extra copy.
  static constexpr std::size_t frame_data_alignment = 64;
  static constexpr std::size_t frame_bytes =
      static_cast<std::size_t>(camera_info.step) * static_cast<std::size_t>(camera_info.height);

  // One published frame: monotonic timing metadata plus tightly packed pixels.
  struct alignas(frame_data_alignment) Frame
  {
    // Capture time in microseconds and a monotonically increasing frame id.
    uint64_t timestamp_us;
    uint64_t sequence;

    // Pixel bytes are stored as a single packed image buffer.
    alignas(frame_data_alignment) std::array<uint8_t, frame_bytes> data;
  };

  // Preserve an explicit name for the shared-topic payload type used by camera
  // publishers/consumers.
  using SharedImageFrame = Frame;

  // Enforce a POD-like payload layout so shared-memory/topic transport can treat
  // Frame as a plain byte block.
  static_assert(camera_info.width > 0, "CameraBase requires non-zero width");
  static_assert(camera_info.height > 0, "CameraBase requires non-zero height");
  static_assert(camera_info.step > 0, "CameraBase requires non-zero step");
  static_assert(frame_bytes > 0, "CameraBase requires non-zero frame bytes");
  static_assert(std::is_trivial_v<Frame>, "Frame must be trivial");
  static_assert(std::is_trivially_copyable_v<Frame>, "Frame must be trivially copyable");
  static_assert(std::is_standard_layout_v<Frame>, "Frame must be standard layout");
  static_assert(alignof(Frame) >= frame_data_alignment, "Frame alignment is too small");
  static_assert(offsetof(Frame, data) % frame_data_alignment == 0,
                "Frame data must be aligned");

  CameraBase(LibXR::HardwareContainer& hw, const char* name = "camera")
      : name_(name), cmd_file_(LibXR::RamFS::CreateFile(name, CommandFun, this))
  {
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
  }

  virtual void SetExposure(double exposure) = 0;
  virtual void SetGain(double gain) = 0;

  // RamFS command entry used for quick manual tuning in bring-up.
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

  static int CommandAdapter(void* instance, int argc, char** argv)
  {
    // Bridge the generic RamFS callback signature back to the typed handler.
    return CommandFun(static_cast<CameraBase*>(instance), argc, argv);
  }

 protected:
  const char* name_;
  LibXR::RamFS::File cmd_file_;
};
