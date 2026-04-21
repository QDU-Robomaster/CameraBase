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
#include <cstdint>
#include <cstdlib>
#include <cstring>
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

  // shared-memory 传输会直接把这些载荷当裸字节块处理，所以这里只保留
  // 真正影响静态几何合法性的约束。
  static_assert(camera_info.width > 0, "CameraBase requires non-zero width");
  static_assert(camera_info.height > 0, "CameraBase requires non-zero height");
  static_assert(camera_info.step > 0, "CameraBase requires non-zero step");

  CameraBase(LibXR::HardwareContainer& hw, const char* name = "camera")
      : name_(name), cmd_file_(LibXR::RamFS::CreateFile(name, CommandFun, this))
  {
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
  }

  virtual ~CameraBase() = default;

  virtual void SetExposure(double exposure) = 0;
  virtual void SetGain(double gain) = 0;

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
  const char* name_;
  LibXR::RamFS::File cmd_file_;
};
