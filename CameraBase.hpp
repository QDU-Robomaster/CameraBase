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
#include "message.hpp"
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
  static constexpr std::size_t image_alignment = 64;

  // 这一份静态相机描述会被整条视觉链共享，后续模块可直接把分辨率、
  // 内参、畸变模型当作编译期常量使用。
  static inline constexpr CameraInfo camera_info = CameraInfoV;
  static constexpr std::size_t image_bytes =
      static_cast<std::size_t>(camera_info.step) * static_cast<std::size_t>(camera_info.height);

  struct alignas(image_alignment) ImageFrame
  {
    uint64_t timestamp_us;
    alignas(image_alignment) std::array<uint8_t, image_bytes> data;
  };

  struct ImuStamped
  {
    uint64_t timestamp_us;
    std::array<float, 4> rotation_wxyz;
    std::array<float, 3> translation_xyz;
    std::array<float, 3> angular_velocity_xyz;
    std::array<float, 3> linear_acceleration_xyz;
  };

  using ImageCommitCallback = ImageFrame* (*)(void* image_sink_context);

  // 共享图像和 imu 都会跨模块搬运，这里只保留真正影响 ABI 的约束。
  static_assert(camera_info.width > 0, "CameraBase requires non-zero width");
  static_assert(camera_info.height > 0, "CameraBase requires non-zero height");
  static_assert(camera_info.step > 0, "CameraBase requires non-zero step");
  static_assert(image_bytes > 0, "CameraBase requires non-zero image bytes");
  static_assert(std::is_trivially_copyable_v<ImageFrame>,
                "CameraBase::ImageFrame must be trivially copyable");
  static_assert(std::is_standard_layout_v<ImageFrame>,
                "CameraBase::ImageFrame must be standard layout");
  static_assert(std::is_trivially_copyable_v<ImuStamped>,
                "CameraBase::ImuStamped must be trivially copyable");
  static_assert(std::is_standard_layout_v<ImuStamped>,
                "CameraBase::ImuStamped must be standard layout");
  static_assert(alignof(ImageFrame) >= image_alignment,
                "CameraBase::ImageFrame alignment is too small");
  static_assert(offsetof(ImageFrame, data) % image_alignment == 0,
                "CameraBase::ImageFrame image payload must stay aligned");

  CameraBase(LibXR::HardwareContainer& hw, const char* name = "camera",
             const char* image_topic_name = "camera_image",
             const char* imu_topic_name = "camera_imu")
      : name_(name),
        image_topic_name_(image_topic_name),
        imu_topic_name_(imu_topic_name),
        cmd_file_(LibXR::RamFS::CreateFile(name, CommandFun, this)),
        imu_topic_(LibXR::Topic::FindOrCreate<ImuStamped>(imu_topic_name_))
  {
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
  }

  virtual ~CameraBase() = default;

  virtual void SetExposure(double exposure) = 0;
  virtual void SetGain(double gain) = 0;

  const char* ImageTopicName() const { return image_topic_name_; }

  const char* ImuTopicName() const { return imu_topic_name_; }

  void PublishImu(ImuStamped imu) { imu_topic_.Publish(imu); }

  bool RegisterImageSink(void* image_sink_context, ImageFrame* initial_image,
                         ImageCommitCallback commit_callback)
  {
    if (image_sink_context == nullptr || initial_image == nullptr || commit_callback == nullptr)
    {
      XR_LOG_ERROR("CameraBase(%s): image sink registration got null argument", name_);
      return false;
    }
    if (image_sink_registered_.load(std::memory_order_acquire))
    {
      XR_LOG_ERROR("CameraBase(%s): image sink already registered", name_);
      return false;
    }

    image_sink_context_ = image_sink_context;
    writable_image_ = initial_image;
    image_commit_callback_ = commit_callback;
    image_sink_registered_.store(true, std::memory_order_release);
    return true;
  }

  bool ImageSinkReady() const
  {
    return image_sink_registered_.load(std::memory_order_acquire) &&
           writable_image_ != nullptr;
  }

  ImageFrame* GetWritableImage() { return ImageSinkReady() ? writable_image_ : nullptr; }

  bool CommitImage()
  {
    if (!ImageSinkReady() || image_commit_callback_ == nullptr)
    {
      return false;
    }

    ImageFrame* next_image = image_commit_callback_(image_sink_context_);
    if (next_image == nullptr)
    {
      XR_LOG_ERROR("CameraBase(%s): image sink callback returned null writable image", name_);
      return false;
    }

    writable_image_ = next_image;
    return true;
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
  const char* name_;
  const char* image_topic_name_;
  const char* imu_topic_name_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Topic imu_topic_;
  void* image_sink_context_{nullptr};
  ImageFrame* writable_image_{nullptr};
  ImageCommitCallback image_commit_callback_{nullptr};
  std::atomic<bool> image_sink_registered_{false};
};
