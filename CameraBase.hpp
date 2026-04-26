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

/**
 * @class CameraTypes
 * @brief 相机静态类型定义容器。
 */
class CameraTypes
{
 public:
  /**
   * @enum Encoding
   * @brief 图像像素编码格式。
   */
  enum Encoding : uint8_t
  {
    INVALID = 0,   ///< 无效或未初始化的编码占位。
    RGB8,          ///< 8 位 RGB 三通道。
    BGR8,          ///< 8 位 BGR 三通道。
    RGBA8,         ///< 8 位 RGBA 四通道。
    BGRA8,         ///< 8 位 BGRA 四通道。
    RGB16,         ///< 16 位 RGB 三通道。
    BGR16,         ///< 16 位 BGR 三通道。
    RGBA16,        ///< 16 位 RGBA 四通道。
    BGRA16,        ///< 16 位 BGRA 四通道。
    MONO8,         ///< 8 位单通道灰度图。
    MONO16,        ///< 16 位单通道灰度图。
    BAYER_RGGB8,   ///< 8 位 Bayer，RGGB 排列。
    BAYER_GRBG8,   ///< 8 位 Bayer，GRBG 排列。
    BAYER_GBRG8,   ///< 8 位 Bayer，GBRG 排列。
    BAYER_BGGR8,   ///< 8 位 Bayer，BGGR 排列。
    BAYER_RGGB16,  ///< 16 位 Bayer，RGGB 排列。
    BAYER_GRBG16,  ///< 16 位 Bayer，GRBG 排列。
    BAYER_GBRG16,  ///< 16 位 Bayer，GBRG 排列。
    BAYER_BGGR16,  ///< 16 位 Bayer，BGGR 排列。
    YUV422         ///< 打包 YUV 4:2:2。
  };

  /**
   * @enum DistortionModel
   * @brief 相机畸变模型。
   */
  enum class DistortionModel : uint8_t
  {
    NONE = 0,             ///< 无畸变模型。
    PLUMB_BOB,            ///< Brown-Conrady / plumb_bob。
    RATIONAL_POLYNOMIAL,  ///< OpenCV 扩展有理多项式模型。
    EQUIDISTANT,          ///< 等距鱼眼模型。
    FOV,                  ///< FOV 畸变模型。
    OMNI,                 ///< 统一全向模型。
    EXTENDED_UNIFIED,     ///< 扩展统一相机模型。
    DOUBLE_SPHERE,        ///< 双球模型。
    THIN_PRISM,           ///< 薄棱镜模型。
    UNKNOWN               ///< 未知或自定义模型。
  };

  /**
   * @struct CameraInfo
   * @brief 编译期静态相机描述。
   */
  struct CameraInfo
  {
    uint32_t width{};  ///< 图像宽度，单位像素。
    uint32_t height{};  ///< 图像高度，单位像素。
    uint32_t step{};  ///< 每行字节数，即 stride。
    Encoding encoding{};  ///< 像素编码格式。
    std::array<double, 9> camera_matrix;  ///< 3x3 相机内参矩阵 K，按行优先存储。
    DistortionModel distortion_model{};  ///< 畸变模型类型。
    std::array<double, 14> distortion_coefficients;  ///< 畸变参数，布局跟随 ROS CameraInfo。
    std::array<double, 9> rectification_matrix;  ///< 3x3 校正旋转矩阵 R，按行优先存储。
    std::array<double, 12> projection_matrix;  ///< 3x4 投影矩阵 P，按行优先存储。
  };

  /**
   * @struct PnPDistCoeffs
   * @brief PnP 使用的固定尺寸畸变系数描述。
   *
   * @note 这里保持为纯静态数据，便于编译期生成，再由运行时封装成 `cv::Mat`。
   */
  struct PnPDistCoeffs
  {
    std::array<double, 8> values{};  ///< 当前 PnP 后端会消费的畸变系数。
    uint8_t size{};  ///< `values` 中有效系数个数。
    bool uses_rational_polynomial_extension{};  ///< 是否命中 8 项 rational 扩展路径。
    bool requires_undistort_first{};  ///< 当前模型是否应先去畸变再进入 PnP。
  };

  /**
   * @brief 按当前静态相机模型生成 PnP 所需的固定畸变系数描述。
   *
   * @note 当前只直接支持 OpenCV 常用 pinhole / rational 两类输入。
   *       其他模型后续应先做去畸变，再按无畸变 pinhole 进入 PnP。
   */
  [[nodiscard]] static constexpr PnPDistCoeffs BuildPnPDistCoeffs(
      const CameraInfo& info)
  {
    PnPDistCoeffs dc{};
    switch (info.distortion_model)
    {
      case DistortionModel::NONE:
        break;

      case DistortionModel::PLUMB_BOB:
        dc.values[0] = info.distortion_coefficients[0];
        dc.values[1] = info.distortion_coefficients[1];
        dc.values[2] = info.distortion_coefficients[2];
        dc.values[3] = info.distortion_coefficients[3];
        dc.values[4] = info.distortion_coefficients[4];
        dc.size = 5;
        break;

      case DistortionModel::RATIONAL_POLYNOMIAL:
        dc.values[0] = info.distortion_coefficients[0];
        dc.values[1] = info.distortion_coefficients[1];
        dc.values[2] = info.distortion_coefficients[2];
        dc.values[3] = info.distortion_coefficients[3];
        dc.values[4] = info.distortion_coefficients[4];
        dc.values[5] = info.distortion_coefficients[5];
        dc.values[6] = info.distortion_coefficients[6];
        dc.values[7] = info.distortion_coefficients[7];
        dc.size = 8;
        dc.uses_rational_polynomial_extension = true;
        break;

      case DistortionModel::EQUIDISTANT:
      case DistortionModel::FOV:
      case DistortionModel::OMNI:
      case DistortionModel::EXTENDED_UNIFIED:
      case DistortionModel::DOUBLE_SPHERE:
      case DistortionModel::THIN_PRISM:
      case DistortionModel::UNKNOWN:
      default:
        dc.requires_undistort_first = true;
        break;
    }
    return dc;
  }
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

  /**
   * @struct ImageFrame
   * @brief 固定尺寸的原始图像帧载荷。
   */
  struct alignas(image_alignment) ImageFrame
  {
    uint64_t timestamp_us;  ///< 图像时间戳，单位微秒。
    alignas(image_alignment) std::array<uint8_t, image_bytes> data;  ///< 图像字节负载。
  };

  /**
   * @struct ImuStamped
   * @brief 与图像同步搬运的位姿与惯导采样。
   */
  struct ImuStamped
  {
    uint64_t timestamp_us;  ///< IMU/位姿时间戳，单位微秒。
    std::array<float, 4> rotation_wxyz;  ///< 姿态四元数，顺序为 wxyz。
    std::array<float, 3> translation_xyz;  ///< 相机平移，单位米。
    std::array<float, 3> angular_velocity_xyz;  ///< 角速度，单位 rad/s。
    std::array<float, 3> linear_acceleration_xyz;  ///< 线加速度，单位 m/s^2。
  };

  /// 图像生产者提交一帧后，用于切换到下一个可写槽位的回调。
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

  // 调试/bring-up 阶段的临时命令入口。
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
