#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机基础类型、像素编码和图像/IMU 同步 ABI
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

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <type_traits>

/**
 * @class CameraTypes
 * @brief 相机 ABI 相关的纯类型定义容器。
 *
 * 本类不保存运行时状态。视觉链路中各模块通过这些类型共享图像像素格式、
 * CameraInfo 内参和同步 IMU 载荷约定。
 */
class CameraTypes
{
 public:
  /**
   * @enum Encoding
   * @brief 图像像素编码格式 ABI。
   *
   * 多字节通道值使用目标平台原生字节序。`step` 字段负责描述每行实际字节数，
   * 因此消费者不能从 `width` 和 `Encoding` 反推出行跨度。
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
   * @brief 相机畸变模型 ABI。
   *
   * 枚举值用于解释 `CameraInfo::distortion_coefficients` 的含义。当前 PnP
   * 直通路径只消费 pinhole 常用模型，其它模型需要调用方先完成去畸变。
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
   *
   * 该结构作为模板参数在视觉模块间传播，所有数组均按行优先存储。这里不携带
   * 手眼外参、坐标系固定旋转或运行时采集参数；这些职责属于上层视觉模块。
   */
  struct CameraInfo
  {
    uint32_t width{};   ///< 图像宽度，单位像素。
    uint32_t height{};  ///< 图像高度，单位像素。
    uint32_t step{};    ///< 每行字节数，即 stride；不是每行像素数。
    Encoding encoding{};  ///< 像素编码格式，决定 `ImageFrame::data` 的解释方式。
    std::array<double, 9> camera_matrix;  ///< 3x3 相机内参矩阵 K，row-major。
    DistortionModel distortion_model{};   ///< 畸变模型类型。
    std::array<double, 14> distortion_coefficients;  ///< 畸变参数，布局跟随 ROS CameraInfo。
    std::array<double, 9> rectification_matrix;      ///< 3x3 校正旋转矩阵 R，row-major。
    std::array<double, 12> projection_matrix;        ///< 3x4 投影矩阵 P，row-major。
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
   * @param info 编译期或运行期可见的相机静态描述。
   * @return 固定长度畸变系数和调用方处理建议。
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

#include "CameraBaseIntrinsicSanity.hpp"

/**
 * @class CameraBase
 * @brief 编译期绑定相机静态信息的相机生产者基类。
 *
 * `CameraBase` 只定义相机 ABI 和图像提交边界：具体相机驱动负责填充
 * `ImageFrame`，同步模块通过 `RegisterImageSink()` 提供可写槽位并接收提交。
 * 本类不负责记录、标定、预览、图像同步或坐标系转换。
 *
 * @tparam CameraInfoV 编译期相机内参、分辨率和像素格式描述。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CameraBase
{
 public:
  using Encoding = CameraTypes::Encoding;  ///< 像素编码枚举的局部别名。
  using DistortionModel = CameraTypes::DistortionModel;  ///< 畸变模型枚举的局部别名。
  using CameraInfo = CameraTypes::CameraInfo;  ///< 静态相机描述结构的局部别名。
  /// 图像帧对象和像素负载的最小对齐字节数。
  static constexpr std::size_t image_alignment = 64;

  /// 整条视觉链共享的编译期相机描述。
  static inline constexpr CameraInfo camera_info = CameraInfoV;
  /// 单帧图像负载字节数，等于 `CameraInfo::step * CameraInfo::height`。
  static constexpr std::size_t image_bytes =
      static_cast<std::size_t>(camera_info.step) * static_cast<std::size_t>(camera_info.height);

  /**
   * @struct ImageFrame
   * @brief 固定尺寸的图像帧载荷。
   *
   * `data` 按 `CameraInfo::encoding` 和 `CameraInfo::step` 解释。生产者只能在当前
   * writable slot 生命周期内写入该对象；调用 `CommitImage()` 后不得继续访问旧槽位。
   */
  struct alignas(image_alignment) ImageFrame
  {
    LibXR::MicrosecondTimestamp timestamp_us;  ///< 图像采集时间戳，单位微秒，语义由具体相机源定义。
    alignas(image_alignment) std::array<uint8_t, image_bytes> data;  ///< 图像字节负载，含每行 padding。
  };

  /**
   * @struct ImuStamped
   * @brief 与图像同步搬运的位姿与惯导采样 ABI。
   *
   * `CameraBase` 只提供发布通道；同步、插值和坐标系定义由上游 IMU/同步模块保证。
   */
  struct ImuStamped
  {
    LibXR::MicrosecondTimestamp timestamp_us;  ///< 同步后时间戳，单位微秒。
    std::array<float, 4> rotation_wxyz;        ///< 姿态四元数，顺序为 wxyz。
    std::array<float, 3> translation_xyz;      ///< 平移，单位米；无平移来源时置零。
    std::array<float, 3> angular_velocity_xyz;      ///< 角速度，单位 rad/s。
    std::array<float, 3> linear_acceleration_xyz;   ///< 线加速度，单位 m/s^2。
  };

  /**
   * @brief 图像生产者提交一帧后由 sink 返回下一个可写槽位的回调。
   *
   * 回调参数为输出引用：sink 必须把下一帧可写的 `ImageFrame*` 写回该引用。
   */
  using ImageCommitCallback = LibXR::Callback<ImageFrame*&>;

  /// ABI 约束：时间戳必须保持 64-bit 标准布局。
  static_assert(sizeof(LibXR::MicrosecondTimestamp) == sizeof(uint64_t),
                "CameraBase timestamp ABI must stay 64-bit");
  static_assert(alignof(LibXR::MicrosecondTimestamp) == alignof(uint64_t),
                "CameraBase timestamp ABI alignment changed");
  static_assert(std::is_standard_layout_v<LibXR::MicrosecondTimestamp>,
                "CameraBase timestamp wrapper must stay standard layout");
  static_assert(camera_info.width > 0, "CameraBase requires non-zero width");
  static_assert(camera_info.height > 0, "CameraBase requires non-zero height");
  static_assert(camera_info.step > 0, "CameraBase requires non-zero step");
  static_assert(image_bytes > 0, "CameraBase requires non-zero image bytes");
  static_assert(CameraBaseIntrinsicSanity::CameraInfoReasonable(camera_info),
                "CameraBase compile-time CameraInfo intrinsics are unreasonable");
  static_assert(std::is_standard_layout_v<ImageFrame>,
                "CameraBase::ImageFrame must be standard layout");
  static_assert(std::is_standard_layout_v<ImuStamped>,
                "CameraBase::ImuStamped must be standard layout");
  static_assert(alignof(ImageFrame) >= image_alignment,
                "CameraBase::ImageFrame alignment is too small");
  static_assert(offsetof(ImageFrame, data) % image_alignment == 0,
                "CameraBase::ImageFrame image payload must stay aligned");

  /**
   * @brief 构造相机基础对象并注册调试命令文件。
   *
   * @param hw 硬件容器，必须包含名为 `ramfs` 的 `LibXR::RamFS`。
   * @param name 相机实例名，同时作为 RamFS 命令文件名。
   * @param image_topic_name 图像逻辑 topic 名称，仅用于向下游暴露 ABI 名称。
   * @param imu_topic_name 同步 IMU topic 名称，`PublishImu()` 会发布到该 topic。
   */
  CameraBase(LibXR::HardwareContainer& hw, std::string_view name = "camera",
             std::string_view image_topic_name = "camera_image",
             std::string_view imu_topic_name = "camera_imu")
      : name_(name),
        image_topic_name_(image_topic_name),
        imu_topic_name_(imu_topic_name),
        cmd_file_(LibXR::RamFS::CreateFile(name_.c_str(), CommandFun, this)),
        imu_topic_(LibXR::Topic::FindOrCreate<ImuStamped>(imu_topic_name_.c_str()))
  {
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
  }

  /**
   * @brief 虚析构，允许通过基类指针释放具体相机实现。
   */
  virtual ~CameraBase() = default;

  /**
   * @brief 设置曝光参数。
   *
   * @param exposure 具体单位由相机实现定义，HikCamera 使用微秒。
   */
  virtual void SetExposure(double exposure) = 0;

  /**
   * @brief 设置增益参数。
   *
   * @param gain 具体单位和量纲由相机实现定义。
   */
  virtual void SetGain(double gain) = 0;

  /**
   * @brief 返回相机实例名视图。
   */
  std::string_view NameView() const { return name_; }

  /**
   * @brief 返回以空字符结尾的相机实例名。
   */
  const char* Name() const { return name_.c_str(); }

  /**
   * @brief 返回图像逻辑 topic 名称视图。
   */
  std::string_view ImageTopicNameView() const { return image_topic_name_; }

  /**
   * @brief 返回以空字符结尾的图像逻辑 topic 名称。
   */
  const char* ImageTopicName() const { return image_topic_name_.c_str(); }

  /**
   * @brief 返回同步 IMU topic 名称视图。
   */
  std::string_view ImuTopicNameView() const { return imu_topic_name_; }

  /**
   * @brief 返回以空字符结尾的同步 IMU topic 名称。
   */
  const char* ImuTopicName() const { return imu_topic_name_.c_str(); }

  /**
   * @brief 发布已经由同步模块对齐完成的 IMU 包。
   *
   * @param imu 标准布局 IMU 载荷，会按 `imu_topic_name` 发布。
   */
  void PublishImu(ImuStamped imu) { imu_topic_.Publish(imu); }

  /**
   * @brief 注册唯一图像 sink。
   *
   * @param initial_image 首个可写图像槽位，必须非空。
   * @param commit_callback 提交当前帧并返回下一个可写槽位的回调。
   * @return 注册成功返回 true；参数非法或重复注册返回 false。
   *
   * 具体相机只写入当前租借槽位，不拥有后级队列。当前实现只允许注册一个 sink。
   */
  bool RegisterImageSink(ImageFrame* initial_image, ImageCommitCallback commit_callback)
  {
    if (initial_image == nullptr || commit_callback.Empty())
    {
      XR_LOG_ERROR("CameraBase(%s): image sink registration got invalid argument", name_.c_str());
      return false;
    }
    if (!image_commit_callback_.Empty())
    {
      XR_LOG_ERROR("CameraBase(%s): image sink already registered", name_.c_str());
      return false;
    }

    writable_image_ = initial_image;
    image_commit_callback_ = commit_callback;
    return true;
  }

  /**
   * @brief 查询图像 sink 是否已经可用。
   *
   * @return 已注册 commit 回调且当前可写槽位非空时返回 true。
   */
  bool ImageSinkReady() const
  {
    return !image_commit_callback_.Empty() && writable_image_ != nullptr;
  }

  /**
   * @brief 获取当前可写图像槽位。
   *
   * @return sink 未就绪时返回 nullptr；否则返回生产者可写入的 `ImageFrame`。
   */
  ImageFrame* GetWritableImage() { return ImageSinkReady() ? writable_image_ : nullptr; }

  /**
   * @brief 提交当前可写图像并切换到 sink 返回的下一槽位。
   *
   * @return 提交成功且获得下一可写槽位时返回 true。
   */
  bool CommitImage()
  {
    if (!ImageSinkReady() || image_commit_callback_.Empty())
    {
      return false;
    }

    ImageFrame* next_image = nullptr;
    image_commit_callback_.Run(false, next_image);
    if (next_image == nullptr)
    {
      XR_LOG_ERROR("CameraBase(%s): image sink callback returned null writable image",
                   name_.c_str());
      return false;
    }

    writable_image_ = next_image;
    return true;
  }

  /**
   * @brief RamFS 调试命令入口。
   *
   * @param self 当前相机实例。
   * @param argc 参数个数。
   * @param argv 参数数组，支持 `set_exposure` 和 `set_gain`。
   * @return 命令成功返回 0，未知命令返回 -1。
   */
  static int CommandFun(CameraBase* self, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Camera: %s\n\n">(self->name_.c_str());
      LibXR::STDIO::Printf<"用法:\r\n">();
      LibXR::STDIO::Printf<"  set_exposure <曝光>\r\n">();
      LibXR::STDIO::Printf<"  set_gain <增益>\r\n">();
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

    LibXR::STDIO::Printf<"未知命令：%s\n">(argv[1]);
    return -1;
  }

 private:
  std::string name_;  ///< 相机实例名和 RamFS 命令文件名。
  std::string image_topic_name_;  ///< 下游识别图像流时使用的逻辑 topic 名称。
  std::string imu_topic_name_;    ///< `PublishImu()` 发布同步 IMU 的 topic 名称。
  LibXR::RamFS::File cmd_file_;   ///< 曝光/增益调试命令入口。
  LibXR::Topic imu_topic_;        ///< 同步 IMU 发布 topic。
  ImageFrame* writable_image_{nullptr};  ///< 当前由 sink 租借给生产者的可写图像槽位。
  ImageCommitCallback image_commit_callback_{};  ///< 提交图像并换取下一槽位的回调。
};
