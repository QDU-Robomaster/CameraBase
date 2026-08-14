#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机基础类型、像素编码和图像/IMU 数据结构
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <type_traits>

#include "app_framework.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "ramfs.hpp"

/**
 * @class CameraTypes
 * @brief 相机相关的纯类型定义容器。
 *
 * 本类不保存运行时状态。视觉链路中各模块通过这些类型共享图像像素格式、
 * 帧存储布局、原生相机标定、逐帧采样几何和同步 IMU 数据格式。
 */
class CameraTypes
{
 public:
  /**
   * @enum Encoding
   * @brief 图像像素编码格式。
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
    YUV422         ///< 打包 YUV 4:2:2；具体字节顺序由图像源约定。
  };

  /**
   * @enum DistortionModel
   * @brief 相机畸变模型。
   *
   * 枚举值用于解释 `CameraCalibration::distortion_coefficients` 的含义。当前 PnP
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
   * @struct FrameLayout
   * @brief 编译期固定的图像尺寸、行跨度、存储容量和像素编码。
   *
   * `width`、`height` 和 `step` 同时定义每帧有效布局与 `ImageFrame::data` 容量。
   * `FrameGeometry` 必须保持这三个字段相同，仅描述 ROI、采样和方向关系；编码在同一
   * 模板实例内保持不变。
   */
  struct FrameLayout
  {
    uint32_t width{};     ///< 每帧有效宽度，单位像素。
    uint32_t height{};    ///< 每帧有效高度，单位像素。
    uint32_t step{};      ///< 每帧行跨度，单位字节。
    Encoding encoding{};  ///< 整个模板实例固定的像素编码。
  };

  /**
   * @struct CameraCalibration
   * @brief 原生传感器坐标系下的不可变运行时标定。
   *
   * 数组均按行优先存储。该结构不描述 ROI、下采样或翻转；这些逐帧采样关系由
   * `FrameGeometry` 携带。
   */
  struct CameraCalibration
  {
    uint32_t native_width{};                ///< 标定使用的原生传感器宽度，单位像素。
    uint32_t native_height{};               ///< 标定使用的原生传感器高度，单位像素。
    std::array<double, 9> camera_matrix{};  ///< 3x3 原生相机内参矩阵 K。
    DistortionModel distortion_model{};     ///< 畸变模型类型。
    std::array<double, 14>
        distortion_coefficients{};  ///< 畸变参数，布局跟随 ROS CameraInfo。
    std::array<double, 9> rectification_matrix{};  ///< 3x3 校正旋转矩阵 R。
    std::array<double, 12> projection_matrix{};    ///< 3x4 原生投影矩阵 P。
  };

  /**
   * @enum FrameGeometryFlags
   * @brief 从帧坐标映射回原生传感器坐标时使用的离散变换。
   */
  enum FrameGeometryFlags : uint16_t
  {
    FRAME_GEOMETRY_NONE = 0,
    FRAME_GEOMETRY_REVERSE_X = 1U << 0,
    FRAME_GEOMETRY_REVERSE_Y = 1U << 1,
  };

  /**
   * @struct FrameGeometry
   * @brief 一帧图像相对原生传感器坐标系的采样关系。
   *
   * `roi_offset_*_native` 和 `sample_phase_*_native` 均使用原生像素中心坐标。
   * `epoch` 必须非零；是否允许运行期变化由接收图像的 sink 决定。结构按值进入
   * `ImageFrame`，可安全跨线程和共享内存。
   */
  struct FrameGeometry
  {
    uint32_t epoch{};                ///< 几何配置代次；0 为无效值。
    uint32_t width{};                ///< 当前帧有效宽度，单位像素。
    uint32_t height{};               ///< 当前帧有效高度，单位像素。
    uint32_t step{};                 ///< 当前帧有效行跨度，单位字节。
    uint32_t roi_offset_x_native{};  ///< ROI 左上角在原生传感器中的 x 偏移。
    uint32_t roi_offset_y_native{};  ///< ROI 左上角在原生传感器中的 y 偏移。
    uint16_t decimation_x{};         ///< 横向相邻帧像素对应的原生像素间距。
    uint16_t decimation_y{};         ///< 纵向相邻帧像素对应的原生像素间距。
    uint16_t flags{};                ///< `FrameGeometryFlags` 位集合。
    uint16_t reserved{};             ///< ABI 保留字段，当前必须为 0。
    float sample_phase_x_native{};   ///< 第 0 列像素中心相对 ROI 起点的原生 x 相位。
    float sample_phase_y_native{};   ///< 第 0 行像素中心相对 ROI 起点的原生 y 相位。
  };

  static_assert(sizeof(FrameGeometry) == 40, "CameraTypes::FrameGeometry ABI changed");
  static_assert(std::is_trivially_copyable_v<FrameLayout>);
  static_assert(std::is_standard_layout_v<FrameLayout>);
  static_assert(std::is_trivially_copyable_v<CameraCalibration>);
  static_assert(std::is_standard_layout_v<CameraCalibration>);
  static_assert(std::is_aggregate_v<CameraCalibration>);
  static_assert(std::is_trivially_copyable_v<FrameGeometry>);
  static_assert(std::is_standard_layout_v<FrameGeometry>);

  /**
   * @brief 返回固定编码下每个像素占用的字节数；不支持的编码返回 0。
   */
  [[nodiscard]] static constexpr uint32_t BytesPerPixel(Encoding encoding)
  {
    switch (encoding)
    {
      case RGB8:
      case BGR8:
        return 3;
      case RGBA8:
      case BGRA8:
        return 4;
      case RGB16:
      case BGR16:
        return 6;
      case RGBA16:
      case BGRA16:
        return 8;
      case MONO8:
      case BAYER_RGGB8:
      case BAYER_GRBG8:
      case BAYER_GBRG8:
      case BAYER_BGGR8:
        return 1;
      case MONO16:
      case BAYER_RGGB16:
      case BAYER_GRBG16:
      case BAYER_GBRG16:
      case BAYER_BGGR16:
      case YUV422:
        return 2;
      case INVALID:
      default:
        return 0;
    }
  }

  /**
   * @brief 检查编译期帧容量是否自洽且不会溢出 `size_t`。
   */
  [[nodiscard]] static constexpr bool ValidateFrameLayout(const FrameLayout& layout)
  {
    const uint32_t bytes_per_pixel = BytesPerPixel(layout.encoding);
    const uint64_t minimum_step = static_cast<uint64_t>(layout.width) * bytes_per_pixel;
    const uint64_t image_bytes = static_cast<uint64_t>(layout.step) * layout.height;
    const bool packed_width_valid = layout.encoding != YUV422 || layout.width % 2U == 0U;
    return layout.width != 0 && layout.height != 0 && layout.step != 0 &&
           bytes_per_pixel != 0 && packed_width_valid && layout.step >= minimum_step &&
           image_bytes != 0 && image_bytes <= std::numeric_limits<std::size_t>::max();
  }

  /**
   * @brief 判断逐帧几何是否能由给定缓冲区承载并落在原生标定范围内。
   */
  [[nodiscard]] static constexpr bool ValidateFrameGeometry(
      const FrameLayout& layout, const CameraCalibration& calibration,
      const FrameGeometry& geometry)
  {
    constexpr uint16_t supported_flags =
        FRAME_GEOMETRY_REVERSE_X | FRAME_GEOMETRY_REVERSE_Y;
    const uint32_t bytes_per_pixel = BytesPerPixel(layout.encoding);
    const uint64_t minimum_step = static_cast<uint64_t>(geometry.width) * bytes_per_pixel;
    const uint64_t active_bytes = static_cast<uint64_t>(geometry.step) * geometry.height;
    const uint64_t capacity_bytes = static_cast<uint64_t>(layout.step) * layout.height;
    const bool phases_finite =
        geometry.sample_phase_x_native == geometry.sample_phase_x_native &&
        geometry.sample_phase_y_native == geometry.sample_phase_y_native;

    if (!ValidateFrameLayout(layout) || calibration.native_width == 0 ||
        calibration.native_height == 0 || geometry.epoch == 0 || geometry.width == 0 ||
        geometry.height == 0 || geometry.step == 0 || geometry.width != layout.width ||
        geometry.height != layout.height || geometry.step != layout.step ||
        geometry.step < minimum_step || active_bytes > capacity_bytes ||
        geometry.decimation_x == 0 || geometry.decimation_y == 0 ||
        (geometry.flags & static_cast<uint16_t>(~supported_flags)) != 0 ||
        geometry.reserved != 0 || !phases_finite ||
        geometry.sample_phase_x_native < 0.0F || geometry.sample_phase_y_native < 0.0F ||
        geometry.sample_phase_x_native >= geometry.decimation_x ||
        geometry.sample_phase_y_native >= geometry.decimation_y)
    {
      return false;
    }

    const double native_max_x =
        static_cast<double>(geometry.roi_offset_x_native) +
        static_cast<double>(geometry.sample_phase_x_native) +
        static_cast<double>(geometry.decimation_x) * (geometry.width - 1U);
    const double native_max_y =
        static_cast<double>(geometry.roi_offset_y_native) +
        static_cast<double>(geometry.sample_phase_y_native) +
        static_cast<double>(geometry.decimation_y) * (geometry.height - 1U);
    return native_max_x <= static_cast<double>(calibration.native_width - 1U) &&
           native_max_y <= static_cast<double>(calibration.native_height - 1U);
  }

  /**
   * @brief 比较两个逐帧几何快照是否描述完全相同的采样关系。
   */
  [[nodiscard]] static constexpr bool SameFrameGeometry(const FrameGeometry& lhs,
                                                        const FrameGeometry& rhs)
  {
    return lhs.epoch == rhs.epoch && lhs.width == rhs.width && lhs.height == rhs.height &&
           lhs.step == rhs.step && lhs.roi_offset_x_native == rhs.roi_offset_x_native &&
           lhs.roi_offset_y_native == rhs.roi_offset_y_native &&
           lhs.decimation_x == rhs.decimation_x && lhs.decimation_y == rhs.decimation_y &&
           lhs.flags == rhs.flags && lhs.reserved == rhs.reserved &&
           lhs.sample_phase_x_native == rhs.sample_phase_x_native &&
           lhs.sample_phase_y_native == rhs.sample_phase_y_native;
  }

  [[nodiscard]] static constexpr bool HasGeometryFlag(const FrameGeometry& geometry,
                                                      FrameGeometryFlags flag)
  {
    return (geometry.flags & static_cast<uint16_t>(flag)) != 0;
  }

  [[nodiscard]] static constexpr double FrameToNativeX(const FrameGeometry& geometry,
                                                       double frame_x)
  {
    const double oriented_x = HasGeometryFlag(geometry, FRAME_GEOMETRY_REVERSE_X)
                                  ? static_cast<double>(geometry.width - 1U) - frame_x
                                  : frame_x;
    return static_cast<double>(geometry.roi_offset_x_native) +
           static_cast<double>(geometry.sample_phase_x_native) +
           static_cast<double>(geometry.decimation_x) * oriented_x;
  }

  [[nodiscard]] static constexpr double FrameToNativeY(const FrameGeometry& geometry,
                                                       double frame_y)
  {
    const double oriented_y = HasGeometryFlag(geometry, FRAME_GEOMETRY_REVERSE_Y)
                                  ? static_cast<double>(geometry.height - 1U) - frame_y
                                  : frame_y;
    return static_cast<double>(geometry.roi_offset_y_native) +
           static_cast<double>(geometry.sample_phase_y_native) +
           static_cast<double>(geometry.decimation_y) * oriented_y;
  }

  [[nodiscard]] static constexpr std::array<double, 2> FrameToNative(
      const FrameGeometry& geometry, double frame_x, double frame_y)
  {
    return {FrameToNativeX(geometry, frame_x), FrameToNativeY(geometry, frame_y)};
  }

  [[nodiscard]] static constexpr double NativeToFrameX(const FrameGeometry& geometry,
                                                       double native_x)
  {
    const double oriented_x =
        (native_x - static_cast<double>(geometry.roi_offset_x_native) -
         static_cast<double>(geometry.sample_phase_x_native)) /
        static_cast<double>(geometry.decimation_x);
    return HasGeometryFlag(geometry, FRAME_GEOMETRY_REVERSE_X)
               ? static_cast<double>(geometry.width - 1U) - oriented_x
               : oriented_x;
  }

  [[nodiscard]] static constexpr double NativeToFrameY(const FrameGeometry& geometry,
                                                       double native_y)
  {
    const double oriented_y =
        (native_y - static_cast<double>(geometry.roi_offset_y_native) -
         static_cast<double>(geometry.sample_phase_y_native)) /
        static_cast<double>(geometry.decimation_y);
    return HasGeometryFlag(geometry, FRAME_GEOMETRY_REVERSE_Y)
               ? static_cast<double>(geometry.height - 1U) - oriented_y
               : oriented_y;
  }

  [[nodiscard]] static constexpr std::array<double, 2> NativeToFrame(
      const FrameGeometry& geometry, double native_x, double native_y)
  {
    return {NativeToFrameX(geometry, native_x), NativeToFrameY(geometry, native_y)};
  }

  /**
   * @struct PnPDistCoeffs
   * @brief PnP 使用的固定尺寸畸变系数描述。
   *
   * @note 这里保持为纯静态数据，便于编译期生成，再由运行时封装成 `cv::Mat`。
   */
  struct PnPDistCoeffs
  {
    std::array<double, 8> values{};             ///< 当前 PnP 后端会消费的畸变系数。
    uint8_t size{};                             ///< `values` 中有效系数个数。
    bool uses_rational_polynomial_extension{};  ///< 是否命中 8 项 rational 扩展路径。
    bool requires_undistort_first{};            ///< 当前模型是否应先去畸变再进入 PnP。
  };

  /**
   * @brief 按原生相机模型生成 PnP 所需的固定畸变系数描述。
   *
   * @param calibration 运行期不可变的原生相机标定。
   * @return 固定长度畸变系数和调用方处理建议。
   *
   * @note 当前只直接支持 OpenCV 常用 pinhole / rational 两类输入。
   *       其他模型后续应先做去畸变，再按无畸变 pinhole 进入 PnP。
   */
  [[nodiscard]] static constexpr PnPDistCoeffs BuildPnPDistCoeffs(
      const CameraCalibration& calibration)
  {
    PnPDistCoeffs dc{};
    switch (calibration.distortion_model)
    {
      case DistortionModel::NONE:
        break;

      case DistortionModel::PLUMB_BOB:
        dc.values[0] = calibration.distortion_coefficients[0];
        dc.values[1] = calibration.distortion_coefficients[1];
        dc.values[2] = calibration.distortion_coefficients[2];
        dc.values[3] = calibration.distortion_coefficients[3];
        dc.values[4] = calibration.distortion_coefficients[4];
        dc.size = 5;
        break;

      case DistortionModel::RATIONAL_POLYNOMIAL:
        dc.values[0] = calibration.distortion_coefficients[0];
        dc.values[1] = calibration.distortion_coefficients[1];
        dc.values[2] = calibration.distortion_coefficients[2];
        dc.values[3] = calibration.distortion_coefficients[3];
        dc.values[4] = calibration.distortion_coefficients[4];
        dc.values[5] = calibration.distortion_coefficients[5];
        dc.values[6] = calibration.distortion_coefficients[6];
        dc.values[7] = calibration.distortion_coefficients[7];
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

namespace CameraBaseDetail
{
enum class ImageSinkRegistrationResult : uint8_t
{
  REGISTERED,
  INVALID_ARGUMENT,
  ALREADY_REGISTERED,
};

enum class ImageCommitResult : uint8_t
{
  COMMITTED,
  NOT_READY,
  NO_WRITABLE_IMAGE,
};

/**
 * @brief 单生产者图像槽位交接状态机。
 *
 * CameraBase 在回调执行前撤销生产者的写权限。sink 必须同步处理提交，并返回一块
 * 独占可写槽位；若本帧未发布，可以返回刚提交的原槽位表示丢弃并复用。
 * `Register()` 只由初始化线程调用一次；发布就绪后仅一个采集线程可调用
 * `WritableImage()` 和 `Commit()`，其他线程最多查询 `Ready()`。
 *
 * @tparam Frame 带有 `publish_token` 字段的图像帧类型。
 */
template <typename Frame>
class ImageSinkState
{
 public:
  using CommitCallback = LibXR::Callback<Frame*&>;

  ImageSinkState() = default;
  ImageSinkState(const ImageSinkState&) = delete;
  ImageSinkState& operator=(const ImageSinkState&) = delete;
  ImageSinkState(ImageSinkState&&) = delete;
  ImageSinkState& operator=(ImageSinkState&&) = delete;

  [[nodiscard]] ImageSinkRegistrationResult Register(Frame* initial_image,
                                                     CommitCallback callback)
  {
    if (initial_image == nullptr || callback.Empty())
    {
      return ImageSinkRegistrationResult::INVALID_ARGUMENT;
    }
    if (!commit_callback_.Empty())
    {
      return ImageSinkRegistrationResult::ALREADY_REGISTERED;
    }

    initial_image->publish_token = 0U;
    writable_image_ = initial_image;
    commit_callback_ = callback;
    ready_.store(true, std::memory_order_release);
    return ImageSinkRegistrationResult::REGISTERED;
  }

  [[nodiscard]] bool Ready() const noexcept
  {
    return ready_.load(std::memory_order_acquire);
  }

  [[nodiscard]] Frame* WritableImage() noexcept
  {
    return Ready() ? writable_image_ : nullptr;
  }

  [[nodiscard]] ImageCommitResult Commit()
  {
    if (!Ready())
    {
      return ImageCommitResult::NOT_READY;
    }

    Frame* committed_image = writable_image_;
    ready_.store(false, std::memory_order_release);
    writable_image_ = nullptr;
    committed_image->publish_token = 0U;

    Frame* next_image = nullptr;
    commit_callback_.Run(false, next_image);
    if (next_image == nullptr)
    {
      return ImageCommitResult::NO_WRITABLE_IMAGE;
    }

    // 原槽丢帧复用和新槽租出都从生产者持有状态开始。
    next_image->publish_token = 0U;
    writable_image_ = next_image;
    ready_.store(true, std::memory_order_release);
    return ImageCommitResult::COMMITTED;
  }

 private:
  Frame* writable_image_{nullptr};
  CommitCallback commit_callback_{};
  std::atomic<bool> ready_{false};
};
}  // namespace CameraBaseDetail

/**
 * @class CameraBase
 * @brief 编译期绑定帧存储布局、运行期持有原生标定的相机生产者基类。
 *
 * `CameraBase` 定义相机类型和图像提交方式。具体相机驱动负责填充
 * `ImageFrame`，其他模块通过 `RegisterImageSink()` 提供可写槽位并接收图像。
 * 本类不分配图像槽、不保存队列，也不负责记录、标定、预览、图像同步或坐标系转换。
 * sink 回调与 `CommitImage()` 同步执行，是发布前预处理、背压和丢帧决策的唯一边界。
 *
 * @tparam FrameLayoutV 编译期图像存储容量和像素格式描述。
 */
template <CameraTypes::FrameLayout FrameLayoutV>
class CameraBase
{
 public:
  using Encoding = CameraTypes::Encoding;                    ///< 像素编码枚举的局部别名。
  using DistortionModel = CameraTypes::DistortionModel;      ///< 畸变模型枚举的局部别名。
  using FrameLayout = CameraTypes::FrameLayout;              ///< 编译期帧存储布局。
  using CameraCalibration = CameraTypes::CameraCalibration;  ///< 原生相机标定。
  using FrameGeometry = CameraTypes::FrameGeometry;          ///< 逐帧采样几何。
  /// 图像帧对象和像素负载的最小对齐字节数。
  static constexpr std::size_t image_alignment = 64;

  /// 整条视觉链共享的编译期图像存储布局。
  static inline constexpr FrameLayout frame_layout = FrameLayoutV;
  /// 单帧图像负载字节数，等于 `FrameLayout::step * FrameLayout::height`。
  static constexpr std::size_t image_bytes =
      static_cast<std::size_t>(frame_layout.step) *
      static_cast<std::size_t>(frame_layout.height);

  /**
   * @struct ImageFrame
   * @brief 固定尺寸的图像帧载荷。
   *
   * `data` 按 `frame_layout.encoding` 和逐帧 `geometry.step` 解释。生产者只能在当前
   * writable slot 生命周期内写入该对象；调用 `CommitImage()` 后不得继续访问旧槽位。
   * `timestamp_us` 必须保留源传感器或录制文件的采样时间，不能改写成主机到达、解码、
   * 预处理或发布时间。实时采集应保留设备时钟的采样语义。CameraBase 不比较或重映射
   * 时间域；重复、回退、回绕或复位由 sink 的同步状态机处理。确定性回放必须保留原始
   * 顺序和时间，包括原始重复值。
   */
  struct alignas(image_alignment) ImageFrame
  {
    LibXR::MicrosecondTimestamp
        timestamp_us;          ///< 传感器采样时间，单位微秒；不是主机到达或发布时间。
    uint64_t publish_token{};  ///< sink 分配的进程内发布标识；生产者持有时为 0。
    FrameGeometry geometry;    ///< 当前像素负载到原生传感器坐标系的映射。
    alignas(image_alignment)
        std::array<uint8_t, image_bytes> data;  ///< 图像字节负载，含每行 padding。
  };

  /**
   * @struct ImuStamped
   * @brief 与图像同步搬运的位姿与惯导采样。
   *
   * `CameraBase` 只负责发布该数据；同步、插值和坐标系定义由生成该数据的模块保证。
   * 同一同步结果的 `timestamp_us` 和 `publish_token` 必须分别与图像一致。
   */
  struct alignas(8) ImuStamped
  {
    LibXR::MicrosecondTimestamp timestamp_us;      ///< 同步后时间戳，单位微秒。
    uint64_t publish_token{};                      ///< 对应共享图像的发布标识。
    std::array<float, 4> rotation_wxyz;            ///< 姿态四元数，顺序为 wxyz。
    std::array<float, 3> translation_xyz;          ///< 平移，单位米；无平移来源时置零。
    std::array<float, 3> angular_velocity_xyz;     ///< 角速度，单位 rad/s。
    std::array<float, 3> linear_acceleration_xyz;  ///< 线加速度，单位 m/s^2。
  };

  /**
   * @brief 图像生产者提交一帧后由 sink 返回下一个可写槽位的回调。
   *
   * 回调在调用 `CommitImage()` 的生产者线程内同步执行。sink 已持有当前槽位句柄，回调
   * 参数只用于返回下一帧独占可写的 `ImageFrame*`。发布前预处理必须在回调发起发布前
   * 完成。没有可发布槽位或预处理拒绝时，sink 可不发布并返回当前槽位表示丢帧复用；
   * 返回 nullptr 是不可恢复的契约错误，会让生产者保持未就绪。
   */
  using ImageCommitCallback = LibXR::Callback<ImageFrame*&>;

  /// 时间戳必须保持 64-bit 标准布局。
  static_assert(sizeof(LibXR::MicrosecondTimestamp) == sizeof(uint64_t),
                "CameraBase timestamp must stay 64-bit");
  static_assert(alignof(LibXR::MicrosecondTimestamp) == alignof(uint64_t),
                "CameraBase timestamp alignment changed");
  static_assert(std::is_standard_layout_v<LibXR::MicrosecondTimestamp>,
                "CameraBase timestamp type must stay standard layout");
  static_assert(CameraTypes::ValidateFrameLayout(frame_layout),
                "CameraBase requires a valid frame layout");
  static_assert(image_bytes > 0, "CameraBase requires non-zero image bytes");
  static_assert(std::is_trivially_copyable_v<ImageFrame>,
                "CameraBase::ImageFrame must be trivially copyable");
  static_assert(std::is_standard_layout_v<ImageFrame>,
                "CameraBase::ImageFrame must be standard layout");
  static_assert(std::is_standard_layout_v<ImuStamped>,
                "CameraBase::ImuStamped must be standard layout");
  static_assert(std::is_trivially_copyable_v<ImuStamped>,
                "CameraBase::ImuStamped must be trivially copyable");
  static_assert(alignof(ImuStamped) == 8, "CameraBase::ImuStamped alignment ABI changed");
  static_assert(alignof(ImageFrame) >= image_alignment,
                "CameraBase::ImageFrame alignment is too small");
  static_assert(offsetof(ImageFrame, timestamp_us) == 0,
                "CameraBase::ImageFrame timestamp offset ABI changed");
  static_assert(offsetof(ImageFrame, publish_token) == 8,
                "CameraBase::ImageFrame publish token offset ABI changed");
  static_assert(offsetof(ImageFrame, geometry) == 16,
                "CameraBase::ImageFrame geometry offset ABI changed");
  static_assert(offsetof(ImageFrame, data) % image_alignment == 0,
                "CameraBase::ImageFrame image data must stay aligned");
  static_assert(offsetof(ImageFrame, data) == image_alignment,
                "CameraBase::ImageFrame image data offset ABI changed");
  static_assert(sizeof(ImageFrame) ==
                    ((image_alignment + image_bytes + image_alignment - 1U) /
                     image_alignment) *
                        image_alignment,
                "CameraBase::ImageFrame size ABI changed");
  static_assert(offsetof(ImuStamped, timestamp_us) == 0,
                "CameraBase::ImuStamped timestamp offset ABI changed");
  static_assert(offsetof(ImuStamped, publish_token) == 8,
                "CameraBase::ImuStamped publish token offset ABI changed");
  static_assert(offsetof(ImuStamped, rotation_wxyz) == 16,
                "CameraBase::ImuStamped rotation offset ABI changed");
  static_assert(offsetof(ImuStamped, translation_xyz) == 32,
                "CameraBase::ImuStamped translation offset ABI changed");
  static_assert(offsetof(ImuStamped, angular_velocity_xyz) == 44,
                "CameraBase::ImuStamped angular velocity offset ABI changed");
  static_assert(offsetof(ImuStamped, linear_acceleration_xyz) == 56,
                "CameraBase::ImuStamped linear acceleration offset ABI changed");
  static_assert(sizeof(ImuStamped) == 72, "CameraBase::ImuStamped size ABI changed");

  /**
   * @brief 构造相机基础对象并注册调试命令文件。
   *
   * @param hw 硬件容器，必须包含名为 `ramfs` 的 `LibXR::RamFS`。
   * @param calibration 原生传感器坐标系下的不可变相机标定，按值持有。
   * @param name 相机实例名，同时作为 RamFS 命令文件名。
   * @param image_topic_name 图像逻辑 topic 名称。
   * @param imu_topic_name 同步 IMU topic 名称，`PublishImu()` 会发布到该 topic。
   */
  CameraBase(LibXR::HardwareContainer& hw, CameraCalibration calibration,
             std::string_view name = "camera",
             std::string_view image_topic_name = "camera_image",
             std::string_view imu_topic_name = "camera_imu")
      : calibration_(calibration),
        name_(name),
        image_topic_name_(image_topic_name),
        imu_topic_name_(imu_topic_name),
        cmd_file_(LibXR::RamFS::CreateFile(name_.c_str(), CommandFun, this)),
        imu_topic_(LibXR::Topic::FindOrCreate<ImuStamped>(imu_topic_name_.c_str()))
  {
    ASSERT(CameraBaseIntrinsicSanity::CameraCalibrationReasonable(calibration_));
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
  }

  CameraBase(const CameraBase&) = delete;
  CameraBase& operator=(const CameraBase&) = delete;
  CameraBase(CameraBase&&) = delete;
  CameraBase& operator=(CameraBase&&) = delete;

  /**
   * @brief 仅提供多态类型完整性，不承担 sink 注销或线程停止语义。
   *
   * CameraBase 与已注册 sink 按进程生命周期使用。析构不是并发边界，调用方不得依赖
   * 本析构等待回调、回收槽位或终止派生类工作线程。
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
   * @brief 返回本相机实例持有的原生标定。
   *
   * 返回引用在相机对象完整生命周期内有效，调用方不得修改标定内容。
   */
  const CameraCalibration& Calibration() const noexcept { return calibration_; }

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
   * @brief 发布同步 IMU 数据。
   *
   * @param imu 标准布局 IMU 载荷，会按 `imu_topic_name` 发布。其时间戳与发布标识由同步
   * 模块填写，CameraBase 不做转换或校验。
   */
  void PublishImu(ImuStamped imu) { imu_topic_.Publish(imu); }

  /**
   * @brief 注册唯一图像 sink。
   *
   * @param initial_image 首个可写图像槽位，必须非空。
   * @param commit_callback 提交当前帧并返回下一个可写槽位的回调。
   * @return 注册成功返回 true；参数非法或重复注册返回 false。
   *
   * 具体相机只写入当前可写槽位，不拥有图像队列。当前实现只允许注册一个进程生命周期
   * sink，不支持注销或替换。注册可发生在采集线程已经开始轮询之后；初始槽位和回调会
   * 通过 release/acquire 就绪门闩一次性发布。仅初始化线程可调用本方法一次。
   */
  bool RegisterImageSink(ImageFrame* initial_image, ImageCommitCallback commit_callback)
  {
    const auto result = image_sink_.Register(initial_image, commit_callback);
    if (result == CameraBaseDetail::ImageSinkRegistrationResult::INVALID_ARGUMENT)
    {
      XR_LOG_ERROR("CameraBase(%s): image sink registration got invalid argument",
                   name_.c_str());
      return false;
    }
    if (result == CameraBaseDetail::ImageSinkRegistrationResult::ALREADY_REGISTERED)
    {
      XR_LOG_ERROR("CameraBase(%s): image sink already registered", name_.c_str());
      return false;
    }
    return true;
  }

  /**
   * @brief 查询图像 sink 是否已经可用。
   *
   * @return 已注册 commit 回调且当前可写槽位非空时返回 true。采集线程可在启动阶段
   * 轮询该状态；它不代表下游曾成功发布图像。此只读查询可从其他线程调用。
   */
  bool ImageSinkReady() const noexcept { return image_sink_.Ready(); }

  /**
   * @brief 获取当前可写图像槽位。
   *
   * @return sink 未就绪时返回 nullptr；否则返回生产者独占写入的 `ImageFrame`。
   * 注册完成后，本方法和 `CommitImage()` 只能由同一个采集线程调用。
   */
  ImageFrame* GetWritableImage() noexcept { return image_sink_.WritableImage(); }

  /**
   * @brief 提交当前可写图像并切换到 sink 返回的下一槽位。
   *
   * 回调执行期间生产者写权限被撤销。返回 true 只表示回调已完成且获得下一写租约，
   * 不表示当前帧已发布或被下游接受；发布、背压和丢帧结果由 sink 自己记录。回调可以
   * 查询 `ImageSinkReady()`，但不得重新注册 sink、递归提交或生产下一帧。
   *
   * @return 获得下一可写槽位时返回 true；未注册或回调返回 nullptr 时返回 false。
   */
  bool CommitImage()
  {
    const auto result = image_sink_.Commit();
    if (result == CameraBaseDetail::ImageCommitResult::NOT_READY)
    {
      return false;
    }
    if (result == CameraBaseDetail::ImageCommitResult::NO_WRITABLE_IMAGE)
    {
      XR_LOG_ERROR("CameraBase(%s): image sink callback returned null writable image",
                   name_.c_str());
      return false;
    }
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
  const CameraCalibration calibration_;  ///< 构造期固定的原生相机标定。
  std::string name_;                     ///< 相机实例名和 RamFS 命令文件名。
  std::string image_topic_name_;         ///< 图像逻辑 topic 名称。
  std::string imu_topic_name_;           ///< `PublishImu()` 发布同步 IMU 的 topic 名称。
  LibXR::RamFS::File cmd_file_;          ///< 曝光/增益调试命令入口。
  LibXR::Topic imu_topic_;               ///< 同步 IMU 发布 topic。
  CameraBaseDetail::ImageSinkState<ImageFrame> image_sink_;  ///< 图像槽位交接状态。
};
