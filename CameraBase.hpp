#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机共用类型与单进程内共享图像所有权的基类 / Common camera types and a base class for in-process shared image ownership
depends: []
standalone: false
=== END MANIFEST === */
// clang-format on

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>

#include "libxr_def.hpp"
#include "libxr_string.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "object_pool.hpp"
#include "ramfs.hpp"

/**
 * @class CameraTypes
 * @brief 相机相关的纯类型定义容器。
 *        Container of camera-related pure type definitions.
 *
 * 本类不保存运行时状态。视觉链路中各模块通过这些类型共享图像像素格式、
 * 帧存储布局、原生相机标定、逐帧采样几何和同步 IMU 数据格式。
 * The class holds no runtime state. Modules in the vision chain share the pixel
 * format, frame storage layout, native camera calibration, per-frame sampling
 * geometry and synchronized IMU format through these types.
 */
class CameraTypes
{
 public:
  /**
   * @enum Encoding
   * @brief 图像像素编码格式。
   *        Image pixel encoding.
   *
   * 多字节通道值使用目标平台原生字节序。行跨度由 `step` 字段给出，
   * 不由 `width` 和 `Encoding` 推算。`YUV422` 的字节顺序由图像源约定。
   * Multi-byte channel values use the native byte order of the target platform.
   * The row stride is given by `step` and is not derived from `width` and
   * `Encoding`. The byte order of `YUV422` is set by the image source.
   */
  enum Encoding : uint8_t
  {
    INVALID = 0,   ///< 无效或未初始化 Invalid or uninitialized
    RGB8,          ///< 8 位 RGB 8-bit RGB
    BGR8,          ///< 8 位 BGR 8-bit BGR
    RGBA8,         ///< 8 位 RGBA 8-bit RGBA
    BGRA8,         ///< 8 位 BGRA 8-bit BGRA
    RGB16,         ///< 16 位 RGB 16-bit RGB
    BGR16,         ///< 16 位 BGR 16-bit BGR
    RGBA16,        ///< 16 位 RGBA 16-bit RGBA
    BGRA16,        ///< 16 位 BGRA 16-bit BGRA
    MONO8,         ///< 8 位灰度 8-bit grayscale
    MONO16,        ///< 16 位灰度 16-bit grayscale
    BAYER_RGGB8,   ///< 8 位 Bayer RGGB 8-bit Bayer RGGB
    BAYER_GRBG8,   ///< 8 位 Bayer GRBG 8-bit Bayer GRBG
    BAYER_GBRG8,   ///< 8 位 Bayer GBRG 8-bit Bayer GBRG
    BAYER_BGGR8,   ///< 8 位 Bayer BGGR 8-bit Bayer BGGR
    BAYER_RGGB16,  ///< 16 位 Bayer RGGB 16-bit Bayer RGGB
    BAYER_GRBG16,  ///< 16 位 Bayer GRBG 16-bit Bayer GRBG
    BAYER_GBRG16,  ///< 16 位 Bayer GBRG 16-bit Bayer GBRG
    BAYER_BGGR16,  ///< 16 位 Bayer BGGR 16-bit Bayer BGGR
    YUV422         ///< 打包 YUV 4:2:2 Packed YUV 4:2:2
  };

  /**
   * @enum DistortionModel
   * @brief 相机畸变模型。
   *        Camera distortion model.
   *
   * 枚举值用于解释 `CameraCalibration::distortion_coefficients` 的含义。PnP 直通路径
   * 消费 pinhole 常用模型，其它模型由调用方先完成去畸变。
   * The value selects how `CameraCalibration::distortion_coefficients` is read. The
   * PnP pass-through path consumes the common pinhole models; the caller undistorts
   * for the other models first.
   */
  enum class DistortionModel : uint8_t
  {
    NONE = 0,             ///< 无畸变 No distortion
    PLUMB_BOB,            ///< Brown-Conrady / plumb_bob 模型 model
    RATIONAL_POLYNOMIAL,  ///< OpenCV 扩展有理多项式 OpenCV extended rational polynomial
    EQUIDISTANT,          ///< 等距鱼眼 Equidistant fisheye
    FOV,                  ///< FOV 模型 FOV model
    OMNI,                 ///< 统一全向 Unified omnidirectional
    EXTENDED_UNIFIED,     ///< 扩展统一相机 Extended unified camera
    DOUBLE_SPHERE,        ///< 双球 Double sphere
    THIN_PRISM,           ///< 薄棱镜 Thin prism
    UNKNOWN               ///< 未知或自定义 Unknown or custom
  };

  /**
   * @struct FrameLayout
   * @brief 编译期固定的图像尺寸、行跨度、存储容量和像素编码。
   *        Compile-time image size, row stride, storage capacity and pixel encoding.
   *
   * `width`、`height` 和 `step` 同时定义每帧有效布局与 `ImageFrame::data` 容量。
   * `FrameGeometry` 的这三个字段与之相同，仅描述 ROI、采样和方向关系；编码在同一
   * 模板实例内保持不变。
   * `width`, `height` and `step` define both the valid layout of each frame and the
   * capacity of `ImageFrame::data`. `FrameGeometry` carries the same three fields
   * and describes only ROI, sampling and orientation; the encoding is fixed within
   * one template instance.
   */
  struct FrameLayout
  {
    uint32_t width{};     ///< 有效宽度，单位像素 Valid width in pixels
    uint32_t height{};    ///< 有效高度，单位像素 Valid height in pixels
    uint32_t step{};      ///< 行跨度，单位字节 Row stride in bytes
    Encoding encoding{};  ///< 像素编码 Pixel encoding
  };

  /**
   * @struct CameraCalibration
   * @brief 原生传感器坐标系下的不可变运行时标定。
   *        Immutable runtime calibration in native sensor coordinates.
   *
   * 数组均按行优先存储，`distortion_coefficients` 的布局跟随 ROS CameraInfo。
   * ROI、下采样和翻转等逐帧采样关系由 `FrameGeometry` 携带。
   * All arrays are stored in row-major order, and `distortion_coefficients` follows
   * the ROS CameraInfo layout. Per-frame sampling relations such as ROI, decimation
   * and flipping are carried by `FrameGeometry`.
   */
  struct CameraCalibration
  {
    uint32_t native_width{};   ///< 原生传感器宽度，像素 Native sensor width, px
    uint32_t native_height{};  ///< 原生传感器高度，像素 Native sensor height, px
    std::array<double, 9> camera_matrix{};  ///< 3x3 内参矩阵 K 3x3 intrinsic matrix K
    DistortionModel distortion_model{};     ///< 畸变模型 Distortion model
    std::array<double, 14>
        distortion_coefficients{};                 ///< 畸变系数 Distortion coefficients
    std::array<double, 9> rectification_matrix{};  ///< 3x3 校正矩阵 R 3x3 rectification R
    std::array<double, 12> projection_matrix{};    ///< 3x4 投影矩阵 P 3x4 projection P
  };

  /**
   * @enum FrameGeometryFlags
   * @brief 从帧坐标映射回原生传感器坐标时使用的翻转标志。
   *        Flip flags used when mapping frame coordinates back to native sensor
   *        coordinates.
   */
  enum FrameGeometryFlags : uint16_t
  {
    FRAME_GEOMETRY_NONE = 0,             ///< 无翻转 No flip
    FRAME_GEOMETRY_REVERSE_X = 1U << 0,  ///< 横向翻转 Horizontal flip
    FRAME_GEOMETRY_REVERSE_Y = 1U << 1,  ///< 纵向翻转 Vertical flip
  };

  /**
   * @enum ProfileId
   * @brief 相机支持的固定采样档位标识。
   *        Identifier of a fixed sampling profile supported by the camera.
   */
  enum class ProfileId : uint8_t
  {
    WIDE = 0,  ///< 宽视场档位 Wide field of view
    NARROW,    ///< 窄视场档位 Narrow field of view
  };

  /**
   * @struct FrameGeometry
   * @brief 一帧图像相对原生传感器坐标系的采样关系。
   *        Sampling relation of one frame to the native sensor coordinates.
   *
   * `roi_offset_*_native` 和 `sample_phase_*_native` 均使用原生像素中心坐标。
   * 结构按值进入 `ImageFrame`，是标准布局、可平凡复制的类型。
   * `roi_offset_*_native` and `sample_phase_*_native` use native pixel-center
   * coordinates. The struct enters `ImageFrame` by value and is standard-layout and
   * trivially copyable.
   */
  struct FrameGeometry
  {
    uint32_t width{};                ///< 帧宽，像素 Frame width, px
    uint32_t height{};               ///< 帧高，像素 Frame height, px
    uint32_t step{};                 ///< 行跨度，字节 Row stride, bytes
    uint32_t roi_offset_x_native{};  ///< ROI 左上角原生 x 偏移 ROI origin x, native px
    uint32_t roi_offset_y_native{};  ///< ROI 左上角原生 y 偏移 ROI origin y, native px
    uint16_t decimation_x{};         ///< 横向像素间距，原生像素 X pitch, native px
    uint16_t decimation_y{};         ///< 纵向像素间距，原生像素 Y pitch, native px
    uint16_t flags{};                ///< `FrameGeometryFlags` 位集合 Bit set of flags
    uint16_t reserved{};             ///< ABI 保留，必须为 0 ABI reserved, must be 0
    float sample_phase_x_native{};   ///< 第 0 列相对 ROI 起点的 x 相位 Column-0 x phase
    float sample_phase_y_native{};   ///< 第 0 行相对 ROI 起点的 y 相位 Row-0 y phase
  };

  /**
   * @struct CameraProfile
   * @brief 相机驱动公开的一个固定采样档位。
   *        One fixed sampling profile exposed by a camera driver.
   *
   * `trigger_period_us` 非零。
   * `trigger_period_us` is non-zero.
   */
  struct CameraProfile
  {
    ProfileId id{ProfileId::WIDE};  ///< 档位标识 Profile identifier
    FrameGeometry geometry{};       ///< 生效后的逐帧几何 Geometry once applied
    uint32_t trigger_period_us{};   ///< 触发周期，微秒 Trigger period, us
  };

  /**
   * @struct AppliedProfile
   * @brief 驱动成功切档后实际生效的档位快照。
   *        Snapshot of the profile in effect after a successful switch.
   */
  struct AppliedProfile
  {
    ProfileId id{ProfileId::WIDE};  ///< 实际生效的档位标识 Profile identifier in effect
    FrameGeometry geometry{};       ///< 后续图像携带的几何 Geometry of following frames
  };

  static_assert(sizeof(FrameGeometry) == 36, "CameraTypes::FrameGeometry ABI changed");
  static_assert(std::is_trivially_copyable_v<FrameLayout>);
  static_assert(std::is_standard_layout_v<FrameLayout>);
  static_assert(std::is_trivially_copyable_v<CameraCalibration>);
  static_assert(std::is_standard_layout_v<CameraCalibration>);
  static_assert(std::is_aggregate_v<CameraCalibration>);
  static_assert(std::is_aggregate_v<FrameGeometry>);
  static_assert(std::is_trivially_copyable_v<FrameGeometry>);
  static_assert(std::is_standard_layout_v<FrameGeometry>);
  static_assert(std::is_trivially_copyable_v<CameraProfile>);
  static_assert(std::is_standard_layout_v<CameraProfile>);
  static_assert(std::is_trivially_copyable_v<AppliedProfile>);
  static_assert(std::is_standard_layout_v<AppliedProfile>);

  /**
   * @brief 返回固定编码下每个像素占用的字节数。
   *        Return the bytes per pixel of an encoding.
   *
   * @param encoding 像素编码。
   *                 Pixel encoding.
   * @return 每像素字节数；不支持的编码返回 0。
   *         Bytes per pixel; 0 for an unsupported encoding.
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
   * @brief 检查帧布局的容量是否自洽且不会溢出 `size_t`。
   *        Check that the frame layout capacity is self-consistent and does not
   *        overflow `size_t`.
   *
   * 宽高和 `step` 非零，编码有效，`step` 不小于一行像素的字节数，`YUV422` 的宽度为偶数。
   * Width, height and `step` are non-zero, the encoding is valid, `step` is at least
   * the bytes of one pixel row, and the width of `YUV422` is even.
   *
   * @param layout 待检查的帧布局。
   *               Frame layout to check.
   * @return 布局有效返回 true。
   *         True when the layout is valid.
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
   *        Check that a per-frame geometry fits the given buffer and lies inside the
   *        native calibration range.
   *
   * @param layout 帧存储布局。
   *               Frame storage layout.
   * @param calibration 原生相机标定。
   *                    Native camera calibration.
   * @param geometry 待检查的逐帧几何。
   *                 Per-frame geometry to check.
   * @return 几何有效返回 true。
   *         True when the geometry is valid.
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
        calibration.native_height == 0 || geometry.width == 0 || geometry.height == 0 ||
        geometry.step == 0 || geometry.width != layout.width ||
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
   * @brief 逐字段比较两个逐帧几何（不含保留字段）。
   *        Compare two per-frame geometries field by field, excluding the reserved
   *        field.
   *
   * @param lhs 第一个几何。
   *            First geometry.
   * @param rhs 第二个几何。
   *            Second geometry.
   * @return 各字段相同返回 true。
   *         True when all compared fields are equal.
   */
  [[nodiscard]] static constexpr bool SameFrameGeometry(const FrameGeometry& lhs,
                                                        const FrameGeometry& rhs)
  {
    return lhs.width == rhs.width && lhs.height == rhs.height && lhs.step == rhs.step &&
           lhs.roi_offset_x_native == rhs.roi_offset_x_native &&
           lhs.roi_offset_y_native == rhs.roi_offset_y_native &&
           lhs.decimation_x == rhs.decimation_x && lhs.decimation_y == rhs.decimation_y &&
           lhs.flags == rhs.flags &&
           lhs.sample_phase_x_native == rhs.sample_phase_x_native &&
           lhs.sample_phase_y_native == rhs.sample_phase_y_native;
  }

  /**
   * @brief 判断几何是否设置了指定翻转标志。
   *        Check whether a geometry has the given flip flag set.
   *
   * @param geometry 逐帧几何。
   *                 Per-frame geometry.
   * @param flag 待检查的标志。
   *             Flag to check.
   * @return 已设置返回 true。
   *         True when the flag is set.
   */
  [[nodiscard]] static constexpr bool HasGeometryFlag(const FrameGeometry& geometry,
                                                      FrameGeometryFlags flag)
  {
    return (geometry.flags & static_cast<uint16_t>(flag)) != 0;
  }

  /**
   * @brief 把帧内 x 坐标映射到原生传感器 x 坐标。
   *        Map a frame x coordinate to the native sensor x coordinate.
   *
   * @param geometry 逐帧几何。
   *                 Per-frame geometry.
   * @param frame_x 帧内 x 坐标，单位像素。
   *                Frame x coordinate in pixels.
   * @return 原生传感器 x 坐标，单位像素。
   *         Native sensor x coordinate in pixels.
   */
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

  /**
   * @brief 把帧内 y 坐标映射到原生传感器 y 坐标。
   *        Map a frame y coordinate to the native sensor y coordinate.
   *
   * @param geometry 逐帧几何。
   *                 Per-frame geometry.
   * @param frame_y 帧内 y 坐标，单位像素。
   *                Frame y coordinate in pixels.
   * @return 原生传感器 y 坐标，单位像素。
   *         Native sensor y coordinate in pixels.
   */
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

  /**
   * @brief 把帧内坐标映射到原生传感器坐标。
   *        Map frame coordinates to native sensor coordinates.
   *
   * @param geometry 逐帧几何。
   *                 Per-frame geometry.
   * @param frame_x 帧内 x 坐标，单位像素。
   *                Frame x coordinate in pixels.
   * @param frame_y 帧内 y 坐标，单位像素。
   *                Frame y coordinate in pixels.
   * @return 原生传感器坐标 {x, y}，单位像素。
   *         Native sensor coordinates {x, y} in pixels.
   */
  [[nodiscard]] static constexpr std::array<double, 2> FrameToNative(
      const FrameGeometry& geometry, double frame_x, double frame_y)
  {
    return {FrameToNativeX(geometry, frame_x), FrameToNativeY(geometry, frame_y)};
  }

  /**
   * @brief 把原生传感器 x 坐标映射到帧内 x 坐标。
   *        Map a native sensor x coordinate to the frame x coordinate.
   *
   * @param geometry 逐帧几何。
   *                 Per-frame geometry.
   * @param native_x 原生传感器 x 坐标，单位像素。
   *                 Native sensor x coordinate in pixels.
   * @return 帧内 x 坐标，单位像素。
   *         Frame x coordinate in pixels.
   */
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

  /**
   * @brief 把原生传感器 y 坐标映射到帧内 y 坐标。
   *        Map a native sensor y coordinate to the frame y coordinate.
   *
   * @param geometry 逐帧几何。
   *                 Per-frame geometry.
   * @param native_y 原生传感器 y 坐标，单位像素。
   *                 Native sensor y coordinate in pixels.
   * @return 帧内 y 坐标，单位像素。
   *         Frame y coordinate in pixels.
   */
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

  /**
   * @brief 把原生传感器坐标映射到帧内坐标。
   *        Map native sensor coordinates to frame coordinates.
   *
   * @param geometry 逐帧几何。
   *                 Per-frame geometry.
   * @param native_x 原生传感器 x 坐标，单位像素。
   *                 Native sensor x coordinate in pixels.
   * @param native_y 原生传感器 y 坐标，单位像素。
   *                 Native sensor y coordinate in pixels.
   * @return 帧内坐标 {x, y}，单位像素。
   *         Frame coordinates {x, y} in pixels.
   */
  [[nodiscard]] static constexpr std::array<double, 2> NativeToFrame(
      const FrameGeometry& geometry, double native_x, double native_y)
  {
    return {NativeToFrameX(geometry, native_x), NativeToFrameY(geometry, native_y)};
  }

  /**
   * @struct PnPDistCoeffs
   * @brief PnP 使用的固定尺寸畸变系数描述。
   *        Fixed-size distortion coefficient description used by PnP.
   *
   * 该结构是纯静态数据，可在编译期生成，再由运行时封装成 `cv::Mat`。
   * The struct is pure static data that can be produced at compile time and wrapped
   * into a `cv::Mat` at runtime.
   */
  struct PnPDistCoeffs
  {
    std::array<double, 8> values{};  ///< PnP 使用的畸变系数 Coefficients used by PnP
    uint8_t size{};                  ///< `values` 的有效个数 Valid count in `values`
    bool uses_rational_polynomial_extension{};  ///< 8 项 rational 扩展 8-term rational
    bool requires_undistort_first{};  ///< 是否需先去畸变再进入 PnP Undistort before PnP
  };

  /**
   * @brief 按原生相机模型生成 PnP 所需的固定畸变系数描述。
   *        Build the fixed distortion coefficient description PnP needs from the
   *        native camera model.
   *
   * @param calibration 原生相机标定。
   *                    Native camera calibration.
   * @return 固定长度畸变系数和是否需先去畸变的标记。
   *         Fixed-length coefficients and the flag for undistorting first.
   *
   * 直接支持 OpenCV 常用的 pinhole / rational 两类输入；其他模型在返回值中标记
   * `requires_undistort_first`，去畸变后按无畸变 pinhole 进入 PnP。
   * The common OpenCV pinhole and rational inputs are supported directly; other
   * models set `requires_undistort_first` and enter PnP as an undistorted pinhole
   * after undistortion.
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
/**
 * @brief 进程内共享所有权对象池。
 *        In-process object pool with shared ownership.
 *
 * 底层 `MPMCObjectPool` 负责固定槽位的分配和回收；每个已借出槽位由一个
 * `SharedHandle` 独占或共享持有。复制 handle 增加原子引用计数，移动转移本地所有权，
 * 最后一个 handle 析构时归还底层独占 pool handle。句柄只提供只读访问；生产者
 * 通过所属 pool 的 `GetWritable()` 取得独占可写指针。
 * The underlying `MPMCObjectPool` allocates and recycles the fixed slots; each
 * borrowed slot is held exclusively or shared by `SharedHandle`s. Copying a handle
 * increments an atomic reference count, moving transfers local ownership, and the
 * last handle to be destroyed returns the underlying exclusive pool handle. Handles
 * give read-only access; the producer obtains the exclusive writable pointer from
 * `GetWritable()` of the owning pool.
 *
 * @tparam Data 槽位中的对象类型。
 *              Object type held in a slot.
 * @tparam SlotCount 固定槽位数。
 *                   Number of fixed slots.
 */
template <typename Data, std::size_t SlotCount>
class SharedObjectPool
{
 public:
  static_assert(SlotCount > 1U,
                "MPMC-backed SharedObjectPool requires at least two slots");

 private:
  using ObjectPool = LibXR::MPMCObjectPool<Data>;
  using UniqueHandle = typename ObjectPool::Handle;

  struct Control
  {
    std::atomic<uint32_t> references{0U};
    std::atomic<uint64_t> generation{0U};
  };

 public:
  /**
   * @brief 一个池槽位的可复制共享所有权句柄。
   *        Copyable shared-ownership handle of one pool slot.
   *
   * 句柄只在当前进程内有效。复制构造和复制赋值增加引用计数；析构和 `Reset()` 减少
   * 引用计数。所有副本只读访问同一个 `Data`。句柄的生命周期不超过所属
   * `SharedObjectPool`。
   * A handle is valid only within the current process. Copy construction and copy
   * assignment increment the reference count; destruction and `Reset()` decrement
   * it. All copies read the same `Data` read-only. A handle does not outlive its
   * `SharedObjectPool`.
   */
  class SharedHandle
  {
   public:
    SharedHandle() = default;

    SharedHandle(const SharedHandle& other) { RetainFrom(other); }

    SharedHandle& operator=(const SharedHandle& other)
    {
      if (SameOwnership(other))
      {
        return *this;
      }

      SharedHandle retained(other);
      *this = std::move(retained);
      return *this;
    }

    SharedHandle(SharedHandle&& other) noexcept
        : pool_(std::exchange(other.pool_, nullptr)),
          slot_index_(std::exchange(other.slot_index_, 0U)),
          generation_(std::exchange(other.generation_, 0U))
    {
    }

    SharedHandle& operator=(SharedHandle&& other) noexcept
    {
      if (this == &other)
      {
        return *this;
      }

      Reset();
      pool_ = std::exchange(other.pool_, nullptr);
      slot_index_ = std::exchange(other.slot_index_, 0U);
      generation_ = std::exchange(other.generation_, 0U);
      return *this;
    }

    ~SharedHandle() { Reset(); }

    /**
     * @brief 句柄是否持有槽位。
     *        Whether the handle holds a slot.
     */
    [[nodiscard]] bool Valid() const noexcept { return pool_ != nullptr; }

    /**
     * @brief 返回只读数据指针。
     *        Return the read-only data pointer.
     *
     * @return 句柄有效时返回槽位数据，否则返回 nullptr。
     *         The slot data when the handle is valid, otherwise nullptr.
     */
    [[nodiscard]] const Data* Get() const noexcept
    {
      return Valid() ? &pool_->Get(slot_index_, generation_) : nullptr;
    }

    [[nodiscard]] const Data* operator->() const noexcept { return Get(); }

    [[nodiscard]] const Data& operator*() const noexcept
    {
      ASSERT(Valid());
      return *Get();
    }

    /**
     * @brief 返回当前引用计数的诊断快照。
     *        Return a diagnostic snapshot of the current reference count.
     *
     * 其他线程可在返回后立即复制或释放句柄，该值不用于判断是否拥有独占访问权。
     * Other threads may copy or release handles right after the call returns, so
     * the value does not indicate exclusive access.
     *
     * @return 引用计数；句柄无效或代次不匹配时为 0。
     *         Reference count; 0 for an invalid handle or a generation mismatch.
     */
    [[nodiscard]] uint32_t UseCount() const noexcept
    {
      return Valid() ? pool_->UseCount(slot_index_, generation_) : 0U;
    }

    /**
     * @brief 释放当前持有的槽位引用，句柄变为无效。
     *        Release the held slot reference and invalidate the handle.
     */
    void Reset() noexcept
    {
      if (!Valid())
      {
        return;
      }

      SharedObjectPool* pool = std::exchange(pool_, nullptr);
      const uint32_t slot_index = std::exchange(slot_index_, 0U);
      const uint64_t generation = std::exchange(generation_, 0U);
      pool->Release(slot_index, generation);
    }

   private:
    friend class SharedObjectPool<Data, SlotCount>;

    SharedHandle(SharedObjectPool* pool, uint32_t slot_index, uint64_t generation)
        : pool_(pool), slot_index_(slot_index), generation_(generation)
    {
    }

    [[nodiscard]] bool SameOwnership(const SharedHandle& other) const noexcept
    {
      return pool_ == other.pool_ && slot_index_ == other.slot_index_ &&
             generation_ == other.generation_;
    }

    void RetainFrom(const SharedHandle& other)
    {
      if (!other.Valid())
      {
        return;
      }

      const bool retained = other.pool_->Retain(other.slot_index_, other.generation_);
      ASSERT(retained);
      pool_ = other.pool_;
      slot_index_ = other.slot_index_;
      generation_ = other.generation_;
    }

    SharedObjectPool* pool_{nullptr};
    uint32_t slot_index_{0U};
    uint64_t generation_{0U};
  };

  SharedObjectPool() : object_pool_(SlotCount) {}

  SharedObjectPool(const SharedObjectPool&) = delete;
  SharedObjectPool& operator=(const SharedObjectPool&) = delete;
  SharedObjectPool(SharedObjectPool&&) = delete;
  SharedObjectPool& operator=(SharedObjectPool&&) = delete;

  /**
   * @brief 获取一个引用计数初值为一的共享句柄。
   *        Acquire a shared handle with an initial reference count of one.
   *
   * @param handle 接收槽位的句柄，须为无效句柄，否则返回 `STATE_ERR`。
   *               Handle that receives the slot; it must be invalid, otherwise
   *               `STATE_ERR` is returned.
   * @return 成功返回 `OK`，没有空闲槽位返回底层 pool 的 `EMPTY`。
   *         `OK` on success, the underlying pool's `EMPTY` when no slot is free.
   */
  [[nodiscard]] LibXR::ErrorCode Acquire(SharedHandle& handle)
  {
    if (handle.Valid())
    {
      return LibXR::ErrorCode::STATE_ERR;
    }

    UniqueHandle unique_handle;
    const LibXR::ErrorCode result = object_pool_.Acquire(unique_handle);
    if (result != LibXR::ErrorCode::OK)
    {
      return result;
    }

    const uint32_t slot_index = unique_handle.Index();
    Control& control = controls_[slot_index];
    ASSERT(control.references.load(std::memory_order_acquire) == 0U);
    ASSERT(!owners_[slot_index].has_value());

    uint64_t generation =
        control.generation.fetch_add(1U, std::memory_order_acq_rel) + 1U;
    if (generation == 0U)
    {
      generation = control.generation.fetch_add(1U, std::memory_order_acq_rel) + 1U;
    }

    owners_[slot_index].emplace(std::move(unique_handle));
    control.references.store(1U, std::memory_order_release);
    handle = SharedHandle(this, slot_index, generation);
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 获取本池唯一所有者当前持有槽位的可写指针。
   *        Get the writable pointer of the slot held by its sole owner in this pool.
   *
   * 调用方独占访问 `handle` 对象，且在使用返回指针期间不复制、移动或重置该 handle。
   * The caller has exclusive access to the `handle` object and does not copy, move
   * or reset it while using the returned pointer.
   *
   * @param handle 持有槽位的句柄。
   *               Handle that holds the slot.
   * @return 可写指针；句柄无效、来自其他池、代次不匹配或已有共享副本时返回 nullptr。
   *         Writable pointer; nullptr when the handle is invalid, belongs to another
   *         pool, has a generation mismatch or has shared copies.
   */
  [[nodiscard]] Data* GetWritable(SharedHandle& handle) noexcept
  {
    if (handle.pool_ != this || handle.slot_index_ >= SlotCount)
    {
      return nullptr;
    }

    Control& control = controls_[handle.slot_index_];
    if (control.generation.load(std::memory_order_acquire) != handle.generation_ ||
        control.references.load(std::memory_order_acquire) != 1U)
    {
      return nullptr;
    }
    return &object_pool_.UnsafeAt(handle.slot_index_);
  }

  /**
   * @brief 返回当前空闲槽位数。
   *        Return the number of free slots.
   */
  [[nodiscard]] std::size_t Available() const noexcept
  {
    return object_pool_.EmptySize();
  }

 private:
  [[nodiscard]] bool Retain(uint32_t slot_index, uint64_t generation) noexcept
  {
    ASSERT(slot_index < SlotCount);
    Control& control = controls_[slot_index];

    while (control.generation.load(std::memory_order_acquire) == generation)
    {
      uint32_t references = control.references.load(std::memory_order_acquire);
      if (references == 0U || references == std::numeric_limits<uint32_t>::max())
      {
        return false;
      }
      if (control.references.compare_exchange_weak(references, references + 1U,
                                                   std::memory_order_acq_rel,
                                                   std::memory_order_acquire))
      {
        if (control.generation.load(std::memory_order_acquire) == generation)
        {
          return true;
        }

        ASSERT(!ReleaseReference(slot_index));
        return false;
      }
    }
    return false;
  }

  void Release(uint32_t slot_index, uint64_t generation) noexcept
  {
    ASSERT(slot_index < SlotCount);
    ASSERT(controls_[slot_index].generation.load(std::memory_order_acquire) ==
           generation);
    if (!ReleaseReference(slot_index))
    {
      return;
    }

    ASSERT(owners_[slot_index].has_value());
    UniqueHandle unique_handle = std::move(*owners_[slot_index]);
    owners_[slot_index].reset();
    // 先清空本槽 owner，再由 Reset() 把索引发布回 MPMC 队列。新的 Acquire()
    // 在入队完成前取不到该索引，不会与这里并发改写同一个 optional。
    unique_handle.Reset();
  }

  /** @return 当前调用是否释放了最后一个引用。 */
  [[nodiscard]] bool ReleaseReference(uint32_t slot_index) noexcept
  {
    const uint32_t previous =
        controls_[slot_index].references.fetch_sub(1U, std::memory_order_acq_rel);
    ASSERT(previous > 0U);
    return previous == 1U;
  }

  [[nodiscard]] Data& Get(uint32_t slot_index, uint64_t generation) noexcept
  {
    ASSERT(slot_index < SlotCount);
    ASSERT(controls_[slot_index].generation.load(std::memory_order_acquire) ==
           generation);
    ASSERT(controls_[slot_index].references.load(std::memory_order_acquire) > 0U);
    return object_pool_.UnsafeAt(slot_index);
  }

  [[nodiscard]] uint32_t UseCount(uint32_t slot_index, uint64_t generation) const noexcept
  {
    ASSERT(slot_index < SlotCount);
    if (controls_[slot_index].generation.load(std::memory_order_acquire) != generation)
    {
      return 0U;
    }
    return controls_[slot_index].references.load(std::memory_order_acquire);
  }

  ObjectPool object_pool_;
  std::array<Control, SlotCount> controls_{};
  std::array<std::optional<UniqueHandle>, SlotCount> owners_{};
};
}  // namespace CameraBaseDetail

/**
 * @class CameraBase
 * @brief 编译期绑定帧存储布局、运行期持有原生标定的相机生产者基类。
 *        Camera producer base class bound to a frame storage layout at compile time
 *        and holding the native calibration at runtime.
 *
 * `CameraBase` 定义相机类型和图像提交方式。具体相机驱动负责填充本类两槽对象池中的
 * `ImageFrame`。`CommitImage()` 通过普通 Topic 同步发布临时 `SharedFrame` 指针；
 * 订阅回调复制句柄后可把同一图像槽位交给异步线程，图像字节保持在原槽位。
 * A concrete camera driver fills the `ImageFrame` of the two-slot object pool of
 * this class. `CommitImage()` synchronously publishes a temporary `SharedFrame`
 * pointer on an ordinary Topic; a subscriber callback copies the handle to pass the
 * same image slot to an asynchronous thread, and the image bytes stay in the slot.
 *
 * @tparam FrameLayoutV 编译期图像存储容量和像素格式描述。
 *                      Compile-time image storage capacity and pixel format.
 */
template <CameraTypes::FrameLayout FrameLayoutV>
class CameraBase
{
 public:
  using Encoding = CameraTypes::Encoding;                ///< 像素编码 Pixel encoding
  using DistortionModel = CameraTypes::DistortionModel;  ///< 畸变模型 Distortion model
  using FrameLayout = CameraTypes::FrameLayout;          ///< 帧存储布局 Frame layout
  using CameraCalibration =
      CameraTypes::CameraCalibration;                  ///< 原生标定 Native calibration
  using FrameGeometry = CameraTypes::FrameGeometry;    ///< 逐帧几何 Per-frame geometry
  using ProfileId = CameraTypes::ProfileId;            ///< 档位标识 Profile identifier
  using CameraProfile = CameraTypes::CameraProfile;    ///< 档位描述 Profile description
  using AppliedProfile = CameraTypes::AppliedProfile;  ///< 生效档位快照 Applied profile
  /// 图像帧对象和像素负载的最小对齐字节数。
  /// Minimum alignment in bytes of the frame object and pixel payload.
  static constexpr std::size_t image_alignment = 64;

  /// 整条视觉链共享的编译期图像存储布局。
  /// Compile-time image storage layout shared by the whole vision chain.
  static inline constexpr FrameLayout frame_layout = FrameLayoutV;
  /// 单帧图像负载字节数，等于 `step * height`。
  /// Image payload bytes per frame, equal to `step * height`.
  static constexpr std::size_t image_bytes =
      static_cast<std::size_t>(frame_layout.step) *
      static_cast<std::size_t>(frame_layout.height);

  /**
   * @struct ImageFrame
   * @brief 固定尺寸的图像帧载荷。
   *        Fixed-size image frame payload.
   *
   * `data` 按 `frame_layout.encoding` 和逐帧 `geometry.step` 解释，含每行
   * padding。生产者在当前 可写槽位的生命周期内写入该对象；调用 `CommitImage()`
   * 后改用下一次 `GetWritableImage()` 返回的指针。`timestamp_us`
   * 保存源传感器或录制文件的采样 时间，实时采集保留设备时钟的采样语义。CameraBase
   * 保存时间戳原值；重复、回退、
   * 回绕或复位由下游同步状态机处理。确定性回放保持原始顺序和时间，包括原始重复值。
   * `data` is interpreted by `frame_layout.encoding` and the per-frame
   * `geometry.step` and includes row padding. The producer writes the object during the
   * lifetime of the current writable slot and, after `CommitImage()`, uses the pointer
   * returned by the next `GetWritableImage()`. `timestamp_us` holds the sampling time of
   * the source sensor or recorded file; live capture keeps the sampling semantics of the
   * device clock. CameraBase stores the timestamp as given; duplicates, regressions,
   * wrap-around and resets are handled by the downstream synchronization state machine.
   * Deterministic replay keeps the original order and times, including original
   * duplicates.
   */
  struct alignas(image_alignment) ImageFrame
  {
    LibXR::MicrosecondTimestamp
        timestamp_us;        ///< 传感器采样时间，微秒 Sensor sampling time, us
    FrameGeometry geometry;  ///< 映射到原生坐标 Mapping to native coordinates
    alignas(
        image_alignment) std::array<uint8_t, image_bytes> data;  ///< 图像字节 Image bytes
  };

  /// 进程内图像池的固定槽位数，对应两帧背压窗口。
  /// Fixed slot count of the in-process image pool, a two-frame back-pressure window.
  static constexpr std::size_t image_slot_count = 2U;
  static_assert(
      image_slot_count == 2U,
      "CameraBase image pool capacity is part of the two-slot pipeline contract");
  /// CameraBase 自有的共享图像对象池。
  /// Shared image object pool owned by CameraBase.
  using ImagePool = CameraBaseDetail::SharedObjectPool<ImageFrame, image_slot_count>;
  /// 图像槽位的可复制进程内共享所有权句柄。
  /// Copyable in-process shared-ownership handle of an image slot.
  using SharedFrame = typename ImagePool::SharedHandle;
  /// 图像 Topic 载荷，仅在同步回调期间借用，不拥有 `SharedFrame`。
  /// Image Topic payload, borrowed only during the synchronous callback and not
  /// owning the `SharedFrame`.
  using ImageTopicPayload = const SharedFrame*;

  /**
   * @struct ImuStamped
   * @brief 与图像同步搬运的位姿与惯导采样。
   *        Pose and inertial sample carried in sync with images.
   *
   * `CameraBase` 定义并发布该载荷；同步、插值、时间域和坐标系由生成该数据的 Module
   * 决定。阶段包装可以把相机本地时间的 `ImageFrame` 与 MCU 时间的本对象关联起来，
   * 两个 `timestamp_us` 的数值可以不同。`translation_xyz` 在没有平移来源时置零。
   * `CameraBase` defines and publishes the payload; synchronization, interpolation,
   * time domain and coordinate frame are determined by the Module that produces the
   * data. A stage wrapper can associate an `ImageFrame` in camera-local time with
   * this object in MCU time, and the two `timestamp_us` values may differ.
   * `translation_xyz` is zero when there is no translation source.
   */
  struct alignas(8) ImuStamped
  {
    LibXR::MicrosecondTimestamp
        timestamp_us;  ///< 生成 Module 定义的时间，微秒 Producer-defined, us
    std::array<float, 4> rotation_wxyz;         ///< 姿态四元数 wxyz Quaternion, wxyz
    std::array<float, 3> translation_xyz;       ///< 平移，米 Translation, m
    std::array<float, 3> angular_velocity_xyz;  ///< 角速度，rad/s Angular velocity, rad/s
    std::array<float, 3>
        linear_acceleration_xyz;  ///< 线加速度，m/s^2 Acceleration, m/s^2
  };

  // 时间戳保持 64-bit 标准布局。
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
  static_assert(offsetof(ImageFrame, geometry) == 8,
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
  static_assert(offsetof(ImuStamped, rotation_wxyz) == 8,
                "CameraBase::ImuStamped rotation offset ABI changed");
  static_assert(offsetof(ImuStamped, translation_xyz) == 24,
                "CameraBase::ImuStamped translation offset ABI changed");
  static_assert(offsetof(ImuStamped, angular_velocity_xyz) == 36,
                "CameraBase::ImuStamped angular velocity offset ABI changed");
  static_assert(offsetof(ImuStamped, linear_acceleration_xyz) == 48,
                "CameraBase::ImuStamped linear acceleration offset ABI changed");
  static_assert(sizeof(ImuStamped) == 64, "CameraBase::ImuStamped size ABI changed");

  /**
   * @brief 构造相机基础对象并注册调试命令文件。
   *        Construct the camera base object and register the debug command file.
   *
   * 构造时检查原生标定的合理性并取得第一个可写图像槽位。
   * Construction checks the native calibration for plausibility and acquires the
   * first writable image slot.
   *
   * @param ramfs 注册调试命令文件的 RamFS。
   *              RamFS that receives the debug command file.
   * @param calibration 原生传感器坐标系下的不可变相机标定，按值持有。
   *                    Immutable camera calibration in native sensor coordinates,
   *                    held by value.
   * @param name 相机实例名，同时作为 RamFS 命令文件名；构造时复制并持有。
   *             Camera instance name, also the RamFS command file name; copied and
   *             held at construction.
   * @param image_topic_name 图像 Topic 名称；构造时复制并持有。
   *                         Image Topic name; copied and held at construction.
   * @param imu_topic_name 同步 IMU Topic 名称；构造时复制并持有，`PublishImu()`
   *                       发布到该 Topic。
   *                       Synchronized IMU Topic name; copied and held at
   *                       construction, and the Topic `PublishImu()` publishes to.
   */
  CameraBase(LibXR::RamFS& ramfs, CameraCalibration calibration,
             std::string_view name = "camera",
             std::string_view image_topic_name = "camera_image",
             std::string_view imu_topic_name = "camera_imu")
      : calibration_(calibration),
        name_(name),
        image_topic_name_(image_topic_name),
        imu_topic_name_(imu_topic_name),
        cmd_file_(LibXR::RamFS::CreateFile(name_.CStr(), CommandFun, this)),
        image_topic_(
            LibXR::Topic::FindOrCreate<ImageTopicPayload>(image_topic_name_.CStr())),
        imu_topic_(LibXR::Topic::FindOrCreate<ImuStamped>(imu_topic_name_.CStr()))
  {
    REQUIRE(CameraBaseIntrinsicSanity::CameraCalibrationReasonable(calibration_));
    const auto result = image_pool_.Acquire(writable_frame_);
    REQUIRE(result == LibXR::ErrorCode::OK);
    ramfs.Add(cmd_file_);
  }

  CameraBase(const CameraBase&) = delete;
  CameraBase& operator=(const CameraBase&) = delete;
  CameraBase(CameraBase&&) = delete;
  CameraBase& operator=(CameraBase&&) = delete;

  /**
   * @brief 虚析构函数，只销毁对象。
   *        Virtual destructor that only destroys the object.
   *
   * CameraBase、图像订阅和派生类工作线程按进程生命周期使用。析构不是并发边界：
   * 销毁前所有 `SharedFrame` 已经释放，回调注销和工作线程的停止在析构之外完成。
   * CameraBase, image subscriptions and derived worker threads live for the whole
   * process. Destruction is not a concurrency boundary: all `SharedFrame` handles
   * are released before it, and callback unregistration and worker thread stopping
   * happen outside the destructor.
   */
  virtual ~CameraBase() = default;

  /**
   * @brief 设置曝光参数。
   *        Set the exposure.
   *
   * @param exposure 曝光值，单位由相机实现定义，HikCamera 使用微秒。
   *                 Exposure value; the unit is defined by the camera
   *                 implementation, HikCamera uses microseconds.
   */
  virtual void SetExposure(double exposure) = 0;

  /**
   * @brief 设置增益参数。
   *        Set the gain.
   *
   * @param gain 增益值，单位和量纲由相机实现定义。
   *             Gain value; the unit and scale are defined by the camera
   *             implementation.
   */
  virtual void SetGain(double gain) = 0;

  /**
   * @brief 返回本相机支持的固定采样档位表。
   *        Return the table of fixed sampling profiles the camera supports.
   *
   * 返回的 span 非空，底层存储及顺序在相机完整生命周期内保持稳定。首项描述相机
   * 构造完成时的当前档位；每个 `id` 唯一，`trigger_period_us` 非零。
   * The returned span is non-empty, and its storage and order stay stable for the
   * whole camera lifetime. The first entry describes the profile in effect after
   * construction; every `id` is unique and every `trigger_period_us` is non-zero.
   *
   * @return 档位表视图。
   *         View of the profile table.
   */
  [[nodiscard]] virtual std::span<const CameraProfile> Profiles() const noexcept = 0;

  /**
   * @brief 阻塞切换到指定固定采样档位。
   *        Switch to the given fixed sampling profile, blocking.
   *
   * 本调用在相机侧切档尝试结束后返回。请求当前档位时返回 `OK` 并填写 `applied`；
   * 不支持的档位返回 `NOT_SUPPORT`。失败时 `applied` 保持原值，部分配置不作为成功
   * 结果返回。失败后的原档回滚和采集恢复由派生驱动与上层状态机处理。
   * The call returns after the camera-side switch attempt ends. Requesting the
   * current profile returns `OK` and fills `applied`; an unsupported profile returns
   * `NOT_SUPPORT`. On failure `applied` keeps its value and a partial configuration
   * is not reported as success. Rolling back to the previous profile and resuming
   * capture after a failure are handled by the derived driver and the upper-level
   * state machine.
   *
   * @param id 请求的档位标识。
   *           Requested profile identifier.
   * @param applied 成功时写入实际生效的档位和逐帧几何；失败时保持原值。
   *                On success receives the profile in effect and its per-frame
   *                geometry; unchanged on failure.
   * @return 成功返回 `OK`，不支持返回 `NOT_SUPPORT`，其他失败返回对应错误码。
   *         `OK` on success, `NOT_SUPPORT` for an unsupported profile, otherwise the
   *         matching error code.
   */
  virtual LibXR::ErrorCode SwitchProfile(ProfileId id, AppliedProfile& applied) = 0;

  /**
   * @brief 返回本相机实例持有的原生标定。
   *        Return the native calibration held by this camera instance.
   *
   * 返回的常量引用在相机对象完整生命周期内有效。
   * The returned const reference is valid for the whole lifetime of the camera
   * object.
   *
   * @return 原生标定。
   *         Native calibration.
   */
  const CameraCalibration& Calibration() const noexcept { return calibration_; }

  /**
   * @brief 返回相机实例名视图。
   *        Return a view of the camera instance name.
   */
  std::string_view NameView() const { return name_.View(); }

  /**
   * @brief 返回以空字符结尾的相机实例名。
   *        Return the null-terminated camera instance name.
   */
  const char* Name() const { return name_.CStr(); }

  /**
   * @brief 返回图像 Topic 名称视图。
   *        Return a view of the image Topic name.
   */
  std::string_view ImageTopicNameView() const { return image_topic_name_.View(); }

  /**
   * @brief 返回以空字符结尾的图像 Topic 名称。
   *        Return the null-terminated image Topic name.
   */
  const char* ImageTopicName() const { return image_topic_name_.CStr(); }

  /**
   * @brief 返回同步 IMU Topic 名称视图。
   *        Return a view of the synchronized IMU Topic name.
   */
  std::string_view ImuTopicNameView() const { return imu_topic_name_.View(); }

  /**
   * @brief 返回以空字符结尾的同步 IMU Topic 名称。
   *        Return the null-terminated synchronized IMU Topic name.
   */
  const char* ImuTopicName() const { return imu_topic_name_.CStr(); }

  /**
   * @brief 发布同步 IMU 数据。
   *        Publish synchronized IMU data.
   *
   * @param imu 标准布局 IMU 载荷，发布到 `imu_topic_name`。时间戳由同步 Module
   *            填写，CameraBase 按原值发布。
   *            Standard-layout IMU payload published on `imu_topic_name`. The
   *            timestamp is set by the synchronization Module and CameraBase
   *            publishes it as given.
   */
  void PublishImu(ImuStamped imu) { imu_topic_.Publish(imu); }

  /**
   * @brief 获取生产者当前独占的可写图像槽位。
   *        Get the writable image slot currently owned exclusively by the producer.
   *
   * 当前没有可写槽位时，本方法从两槽池中获取一块。池中所有槽位仍被下游
   * `SharedFrame` 持有时返回 nullptr；下游释放任一槽位后，后续调用可再次成功。
   * 本方法和 `CommitImage()` 由同一个采集线程调用。
   * When no writable slot is held, the method takes one from the two-slot pool. It
   * returns nullptr while downstream `SharedFrame`s hold all slots, and a later call
   * succeeds after downstream releases a slot. This method and `CommitImage()` are
   * called from the same capture thread.
   *
   * @return 可写图像指针；没有可用槽位时为 nullptr。
   *         Writable image pointer; nullptr when no slot is available.
   */
  ImageFrame* GetWritableImage() noexcept
  {
    if (!writable_frame_.Valid())
    {
      if (image_pool_.Acquire(writable_frame_) != LibXR::ErrorCode::OK)
      {
        return nullptr;
      }
    }
    return image_pool_.GetWritable(writable_frame_);
  }

  /**
   * @brief 返回当前可立即获取的空闲图像槽位数。
   *        Return the number of free image slots that can be acquired right now.
   *
   * 生产者已经持有的可写槽位不计入该值。该值用于监控，其他线程可能在返回后
   * 立即释放槽位，是否取得槽位以 `GetWritableImage()` 的结果为准。
   * The writable slot already held by the producer is not counted. The value serves
   * monitoring: other threads may release slots right after the call returns, and
   * `GetWritableImage()` decides whether a slot is obtained.
   *
   * @return 空闲槽位数。
   *         Number of free slots.
   */
  [[nodiscard]] std::size_t AvailableImageSlots() const noexcept
  {
    return image_pool_.Available();
  }

  /**
   * @brief 发布当前图像并放弃生产者所有权。
   *        Publish the current image and give up producer ownership.
   *
   * Topic 载荷是指向栈上 `SharedFrame` 的临时借用指针，只在 `Publish()` 的同步
   * 回调期间有效。需要在回调之后继续使用图像的订阅者在回调内复制 `SharedFrame`，
   * 再把该副本移动到稳定的异步工作槽位；图像所有权通过回调建立，
   * `SyncSubscriber` 和 `QueuedSubscriber` 可能错过或丢弃消息。回调只做
   * 保留和入队，重处理在工作线程中进行。最后一个句柄析构后，槽位自动返回池中。
   * The Topic payload is a temporary borrowed pointer to a `SharedFrame` on the
   * stack, valid only during the synchronous callback of `Publish()`. A subscriber
   * that uses the image after the callback copies the `SharedFrame` inside the
   * callback and moves the copy to a stable asynchronous worker slot; ownership is
   * established through the callback, because `SyncSubscriber` and
   * `QueuedSubscriber` can miss or drop messages. The callback only retains and
   * enqueues, and heavy processing runs on the worker thread. The slot returns to
   * the pool after the last handle is destroyed.
   *
   * @return 存在可写帧并完成同步发布时返回 true；没有可写帧时返回 false。
   *         True when a writable frame existed and was published synchronously;
   *         false when there is no writable frame.
   */
  bool CommitImage()
  {
    if (!writable_frame_.Valid())
    {
      return false;
    }

    SharedFrame published_frame = std::move(writable_frame_);
    ImageTopicPayload message = &published_frame;
    image_topic_.Publish(message);
    return true;
  }

  /**
   * @brief RamFS 调试命令入口。
   *        RamFS debug command entry.
   *
   * 不带参数时打印用法；`set_exposure <值>` 和 `set_gain <值>` 调用对应的设置函数。
   * Without arguments it prints the usage; `set_exposure <value>` and
   * `set_gain <value>` call the matching setter.
   *
   * @param self 当前相机实例。
   *             Current camera instance.
   * @param argc 参数个数。
   *             Argument count.
   * @param argv 参数数组，支持 `set_exposure` 和 `set_gain`。
   *             Argument array; `set_exposure` and `set_gain` are supported.
   * @return 命令成功返回 0，未知命令返回 -1。
   *         0 on success, -1 for an unknown command.
   */
  static int CommandFun(CameraBase* self, int argc, char** argv)
  {
    const char* command = argc > 1 && argv != nullptr ? argv[1] : nullptr;
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Camera: %s\n\n">(self->Name());
      LibXR::STDIO::Printf<"用法:\r\n">();
      LibXR::STDIO::Printf<"  set_exposure <曝光>\r\n">();
      LibXR::STDIO::Printf<"  set_gain <增益>\r\n">();
      return 0;
    }
    else if (argc == 3 && command != nullptr && argv[2] != nullptr)
    {
      if (strcmp(command, "set_exposure") == 0)
      {
        self->SetExposure(atof(argv[2]));
        return 0;
      }
      if (strcmp(command, "set_gain") == 0)
      {
        self->SetGain(atof(argv[2]));
        return 0;
      }
    }

    LibXR::STDIO::Printf<"未知命令：%s\n">(command != nullptr ? command : "");
    return -1;
  }

 protected:
  /**
   * @brief 丢弃生产者当前尚未提交的可写图像槽位。
   *        Discard the writable image slot the producer holds and has not committed.
   *
   * 采集运行期间由调用 `GetWritableImage()` 和 `CommitImage()` 的采集线程调用。
   * 派生驱动停流并 join 采集线程后，控制线程可在没有并发图像槽访问时调用。已有
   * 下游 `SharedFrame` 不受影响，本调用不等待其他槽位归还；当前没有可写槽位时无
   * 操作。后续 `GetWritableImage()` 按需重新取槽。
   * While capturing, it is called by the capture thread that calls
   * `GetWritableImage()` and `CommitImage()`. After the derived driver stops the
   * stream and joins the capture thread, the control thread can call it when no
   * image slot is accessed concurrently. Existing downstream `SharedFrame`s are
   * unaffected and the call does not wait for other slots to return; it does nothing
   * when no writable slot is held. A later `GetWritableImage()` takes a slot again
   * on demand.
   */
  void DiscardWritableImage() noexcept { writable_frame_.Reset(); }

 private:
  const CameraCalibration calibration_;  ///< 原生标定 Native calibration
  LibXR::RuntimeStringView<> name_;      ///< 实例名和命令文件名 Name and command file
  LibXR::RuntimeStringView<> image_topic_name_;  ///< 图像 Topic 名称 Image Topic name
  LibXR::RuntimeStringView<> imu_topic_name_;    ///< IMU Topic 名称 IMU Topic name
  LibXR::RamFS::File cmd_file_;  ///< 曝光/增益命令文件 Exposure/gain command file
  LibXR::Topic image_topic_;     ///< 图像发布 Topic Image publication Topic
  LibXR::Topic imu_topic_;       ///< IMU 发布 Topic IMU publication Topic
  ImagePool image_pool_;         ///< 两槽图像池 Two-slot image pool
  SharedFrame writable_frame_;   ///< 当前可写槽位 Current writable slot
};
