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
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "app_framework.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "object_pool.hpp"
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
   * @enum ProfileId
   * @brief 相机支持的固定采样档位标识。
   */
  enum class ProfileId : uint8_t
  {
    WIDE = 0,  ///< 宽视场档位。
    NARROW,    ///< 窄视场档位。
  };

  /**
   * @struct FrameGeometry
   * @brief 一帧图像相对原生传感器坐标系的采样关系。
   *
   * `roi_offset_*_native` 和 `sample_phase_*_native` 均使用原生像素中心坐标。
   * 结构按值进入 `ImageFrame`，可安全跨线程和共享内存。
   */
  struct FrameGeometry
  {
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

  /**
   * @struct CameraProfile
   * @brief 相机驱动公开的一个固定采样档位。
   */
  struct CameraProfile
  {
    ProfileId id{ProfileId::WIDE};  ///< 档位标识。
    FrameGeometry geometry{};       ///< 该档位成功生效后的逐帧几何。
    uint32_t trigger_period_us{};   ///< 该档位要求的触发周期，单位微秒且必须非零。
  };

  /**
   * @struct AppliedProfile
   * @brief 驱动成功切档后实际生效的档位快照。
   */
  struct AppliedProfile
  {
    ProfileId id{ProfileId::WIDE};  ///< 实际生效的档位标识。
    FrameGeometry geometry{};       ///< 后续图像逐帧携带的实际几何。
  };

  static_assert(sizeof(FrameGeometry) == 36, "CameraTypes::FrameGeometry ABI changed");
  static_assert(std::is_trivially_copyable_v<FrameLayout>);
  static_assert(std::is_standard_layout_v<FrameLayout>);
  static_assert(std::is_trivially_copyable_v<CameraCalibration>);
  static_assert(std::is_standard_layout_v<CameraCalibration>);
  static_assert(std::is_aggregate_v<CameraCalibration>);
  static_assert(std::is_trivially_copyable_v<FrameGeometry>);
  static_assert(std::is_standard_layout_v<FrameGeometry>);
  static_assert(std::is_trivially_copyable_v<CameraProfile>);
  static_assert(std::is_standard_layout_v<CameraProfile>);
  static_assert(std::is_trivially_copyable_v<AppliedProfile>);
  static_assert(std::is_standard_layout_v<AppliedProfile>);

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
   * @brief 比较两个逐帧几何快照是否描述完全相同的采样关系。
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
/**
 * @brief 进程内共享所有权对象池。
 *
 * 底层 `MPMCObjectPool` 继续负责固定槽位的分配和回收；每个已借出槽位由一个
 * `SharedHandle` 独占或共享持有。复制 handle 增加原子引用计数，移动只转移本地所有权，
 * 最后一个 handle 析构时才把底层独占 pool handle 归还。句柄只暴露只读访问；生产者
 * 必须通过所属 pool 的 `GetWritable()` 取得独占可写指针。
 *
 * @tparam Data 槽位中的对象类型。
 * @tparam SlotCount 固定槽位数。
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
   *
   * 句柄只在当前进程内有效。复制构造和复制赋值会增加引用计数；析构和 `Reset()` 会
   * 减少引用计数。所有副本只读访问同一个 `Data`。调用方不得让 handle 的生命周期
   * 超过所属 `SharedObjectPool`。
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

    [[nodiscard]] bool Valid() const noexcept { return pool_ != nullptr; }

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
     *
     * 其他线程可在返回后立即复制或释放句柄，调用方不得用该值决定是否拥有独占访问权。
     */
    [[nodiscard]] uint32_t UseCount() const noexcept
    {
      return Valid() ? pool_->UseCount(slot_index_, generation_) : 0U;
    }

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
   * @return 成功返回 `OK`，没有空闲槽位返回底层 pool 的 `EMPTY`。
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
   *
   * 调用方必须独占访问 `handle` 对象，且在使用返回指针期间不得复制、移动或重置该
   * handle。句柄无效、来自其他池、代次不匹配或已有共享副本时返回 nullptr。
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
    // 在入队完成前不可能取得该索引，因此不会与这里并发改写同一个 optional。
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
 *
 * `CameraBase` 定义相机类型和图像提交方式。具体相机驱动负责填充本类八槽对象池中的
 * `ImageFrame`。`CommitImage()` 通过普通 topic 同步发布临时 `SharedFrame` 指针；
 * 订阅回调复制句柄后可把同一图像槽位交给异步线程，不复制图像字节。
 * 本类不负责记录、标定、预览、图像同步或坐标系转换。
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
  using ProfileId = CameraTypes::ProfileId;                  ///< 固定采样档位标识。
  using CameraProfile = CameraTypes::CameraProfile;          ///< 可选档位描述。
  using AppliedProfile = CameraTypes::AppliedProfile;        ///< 实际生效档位快照。
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
   * 时间域；重复、回退、回绕或复位由下游同步状态机处理。确定性回放必须保留原始
   * 顺序和时间，包括原始重复值。
   */
  struct alignas(image_alignment) ImageFrame
  {
    LibXR::MicrosecondTimestamp
        timestamp_us;        ///< 传感器采样时间，单位微秒；不是主机到达或发布时间。
    FrameGeometry geometry;  ///< 当前像素负载到原生传感器坐标系的映射。
    alignas(image_alignment)
        std::array<uint8_t, image_bytes> data;  ///< 图像字节负载，含每行 padding。
  };

  /// CameraBase 进程内图像池的固定槽位数。
  static constexpr std::size_t image_slot_count = 8U;
  /// CameraBase 自有的共享图像对象池。
  using ImagePool = CameraBaseDetail::SharedObjectPool<ImageFrame, image_slot_count>;
  /// 一个图像槽位的可复制进程内共享所有权句柄。
  using SharedFrame = typename ImagePool::SharedHandle;
  /// 普通 topic 只在同步回调期间借用该指针，不拥有 `SharedFrame`。
  using ImageTopicPayload = const SharedFrame*;

  /**
   * @struct ImuStamped
   * @brief 与图像同步搬运的位姿与惯导采样。
   *
   * `CameraBase` 只定义并发布该载荷；同步、插值、时间域和坐标系由生成该数据的模块
   * 保证。阶段包装可以把相机本地时间的 `ImageFrame` 与 MCU 时间的本对象关联起来，
   * 两个 `timestamp_us` 不要求数值相等。
   */
  struct alignas(8) ImuStamped
  {
    LibXR::MicrosecondTimestamp timestamp_us;      ///< 生成模块定义的权威时间，单位微秒。
    std::array<float, 4> rotation_wxyz;            ///< 姿态四元数，顺序为 wxyz。
    std::array<float, 3> translation_xyz;          ///< 平移，单位米；无平移来源时置零。
    std::array<float, 3> angular_velocity_xyz;     ///< 角速度，单位 rad/s。
    std::array<float, 3> linear_acceleration_xyz;  ///< 线加速度，单位 m/s^2。
  };

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
        image_topic_(
            LibXR::Topic::FindOrCreate<ImageTopicPayload>(image_topic_name_.c_str())),
        imu_topic_(LibXR::Topic::FindOrCreate<ImuStamped>(imu_topic_name_.c_str()))
  {
    ASSERT(CameraBaseIntrinsicSanity::CameraCalibrationReasonable(calibration_));
    const auto result = image_pool_.Acquire(writable_frame_);
    ASSERT(result == LibXR::ErrorCode::OK);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
  }

  CameraBase(const CameraBase&) = delete;
  CameraBase& operator=(const CameraBase&) = delete;
  CameraBase(CameraBase&&) = delete;
  CameraBase& operator=(CameraBase&&) = delete;

  /**
   * @brief 仅提供多态类型完整性，不承担订阅注销或线程停止语义。
   *
   * CameraBase、图像订阅和派生类工作线程按进程生命周期使用。析构不是并发边界，调用
   * 方必须保证所有 `SharedFrame` 已经释放，不得依赖本析构等待回调或终止工作线程。
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
   * @brief 返回本相机支持的固定采样档位表。
   *
   * 返回的 span 必须非空，底层存储及顺序在相机完整生命周期内保持稳定。首项描述相机
   * 构造完成时的当前档位；每个 `id` 必须唯一，`trigger_period_us` 必须非零。
   */
  [[nodiscard]] virtual std::span<const CameraProfile> Profiles() const noexcept = 0;

  /**
   * @brief 阻塞切换到指定固定采样档位。
   *
   * 本调用在相机侧切档尝试结束后返回。请求当前档位也必须返回 `OK` 并填写
   * `applied`；不支持的档位返回 `NOT_SUPPORT`。任何失败都不得修改 `applied`，也不得
   * 把部分配置暴露为成功结果。本接口不承诺失败后自动回滚原档或恢复采集。
   *
   * @param id 请求的档位标识。
   * @param applied 成功时写入实际生效的档位和逐帧几何；失败时保持原值。
   * @return 成功返回 `OK`，不支持返回 `NOT_SUPPORT`，其他失败返回对应错误码。
   */
  virtual LibXR::ErrorCode SwitchProfile(ProfileId id, AppliedProfile& applied) = 0;

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
   * @param imu 标准布局 IMU 载荷，会按 `imu_topic_name` 发布。其时间戳由同步模块填写，
   * CameraBase 不做转换或校验。
   */
  void PublishImu(ImuStamped imu) { imu_topic_.Publish(imu); }

  /**
   * @brief 获取生产者当前独占的可写图像槽位。
   *
   * 若当前没有写槽位，本方法从八槽池中尝试获取一块。池中所有槽位仍被下游
   * `SharedFrame` 持有时返回 nullptr；下游释放任一槽位后，后续调用可再次成功。
   * 本方法和 `CommitImage()` 只能由同一个采集线程调用。
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
   *
   * 当前生产者已经持有的可写槽位不计入该值。该值只用于监控，其他线程可能在返回后
   * 立即释放槽位，因此调用方不得据此代替 `GetWritableImage()` 的结果。
   */
  [[nodiscard]] std::size_t AvailableImageSlots() const noexcept
  {
    return image_pool_.Available();
  }

  /**
   * @brief 发布当前图像并放弃生产者所有权。
   *
   * topic payload 是指向栈上 `SharedFrame` 的临时借用指针，只在 `Publish()` 的同步
   * 回调期间有效。需要跨越回调继续使用图像的订阅者必须在回调内复制 `SharedFrame`，
   * 再把该副本移动到稳定的异步工作槽位；不得保存裸指针，也不得用
   * `SyncSubscriber` 或 `QueuedSubscriber` 承担图像所有权。回调应只做
   * retain/enqueue，重处理留在工作线程。最后一个句柄析构后，槽位自动返回池中。
   *
   * @return 当前存在可写帧并完成同步发布时返回 true；没有可写帧时返回 false。
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

 protected:
  /**
   * @brief 丢弃生产者当前尚未提交的可写图像槽位。
   *
   * 采集运行期间只能由调用 `GetWritableImage()` 和 `CommitImage()` 的采集线程调用。
   * 派生驱动停流并 join 采集线程后，控制线程可在确认没有并发图像槽访问时调用。已有
   * 下游 `SharedFrame` 不受影响，本调用不等待其他槽位归还；当前没有可写槽位时为空
   * 操作。后续 `GetWritableImage()` 会按需重新取槽。
   */
  void DiscardWritableImage() noexcept { writable_frame_.Reset(); }

 private:
  const CameraCalibration calibration_;  ///< 构造期固定的原生相机标定。
  std::string name_;                     ///< 相机实例名和 RamFS 命令文件名。
  std::string image_topic_name_;         ///< 图像逻辑 topic 名称。
  std::string imu_topic_name_;           ///< `PublishImu()` 发布同步 IMU 的 topic 名称。
  LibXR::RamFS::File cmd_file_;          ///< 曝光/增益调试命令入口。
  LibXR::Topic image_topic_;             ///< 临时 `SharedFrame` 指针发布 topic。
  LibXR::Topic imu_topic_;               ///< 同步 IMU 发布 topic。
  ImagePool image_pool_;                 ///< CameraBase 自有八槽图像池。
  SharedFrame writable_frame_;           ///< 生产者当前独占的可写槽位。
};
