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

// STL
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

/**
 * @class CameraBase
 * @brief 相机通用类型容器（仅定义公共类型/结构，不包含具体接口）。
 *
 * 说明：
 * - 像素编码与 ROS `sensor_msgs/Image.encoding` 的命名/含义保持一致或可直观映射。
 * - 所有矩阵按 **行优先（Row-major）** 存储。
 * - `step` 为固定输出模式下的每行字节数（stride, bytes per row），**不是每像素字节数**。
 * - 帧级时间戳等动态元数据由具体帧载荷承载，不放在 `CameraInfo` 中。
 */
class CameraBase
{
 public:
  /**
   * @enum Encoding
   * @brief 图像像素编码类型。
   */
  enum Encoding : uint8_t
  {
    INVALID = 0,   ///< 无效/未设置的编码占位。
    RGB8,          ///< 8 位/通道，RGB 通道顺序，24bpp。
    BGR8,          ///< 8 位/通道，BGR 通道顺序，24bpp。
    RGBA8,         ///< 8 位/通道，RGBA（含 alpha），32bpp。
    BGRA8,         ///< 8 位/通道，BGRA（含 alpha），32bpp。
    RGB16,         ///< 16 位/通道，RGB，48bpp。
    BGR16,         ///< 16 位/通道，BGR，48bpp。
    RGBA16,        ///< 16 位/通道，RGBA，64bpp。
    BGRA16,        ///< 16 位/通道，BGRA，64bpp。
    MONO8,         ///< 8 位单通道灰度（Y）。
    MONO16,        ///< 16 位单通道灰度（Y）。
    BAYER_RGGB8,   ///< 8 位 Bayer RAW，排列 RGGB。
    BAYER_GRBG8,   ///< 8 位 Bayer RAW，排列 GRBG。
    BAYER_GBRG8,   ///< 8 位 Bayer RAW，排列 GBRG。
    BAYER_BGGR8,   ///< 8 位 Bayer RAW，排列 BGGR。
    BAYER_RGGB16,  ///< 16 位 Bayer RAW，排列 RGGB。
    BAYER_GRBG16,  ///< 16 位 Bayer RAW，排列 GRBG。
    BAYER_GBRG16,  ///< 16 位 Bayer RAW，排列 GBRG。
    BAYER_BGGR16,  ///< 16 位 Bayer RAW，排列 BGGR。
    YUV422         ///< 打包 YUV 4:2:2（常见布局
                   ///< YUYV/UYVY）。如需精确到子格式，请在协议层另行注明。
  };

  /**
   * @enum DistortionModel
   * @brief 畸变模型枚举。
   */
  enum class DistortionModel : uint8_t
  {
    NONE = 0,             ///< 无畸变；所有系数视为 0。
    PLUMB_BOB,            ///< Brown–Conrady（radtan）；k1,k2,p1,p2,k3。
    RATIONAL_POLYNOMIAL,  ///< OpenCV 扩展 8/12/14 参：+k4..k6,+s1..s4,+tauX,tauY。
    EQUIDISTANT,          ///< 等距（KB4 fisheye）：k1..k4。
    FOV,                  ///< Devernay–Basu FOV：常用单参 w。
    OMNI,                 ///< 统一全向（Mei/Scaramuzza）：含 xi。
    EXTENDED_UNIFIED,     ///< 扩展统一（EUCM）：alpha,beta。
    DOUBLE_SPHERE,        ///< 双球：xi,alpha。
    THIN_PRISM,           ///< 薄棱镜项 s1..s4；常与 radtan 组合。
    UNKNOWN               ///< 未知/自定义。
  };

  /**
   * @struct SharedImageFrame
   * @brief Linux shared image topic payload.
   *
   * 说明：
   * - 这是给 `LinuxSharedTopic` 用的固定容量帧结构；
   * - `width/height/step/encoding` 描述当前有效图像；
   * - `data_size` 为本帧有效字节数，必须不超过 @ref SharedImageFrame::max_bytes；
   * - `data` 显式按缓存行对齐，避免共享内存图像负载起始地址靠默认布局碰运气。
   */
  struct alignas(64) SharedImageFrame
  {
    static constexpr uint32_t max_width = 1920;
    static constexpr uint32_t max_height = 1080;
    static constexpr uint32_t max_channels = 4;
    static constexpr size_t max_bytes =
        static_cast<size_t>(max_width) * max_height * max_channels;
    static constexpr size_t data_alignment = 64;
    static constexpr const char* topic_name = "image_frame";

    uint64_t timestamp_us;
    uint64_t sequence;
    uint32_t width;
    uint32_t height;
    uint32_t step;
    uint32_t data_size;
    Encoding encoding;
    alignas(data_alignment) std::array<uint8_t, max_bytes> data;

    [[nodiscard]] static constexpr bool HasValidPayload(const SharedImageFrame& frame)
    {
      return frame.width > 0 && frame.height > 0 && frame.step > 0 &&
             frame.data_size > 0 && frame.data_size <= max_bytes &&
             static_cast<size_t>(frame.step) * static_cast<size_t>(frame.height) <=
                 frame.data_size;
    }
  };

  static_assert(std::is_trivial_v<SharedImageFrame>,
                "SharedImageFrame must be trivial");
  static_assert(std::is_trivially_copyable_v<SharedImageFrame>,
                "SharedImageFrame must be trivially copyable");
  static_assert(std::is_standard_layout_v<SharedImageFrame>,
                "SharedImageFrame must be standard layout");
  static_assert(alignof(SharedImageFrame) >= SharedImageFrame::data_alignment,
                "SharedImageFrame alignment is too small");
  static_assert(offsetof(SharedImageFrame, data) % SharedImageFrame::data_alignment == 0,
                "SharedImageFrame data must be aligned");

  /**
   * @struct CameraInfo
   * @brief 静态图像尺寸、编码与标定信息。
   *
   * 设计目标：
   * - 仅保留结构化字段，便于作为项目级 `inline constexpr` 常量；
   * - 可直接作为非类型模板参数（NTTP）；
   * - 帧级动态元数据由帧载荷自身承载，不再通过 `camera_info` topic 发布。
   */
  struct CameraInfo
  {
    // 基本图像信息
    uint32_t width{};                         ///< 图像宽度（像素）。
    uint32_t height{};                        ///< 图像高度（像素）。
    uint32_t step{};                          ///< 每行字节数（bytes per row / stride）。
    Encoding encoding{};                      ///< 像素编码类型。

    // 1) 内参矩阵 K = [ fx  0 cx ; 0 fy cy ; 0 0 1 ]（3×3，行优先）
    std::array<double, 9> camera_matrix;

    // 2) 畸变模型与参数（依据所选模型使用前 N 项，其余置 0）
    DistortionModel distortion_model{};               ///< 见 @ref DistortionModel 。
    std::array<double, 14> distortion_coefficients;  ///< 最多 14 项。

    // 3) 矫正后的旋转矩阵 R（3×3，行优先）
    std::array<double, 9> rectification_matrix;

    // 4) 投影矩阵 P（3×4，行优先）
    // P = [ fx'  0   cx'  Tx ;
    //        0  fy' cy'  Ty ;
    //        0   0   1    0 ]
    std::array<double, 12> projection_matrix;

    static inline std::vector<double> ToPnPDistCoeffs(
        CameraBase::DistortionModel model, const std::array<double, 14>& distortion_coeffs)
    {
      std::vector<double> dc;
      switch (model)
      {
        case CameraBase::DistortionModel::NONE:
          // 无畸变：传空向量
          break;

        case CameraBase::DistortionModel::PLUMB_BOB:
          // Brown–Conrady：k1,k2,p1,p2,k3
          dc = {distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2],
                distortion_coeffs[3], distortion_coeffs[4]};
          break;

        case CameraBase::DistortionModel::RATIONAL_POLYNOMIAL:
          // 多数 PnP 实现接受前 8 项：k1,k2,p1,p2,k3,k4,k5,k6
          dc = {distortion_coeffs[0], distortion_coeffs[1], distortion_coeffs[2],
                distortion_coeffs[3], distortion_coeffs[4], distortion_coeffs[5],
                distortion_coeffs[6], distortion_coeffs[7]};
          XR_LOG_WARN(
              "PnPSolver: using 8-term rational; extend to 14 if backend supports.");
          break;

        case CameraBase::DistortionModel::EQUIDISTANT:
        case CameraBase::DistortionModel::FOV:
        case CameraBase::DistortionModel::OMNI:
        case CameraBase::DistortionModel::EXTENDED_UNIFIED:
        case CameraBase::DistortionModel::DOUBLE_SPHERE:
        case CameraBase::DistortionModel::THIN_PRISM:
        case CameraBase::DistortionModel::UNKNOWN:
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

  CameraBase(LibXR::HardwareContainer& hw, const char* name = "camera")
      : name_(name), cmd_file_(LibXR::RamFS::CreateFile(name, CommandFun, this))
  {
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
  }

  virtual void SetExposure(double exposure) = 0;

  virtual void SetGain(double gain) = 0;

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
      else if (strcmp(argv[1], "set_gain") == 0)
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
    return CommandFun(static_cast<CameraBase*>(instance), argc, argv);
  }

  const char* name_;
  LibXR::RamFS::File cmd_file_;
};
