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
#include "libxr.hpp"
#include "libxr_rw.hpp"
#include "ramfs.hpp"

// STL
#include <array>
#include <cstdint>
#include <initializer_list>
#include <vector>

/**
 * @class CameraBase
 * @brief 相机通用类型容器（仅定义公共类型/结构，不包含具体接口）。
 *
 * 说明：
 * - 像素编码与 ROS `sensor_msgs/Image.encoding` 的命名/含义保持一致或可直观映射。
 * - 所有矩阵按 **行优先（Row-major）** 存储。
 * - `step` 为每行字节数（stride, bytes per row），**不是每像素字节数**。
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
   * @union DistortionCoeffs
   * @brief 畸变系数共用体：共享同一段 `double[14]` 存储。
   */
  union DistortionCoeffs
  {
    std::array<double, 14> raw;  ///< 原始系数存储（索引 0..13）。

    struct
    {
      double k1, k2, p1, p2, k3;
    } plumb_bob;  ///< 5
    struct
    {
      double k1, k2, k3, k4;
    } equidistant;  ///< 4
    struct
    {
      double w;
    } fov;  ///< 1
    struct
    {
      double xi, k1, k2, p1, p2, k3;
    } omni;  ///< 6
    struct
    {
      double alpha, beta;
    } extended_unified;  ///< 2
    struct
    {
      double xi, alpha;
    } double_sphere;  ///< 2
    struct
    {
      double s1, s2, s3, s4;
    } thin_prism;  ///< 4
    struct
    {  // 14（OpenCV 完整有理多项式）
      double k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, tauX, tauY;
    } rational14;  ///< 14

    DistortionCoeffs() : raw{} {}  ///< 零初始化并激活 raw 成员。

    DistortionCoeffs(std::initializer_list<double> init)
    {
      ASSERT(init.size() <= 14);
      for (size_t i = 0; i < init.size(); i++)
      {
        raw[i] = init.begin()[i];
      }
    }
  };
  static_assert(sizeof(DistortionCoeffs) == sizeof(std::array<double, 14>),
                "overlay size mismatch");
  static_assert(alignof(DistortionCoeffs) == alignof(std::array<double, 14>),
                "overlay align mismatch");

  /**
   * @struct CameraInfo
   * @brief 图像尺寸、时间戳、编码与标定信息。
   *
   * 字段顺序与常见相机模型表达一致：
   * 1) 内参矩阵 K；2) 畸变模型与参数；3) 矫正旋转 R；4) 投影矩阵 P。
   */
  struct CameraInfo
  {
    // 基本图像信息
    uint32_t width{};   ///< 图像宽度（像素）。
    uint32_t height{};  ///< 图像高度（像素）。
    uint32_t step{};    ///< 每行字节数（bytes per row / stride）。
    LibXR::MicrosecondTimestamp timestamp{};  ///< 采集时间戳（微秒）。
    Encoding encoding{};                      ///< 像素编码类型。

    // 1) 内参矩阵 K = [ fx  0 cx ; 0 fy cy ; 0 0 1 ]（3×3，行优先）
    std::array<double, 9> camera_matrix;

    // 2) 畸变模型与参数（依据所选模型使用前 N 项，其余置 0）
    DistortionModel distortion_model{};        ///< 见 @ref DistortionModel 。
    DistortionCoeffs distortion_coefficients;  ///< 最多 14 项。

    // 3) 矫正后的旋转矩阵 R（3×3，行优先）
    std::array<double, 9> rectification_matrix;

    // 4) 投影矩阵 P（3×4，行优先）
    // P = [ fx'  0   cx'  Tx ;
    //        0  fy' cy'  Ty ;
    //        0   0   1    0 ]
    std::array<double, 12> projection_matrix;

    static inline std::vector<double> ToPnPDistCoeffs(
        CameraBase::DistortionModel model, const CameraBase::DistortionCoeffs& D)
    {
      std::vector<double> dc;
      switch (model)
      {
        case CameraBase::DistortionModel::NONE:
          // 无畸变：传空向量
          break;

        case CameraBase::DistortionModel::PLUMB_BOB:
          // Brown–Conrady：k1,k2,p1,p2,k3
          dc = {D.plumb_bob.k1, D.plumb_bob.k2, D.plumb_bob.p1, D.plumb_bob.p2,
                D.plumb_bob.k3};
          break;

        case CameraBase::DistortionModel::RATIONAL_POLYNOMIAL:
          // 多数 PnP 实现接受前 8 项：k1,k2,p1,p2,k3,k4,k5,k6
          dc = {D.rational14.k1, D.rational14.k2, D.rational14.p1, D.rational14.p2,
                D.rational14.k3, D.rational14.k4, D.rational14.k5, D.rational14.k6};
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

  const char* name_;
  LibXR::RamFS::File cmd_file_;
};
