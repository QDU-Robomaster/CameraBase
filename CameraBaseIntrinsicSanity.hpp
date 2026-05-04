#pragma once

#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

/**
 * @brief 相机内参合理性检查 helper。
 *
 * 该 helper 同时服务两条路径：
 * - CameraBase 普通运行路径用 constexpr 检查编译期 CameraInfo。
 * - 在线标定保存路径用同一套规则输出可读质量报告。
 */
class CameraBaseIntrinsicSanity
{
 public:
  /// 焦距相对图像长边的最小合理比例，留出广角镜头余量。
  static constexpr double min_focal_to_image = 0.15;
  /// 焦距相对图像长边的最大合理比例，覆盖窄视场镜头但拒绝明显错量纲。
  static constexpr double max_focal_to_image = 10.0;
  /// 方形像素相机的 fx/fy 宽松比例范围下限。
  static constexpr double min_fx_fy_ratio = 0.5;
  /// 方形像素相机的 fx/fy 宽松比例范围上限。
  static constexpr double max_fx_fy_ratio = 2.0;
  /// 主点允许落在图像外的归一化余量，用于兼容裁剪和轻微外推。
  static constexpr double principal_padding_ratio = 0.25;
  /// 内参矩阵非关键项允许的数值误差。
  static constexpr double matrix_eps = 1e-9;
  /// 畸变系数绝对值的宽松上限，用于拒绝 NaN/Inf 或明显爆炸的结果。
  static constexpr double max_abs_distortion = 10.0;
  /// constexpr 有限性检查的数值护栏。
  static constexpr double finite_guard = 1e12;

  /**
   * @brief 内参检查指标。
   */
  struct Metrics
  {
    bool matrix_shape_ok{false};       ///< K 是否符合 pinhole 矩阵形状。
    bool focal_ok{false};              ///< 焦距是否为正且量级合理。
    bool focal_ratio_ok{false};        ///< fx/fy 是否在宽松方形像素范围内。
    bool principal_point_ok{false};    ///< 主点是否落在合理图像范围内。
    bool distortion_ok{false};         ///< 畸变系数是否有限且不过度爆炸。
    bool all_ok{false};                ///< 全部检查是否通过。
    double fx{0.0};                    ///< 像素焦距 fx。
    double fy{0.0};                    ///< 像素焦距 fy。
    double cx{0.0};                    ///< 主点 x。
    double cy{0.0};                    ///< 主点 y。
    double fx_fy_ratio{0.0};           ///< fx / fy。
    double focal_to_image{0.0};        ///< max(fx, fy) / max(width, height)。
    double principal_dx_norm{0.0};     ///< 主点相对图像中心的 x 偏移，按宽度归一化。
    double principal_dy_norm{0.0};     ///< 主点相对图像中心的 y 偏移，按高度归一化。
    double max_abs_distortion_value{0.0};  ///< 最大畸变系数绝对值。
  };

  static constexpr double Abs(double value)
  {
    return value < 0.0 ? -value : value;
  }

  static constexpr double Max(double lhs, double rhs)
  {
    return lhs > rhs ? lhs : rhs;
  }

  static constexpr bool IsFiniteLike(double value)
  {
    return value == value && value > -finite_guard && value < finite_guard;
  }

  static constexpr bool MatrixShapeReasonable(const std::array<double, 9>& k)
  {
    return IsFiniteLike(k[0]) && IsFiniteLike(k[1]) &&
           IsFiniteLike(k[2]) && IsFiniteLike(k[3]) &&
           IsFiniteLike(k[4]) && IsFiniteLike(k[5]) &&
           IsFiniteLike(k[6]) && IsFiniteLike(k[7]) &&
           IsFiniteLike(k[8]) && Abs(k[1]) <= matrix_eps &&
           Abs(k[3]) <= matrix_eps && Abs(k[6]) <= matrix_eps &&
           Abs(k[7]) <= matrix_eps && Abs(k[8] - 1.0) <= matrix_eps;
  }

  static constexpr bool FocalReasonable(uint32_t width, uint32_t height,
                                        const std::array<double, 9>& k)
  {
    const double max_dim = Max(static_cast<double>(width),
                               static_cast<double>(height));
    const double max_focal = Max(k[0], k[4]);
    return width > 0 && height > 0 && k[0] > 0.0 && k[4] > 0.0 &&
           max_dim > 0.0 &&
           max_focal >= max_dim * min_focal_to_image &&
           max_focal <= max_dim * max_focal_to_image;
  }

  static constexpr bool FocalRatioReasonable(const std::array<double, 9>& k)
  {
    return k[4] > 0.0 && k[0] / k[4] >= min_fx_fy_ratio &&
           k[0] / k[4] <= max_fx_fy_ratio;
  }

  static constexpr bool PrincipalPointReasonable(uint32_t width, uint32_t height,
                                                 const std::array<double, 9>& k)
  {
    const double pad_x = static_cast<double>(width) * principal_padding_ratio;
    const double pad_y = static_cast<double>(height) * principal_padding_ratio;
    return width > 0 && height > 0 && k[2] >= -pad_x &&
           k[2] <= static_cast<double>(width) + pad_x &&
           k[5] >= -pad_y &&
           k[5] <= static_cast<double>(height) + pad_y;
  }

  static constexpr bool DistortionReasonable(const std::array<double, 14>& d)
  {
    for (double value : d)
    {
      if (!IsFiniteLike(value) || Abs(value) > max_abs_distortion)
      {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief 编译期 CameraInfo 内参是否合理。
   */
  template <typename CameraInfo>
  static constexpr bool CameraInfoReasonable(const CameraInfo& info)
  {
    return MatrixShapeReasonable(info.camera_matrix) &&
           FocalReasonable(info.width, info.height, info.camera_matrix) &&
           FocalRatioReasonable(info.camera_matrix) &&
           PrincipalPointReasonable(info.width, info.height, info.camera_matrix) &&
           DistortionReasonable(info.distortion_coefficients);
  }

  /**
   * @brief 计算内参质量指标，供标定报告使用。
   */
  static Metrics Evaluate(uint32_t width, uint32_t height,
                          const std::array<double, 9>& camera_matrix,
                          const std::array<double, 14>& distortion)
  {
    Metrics metrics{};
    metrics.fx = camera_matrix[0];
    metrics.fy = camera_matrix[4];
    metrics.cx = camera_matrix[2];
    metrics.cy = camera_matrix[5];
    metrics.fx_fy_ratio = metrics.fy != 0.0 ? metrics.fx / metrics.fy : 0.0;

    const double max_dim = Max(static_cast<double>(width),
                               static_cast<double>(height));
    metrics.focal_to_image =
        max_dim > 0.0 ? Max(metrics.fx, metrics.fy) / max_dim : 0.0;
    metrics.principal_dx_norm =
        width > 0 ? (metrics.cx - static_cast<double>(width) * 0.5) /
                        static_cast<double>(width)
                  : 0.0;
    metrics.principal_dy_norm =
        height > 0 ? (metrics.cy - static_cast<double>(height) * 0.5) /
                         static_cast<double>(height)
                   : 0.0;
    for (double value : distortion)
    {
      metrics.max_abs_distortion_value =
          Max(metrics.max_abs_distortion_value, Abs(value));
    }

    metrics.matrix_shape_ok = MatrixShapeReasonable(camera_matrix);
    metrics.focal_ok = FocalReasonable(width, height, camera_matrix);
    metrics.focal_ratio_ok = FocalRatioReasonable(camera_matrix);
    metrics.principal_point_ok =
        PrincipalPointReasonable(width, height, camera_matrix);
    metrics.distortion_ok = DistortionReasonable(distortion);
    metrics.all_ok = metrics.matrix_shape_ok && metrics.focal_ok &&
                     metrics.focal_ratio_ok && metrics.principal_point_ok &&
                     metrics.distortion_ok;
    return metrics;
  }

  /**
   * @brief 格式化内参合理性报告。
   */
  static std::string FormatReport(const Metrics& metrics)
  {
    std::ostringstream out;
    out << std::setprecision(10);
    out << "intrinsics_ok: " << (metrics.all_ok ? 1 : 0) << "\n";
    out << "matrix_shape_ok: " << (metrics.matrix_shape_ok ? 1 : 0) << "\n";
    out << "focal_ok: " << (metrics.focal_ok ? 1 : 0) << "\n";
    out << "focal_ratio_ok: " << (metrics.focal_ratio_ok ? 1 : 0) << "\n";
    out << "principal_point_ok: "
        << (metrics.principal_point_ok ? 1 : 0) << "\n";
    out << "distortion_ok: " << (metrics.distortion_ok ? 1 : 0) << "\n";
    out << "fx: " << metrics.fx << "\n";
    out << "fy: " << metrics.fy << "\n";
    out << "cx: " << metrics.cx << "\n";
    out << "cy: " << metrics.cy << "\n";
    out << "fx_fy_ratio: " << metrics.fx_fy_ratio << "\n";
    out << "focal_to_image: " << metrics.focal_to_image << "\n";
    out << "principal_dx_norm: " << metrics.principal_dx_norm << "\n";
    out << "principal_dy_norm: " << metrics.principal_dy_norm << "\n";
    out << "max_abs_distortion: "
        << metrics.max_abs_distortion_value << "\n";
    return out.str();
  }
};
