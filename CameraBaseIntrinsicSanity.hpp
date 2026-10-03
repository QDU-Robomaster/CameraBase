#pragma once

#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

/**
 * @brief 相机内参合理性检查工具。
 *        Sanity-check helpers for camera intrinsics.
 *
 * 同一套规则服务两条路径：
 * - CameraBase 构造时检查运行时原生 `CameraCalibration`。
 * - 在线标定保存时输出可读的质量报告。
 * The same rules serve two paths:
 * - CameraBase checks the runtime native `CameraCalibration` at construction.
 * - Online calibration saving prints a readable quality report.
 */
class CameraBaseIntrinsicSanity
{
 public:
  /// 焦距相对图像长边的最小比例，留出广角镜头余量。
  /// Minimum focal length relative to the longer image side, leaving margin for
  /// wide-angle lenses.
  static constexpr double min_focal_to_image = 0.15;
  /// 焦距相对图像长边的最大比例，覆盖窄视场镜头。
  /// Maximum focal length relative to the longer image side, covering narrow-FOV
  /// lenses.
  static constexpr double max_focal_to_image = 10.0;
  /// 方形像素相机 fx/fy 比例下限。
  /// Lower bound of the fx/fy ratio for square-pixel cameras.
  static constexpr double min_fx_fy_ratio = 0.5;
  /// 方形像素相机 fx/fy 比例上限。
  /// Upper bound of the fx/fy ratio for square-pixel cameras.
  static constexpr double max_fx_fy_ratio = 2.0;
  /// 主点允许落在图像外的归一化余量，兼容裁剪和轻微外推。
  /// Normalized margin by which the principal point may lie outside the image,
  /// allowing cropping and slight extrapolation.
  static constexpr double principal_padding_ratio = 0.25;
  /// 内参矩阵固定项允许的数值误差。
  /// Numeric tolerance of the fixed entries of the intrinsic matrix.
  static constexpr double matrix_eps = 1e-9;
  /// 畸变系数绝对值上限，用于拒绝 NaN/Inf 和过大的值。
  /// Upper bound of the distortion coefficient magnitude, rejecting NaN/Inf and
  /// oversized values.
  static constexpr double max_abs_distortion = 10.0;
  /// constexpr 有限性检查的数值上界。
  /// Numeric bound of the constexpr finiteness check.
  static constexpr double finite_guard = 1e12;

  /**
   * @brief 内参检查指标。
   *        Intrinsic check metrics.
   */
  struct Metrics
  {
    bool matrix_shape_ok{false};     ///< K 为 pinhole 形状 K is pinhole-shaped
    bool focal_ok{false};            ///< 焦距合理 Focal length plausible
    bool focal_ratio_ok{false};      ///< fx/fy 合理 fx/fy plausible
    bool principal_point_ok{false};  ///< 主点合理 Principal point plausible
    bool distortion_ok{false};       ///< 畸变系数合理 Distortion plausible
    bool all_ok{false};              ///< 全部检查通过 All checks passed
    double fx{0.0};                  ///< 像素焦距 fx Focal length fx, px
    double fy{0.0};                  ///< 像素焦距 fy Focal length fy, px
    double cx{0.0};                  ///< 主点 x Principal point x
    double cy{0.0};                  ///< 主点 y Principal point y
    double fx_fy_ratio{0.0};         ///< fx / fy
    double focal_to_image{0.0};      ///< max(fx, fy) / max(width, height)
    double principal_dx_norm{0.0};   ///< 主点 x 偏移（宽度归一化） x offset / width
    double principal_dy_norm{0.0};   ///< 主点 y 偏移（高度归一化） y offset / height
    double max_abs_distortion_value{0.0};  ///< 最大畸变系数绝对值 Max |distortion|
  };

  /**
   * @brief 返回绝对值。
   *        Return the absolute value.
   */
  static constexpr double Abs(double value) { return value < 0.0 ? -value : value; }

  /**
   * @brief 返回较大值。
   *        Return the larger value.
   */
  static constexpr double Max(double lhs, double rhs) { return lhs > rhs ? lhs : rhs; }

  /**
   * @brief 判断数值是否有限（非 NaN 且在 `finite_guard` 之内）。
   *        Check that a value is finite (not NaN and within `finite_guard`).
   */
  static constexpr bool IsFiniteLike(double value)
  {
    return value == value && value > -finite_guard && value < finite_guard;
  }

  /**
   * @brief 把检查结果转换为报告文字。
   *        Convert a check result to report text.
   */
  static constexpr const char* PassFail(bool ok) { return ok ? "通过" : "失败"; }

  /**
   * @brief 检查 K 是否为 pinhole 形状：各项有限，`k[1]`、`k[3]`、`k[6]`、`k[7]` 为
   * 0，`k[8]` 为 1。 Check that K has the pinhole shape: all entries finite, `k[1]`,
   * `k[3]`, `k[6]`, `k[7]` equal to 0 and `k[8]` equal to 1.
   */
  static constexpr bool MatrixShapeReasonable(const std::array<double, 9>& k)
  {
    return IsFiniteLike(k[0]) && IsFiniteLike(k[1]) && IsFiniteLike(k[2]) &&
           IsFiniteLike(k[3]) && IsFiniteLike(k[4]) && IsFiniteLike(k[5]) &&
           IsFiniteLike(k[6]) && IsFiniteLike(k[7]) && IsFiniteLike(k[8]) &&
           Abs(k[1]) <= matrix_eps && Abs(k[3]) <= matrix_eps &&
           Abs(k[6]) <= matrix_eps && Abs(k[7]) <= matrix_eps &&
           Abs(k[8] - 1.0) <= matrix_eps;
  }

  /**
   * @brief 检查 fx、fy 为正，且落在图像长边的 `min_focal_to_image` 到
   * `max_focal_to_image` 倍之间。 Check that fx and fy are positive and lie between
   * `min_focal_to_image` and `max_focal_to_image` times the longer image side.
   */
  static constexpr bool FocalReasonable(uint32_t width, uint32_t height,
                                        const std::array<double, 9>& k)
  {
    const double max_dim = Max(static_cast<double>(width), static_cast<double>(height));
    const double min_focal = max_dim * min_focal_to_image;
    const double max_focal = max_dim * max_focal_to_image;
    return width > 0 && height > 0 && k[0] > 0.0 && k[4] > 0.0 && max_dim > 0.0 &&
           k[0] >= min_focal && k[0] <= max_focal && k[4] >= min_focal &&
           k[4] <= max_focal;
  }

  /**
   * @brief 检查 fx/fy 落在 `min_fx_fy_ratio` 到 `max_fx_fy_ratio` 之间。
   *        Check that fx/fy lies between `min_fx_fy_ratio` and `max_fx_fy_ratio`.
   */
  static constexpr bool FocalRatioReasonable(const std::array<double, 9>& k)
  {
    return k[4] > 0.0 && k[0] / k[4] >= min_fx_fy_ratio && k[0] / k[4] <= max_fx_fy_ratio;
  }

  /**
   * @brief 检查主点落在图像范围按 `principal_padding_ratio` 外扩后的区域内。
   *        Check that the principal point lies inside the image extended by
   * `principal_padding_ratio`.
   */
  static constexpr bool PrincipalPointReasonable(uint32_t width, uint32_t height,
                                                 const std::array<double, 9>& k)
  {
    const double pad_x = static_cast<double>(width) * principal_padding_ratio;
    const double pad_y = static_cast<double>(height) * principal_padding_ratio;
    return width > 0 && height > 0 && k[2] >= -pad_x &&
           k[2] <= static_cast<double>(width) + pad_x && k[5] >= -pad_y &&
           k[5] <= static_cast<double>(height) + pad_y;
  }

  /**
   * @brief 检查全部畸变系数有限且绝对值不超过 `max_abs_distortion`。
   *        Check that all distortion coefficients are finite with magnitude at most
   * `max_abs_distortion`.
   */
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
   * @brief 检查原生相机标定内参是否合理。
   *        Check that the native camera calibration intrinsics are plausible.
   *
   * @tparam CameraCalibration 含 `native_width`、`native_height`、`camera_matrix`
   *         和 `distortion_coefficients` 的标定类型。
   *         Calibration type with `native_width`, `native_height`, `camera_matrix`
   *         and `distortion_coefficients`.
   * @param calibration 待检查的标定。
   *                    Calibration to check.
   * @return 矩阵形状、焦距、焦距比例、主点和畸变系数全部合理时返回 true。
   *         True when matrix shape, focal length, focal ratio, principal point and
   *         distortion are all plausible.
   */
  template <typename CameraCalibration>
  static constexpr bool CameraCalibrationReasonable(const CameraCalibration& calibration)
  {
    return MatrixShapeReasonable(calibration.camera_matrix) &&
           FocalReasonable(calibration.native_width, calibration.native_height,
                           calibration.camera_matrix) &&
           FocalRatioReasonable(calibration.camera_matrix) &&
           PrincipalPointReasonable(calibration.native_width, calibration.native_height,
                                    calibration.camera_matrix) &&
           DistortionReasonable(calibration.distortion_coefficients);
  }

  /**
   * @brief 计算内参质量指标，供标定报告使用。
   *        Compute the intrinsic quality metrics for calibration reports.
   *
   * @param width 图像宽度，单位像素。
   *              Image width in pixels.
   * @param height 图像高度，单位像素。
   *               Image height in pixels.
   * @param camera_matrix 3x3 内参矩阵，按行存储。
   *                      3x3 intrinsic matrix in row-major order.
   * @param distortion 14 项畸变系数。
   *                   14 distortion coefficients.
   * @return 各项检查结果和数值指标。
   *         Check results and numeric metrics.
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

    const double max_dim = Max(static_cast<double>(width), static_cast<double>(height));
    metrics.focal_to_image = max_dim > 0.0 ? Max(metrics.fx, metrics.fy) / max_dim : 0.0;
    metrics.principal_dx_norm =
        width > 0
            ? (metrics.cx - static_cast<double>(width) * 0.5) / static_cast<double>(width)
            : 0.0;
    metrics.principal_dy_norm = height > 0
                                    ? (metrics.cy - static_cast<double>(height) * 0.5) /
                                          static_cast<double>(height)
                                    : 0.0;
    for (double value : distortion)
    {
      if (!IsFiniteLike(value))
      {
        metrics.max_abs_distortion_value = Abs(value);
        break;
      }
      metrics.max_abs_distortion_value =
          Max(metrics.max_abs_distortion_value, Abs(value));
    }

    metrics.matrix_shape_ok = MatrixShapeReasonable(camera_matrix);
    metrics.focal_ok = FocalReasonable(width, height, camera_matrix);
    metrics.focal_ratio_ok = FocalRatioReasonable(camera_matrix);
    metrics.principal_point_ok = PrincipalPointReasonable(width, height, camera_matrix);
    metrics.distortion_ok = DistortionReasonable(distortion);
    metrics.all_ok = metrics.matrix_shape_ok && metrics.focal_ok &&
                     metrics.focal_ratio_ok && metrics.principal_point_ok &&
                     metrics.distortion_ok;
    return metrics;
  }

  /**
   * @brief 格式化内参合理性报告。
   *        Format the intrinsic sanity report.
   *
   * @param metrics `Evaluate()` 返回的指标。
   *                Metrics returned by `Evaluate()`.
   * @return 每行一项的文本报告。
   *         Text report with one item per line.
   */
  static std::string FormatReport(const Metrics& metrics)
  {
    std::ostringstream out;
    out << std::setprecision(10);
    out << "内参判定(intrinsics_ok): " << PassFail(metrics.all_ok) << "\n";
    out << "内参矩阵形状(matrix_shape_ok): " << PassFail(metrics.matrix_shape_ok) << "\n";
    out << "焦距量级(focal_ok): " << PassFail(metrics.focal_ok) << "\n";
    out << "焦距比例(focal_ratio_ok): " << PassFail(metrics.focal_ratio_ok) << "\n";
    out << "主点范围(principal_point_ok): " << PassFail(metrics.principal_point_ok)
        << "\n";
    out << "畸变系数(distortion_ok): " << PassFail(metrics.distortion_ok) << "\n";
    out << "焦距 fx(fx): " << metrics.fx << "\n";
    out << "焦距 fy(fy): " << metrics.fy << "\n";
    out << "主点 cx(cx): " << metrics.cx << "\n";
    out << "主点 cy(cy): " << metrics.cy << "\n";
    out << "焦距比例 fx/fy(fx_fy_ratio): " << metrics.fx_fy_ratio << "\n";
    out << "焦距/图像长边(focal_to_image): " << metrics.focal_to_image << "\n";
    out << "主点 x 偏移归一化(principal_dx_norm): " << metrics.principal_dx_norm << "\n";
    out << "主点 y 偏移归一化(principal_dy_norm): " << metrics.principal_dy_norm << "\n";
    out << "最大畸变系数绝对值(max_abs_distortion): " << metrics.max_abs_distortion_value
        << "\n";
    return out.str();
  }
};
