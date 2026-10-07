#pragma once

#include <cmath>

#include "CameraTypes.hpp"

namespace CameraTypes
{
/**
 * @brief 标定是否合理：用于在启动时发现 YAML 里写错的常量。
 *        Whether a calibration is plausible; catches mistyped YAML constants at
 *        startup.
 *
 * 焦距为正且不超过图像长边的 10 倍，fx/fy 在 0.5–2 之间，主点在图像内，畸变系数有限且
 * 绝对值不超过 10。
 * Focal lengths are positive and at most 10 times the longer side, fx/fy is within
 * 0.5–2, the principal point lies inside the image, and the distortion coefficients
 * are finite with magnitude at most 10.
 */
inline bool CalibrationReasonable(const CameraCalibration& calibration)
{
  const double width = calibration.native_width;
  const double height = calibration.native_height;
  const double longer = width > height ? width : height;
  const bool focal_ok = calibration.fx > 0.0 && calibration.fy > 0.0 &&
                        calibration.fx <= 10.0 * longer &&
                        calibration.fy <= 10.0 * longer;
  const double ratio = calibration.fy > 0.0 ? calibration.fx / calibration.fy : 0.0;
  const bool principal_ok = calibration.cx > 0.0 && calibration.cx < width &&
                            calibration.cy > 0.0 && calibration.cy < height;
  bool distortion_ok = true;
  for (double k : calibration.distortion)
  {
    distortion_ok = distortion_ok && std::isfinite(k) && std::fabs(k) <= 10.0;
  }
  return width > 0.0 && height > 0.0 && focal_ok && ratio >= 0.5 && ratio <= 2.0 &&
         principal_ok && distortion_ok;
}
}  // namespace CameraTypes
