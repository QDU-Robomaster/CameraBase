#pragma once

#include <array>
#include <cstdint>

/**
 * @brief 相机帧尺寸、标定与采样几何 / Camera frame size, calibration and sampling
 *        geometry.
 *
 * 全系统只有一种帧：640×512 BayerRG8，`(0, 0)` 为 R，每行 640 字节。帧通过
 * `FrameGeometry` 对应到原生传感器坐标，标定始终在原生坐标下给出。
 * The whole system uses one frame layout: 640×512 BayerRG8 with R at `(0, 0)` and
 * 640 bytes per row. `FrameGeometry` maps a frame to native sensor coordinates, where
 * the calibration is always given.
 */
namespace CameraTypes
{
inline constexpr uint32_t FRAME_WIDTH = 640;   ///< 帧宽，像素 / Frame width, px
inline constexpr uint32_t FRAME_HEIGHT = 512;  ///< 帧高，像素 / Frame height, px
inline constexpr uint32_t FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT;  ///< 每帧字节数

/// 像素坐标 / Pixel coordinate.
struct Point2d
{
  double x;
  double y;
};

/**
 * @brief 原生传感器坐标下的针孔标定，畸变为 OpenCV plumb_bob 的 5 个系数。
 *        Pinhole calibration in native sensor coordinates; distortion uses the five
 *        OpenCV plumb_bob coefficients.
 */
struct CameraCalibration
{
  uint32_t native_width;             ///< 原生传感器宽度，像素 / Native sensor width, px
  uint32_t native_height;            ///< 原生传感器高度，像素 / Native sensor height, px
  double fx;                         ///< 焦距 x，像素 / Focal length x, px
  double fy;                         ///< 焦距 y，像素 / Focal length y, px
  double cx;                         ///< 主点 x，像素 / Principal point x, px
  double cy;                         ///< 主点 y，像素 / Principal point y, px
  std::array<double, 5> distortion;  ///< k1, k2, p1, p2, k3
};

/**
 * @brief 一帧相对原生传感器的采样关系 / Sampling relation of one frame to the native
 *        sensor.
 *
 * 帧覆盖原生窗口 `[roi_x, roi_x + 640·decimation) × [roi_y, roi_y + 512·decimation)`。
 * decimation 为 2 时相机按 Bayer 单元跳采：保留每个 4×4 原生块左上角的 2×2 单元。
 * The frame covers the native window starting at `(roi_x, roi_y)`. With decimation 2
 * the camera skips by Bayer cell and keeps the top-left 2×2 cell of every 4×4 native
 * block.
 */
struct FrameGeometry
{
  uint32_t roi_x;       ///< 原生窗口左上角 x / Native window origin x
  uint32_t roi_y;       ///< 原生窗口左上角 y / Native window origin y
  uint16_t decimation;  ///< 1（裁剪）或 2（跳采） / 1 (crop) or 2 (skip)
};

/// 两个几何是否相同 / Whether two geometries are equal.
constexpr bool operator==(const FrameGeometry& lhs, const FrameGeometry& rhs)
{
  return lhs.roi_x == rhs.roi_x && lhs.roi_y == rhs.roi_y &&
         lhs.decimation == rhs.decimation;
}

/**
 * @brief 采样相位：帧坐标 0 对应的原生坐标相对窗口起点的偏移。
 *        Sampling phase: native offset of frame coordinate 0 from the window origin.
 *
 * 跳采 2 时偶数列取原生 `4k`、奇数列取 `4k + 1`，连续坐标的平均映射为 `2x − 0.5`
 * （13T 实测，2026-10-05）。
 * With decimation 2, even columns sample native `4k` and odd ones `4k + 1`; the
 * average continuous mapping is `2x − 0.5` (measured on PI-HAILO-13T, 2026-10-05).
 */
constexpr double SamplingPhase(uint16_t decimation)
{
  return decimation == 2 ? -0.5 : 0.0;
}

/// 帧坐标映射到原生坐标 / Map frame coordinates to native coordinates.
constexpr Point2d FrameToNative(const FrameGeometry& geometry, Point2d frame)
{
  const double phase = SamplingPhase(geometry.decimation);
  return {geometry.roi_x + phase + geometry.decimation * frame.x,
          geometry.roi_y + phase + geometry.decimation * frame.y};
}

/// 原生坐标映射到帧坐标 / Map native coordinates to frame coordinates.
constexpr Point2d NativeToFrame(const FrameGeometry& geometry, Point2d native)
{
  const double phase = SamplingPhase(geometry.decimation);
  return {(native.x - geometry.roi_x - phase) / geometry.decimation,
          (native.y - geometry.roi_y - phase) / geometry.decimation};
}

/// 几何是否完整落在传感器内 / Whether the geometry lies inside the sensor.
constexpr bool GeometryInsideSensor(const FrameGeometry& geometry,
                                    const CameraCalibration& calibration)
{
  const uint64_t right =
      static_cast<uint64_t>(geometry.roi_x) + uint64_t{FRAME_WIDTH} * geometry.decimation;
  const uint64_t bottom = static_cast<uint64_t>(geometry.roi_y) +
                          uint64_t{FRAME_HEIGHT} * geometry.decimation;
  // 偏移为偶数才保持 RGGB 排列 / Even offsets keep the RGGB order.
  return (geometry.decimation == 1 || geometry.decimation == 2) &&
         geometry.roi_x % 2 == 0 && geometry.roi_y % 2 == 0 &&
         right <= calibration.native_width && bottom <= calibration.native_height;
}
}  // namespace CameraTypes
