#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "CameraBase.hpp"

namespace
{
void Expect(bool condition, const char* label)
{
  if (!condition)
  {
    std::cerr << label << '\n';
    std::exit(EXIT_FAILURE);
  }
}

std::array<double, 9> MakeCameraMatrix(double fx, double fy)
{
  return {fx, 0.0, 720.0, 0.0, fy, 540.0, 0.0, 0.0, 1.0};
}

void TestBothFocalAxesAreChecked()
{
  constexpr uint32_t width = 1440;
  constexpr uint32_t height = 1080;
  Expect(CameraBaseIntrinsicSanity::FocalReasonable(width, height,
                                                    MakeCameraMatrix(1000.0, 1000.0)),
         "ordinary focal lengths must pass");
  Expect(!CameraBaseIntrinsicSanity::FocalReasonable(width, height,
                                                     MakeCameraMatrix(144.0, 288.0)),
         "too-small fx must not be hidden by fy");
  Expect(!CameraBaseIntrinsicSanity::FocalReasonable(width, height,
                                                     MakeCameraMatrix(288.0, 144.0)),
         "too-small fy must not be hidden by fx");
  Expect(!CameraBaseIntrinsicSanity::FocalReasonable(width, height,
                                                     MakeCameraMatrix(15000.0, 1000.0)),
         "too-large fx must not pass");
  Expect(!CameraBaseIntrinsicSanity::FocalReasonable(width, height,
                                                     MakeCameraMatrix(1000.0, 15000.0)),
         "too-large fy must not pass");
}

void TestNonFiniteDistortionRemainsVisibleInReportMetrics()
{
  std::array<double, 14> distortion{};
  distortion[2] = std::numeric_limits<double>::quiet_NaN();
  distortion[3] = 0.5;
  const auto metrics = CameraBaseIntrinsicSanity::Evaluate(
      1440, 1080, MakeCameraMatrix(1000.0, 1000.0), distortion);
  Expect(!metrics.distortion_ok, "non-finite distortion must fail sanity");
  Expect(std::isnan(metrics.max_abs_distortion_value),
         "report metrics must retain the non-finite distortion signal");

  distortion = {};
  distortion[0] = -std::numeric_limits<double>::infinity();
  const auto infinite_metrics = CameraBaseIntrinsicSanity::Evaluate(
      1440, 1080, MakeCameraMatrix(1000.0, 1000.0), distortion);
  Expect(std::isinf(infinite_metrics.max_abs_distortion_value) &&
             infinite_metrics.max_abs_distortion_value > 0.0,
         "maximum absolute distortion must normalize negative infinity");
}
}  // namespace

int main()
{
  TestBothFocalAxesAreChecked();
  TestNonFiniteDistortionRemainsVisibleInReportMetrics();
  return EXIT_SUCCESS;
}
