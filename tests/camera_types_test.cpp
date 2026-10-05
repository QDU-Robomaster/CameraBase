#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "CameraCalibrationCheck.hpp"
#include "CameraTypes.hpp"

namespace
{
void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

bool Near(double a, double b) { return std::fabs(a - b) < 1e-9; }

constexpr CameraTypes::CameraCalibration CALIBRATION{
    1440,
    1080,
    2328.69,
    2328.67,
    733.36,
    540.62,
    {-0.0918, 0.4640, 0.0026, 0.0010, -0.4751}};

void TestMapping()
{
  using namespace CameraTypes;
  constexpr FrameGeometry wide{80, 24, 2};
  // 13T 实测：偶数列取原生 80 + 4k，奇数列取 80 + 4k + 1，平均 2x + 79.5。
  Expect(Near(FrameToNative(wide, {0.0, 0.0}).x, 79.5), "wide x0");
  Expect(Near(FrameToNative(wide, {0.0, 0.0}).y, 23.5), "wide y0");
  Expect(Near(FrameToNative(wide, {10.0, 3.0}).x, 99.5), "wide x10");
  constexpr FrameGeometry narrow{400, 284, 1};
  Expect(Near(FrameToNative(narrow, {5.0, 7.0}).x, 405.0), "narrow x");
  Expect(Near(FrameToNative(narrow, {5.0, 7.0}).y, 291.0), "narrow y");
  for (const FrameGeometry& g : {wide, narrow})
  {
    const Point2d back = NativeToFrame(g, FrameToNative(g, {123.25, 45.75}));
    Expect(Near(back.x, 123.25) && Near(back.y, 45.75), "round trip");
  }
}

void TestInsideSensor()
{
  using namespace CameraTypes;
  Expect(GeometryInsideSensor({80, 24, 2}, CALIBRATION), "wide inside");
  Expect(GeometryInsideSensor({800, 568, 1}, CALIBRATION), "narrow corner inside");
  Expect(!GeometryInsideSensor({804, 568, 1}, CALIBRATION), "narrow past right edge");
  Expect(!GeometryInsideSensor({401, 284, 1}, CALIBRATION), "odd offset breaks Bayer");
  Expect(!GeometryInsideSensor({80, 24, 3}, CALIBRATION), "unsupported decimation");
}

void TestCalibrationCheck()
{
  using namespace CameraTypes;
  Expect(CalibrationReasonable(CALIBRATION), "real calibration");
  CameraCalibration bad = CALIBRATION;
  bad.fx = -1.0;
  Expect(!CalibrationReasonable(bad), "negative focal");
  bad = CALIBRATION;
  bad.cx = 2000.0;
  Expect(!CalibrationReasonable(bad), "principal point outside");
  bad = CALIBRATION;
  bad.distortion[0] = NAN;
  Expect(!CalibrationReasonable(bad), "NaN distortion");
}
}  // namespace

int main()
{
  TestMapping();
  TestInsideSensor();
  TestCalibrationCheck();
  std::puts("camera_types_test passed");
  return 0;
}
