#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "CameraBase.hpp"

namespace
{
constexpr CameraTypes::FrameLayout kLayout{720, 540, 2160, CameraTypes::Encoding::BGR8};

constexpr CameraTypes::CameraCalibration MakeCalibration()
{
  CameraTypes::CameraCalibration calibration{};
  calibration.native_width = 1440;
  calibration.native_height = 1080;
  return calibration;
}

constexpr CameraTypes::FrameGeometry MakeWideGeometry()
{
  return {
      .epoch = 1,
      .width = 720,
      .height = 540,
      .step = 2160,
      .roi_offset_x_native = 0,
      .roi_offset_y_native = 0,
      .decimation_x = 2,
      .decimation_y = 2,
      .flags = CameraTypes::FRAME_GEOMETRY_NONE,
      .reserved = 0,
      .sample_phase_x_native = 0.0F,
      .sample_phase_y_native = 0.0F,
  };
}

void Expect(bool condition, const char* label)
{
  if (!condition)
  {
    std::cerr << label << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void ExpectNear(double actual, double expected, const char* label)
{
  if (std::abs(actual - expected) > 1e-9)
  {
    std::cerr << label << ": actual=" << actual << " expected=" << expected << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void TestWideMapping()
{
  constexpr auto geometry = MakeWideGeometry();
  const auto first = CameraTypes::FrameToNative(geometry, 0.0, 0.0);
  const auto last = CameraTypes::FrameToNative(geometry, 719.0, 539.0);
  ExpectNear(first[0], 0.0, "wide first x");
  ExpectNear(first[1], 0.0, "wide first y");
  ExpectNear(last[0], 1438.0, "wide last x");
  ExpectNear(last[1], 1078.0, "wide last y");
}

void TestCenteredRoiAndReverse()
{
  auto roi = MakeWideGeometry();
  roi.epoch = 2;
  roi.roi_offset_x_native = 360;
  roi.roi_offset_y_native = 270;
  roi.decimation_x = 1;
  roi.decimation_y = 1;
  const auto roi_first = CameraTypes::FrameToNative(roi, 0.0, 0.0);
  const auto roi_last = CameraTypes::FrameToNative(roi, 719.0, 539.0);
  ExpectNear(roi_first[0], 360.0, "roi first x");
  ExpectNear(roi_first[1], 270.0, "roi first y");
  ExpectNear(roi_last[0], 1079.0, "roi last x");
  ExpectNear(roi_last[1], 809.0, "roi last y");

  auto reversed = MakeWideGeometry();
  reversed.flags =
      CameraTypes::FRAME_GEOMETRY_REVERSE_X | CameraTypes::FRAME_GEOMETRY_REVERSE_Y;
  const auto reversed_first = CameraTypes::FrameToNative(reversed, 0.0, 0.0);
  ExpectNear(reversed_first[0], 1438.0, "reversed first x");
  ExpectNear(reversed_first[1], 1078.0, "reversed first y");
}

void TestRoundTrip()
{
  auto geometry = MakeWideGeometry();
  geometry.sample_phase_x_native = 0.25F;
  geometry.sample_phase_y_native = 0.5F;
  const auto native = CameraTypes::FrameToNative(geometry, 123.25, 456.5);
  const auto frame = CameraTypes::NativeToFrame(geometry, native[0], native[1]);
  ExpectNear(frame[0], 123.25, "round-trip x");
  ExpectNear(frame[1], 456.5, "round-trip y");
}

void TestValidationAndIdentity()
{
  constexpr auto calibration = MakeCalibration();
  constexpr auto wide = MakeWideGeometry();
  static_assert(CameraTypes::ValidateFrameGeometry(kLayout, calibration, wide));
  static_assert(CameraTypes::SameFrameGeometry(wide, wide));

  auto changed_same_epoch = wide;
  changed_same_epoch.roi_offset_x_native = 1;
  Expect(CameraTypes::ValidateFrameGeometry(kLayout, calibration, changed_same_epoch),
         "changed same-epoch geometry should remain individually valid");
  Expect(!CameraTypes::SameFrameGeometry(wide, changed_same_epoch),
         "same epoch must not hide a geometry change");

  auto changed_epoch = wide;
  changed_epoch.epoch = 2;
  Expect(!CameraTypes::SameFrameGeometry(wide, changed_epoch),
         "changed epoch must not match the locked geometry");

  auto invalid = wide;
  invalid.epoch = 0;
  Expect(!CameraTypes::ValidateFrameGeometry(kLayout, calibration, invalid),
         "zero epoch must be rejected");
  invalid = wide;
  invalid.decimation_x = 0;
  Expect(!CameraTypes::ValidateFrameGeometry(kLayout, calibration, invalid),
         "zero decimation must be rejected");
  invalid = wide;
  invalid.roi_offset_x_native = 2;
  Expect(!CameraTypes::ValidateFrameGeometry(kLayout, calibration, invalid),
         "native overflow must be rejected");
  invalid = wide;
  invalid.sample_phase_y_native = std::numeric_limits<float>::quiet_NaN();
  Expect(!CameraTypes::ValidateFrameGeometry(kLayout, calibration, invalid),
         "non-finite phase must be rejected");
}
}  // namespace

int main()
{
  TestWideMapping();
  TestCenteredRoiAndReverse();
  TestRoundTrip();
  TestValidationAndIdentity();
  return EXIT_SUCCESS;
}
