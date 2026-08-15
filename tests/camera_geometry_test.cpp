#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <type_traits>

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

constexpr bool CalibrationDefaultsToZero()
{
  CameraTypes::CameraCalibration calibration;
  for (double value : calibration.camera_matrix)
  {
    if (value != 0.0)
    {
      return false;
    }
  }
  for (double value : calibration.distortion_coefficients)
  {
    if (value != 0.0)
    {
      return false;
    }
  }
  for (double value : calibration.rectification_matrix)
  {
    if (value != 0.0)
    {
      return false;
    }
  }
  for (double value : calibration.projection_matrix)
  {
    if (value != 0.0)
    {
      return false;
    }
  }
  return true;
}

static_assert(CalibrationDefaultsToZero());
static_assert(std::is_aggregate_v<CameraTypes::CameraCalibration>);
static_assert(std::is_standard_layout_v<CameraTypes::CameraCalibration>);
static_assert(std::is_trivially_copyable_v<CameraTypes::CameraCalibration>);

constexpr CameraTypes::FrameGeometry MakeWideGeometry()
{
  return {
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

template <typename T>
concept HasEpochMember = requires(T value) { value.epoch; };

static_assert(!HasEpochMember<CameraTypes::FrameGeometry>);
static_assert(sizeof(CameraTypes::FrameGeometry) == 36U);

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
  if (!std::isfinite(actual) || !std::isfinite(expected) ||
      std::abs(actual - expected) > 1e-9)
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

  auto changed_geometry = wide;
  changed_geometry.roi_offset_x_native = 1;
  Expect(CameraTypes::ValidateFrameGeometry(kLayout, calibration, changed_geometry),
         "changed geometry should remain individually valid");
  Expect(!CameraTypes::SameFrameGeometry(wide, changed_geometry),
         "a real geometry change must change identity");

  auto invalid = wide;
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

  auto changed_reserved = wide;
  changed_reserved.reserved = 1;
  Expect(!CameraTypes::ValidateFrameGeometry(kLayout, calibration, changed_reserved),
         "non-zero reserved storage must be rejected");
  Expect(CameraTypes::SameFrameGeometry(wide, changed_reserved),
         "reserved storage must not define geometry identity");

  constexpr CameraTypes::FrameLayout even_yuv_layout{4, 2, 8,
                                                     CameraTypes::Encoding::YUV422};
  constexpr CameraTypes::FrameLayout odd_yuv_layout{3, 2, 6,
                                                    CameraTypes::Encoding::YUV422};
  static_assert(CameraTypes::ValidateFrameLayout(even_yuv_layout));
  static_assert(!CameraTypes::ValidateFrameLayout(odd_yuv_layout));
}

void TestProfileTypes()
{
  auto narrow_geometry = MakeWideGeometry();
  narrow_geometry.roi_offset_x_native = 360;
  narrow_geometry.roi_offset_y_native = 270;
  narrow_geometry.decimation_x = 1;
  narrow_geometry.decimation_y = 1;

  const std::array<CameraTypes::CameraProfile, 2U> profiles{{
      {.id = CameraTypes::ProfileId::WIDE,
       .geometry = MakeWideGeometry(),
       .trigger_period_us = 10000U},
      {.id = CameraTypes::ProfileId::NARROW,
       .geometry = narrow_geometry,
       .trigger_period_us = 5000U},
  }};
  Expect(profiles[0].id == CameraTypes::ProfileId::WIDE,
         "wide profile id must be preserved");
  Expect(profiles[1].id == CameraTypes::ProfileId::NARROW,
         "narrow profile id must be preserved");
  Expect(profiles[0].trigger_period_us != 0U && profiles[1].trigger_period_us != 0U,
         "profile trigger periods must be non-zero");
  Expect(!CameraTypes::SameFrameGeometry(profiles[0].geometry, profiles[1].geometry),
         "profile geometry must be carried by value");

  const CameraTypes::AppliedProfile applied{.id = profiles[1].id,
                                            .geometry = profiles[1].geometry};
  Expect(applied.id == CameraTypes::ProfileId::NARROW,
         "applied profile id must be preserved");
  Expect(CameraTypes::SameFrameGeometry(applied.geometry, profiles[1].geometry),
         "applied profile geometry must be preserved");
}
}  // namespace

int main()
{
  TestWideMapping();
  TestCenteredRoiAndReverse();
  TestRoundTrip();
  TestValidationAndIdentity();
  TestProfileTypes();
  return EXIT_SUCCESS;
}
