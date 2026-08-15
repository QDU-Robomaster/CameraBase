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

constexpr CameraTypes::CameraCalibration MakeCalibrationWithDistortion(
    CameraTypes::DistortionModel model)
{
  auto calibration = MakeCalibration();
  calibration.distortion_model = model;
  for (std::size_t i = 0; i < calibration.distortion_coefficients.size(); ++i)
  {
    calibration.distortion_coefficients[i] = static_cast<double>(i + 1U) * 0.125;
  }
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

constexpr auto kNoDistortion = CameraTypes::BuildPnPDistCoeffs(
    MakeCalibrationWithDistortion(CameraTypes::DistortionModel::NONE));
static_assert(kNoDistortion.size == 0U);
static_assert(!kNoDistortion.uses_rational_polynomial_extension);
static_assert(!kNoDistortion.requires_undistort_first);

constexpr auto kPlumbBobDistortion = CameraTypes::BuildPnPDistCoeffs(
    MakeCalibrationWithDistortion(CameraTypes::DistortionModel::PLUMB_BOB));
static_assert(kPlumbBobDistortion.size == 5U);
static_assert(kPlumbBobDistortion.values[0] == 0.125);
static_assert(kPlumbBobDistortion.values[1] == 0.25);
static_assert(kPlumbBobDistortion.values[2] == 0.375);
static_assert(kPlumbBobDistortion.values[3] == 0.5);
static_assert(kPlumbBobDistortion.values[4] == 0.625);
static_assert(!kPlumbBobDistortion.uses_rational_polynomial_extension);
static_assert(!kPlumbBobDistortion.requires_undistort_first);

constexpr auto kRationalDistortion = CameraTypes::BuildPnPDistCoeffs(
    MakeCalibrationWithDistortion(CameraTypes::DistortionModel::RATIONAL_POLYNOMIAL));
static_assert(kRationalDistortion.size == 8U);
static_assert(kRationalDistortion.values[0] == 0.125);
static_assert(kRationalDistortion.values[1] == 0.25);
static_assert(kRationalDistortion.values[2] == 0.375);
static_assert(kRationalDistortion.values[3] == 0.5);
static_assert(kRationalDistortion.values[4] == 0.625);
static_assert(kRationalDistortion.values[5] == 0.75);
static_assert(kRationalDistortion.values[6] == 0.875);
static_assert(kRationalDistortion.values[7] == 1.0);
static_assert(kRationalDistortion.uses_rational_polynomial_extension);
static_assert(!kRationalDistortion.requires_undistort_first);

constexpr CameraTypes::FrameGeometry MakeWideGeometry()
{
  return {720, 540, 2160, 0, 0, 2, 2, CameraTypes::FRAME_GEOMETRY_NONE, 0, 0.0F, 0.0F};
}

constexpr CameraTypes::FrameGeometry kLegacyWideGeometry{
    17, 720, 540, 2160, 0, 0, 2, 2, CameraTypes::FRAME_GEOMETRY_NONE, 0, 0.0F, 0.0F};

constexpr bool ExactGeometryFieldsEqual(const CameraTypes::FrameGeometry& lhs,
                                        const CameraTypes::FrameGeometry& rhs)
{
  return lhs.width == rhs.width && lhs.height == rhs.height && lhs.step == rhs.step &&
         lhs.roi_offset_x_native == rhs.roi_offset_x_native &&
         lhs.roi_offset_y_native == rhs.roi_offset_y_native &&
         lhs.decimation_x == rhs.decimation_x && lhs.decimation_y == rhs.decimation_y &&
         lhs.flags == rhs.flags && lhs.reserved == rhs.reserved &&
         lhs.sample_phase_x_native == rhs.sample_phase_x_native &&
         lhs.sample_phase_y_native == rhs.sample_phase_y_native;
}

template <typename T>
concept HasEpochMember = requires(T value) { value.epoch; };

static_assert(!HasEpochMember<CameraTypes::FrameGeometry>);
static_assert(sizeof(CameraTypes::FrameGeometry) == 36U);
static_assert(ExactGeometryFieldsEqual(MakeWideGeometry(), kLegacyWideGeometry));
static_assert(CameraTypes::SameFrameGeometry(MakeWideGeometry(), kLegacyWideGeometry));

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

void TestPnPDistortionContract()
{
  const auto expect_zero_values =
      [](const CameraTypes::PnPDistCoeffs& distortion, const char* label)
  {
    for (double value : distortion.values)
    {
      ExpectNear(value, 0.0, label);
    }
  };

  const auto none = CameraTypes::BuildPnPDistCoeffs(
      MakeCalibrationWithDistortion(CameraTypes::DistortionModel::NONE));
  Expect(none.size == 0U, "none must expose zero coefficients");
  Expect(!none.uses_rational_polynomial_extension,
         "none must not use the rational extension");
  Expect(!none.requires_undistort_first,
         "none must be accepted directly by the pinhole PnP path");
  expect_zero_values(none, "none must not leak stored coefficients");

  const auto plumb_bob = CameraTypes::BuildPnPDistCoeffs(
      MakeCalibrationWithDistortion(CameraTypes::DistortionModel::PLUMB_BOB));
  Expect(plumb_bob.size == 5U, "plumb_bob must expose five coefficients");
  for (std::size_t i = 0; i < plumb_bob.values.size(); ++i)
  {
    const double expected =
        i < plumb_bob.size ? static_cast<double>(i + 1U) * 0.125 : 0.0;
    ExpectNear(plumb_bob.values[i], expected,
               "plumb_bob coefficient order or unused storage");
  }
  Expect(!plumb_bob.uses_rational_polynomial_extension,
         "plumb_bob must not use the rational extension");
  Expect(!plumb_bob.requires_undistort_first,
         "plumb_bob must be accepted directly by the pinhole PnP path");

  const auto rational = CameraTypes::BuildPnPDistCoeffs(
      MakeCalibrationWithDistortion(CameraTypes::DistortionModel::RATIONAL_POLYNOMIAL));
  Expect(rational.size == 8U, "rational must expose eight coefficients");
  for (std::size_t i = 0; i < rational.values.size(); ++i)
  {
    ExpectNear(rational.values[i], static_cast<double>(i + 1U) * 0.125,
               "rational coefficient order");
  }
  Expect(rational.uses_rational_polynomial_extension,
         "rational must advertise its eight-coefficient extension");
  Expect(!rational.requires_undistort_first,
         "rational must be accepted directly by the pinhole PnP path");

  constexpr std::array unsupported_models{
      CameraTypes::DistortionModel::EQUIDISTANT,
      CameraTypes::DistortionModel::FOV,
      CameraTypes::DistortionModel::OMNI,
      CameraTypes::DistortionModel::EXTENDED_UNIFIED,
      CameraTypes::DistortionModel::DOUBLE_SPHERE,
      CameraTypes::DistortionModel::THIN_PRISM,
      CameraTypes::DistortionModel::UNKNOWN,
      static_cast<CameraTypes::DistortionModel>(0xFFU),
  };
  for (const auto model : unsupported_models)
  {
    const auto unsupported =
        CameraTypes::BuildPnPDistCoeffs(MakeCalibrationWithDistortion(model));
    Expect(unsupported.size == 0U, "unsupported model must expose zero coefficients");
    Expect(!unsupported.uses_rational_polynomial_extension,
           "unsupported model must not advertise the rational extension");
    Expect(unsupported.requires_undistort_first,
           "unsupported model must fail closed and require undistortion");
    expect_zero_values(unsupported, "unsupported model must not leak coefficients");
  }
}
}  // namespace

int main()
{
  TestWideMapping();
  TestCenteredRoiAndReverse();
  TestRoundTrip();
  TestValidationAndIdentity();
  TestProfileTypes();
  TestPnPDistortionContract();
  return EXIT_SUCCESS;
}
