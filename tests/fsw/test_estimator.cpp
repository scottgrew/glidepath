// Oracle guardrails for the navigation estimator and atmosphere helpers.
#include "fsw/atmosphere.hpp"
#include "fsw/estimator.hpp"
#include "fsw/ekf_layout.hpp"
#include "fsw/measurements.hpp"
#include "fsw/runway.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
namespace {

using glidepath::fsw::kStateDim;

using glidepath::fsw::AidingUse;
using glidepath::fsw::BaroSample;
using glidepath::fsw::EkfConfig;
using glidepath::fsw::Estimate;
using glidepath::fsw::EstimatorMemory;
using glidepath::fsw::GnssSample;
using glidepath::fsw::ImuSample;
using glidepath::fsw::InitEkf;
using glidepath::fsw::IsaAltitudeM;
using glidepath::fsw::IsaDensityKgM3;
using glidepath::fsw::IsaPressurePa;
using glidepath::fsw::kControlRateHz;
using glidepath::fsw::kDefaultSpecs;
using glidepath::fsw::kMaxPredictGapS;
using glidepath::fsw::kPaloAlto31;
using glidepath::fsw::kPosN;
using glidepath::fsw::kSeaLevelDensityKgM3;
using glidepath::fsw::kStateFields;
using glidepath::fsw::MagSample;
using glidepath::fsw::PitotSample;
using glidepath::fsw::SensorFrame;
using glidepath::fsw::StartEstimator;
using glidepath::fsw::UpdateEstimator;

EkfConfig TestConfig() {
  return {.specs = kDefaultSpecs,
          .terrain_elevation_m = kPaloAlto31.elevation_m};
}

SensorFrame FullFrame(const double time_s) {
  SensorFrame frame{};
  frame.time_s = time_s;
  frame.imu = {.valid = true,
               .gyro_rad_s = {0.0, 0.0, 0.0},
               .accel_m_s2 = {0.0, 0.0, -9.80665}};
  frame.gnss = {.valid = true,
                .latitude_rad = 0.7,
                .longitude_rad = -1.3,
                .north_m_s = 30.0,
                .east_m_s = 1.0,
                .down_m_s = -2.0};
  frame.baro = {.valid = true, .pressure_pa = IsaPressurePa(500.0)};
  frame.mag = {.valid = true, .field_nt = {20000.0, 4000.0, 35000.0}};
  frame.pitot = {.valid = true, .differential_pressure_pa = 551.25};
  frame.radar = {.valid = true, .height_m = 20.0};
  frame.main_gear_on_ground = false;
  frame.nose_gear_on_ground = false;
  return frame;
}

bool StateAllNan(const glidepath::fsw::VehicleState& state) {
  for (const glidepath::fsw::Field<glidepath::fsw::VehicleState>& field :
       kStateFields) {
    if (std::isfinite(state.*field.member)) {
      return false;
    }
  }
  return true;
}

double Covariance(const glidepath::fsw::EkfSnapshot& snapshot,
                  const std::ptrdiff_t row, const std::ptrdiff_t col) {
  return snapshot.covariance[static_cast<std::size_t>(row * kStateDim + col)];
}

}  // namespace

TEST_CASE("estimator: ISA pressure altitude and density match the oracle") {
  CHECK(IsaPressurePa(0.0) == Catch::Approx(101325.0).margin(0.1));
  CHECK(IsaPressurePa(1000.0) == Catch::Approx(89874.6).margin(0.5));
  CHECK(IsaPressurePa(11000.0) == Catch::Approx(22632.1).margin(0.5));
  CHECK(IsaDensityKgM3(0.0) == Catch::Approx(1.225).margin(1e-3));
  CHECK(IsaDensityKgM3(11000.0) == Catch::Approx(0.3639).margin(1e-3));

  const double altitude_m = 2345.0;
  const double pressure = IsaPressurePa(altitude_m);
  CHECK(IsaAltitudeM(pressure) == Catch::Approx(altitude_m).margin(1e-6));
}

TEST_CASE("estimator: pitot CAS passes through sqrt(2 dp / rho0)") {
  const EkfConfig config = TestConfig();
  EstimatorMemory memory = StartEstimator();
  SensorFrame frame = FullFrame(0.0);
  const Estimate result = UpdateEstimator(config, frame, &memory, AidingUse{});
  REQUIRE(result.valid);
  CHECK(result.state.calibrated_airspeed_m_s ==
        Catch::Approx(30.0).margin(1e-6));
  CHECK(frame.pitot.differential_pressure_pa ==
        Catch::Approx(551.25).margin(0));
}

TEST_CASE("estimator: InitEkf decision rows") {
  const EkfConfig config = TestConfig();
  SensorFrame frame = FullFrame(0.0);
  frame.imu.valid = false;
  CHECK(!InitEkf(config, frame).has_value());
  frame = FullFrame(0.0);
  CHECK(InitEkf(config, frame).has_value());
}

TEST_CASE("estimator: cycle before init returns invalid NaN state") {
  const EkfConfig config = TestConfig();
  EstimatorMemory memory = StartEstimator();
  SensorFrame frame = FullFrame(0.0);
  frame.gnss.valid = false;
  const Estimate result = UpdateEstimator(config, frame, &memory, AidingUse{});
  CHECK(!result.valid);
  CHECK(StateAllNan(result.state));
  CHECK(!result.snapshot.valid);
}

TEST_CASE(
    "estimator: GNSS absent 20 s keeps valid and P_posN grows monotonically") {
  const EkfConfig config = TestConfig();
  EstimatorMemory memory = StartEstimator();
  const double dt_s = 1.0 / kControlRateHz;
  SensorFrame frame = FullFrame(0.0);
  UpdateEstimator(config, frame, &memory, AidingUse{});
  REQUIRE(memory.ekf.has_value());

  double previous_p = 0.0;
  const int cycles = static_cast<int>(20.0 * kControlRateHz);
  for (int cycle = 0; cycle < cycles; ++cycle) {
    frame.time_s += dt_s;
    frame.gnss.valid = false;
    const Estimate result =
        UpdateEstimator(config, frame, &memory, AidingUse{});
    REQUIRE(result.valid);
    const double p_pos_n = Covariance(result.snapshot, kPosN, kPosN);
    CHECK(p_pos_n >= previous_p);
    previous_p = p_pos_n;
  }
}

TEST_CASE("estimator: IMU dropout invalidates then recovers") {
  const EkfConfig config = TestConfig();
  EstimatorMemory memory = StartEstimator();
  const double dt_s = 1.0 / kControlRateHz;
  SensorFrame frame = FullFrame(0.0);
  UpdateEstimator(config, frame, &memory, AidingUse{});
  frame.time_s += dt_s;
  frame.imu.valid = false;
  const Estimate dropped = UpdateEstimator(config, frame, &memory, AidingUse{});
  CHECK(!dropped.valid);
  CHECK(!memory.ekf.has_value());

  frame = FullFrame(frame.time_s + dt_s);
  const Estimate recovered =
      UpdateEstimator(config, frame, &memory, AidingUse{});
  CHECK(recovered.valid);
  CHECK(memory.ekf.has_value());
}

TEST_CASE("estimator: predict gap over 0.5 s resets the filter") {
  const EkfConfig config = TestConfig();
  EstimatorMemory memory = StartEstimator();
  SensorFrame frame = FullFrame(0.0);
  UpdateEstimator(config, frame, &memory, AidingUse{});
  REQUIRE(memory.ekf.has_value());

  frame.time_s += kMaxPredictGapS + 0.05;
  const Estimate result = UpdateEstimator(config, frame, &memory, AidingUse{});
  CHECK(!result.valid);
  CHECK(!memory.ekf.has_value());
}