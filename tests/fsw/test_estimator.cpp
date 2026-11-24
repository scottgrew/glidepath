// Guardrails for the navigation estimator and atmosphere helpers.
#include "fsw/atmosphere.hpp"
#include "fsw/estimator.hpp"
#include "fsw/measurements.hpp"
#include "fsw/runway.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>

namespace {

using glidepath::fsw::Estimate;
using glidepath::fsw::EstimatorConfig;
using glidepath::fsw::EstimatorMemory;
using glidepath::fsw::IsaAltitudeM;
using glidepath::fsw::IsaDensityKgM3;
using glidepath::fsw::IsaPressurePa;
using glidepath::fsw::kControlRateHz;
using glidepath::fsw::kDefaultSpecs;
using glidepath::fsw::kMaxPredictGapS;
using glidepath::fsw::kNavStateDim;
using glidepath::fsw::kPaloAlto31;
using glidepath::fsw::kStateFields;
using glidepath::fsw::SensorFrame;
using glidepath::fsw::StartEstimator;
using glidepath::fsw::UpdateEstimator;

EstimatorConfig TestConfig() {
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

double Covariance(const Estimate& estimate, const std::size_t row,
                  const std::size_t col) {
  return estimate.nav.covariance.at(row * kNavStateDim + col);
}

// Runs `cycles` cycles of FullFrame from t = 0 with the radar height given.
Estimate RunWithRadar(const bool radar_valid, const double height_m,
                      const int cycles) {
  const EstimatorConfig config = TestConfig();
  EstimatorMemory memory = StartEstimator();
  Estimate last{};
  for (int cycle = 0; cycle < cycles; ++cycle) {
    SensorFrame frame = FullFrame(cycle / kControlRateHz);
    frame.gnss.valid = cycle == 0;
    frame.baro.pressure_pa = IsaPressurePa(kPaloAlto31.elevation_m + 25.0);
    frame.radar = {.valid = radar_valid, .height_m = height_m};
    last = UpdateEstimator(config, frame, &memory);
  }
  return last;
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

TEST_CASE("estimator: the first full frame starts both filters and publishes") {
  const EstimatorConfig config = TestConfig();
  EstimatorMemory memory = StartEstimator();
  const SensorFrame frame = FullFrame(0.0);
  const Estimate result = UpdateEstimator(config, frame, &memory);
  REQUIRE(result.valid);
  CHECK(result.nav.valid);
  CHECK(memory.attitude.has_value());
  CHECK(memory.nav.has_value());
  CHECK(result.state.calibrated_airspeed_m_s ==
        Catch::Approx(30.0).margin(1e-6));
  CHECK(result.state.altitude_m == Catch::Approx(500.0).margin(1e-6));
  CHECK(result.state.height_above_ground_m ==
        Catch::Approx(500.0 - kPaloAlto31.elevation_m).margin(1e-6));
  CHECK(result.state.latitude_rad == Catch::Approx(0.7).margin(1e-12));
  CHECK(result.state.longitude_rad == Catch::Approx(-1.3).margin(1e-12));
  CHECK(result.state.north_velocity_m_s == Catch::Approx(30.0).margin(1e-9));
  CHECK(result.state.climb_rate_m_s == Catch::Approx(2.0).margin(1e-9));
  CHECK(std::abs(result.state.roll_rad) < 1e-9);
  CHECK(std::abs(result.state.pitch_rad) < 1e-9);
  CHECK(result.state.heading_rad >= 0.0);
  CHECK(result.state.heading_rad < 2.0 * 3.141592653589793);
}

TEST_CASE("estimator: init waits for IMU, magnetometer, GNSS and baro") {
  const EstimatorConfig config = TestConfig();
  for (int missing = 0; missing < 4; ++missing) {
    EstimatorMemory memory = StartEstimator();
    SensorFrame frame = FullFrame(0.0);
    if (missing == 0) frame.imu.valid = false;
    if (missing == 1) frame.mag.valid = false;
    if (missing == 2) frame.gnss.valid = false;
    if (missing == 3) frame.baro.valid = false;
    const Estimate result = UpdateEstimator(config, frame, &memory);
    INFO("missing sensor " << missing);
    CHECK(!result.valid);
    CHECK(!result.nav.valid);
    CHECK(StateAllNan(result.state));
    CHECK(!memory.attitude.has_value());
    CHECK(!memory.nav.has_value());
  }
}

TEST_CASE("estimator: a null memory gives an invalid NaN state") {
  const Estimate result =
      UpdateEstimator(TestConfig(), FullFrame(0.0), nullptr);
  CHECK(!result.valid);
  CHECK(StateAllNan(result.state));
}

TEST_CASE("estimator: without a pitot reading the estimate is invalid") {
  const EstimatorConfig config = TestConfig();
  EstimatorMemory memory = StartEstimator();
  SensorFrame frame = FullFrame(0.0);
  frame.pitot.valid = false;
  const Estimate result = UpdateEstimator(config, frame, &memory);
  CHECK(!result.valid);
  CHECK(StateAllNan(result.state));
  CHECK(memory.nav.has_value());
  frame = FullFrame(1.0 / kControlRateHz);
  CHECK(UpdateEstimator(config, frame, &memory).valid);
}

TEST_CASE("estimator: GNSS absent 20 s keeps valid and P_posN grows") {
  const EstimatorConfig config = TestConfig();
  EstimatorMemory memory = StartEstimator();
  const double dt_s = 1.0 / kControlRateHz;
  SensorFrame frame = FullFrame(0.0);
  UpdateEstimator(config, frame, &memory);
  REQUIRE(memory.nav.has_value());

  double previous_p = 0.0;
  const int cycles = static_cast<int>(20.0 * kControlRateHz);
  for (int cycle = 0; cycle < cycles; ++cycle) {
    frame.time_s += dt_s;
    frame.gnss.valid = false;
    const Estimate result = UpdateEstimator(config, frame, &memory);
    REQUIRE(result.valid);
    const double p_pos_n = Covariance(result, 0, 0);
    CHECK(p_pos_n > previous_p);
    previous_p = p_pos_n;
  }
}

TEST_CASE("estimator: IMU dropout invalidates then recovers") {
  const EstimatorConfig config = TestConfig();
  EstimatorMemory memory = StartEstimator();
  const double dt_s = 1.0 / kControlRateHz;
  SensorFrame frame = FullFrame(0.0);
  UpdateEstimator(config, frame, &memory);
  frame.time_s += dt_s;
  frame.imu.valid = false;
  const Estimate dropped = UpdateEstimator(config, frame, &memory);
  CHECK(!dropped.valid);
  CHECK(!memory.attitude.has_value());
  CHECK(!memory.nav.has_value());

  frame = FullFrame(frame.time_s + dt_s);
  const Estimate recovered = UpdateEstimator(config, frame, &memory);
  CHECK(recovered.valid);
  CHECK(memory.attitude.has_value());
  CHECK(memory.nav.has_value());
}

TEST_CASE("estimator: a gap over 0.5 s or a repeated time resets the filters") {
  const EstimatorConfig config = TestConfig();
  for (const double step_s : {kMaxPredictGapS + 0.05, 0.0}) {
    EstimatorMemory memory = StartEstimator();
    SensorFrame frame = FullFrame(0.0);
    UpdateEstimator(config, frame, &memory);
    REQUIRE(memory.nav.has_value());
    frame.time_s += step_s;
    const Estimate result = UpdateEstimator(config, frame, &memory);
    INFO("step " << step_s);
    CHECK(!result.valid);
    CHECK(!memory.attitude.has_value());
    CHECK(!memory.nav.has_value());
  }
}

TEST_CASE("estimator: radar heights outside 0.5 to 50 m are not used") {
  constexpr int kCycles = 200;
  const Estimate no_radar = RunWithRadar(false, 0.0, kCycles);
  const Estimate too_high = RunWithRadar(true, 60.0, kCycles);
  const Estimate too_low = RunWithRadar(true, 0.2, kCycles);
  const Estimate in_band = RunWithRadar(true, 20.0, kCycles);
  CHECK(too_high.state.altitude_m == no_radar.state.altitude_m);
  CHECK(too_low.state.altitude_m == no_radar.state.altitude_m);
  CHECK(no_radar.state.height_above_ground_m ==
        Catch::Approx(25.0).margin(0.5));
  CHECK(in_band.state.height_above_ground_m == Catch::Approx(20.0).margin(0.1));
}
