#include "fsw/estimator.hpp"

#include "fsw/atmosphere.hpp"
#include "fsw/ekf.hpp"
#include "fsw/ekf_layout.hpp"
#include "fsw/measurements.hpp"
#include "fsw/types.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>

namespace glidepath::fsw {
namespace {

VehicleState InvalidState() noexcept {
  VehicleState state{};
  for (const Field<VehicleState>& field : kStateFields) {
    state.*field.member = std::numeric_limits<double>::quiet_NaN();
  }
  return state;
}

void FillSnapshot(const Ekf& ekf, const Innovations& innovations,
                  EkfSnapshot& snapshot) noexcept {
  snapshot.valid = true;
  snapshot.innovations = innovations;
  const Eigen::Matrix<double, kStateDim, kStateDim, Eigen::RowMajor> row_major =
      ekf.covariance;
  std::copy_n(row_major.data(), kCovarianceSize, snapshot.covariance.begin());
}

struct PublishedStateInputs {
  double pitot_cas_m_s;
  double terrain_elevation_m;
};

VehicleState StateFromEkf(const Ekf& ekf, const ImuSample& imu,
                          const PublishedStateInputs& inputs,
                          const SensorFrame& frame) noexcept {
  const Eigen::Matrix3d rotation = ekf.attitude.normalized().toRotationMatrix();
  VehicleState state{};
  state.time_s = ekf.time_s;
  state.latitude_rad = ekf.latitude_rad;
  state.longitude_rad = ekf.longitude_rad;
  state.altitude_m = ekf.altitude_m;
  state.roll_rad = std::atan2(rotation(2, 1), rotation(2, 2));
  state.pitch_rad = -std::asin(rotation(2, 0));
  state.heading_rad = std::atan2(rotation(1, 0), rotation(0, 0));
  if (state.heading_rad < 0.0) {
    state.heading_rad += 2.0 * std::numbers::pi;
  }
  const Eigen::Vector3d gyro_body(imu.gyro_rad_s[0], imu.gyro_rad_s[1],
                                  imu.gyro_rad_s[2]);
  const Eigen::Vector3d body_rates = gyro_body - ekf.gyro_bias_rad_s;
  state.roll_rate_rad_s = body_rates.x();
  state.pitch_rate_rad_s = body_rates.y();
  state.yaw_rate_rad_s = body_rates.z();
  state.calibrated_airspeed_m_s = inputs.pitot_cas_m_s;
  state.climb_rate_m_s = -ekf.velocity_ned_m_s.z();
  state.north_velocity_m_s = ekf.velocity_ned_m_s.x();
  state.east_velocity_m_s = ekf.velocity_ned_m_s.y();
  state.height_above_ground_m = ekf.altitude_m - inputs.terrain_elevation_m;
  state.main_gear_on_ground = frame.main_gear_on_ground;
  state.nose_gear_on_ground = frame.nose_gear_on_ground;
  return state;
}

}  // namespace

EstimatorMemory StartEstimator() noexcept {
  return {.ekf = std::nullopt,
          .pitot_cas_m_s = std::numeric_limits<double>::quiet_NaN()};
}

Estimate UpdateEstimator(const EkfConfig& config, const SensorFrame& frame,
                         EstimatorMemory* memory,
                         const AidingUse& aiding) noexcept {
  Estimate result{.state = InvalidState(), .valid = false, .snapshot = {}};
  result.snapshot.valid = false;
  if (memory == nullptr) {
    return result;
  }

  bool just_initialized = false;
  if (!memory->ekf.has_value()) {
    memory->ekf = InitEkf(config, frame);
    if (!memory->ekf.has_value() || !IsUsable(frame.imu)) {
      return result;
    }
    just_initialized = true;
  } else {
    const double dt_s = frame.time_s - memory->ekf->time_s;
    if (dt_s <= 0.0 || dt_s > kMaxPredictGapS || !IsUsable(frame.imu)) {
      memory->ekf = std::nullopt;
      return result;
    }
    PredictEkf(config, frame.imu, dt_s, *memory->ekf);
  }

  Innovations innovations{};
  if (!just_initialized) {
    innovations = AidEkf(config, frame, aiding, *memory->ekf);
  }

  if (IsUsable(frame.pitot)) {
    const double dynamic = std::max(frame.pitot.differential_pressure_pa, 0.0);
    memory->pitot_cas_m_s = std::sqrt(2.0 * dynamic / kSeaLevelDensityKgM3);
  }

  const bool pitot_finite = std::isfinite(memory->pitot_cas_m_s);
  if (!IsUsable(frame.imu) || !pitot_finite) {
    memory->ekf = std::nullopt;
    return result;
  }

  return EstimateFromFilter(config, *memory->ekf, frame, memory->pitot_cas_m_s,
                            innovations);
}

Estimate EstimateFromFilter(const EkfConfig& config, const Ekf& ekf,
                            const SensorFrame& frame,
                            const double pitot_cas_m_s,
                            const Innovations& innovations) noexcept {
  Estimate result{.state = InvalidState(), .valid = false, .snapshot = {}};
  result.snapshot.valid = false;
  if (!IsUsable(frame.imu) || !std::isfinite(pitot_cas_m_s)) {
    return result;
  }
  const PublishedStateInputs inputs{
      .pitot_cas_m_s = pitot_cas_m_s,
      .terrain_elevation_m = config.terrain_elevation_m};
  result.state = StateFromEkf(ekf, frame.imu, inputs, frame);
  FillSnapshot(ekf, innovations, result.snapshot);
  result.valid = true;
  return result;
}

}  // namespace glidepath::fsw