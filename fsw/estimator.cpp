// Per-cycle navigation estimate: the attitude filter and the nav filter start
// together on the first complete frame, then step on every cycle.
#include "fsw/estimator.hpp"

#include "fsw/atmosphere.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/attitude.hpp"
#include "fsw/measurements.hpp"
#include "fsw/nav_filter.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/types.hpp"
#include "fsw/units.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>

namespace glidepath::fsw {
namespace {

// Radar heights below this use the tighter radar sigma.
constexpr double kRadarLowHeightM = 1.0;

struct Euler {
  double roll_rad;
  double pitch_rad;
  double heading_rad;
};

VehicleState InvalidState() noexcept {
  VehicleState state{};
  for (const Field<VehicleState>& field : kStateFields) {
    state.*field.member = std::numeric_limits<double>::quiet_NaN();
  }
  return state;
}

Estimate InvalidResult() noexcept {
  return {.state = InvalidState(),
          .valid = false,
          .nav = {.valid = false, .covariance = {}}};
}

Eigen::Vector3d ToVector(const Vec3& vector) noexcept {
  return {vector[0], vector[1], vector[2]};
}

void ResetFilters(EstimatorMemory& memory) noexcept {
  memory.attitude = std::nullopt;
  memory.nav = std::nullopt;
  memory.gnss_accel = {};
}

// Differences successive GNSS velocities and low-passes the result.
void UpdateGnssAcceleration(const GnssSample& gnss, const double time_s,
                            GnssAcceleration& accel) noexcept {
  if (!IsUsable(gnss)) {
    return;
  }
  const Eigen::Vector3d velocity(gnss.north_m_s, gnss.east_m_s, gnss.down_m_s);
  const double dt_s = time_s - accel.time_s;
  if (accel.has_previous && dt_s > 0.0 && dt_s <= kGnssAccelMaxGapS) {
    const Eigen::Vector3d raw = (velocity - accel.velocity_ned_m_s) / dt_s;
    const double blend = std::min(1.0, dt_s / kGnssAccelTauS);
    accel.accel_ned_m_s2 += blend * (raw - accel.accel_ned_m_s2);
  } else {
    accel.accel_ned_m_s2 = Eigen::Vector3d::Zero();
  }
  accel.has_previous = true;
  accel.time_s = time_s;
  accel.velocity_ned_m_s = velocity;
}

bool AllInitInputsUsable(const SensorFrame& frame) noexcept {
  return IsUsable(frame.imu) && IsUsable(frame.mag) && IsUsable(frame.gnss) &&
         IsUsable(frame.baro);
}

// Starts both filters from this frame. False when they cannot both start; the
// filters are then left reset.
bool TryInit(const EstimatorConfig& config, const SensorFrame& frame,
             EstimatorMemory& memory) noexcept {
  if (!AllInitInputsUsable(frame)) {
    return false;
  }
  memory.attitude =
      InitAttitude(frame.imu, frame.mag, config.specs.magnetic_field_ned_nt);
  memory.nav =
      InitNav(config.specs, frame, IsaAltitudeM(frame.baro.pressure_pa));
  if (!memory.attitude || !memory.nav) {
    ResetFilters(memory);
    return false;
  }
  memory.gnss_accel = {};
  UpdateGnssAcceleration(frame.gnss, frame.time_s, memory.gnss_accel);
  return true;
}

void CorrectBaro(const BaroSample& baro, NavFilter& nav) noexcept {
  if (!IsUsable(baro)) {
    return;
  }
  const double altitude_m = IsaAltitudeM(baro.pressure_pa);
  CorrectAltitude(altitude_m, BaroAltitudeSigmaM(altitude_m), nav);
}

void CorrectRadar(const EstimatorConfig& config, const RadarSample& radar,
                  NavFilter& nav) noexcept {
  const SensorSpecs& specs = config.specs;
  if (!IsUsable(radar) || radar.height_m < specs.radar_min_valid_height_m ||
      radar.height_m > specs.radar_max_valid_height_m) {
    return;
  }
  const double sigma_m = radar.height_m < kRadarLowHeightM
                             ? specs.radar_sigma_below_1m_m
                             : specs.radar_sigma_m;
  CorrectAltitude(config.terrain_elevation_m + radar.height_m, sigma_m, nav);
}

// Attitude step, then the nav prediction with the NED acceleration.
void PredictStep(const EstimatorConfig& config, const SensorFrame& frame,
                 const double dt_s, GnssAcceleration& gnss_accel,
                 AttitudeFilter& attitude, NavFilter& nav) noexcept {
  UpdateGnssAcceleration(frame.gnss, frame.time_s, gnss_accel);
  UpdateAttitude(kDefaultAttitudeGains, frame.imu, frame.mag,
                 config.specs.magnetic_field_ned_nt, gnss_accel.accel_ned_m_s2,
                 dt_s, attitude);
  const Eigen::Matrix3d rotation =
      attitude.attitude.normalized().toRotationMatrix();
  const Eigen::Vector3d gravity_ned(0.0, 0.0, kGravity);
  const Eigen::Vector3d accel_ned =
      rotation * ToVector(frame.imu.accel_m_s2) + gravity_ned;
  PredictNav(accel_ned, dt_s, nav);
}

void CorrectNav(const EstimatorConfig& config, const SensorFrame& frame,
                NavFilter& nav) noexcept {
  CorrectGnss(config.specs, frame.gnss, nav);
  CorrectBaro(frame.baro, nav);
  CorrectRadar(config, frame.radar, nav);
}

// Roll, pitch and heading from the body-to-NED rotation; heading in [0, 2pi).
Euler EulerFromRotation(const Eigen::Matrix3d& rotation) noexcept {
  Euler angles{.roll_rad = std::atan2(rotation(2, 1), rotation(2, 2)),
               .pitch_rad = -std::asin(rotation(2, 0)),
               .heading_rad = std::atan2(rotation(1, 0), rotation(0, 0))};
  if (angles.heading_rad < 0.0) {
    angles.heading_rad += 2.0 * std::numbers::pi;
  }
  return angles;
}

VehicleState PublishedState(const EstimatorConfig& config,
                            const SensorFrame& frame,
                            const AttitudeFilter& attitude,
                            const NavFilter& nav,
                            const double pitot_cas_m_s) noexcept {
  const Euler angles =
      EulerFromRotation(attitude.attitude.normalized().toRotationMatrix());
  const Eigen::Vector3d body_rates =
      ToVector(frame.imu.gyro_rad_s) - attitude.gyro_bias_rad_s;
  const GeoPoint position = NavPosition(nav);
  const double altitude_m = NavAltitude(nav);

  VehicleState state{};
  state.time_s = frame.time_s;
  state.latitude_rad = position.latitude_rad;
  state.longitude_rad = position.longitude_rad;
  state.altitude_m = altitude_m;
  state.roll_rad = angles.roll_rad;
  state.pitch_rad = angles.pitch_rad;
  state.heading_rad = angles.heading_rad;
  state.roll_rate_rad_s = body_rates.x();
  state.pitch_rate_rad_s = body_rates.y();
  state.yaw_rate_rad_s = body_rates.z();
  state.calibrated_airspeed_m_s = pitot_cas_m_s;
  state.climb_rate_m_s = -nav.x(5);
  state.north_velocity_m_s = nav.x(3);
  state.east_velocity_m_s = nav.x(4);
  state.height_above_ground_m = altitude_m - config.terrain_elevation_m;
  state.main_gear_on_ground = frame.main_gear_on_ground;
  state.nose_gear_on_ground = frame.nose_gear_on_ground;
  return state;
}

NavSnapshot NavSnapshotOf(const NavFilter& nav) noexcept {
  const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> row_major = nav.P;
  NavSnapshot snapshot{.valid = true, .covariance = {}};
  std::copy_n(row_major.data(), snapshot.covariance.size(),
              snapshot.covariance.begin());
  return snapshot;
}

double PitotCas(const PitotSample& pitot) noexcept {
  const double dynamic_pa = std::max(pitot.differential_pressure_pa, 0.0);
  return std::sqrt(2.0 * dynamic_pa / kSeaLevelDensityKgM3);
}

}  // namespace

EstimatorMemory StartEstimator() noexcept {
  return {.attitude = std::nullopt,
          .nav = std::nullopt,
          .gnss_accel = {},
          .pitot_cas_m_s = std::numeric_limits<double>::quiet_NaN()};
}

Estimate UpdateEstimator(const EstimatorConfig& config,
                         const SensorFrame& frame,
                         EstimatorMemory* memory) noexcept {
  if (memory == nullptr) {
    return InvalidResult();
  }
  if (memory->attitude && memory->nav) {
    const double dt_s = frame.time_s - memory->nav->time_s;
    if (dt_s <= 0.0 || dt_s > kMaxPredictGapS || !IsUsable(frame.imu)) {
      ResetFilters(*memory);
      return InvalidResult();
    }
    PredictStep(config, frame, dt_s, memory->gnss_accel, *memory->attitude,
                *memory->nav);
    CorrectNav(config, frame, *memory->nav);
  } else if (!TryInit(config, frame, *memory)) {
    return InvalidResult();
  }

  if (IsUsable(frame.pitot)) {
    memory->pitot_cas_m_s = PitotCas(frame.pitot);
  }
  if (!std::isfinite(memory->pitot_cas_m_s) || !memory->attitude ||
      !memory->nav) {
    return InvalidResult();
  }
  return {.state = PublishedState(config, frame, *memory->attitude,
                                  *memory->nav, memory->pitot_cas_m_s),
          .valid = true,
          .nav = NavSnapshotOf(*memory->nav)};
}

}  // namespace glidepath::fsw
