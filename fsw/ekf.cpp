#include "fsw/ekf.hpp"

#include "fsw/atmosphere.hpp"
#include "fsw/ekf_layout.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/gnss.hpp"
#include "fsw/measurements.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/units.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>

namespace glidepath::fsw {
namespace {

using Matrix27 = Eigen::Matrix<double, kStateDim, kStateDim>;
using Vector27 = Eigen::Matrix<double, kStateDim, 1>;
using Row27 = Eigen::Matrix<double, 1, kStateDim>;

constexpr double kMinRotationNorm = 1e-12;
constexpr double kMaxPredictDtS = 0.5;
constexpr double kSomiglianaBase = 9.7803253359;
constexpr double kSomiglianaSin2Coeff = 0.00193185265241;
constexpr double kWgs84EccentricitySquared = 6.69437999014e-3;
constexpr double kGravityAltitudeCoeff = 3.086e-6;

struct Wgs84Location {
  double latitude_rad;
  double altitude_m;
};

double LocalGravity(const Wgs84Location& location) noexcept {
  const double sin_lat = std::sin(location.latitude_rad);
  const double sin2 = sin_lat * sin_lat;
  const double ellipsoid = kSomiglianaBase *
                           (1.0 + kSomiglianaSin2Coeff * sin2) /
                           std::sqrt(1.0 - kWgs84EccentricitySquared * sin2);
  return ellipsoid - kGravityAltitudeCoeff * location.altitude_m;
}

double ChannelDecay(const double dt_s) noexcept {
  return std::exp(-dt_s / kUreCorrelationTimeS);
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& vector) noexcept {
  return (Eigen::Matrix3d{} << 0.0, -vector.z(), vector.y(), vector.z(), 0.0,
          -vector.x(), -vector.y(), vector.x(), 0.0)
      .finished();
}

Eigen::Quaterniond RotationFromVector(
    const Eigen::Vector3d& rotation) noexcept {
  const double norm = rotation.norm();
  if (norm < kMinRotationNorm) {
    return Eigen::Quaterniond::Identity();
  }
  const Eigen::AngleAxisd axis(norm, rotation / norm);
  return Eigen::Quaterniond{axis};
}

std::optional<Eigen::Quaterniond> AttitudeFromAccelMag(
    const SensorSpecs& specs, const ImuSample& imu,
    const MagSample& mag) noexcept {
  const double fx = imu.accel_m_s2[0];
  const double fy = imu.accel_m_s2[1];
  const double fz = imu.accel_m_s2[2];
  const double roll = std::atan2(-fy, -fz);
  const double pitch = std::atan2(fx, std::hypot(fy, fz));
  const double cos_roll = std::cos(roll);
  const double sin_roll = std::sin(roll);
  const double cos_pitch = std::cos(pitch);
  const double sin_pitch = std::sin(pitch);

  const double mx = mag.field_nt[0];
  const double my = mag.field_nt[1];
  const double mz = mag.field_nt[2];
  const double m_hx =
      mx * cos_pitch + my * sin_pitch * sin_roll + mz * cos_roll * sin_pitch;
  const double m_hy = my * cos_roll - mz * sin_roll;

  const double b_n = specs.magnetic_field_ned_nt[0];
  const double b_e = specs.magnetic_field_ned_nt[1];
  const double yaw = std::atan2(b_e, b_n) - std::atan2(m_hy, m_hx);

  const Eigen::AngleAxisd roll_axis(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_axis(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_axis(yaw, Eigen::Vector3d::UnitZ());
  return yaw_axis * pitch_axis * roll_axis;
}

Matrix27 Symmetrised(Matrix27 covariance) noexcept {
  return 0.5 * (covariance + covariance.transpose());
}

Eigen::Matrix3d BodyToNed(const Eigen::Quaterniond& attitude) noexcept {
  return attitude.normalized().toRotationMatrix();
}

Matrix27 InitialCovariance(const SensorSpecs& specs) noexcept {
  Matrix27 covariance = Matrix27::Zero();

  const double gnss_var =
      specs.gnss_horizontal_sigma_m * specs.gnss_horizontal_sigma_m;
  covariance(kPosN, kPosN) = gnss_var;
  covariance(kPosE, kPosE) = gnss_var;

  const double baro_sigma = BaroAltitudeSigmaM(0.0);
  covariance(kPosD, kPosD) = baro_sigma * baro_sigma;

  const double vel_var =
      specs.gnss_velocity_sigma_m_s * specs.gnss_velocity_sigma_m_s;
  covariance(kVelN, kVelN) = vel_var;
  covariance(kVelE, kVelE) = vel_var;
  covariance(kVelD, kVelD) = vel_var;

  const double imu_dt = ImuSampleIntervalS();
  const double accel_sample_sigma =
      specs.accel_density_m_s2_per_sqrt_hz / std::sqrt(imu_dt);
  const double sigma_rp =
      std::hypot(accel_sample_sigma, specs.accel_bias_sigma_m_s2) / kGravity;
  const double sigma_rp2 = sigma_rp * sigma_rp;
  covariance(kAttN, kAttN) = sigma_rp2;
  covariance(kAttE, kAttE) = sigma_rp2;

  const double b_n = specs.magnetic_field_ned_nt[0];
  const double b_e = specs.magnetic_field_ned_nt[1];
  const double b_d = specs.magnetic_field_ned_nt[2];
  const double b_h = std::hypot(b_n, b_e);
  const double mag_sigma = specs.magnetometer_sigma_nt;
  const double sigma_yaw =
      std::sqrt((mag_sigma / b_h) * (mag_sigma / b_h) +
                (b_d / b_h * sigma_rp) * (b_d / b_h * sigma_rp));
  covariance(kAttD, kAttD) = sigma_yaw * sigma_yaw;

  const double gyro_bias_var =
      specs.gyro_bias_sigma_rad_s * specs.gyro_bias_sigma_rad_s;
  covariance(kGyroBiasX, kGyroBiasX) = gyro_bias_var;
  covariance(kGyroBiasY, kGyroBiasY) = gyro_bias_var;
  covariance(kGyroBiasZ, kGyroBiasZ) = gyro_bias_var;

  const double accel_bias_var =
      specs.accel_bias_sigma_m_s2 * specs.accel_bias_sigma_m_s2;
  covariance(kAccelBiasX, kAccelBiasX) = accel_bias_var;
  covariance(kAccelBiasY, kAccelBiasY) = accel_bias_var;
  covariance(kAccelBiasZ, kAccelBiasZ) = accel_bias_var;

  covariance(kClockBias, kClockBias) =
      kInitialClockBiasSigmaM * kInitialClockBiasSigmaM;
  covariance(kClockDrift, kClockDrift) =
      kInitialClockDriftSigmaMPerS * kInitialClockDriftSigmaMPerS;

  return covariance;
}

Matrix27 Transition(const Ekf& ekf, const Eigen::Vector3d& specific_force_ned,
                    const double dt_s) noexcept {
  Matrix27 transition = Matrix27::Identity();
  const Eigen::Matrix3d rotation = BodyToNed(ekf.attitude);

  transition(kPosN, kVelN) = dt_s;
  transition(kPosE, kVelE) = dt_s;
  transition(kPosD, kVelD) = dt_s;

  const Eigen::Matrix3d force_skew = Skew(specific_force_ned);
  transition.block<3, 3>(kVelN, kAttN) = -force_skew * dt_s;
  transition.block<3, 3>(kVelN, kAccelBiasX) = -rotation * dt_s;
  transition.block<3, 3>(kAttN, kGyroBiasX) = -rotation * dt_s;

  const double phi = std::exp(-dt_s / kBiasTauSeconds);
  for (const std::ptrdiff_t index : {kGyroBiasX, kGyroBiasY, kGyroBiasZ,
                                     kAccelBiasX, kAccelBiasY, kAccelBiasZ}) {
    transition(index, index) = phi;
  }

  transition(kClockBias, kClockDrift) = dt_s;
  const double channel_decay = ChannelDecay(dt_s);
  Eigen::Index channel_index = kChannelBias0;
  for (const GnssChannel& channel : ekf.channels) {
    transition(channel_index, channel_index) =
        channel.slot >= 0 ? channel_decay : 0.0;
    ++channel_index;
  }

  return transition;
}

Matrix27 ProcessNoise(const SensorSpecs& specs, const Ekf& ekf,
                      const double dt_s) noexcept {
  Matrix27 noise = Matrix27::Zero();
  const double phi = std::exp(-dt_s / specs.bias_tau_s);
  const double bias_drive = 1.0 - phi * phi;

  const double accel_var = specs.accel_density_m_s2_per_sqrt_hz *
                           specs.accel_density_m_s2_per_sqrt_hz * dt_s;
  noise(kVelN, kVelN) = accel_var;
  noise(kVelE, kVelE) = accel_var;
  noise(kVelD, kVelD) = accel_var;

  const double gyro_var = specs.gyro_density_rad_s_per_sqrt_hz *
                          specs.gyro_density_rad_s_per_sqrt_hz * dt_s;
  noise(kAttN, kAttN) = gyro_var;
  noise(kAttE, kAttE) = gyro_var;
  noise(kAttD, kAttD) = gyro_var;

  const double gyro_bias_var =
      specs.gyro_bias_sigma_rad_s * specs.gyro_bias_sigma_rad_s * bias_drive;
  noise(kGyroBiasX, kGyroBiasX) = gyro_bias_var;
  noise(kGyroBiasY, kGyroBiasY) = gyro_bias_var;
  noise(kGyroBiasZ, kGyroBiasZ) = gyro_bias_var;

  const double accel_bias_var =
      specs.accel_bias_sigma_m_s2 * specs.accel_bias_sigma_m_s2 * bias_drive;
  noise(kAccelBiasX, kAccelBiasX) = accel_bias_var;
  noise(kAccelBiasY, kAccelBiasY) = accel_bias_var;
  noise(kAccelBiasZ, kAccelBiasZ) = accel_bias_var;

  const double dt2 = dt_s * dt_s;
  const double dt3 = dt2 * dt_s;
  noise(kClockBias, kClockBias) =
      kClockWhiteFmM2PerS * dt_s + kClockRandomWalkFmM2PerS3 * dt3 / 3.0;
  noise(kClockBias, kClockDrift) = kClockRandomWalkFmM2PerS3 * dt2 / 2.0;
  noise(kClockDrift, kClockBias) = noise(kClockBias, kClockDrift);
  noise(kClockDrift, kClockDrift) = kClockRandomWalkFmM2PerS3 * dt_s;

  const double channel_decay = ChannelDecay(dt_s);
  const double channel_drive =
      kUreSigmaM * kUreSigmaM * (1.0 - channel_decay * channel_decay);
  Eigen::Index channel_index = kChannelBias0;
  for (const GnssChannel& channel : ekf.channels) {
    noise(channel_index, channel_index) =
        channel.slot >= 0 ? channel_drive : 0.0;
    ++channel_index;
  }

  return noise;
}

void Inject(const Vector27& delta, Ekf& ekf) noexcept {
  const GeoPoint origin{.latitude_rad = ekf.latitude_rad,
                        .longitude_rad = ekf.longitude_rad};
  const LocalPoint displacement{.north_m = delta(kPosN),
                                .east_m = delta(kPosE)};
  const GeoPoint moved = ToGeo(displacement, origin);
  ekf.latitude_rad = moved.latitude_rad;
  ekf.longitude_rad = moved.longitude_rad;
  ekf.altitude_m -= delta(kPosD);

  ekf.velocity_ned_m_s.x() += delta(kVelN);
  ekf.velocity_ned_m_s.y() += delta(kVelE);
  ekf.velocity_ned_m_s.z() += delta(kVelD);

  const Eigen::Vector3d attitude_delta(delta(kAttN), delta(kAttE),
                                       delta(kAttD));
  ekf.attitude = RotationFromVector(attitude_delta) * ekf.attitude;
  ekf.attitude.normalize();

  ekf.gyro_bias_rad_s.x() += delta(kGyroBiasX);
  ekf.gyro_bias_rad_s.y() += delta(kGyroBiasY);
  ekf.gyro_bias_rad_s.z() += delta(kGyroBiasZ);

  ekf.accel_bias_m_s2.x() += delta(kAccelBiasX);
  ekf.accel_bias_m_s2.y() += delta(kAccelBiasY);
  ekf.accel_bias_m_s2.z() += delta(kAccelBiasZ);

  ekf.clock_bias_m += delta(kClockBias);
  ekf.clock_drift_m_s += delta(kClockDrift);
  Eigen::Index channel_index = kChannelBias0;
  for (GnssChannel& channel : ekf.channels) {
    channel.bias_m += delta(channel_index);
    ++channel_index;
  }
}

struct ScalarMeasurement {
  double residual{};
  double noise_variance{};
  Row27 h;
};

Innovation ScalarUpdate(const ScalarMeasurement& measurement,
                        Ekf& ekf) noexcept {
  Innovation result{.residual = 0.0, .variance = 0.0};
  const double predicted_variance =
      (measurement.h * ekf.covariance * measurement.h.transpose())(0, 0);
  const double innovation_variance =
      predicted_variance + measurement.noise_variance;
  if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0) {
    return result;
  }

  const Vector27 kalman_gain =
      ekf.covariance * measurement.h.transpose() / innovation_variance;
  const Vector27 delta = kalman_gain * measurement.residual;

  const Matrix27 identity = Matrix27::Identity();
  const Matrix27 i_minus_kh = identity - kalman_gain * measurement.h;
  ekf.covariance =
      i_minus_kh * ekf.covariance * i_minus_kh.transpose() +
      kalman_gain * measurement.noise_variance * kalman_gain.transpose();
  ekf.covariance = Symmetrised(ekf.covariance);
  Inject(delta, ekf);

  result.residual = measurement.residual;
  result.variance = innovation_variance;
  return result;
}

// Tightly coupled GNSS (G11c): one pseudorange and one range rate per channel.
Eigen::Matrix3d NedToEcefMatrix(const double latitude_rad,
                                const double longitude_rad) noexcept {
  const double sin_lat = std::sin(latitude_rad);
  const double cos_lat = std::cos(latitude_rad);
  const double sin_lon = std::sin(longitude_rad);
  const double cos_lon = std::cos(longitude_rad);
  Eigen::Matrix3d basis;
  basis.col(0) =
      Eigen::Vector3d(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
  basis.col(1) = Eigen::Vector3d(-sin_lon, cos_lon, 0.0);
  basis.col(2) =
      Eigen::Vector3d(-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat);
  return basis;
}

Eigen::Vector3d AsVector(const Ecef& point) noexcept {
  return {point[0], point[1], point[2]};
}

struct LineOfSight {
  double range_m;
  Eigen::Vector3d unit_ecef;  // Receiver to satellite.
  Eigen::Vector3d unit_ned;   // The same, in the receiver's NED frame.
  Eigen::Vector3d receiver_velocity_ecef;
};

LineOfSight LineOfSightTo(const Ekf& ekf,
                          const SatelliteState& satellite) noexcept {
  const Eigen::Matrix3d ned_to_ecef =
      NedToEcefMatrix(ekf.latitude_rad, ekf.longitude_rad);
  const GeoPoint site{.latitude_rad = ekf.latitude_rad,
                      .longitude_rad = ekf.longitude_rad};
  const Eigen::Vector3d receiver =
      AsVector(GeodeticToEcef(site, ekf.altitude_m + kKpaoGeoidSeparationM));
  const Eigen::Vector3d separation = AsVector(satellite.position_m) - receiver;
  const double range = separation.norm();
  const Eigen::Vector3d unit = separation / range;
  return {.range_m = range,
          .unit_ecef = unit,
          .unit_ned = ned_to_ecef.transpose() * unit,
          .receiver_velocity_ecef = ned_to_ecef * ekf.velocity_ned_m_s};
}

bool IsBaselineSlot(const int slot) noexcept {
  return slot >= 0 && static_cast<std::size_t>(slot) < kBaselineSlots.size();
}

// The satellite of the sample's baseline slot, found by search.
std::optional<SatelliteState> BaselineSatellite(
    const RangeSample& sample, const double gps_seconds) noexcept {
  int index = 0;
  for (const GpsSlot& candidate : kBaselineSlots) {
    if (index == sample.slot) {
      return SatelliteAt(candidate, gps_seconds);
    }
    ++index;
  }
  return std::nullopt;
}

std::span<const RangeSample> ActiveRanges(const SensorFrame& frame) noexcept {
  return {frame.ranges.data(), std::min(frame.range_count, kMaxChannels)};
}

bool FrameHasSlot(const SensorFrame& frame, const int slot) noexcept {
  return std::ranges::any_of(
      ActiveRanges(frame),
      [slot](const RangeSample& sample) { return sample.slot == slot; });
}

// A channel and its index in the state vector.
struct ChannelHandle {
  GnssChannel* channel;
  Eigen::Index index;
};

ChannelHandle FindChannel(Ekf& ekf, const int slot) noexcept {
  Eigen::Index index = kChannelBias0;
  for (GnssChannel& channel : ekf.channels) {
    if (slot >= 0 && channel.slot == slot) {
      return {.channel = &channel, .index = index};
    }
    ++index;
  }
  return {.channel = nullptr, .index = index};
}

void ZeroChannelCovariance(Ekf& ekf, const Eigen::Index index) noexcept {
  ekf.covariance.row(index).setZero();
  ekf.covariance.col(index).setZero();
}

void ReleaseChannel(Ekf& ekf, const Eigen::Index index,
                    GnssChannel& channel) noexcept {
  channel = GnssChannel{};
  ZeroChannelCovariance(ekf, index);
}

// Bias 0 and variance sigma_URE^2, with no cross terms.
void AssignChannel(Ekf& ekf, const Eigen::Index index, GnssChannel& channel,
                   const int slot) noexcept {
  ZeroChannelCovariance(ekf, index);
  ekf.covariance(index, index) = kUreSigmaM * kUreSigmaM;
  channel = {.slot = slot, .bias_m = 0.0};
}

// A channel whose slot left the frame is freed; a new slot takes the lowest
// free channel, in frame order.
void AssignChannels(const SensorFrame& frame, Ekf& ekf) noexcept {
  Eigen::Index index = kChannelBias0;
  for (GnssChannel& channel : ekf.channels) {
    if (channel.slot >= 0 && !FrameHasSlot(frame, channel.slot)) {
      ReleaseChannel(ekf, index, channel);
    }
    ++index;
  }
  for (const RangeSample& sample : ActiveRanges(frame)) {
    if (!IsBaselineSlot(sample.slot) ||
        FindChannel(ekf, sample.slot).channel != nullptr) {
      continue;
    }
    Eigen::Index free_index = kChannelBias0;
    for (GnssChannel& channel : ekf.channels) {
      if (channel.slot < 0) {
        AssignChannel(ekf, free_index, channel, sample.slot);
        break;
      }
      ++free_index;
    }
  }
}

// Mean over the frame's ranges of (pseudorange - geometric range), with the
// receiver at the initial estimate. Zero when there are no usable ranges.
double InitialClockBiasM(const SensorFrame& frame, const Ekf& ekf) noexcept {
  double sum = 0.0;
  std::size_t used = 0;
  for (const RangeSample& sample : ActiveRanges(frame)) {
    const std::optional<SatelliteState> satellite =
        BaselineSatellite(sample, frame.gps_seconds_since_epoch);
    if (!satellite.has_value()) {
      continue;
    }
    sum += sample.pseudorange_m - LineOfSightTo(ekf, *satellite).range_m;
    used += 1;
  }
  return used == 0 ? 0.0 : sum / static_cast<double>(used);
}

struct NisSum {
  double sum{};
  std::size_t count{};
};

void AccumulateNis(const Innovation& innovation, NisSum& nis) noexcept {
  if (innovation.variance > 0.0) {
    nis.sum += Nis(innovation);
    nis.count += 1;
  }
}

// Slot value for an epoch: the RMS normalised innovation as the residual, with
// unit variance, so Nis() of it is the mean normalised square.
Innovation RmsInnovation(const NisSum& nis) noexcept {
  if (nis.count == 0) {
    return {.residual = 0.0, .variance = 0.0};
  }
  return {.residual = std::sqrt(nis.sum / static_cast<double>(nis.count)),
          .variance = 1.0};
}

struct RangeNis {
  NisSum pseudorange;
  NisSum rate;
};

// Pseudorange, then range rate at the estimate the pseudorange left behind.
void UpdatePseudorange(const RangeSample& sample, const LineOfSight& los,
                       const ChannelHandle& handle, Ekf& ekf,
                       NisSum& nis) noexcept {
  Row27 row = Row27::Zero();
  row.block<1, 3>(0, kPosN) = -los.unit_ned.transpose();
  row(0, kClockBias) = 1.0;
  row(0, handle.index) = 1.0;
  const double predicted =
      los.range_m + ekf.clock_bias_m + handle.channel->bias_m;
  const ScalarMeasurement measurement{
      .residual = sample.pseudorange_m - predicted,
      .noise_variance = kPseudorangeWhiteVarianceM2,
      .h = row,
  };
  AccumulateNis(ScalarUpdate(measurement, ekf), nis);
}

void UpdateRangeRate(const RangeSample& sample, const SatelliteState& satellite,
                     Ekf& ekf, NisSum& nis) noexcept {
  const LineOfSight los = LineOfSightTo(ekf, satellite);
  const double predicted =
      (AsVector(satellite.velocity_m_s) - los.receiver_velocity_ecef)
          .dot(los.unit_ecef) +
      ekf.clock_drift_m_s;
  Row27 row = Row27::Zero();
  row.block<1, 3>(0, kVelN) = -los.unit_ned.transpose();
  row(0, kClockDrift) = 1.0;
  const ScalarMeasurement measurement{
      .residual = sample.range_rate_m_s - predicted,
      .noise_variance = kRangeRateSigmaMPerS * kRangeRateSigmaMPerS,
      .h = row,
  };
  AccumulateNis(ScalarUpdate(measurement, ekf), nis);
}

void UpdateRangeSample(const RangeSample& sample, const double gps_seconds,
                       Ekf& ekf, RangeNis& nis,
                       const int exclude_channel) noexcept {
  const ChannelHandle handle = FindChannel(ekf, sample.slot);
  const std::optional<SatelliteState> satellite =
      BaselineSatellite(sample, gps_seconds);
  if (handle.channel == nullptr || !satellite.has_value()) {
    return;
  }
  if (handle.index - kChannelBias0 == exclude_channel) {
    return;
  }
  UpdatePseudorange(sample, LineOfSightTo(ekf, *satellite), handle, ekf,
                    nis.pseudorange);
  UpdateRangeRate(sample, *satellite, ekf, nis.rate);
}

// Innovation slots: slot 0 holds the pseudorange RMS, slot 2 the range-rate
// RMS (the name kSlotGnssVelNorth is kept; its value is now a range rate).
void UpdateRanges(const SensorFrame& frame, const int exclude_channel, Ekf& ekf,
                  Innovations& innovations) noexcept {
  RangeNis nis{};
  for (const RangeSample& sample : ActiveRanges(frame)) {
    UpdateRangeSample(sample, frame.gps_seconds_since_epoch, ekf, nis,
                      exclude_channel);
  }
  innovations[kSlotGnssNorth] = RmsInnovation(nis.pseudorange);
  innovations[kSlotGnssVelNorth] = RmsInnovation(nis.rate);
}

void UpdateBaro(const EkfConfig& config, const BaroSample& baro, Ekf& ekf,
                Innovations& innovations) noexcept {
  const double measured_altitude = IsaAltitudeM(baro.pressure_pa);
  const double gravity = LocalGravity(
      {.latitude_rad = ekf.latitude_rad, .altitude_m = ekf.altitude_m});
  const double density = IsaDensityKgM3(ekf.altitude_m);
  const double altitude_sigma =
      config.specs.baro_pressure_sigma_pa / (density * gravity);
  const double altitude_var = altitude_sigma * altitude_sigma;

  Row27 row = Row27::Zero();
  row(0, kPosD) = -1.0;
  const ScalarMeasurement measurement{
      .residual = measured_altitude - ekf.altitude_m,
      .noise_variance = altitude_var,
      .h = row,
  };
  innovations[kSlotBaro] = ScalarUpdate(measurement, ekf);
}

Row27 MagnetometerRowForAxis(const Eigen::Matrix3d& rotation,
                             const Eigen::Vector3d& field_ned,
                             const Eigen::Index axis) noexcept {
  const Eigen::Matrix3d sensitivity = rotation.transpose() * Skew(field_ned);
  Row27 row = Row27::Zero();
  row(0, kAttN) = sensitivity(axis, 0);
  row(0, kAttE) = sensitivity(axis, 1);
  row(0, kAttD) = sensitivity(axis, 2);
  return row;
}

void UpdateMagnetometer(const EkfConfig& config, const MagSample& mag, Ekf* ekf,
                        Innovations& innovations) noexcept {
  if (ekf == nullptr) {
    return;
  }
  const Eigen::Vector3d field_ned(config.specs.magnetic_field_ned_nt[0],
                                  config.specs.magnetic_field_ned_nt[1],
                                  config.specs.magnetic_field_ned_nt[2]);
  const Eigen::Vector3d measured(mag.field_nt[0], mag.field_nt[1],
                                 mag.field_nt[2]);
  const double mag_var =
      config.specs.magnetometer_sigma_nt * config.specs.magnetometer_sigma_nt;

  const Eigen::Matrix3d rotation = BodyToNed(ekf->attitude);
  const Eigen::Vector3d predicted = rotation.transpose() * field_ned;
  innovations[kSlotMagX] =
      ScalarUpdate({.residual = measured.x() - predicted.x(),
                    .noise_variance = mag_var,
                    .h = MagnetometerRowForAxis(rotation, field_ned, 0)},
                   *ekf);
  innovations[kSlotMagY] =
      ScalarUpdate({.residual = measured.y() - predicted.y(),
                    .noise_variance = mag_var,
                    .h = MagnetometerRowForAxis(rotation, field_ned, 1)},
                   *ekf);
  innovations[kSlotMagZ] =
      ScalarUpdate({.residual = measured.z() - predicted.z(),
                    .noise_variance = mag_var,
                    .h = MagnetometerRowForAxis(rotation, field_ned, 2)},
                   *ekf);
}

void UpdateRadar(const EkfConfig& config, const RadarSample& radar, Ekf& ekf,
                 Innovations& innovations) noexcept {
  const double predicted_height = ekf.altitude_m - config.terrain_elevation_m;
  const double sigma = radar.height_m < 1.0
                           ? config.specs.radar_sigma_below_1m_m
                           : config.specs.radar_sigma_m;
  const double height_var = sigma * sigma;

  Row27 row = Row27::Zero();
  row(0, kPosD) = -1.0;
  const ScalarMeasurement measurement{
      .residual = radar.height_m - predicted_height,
      .noise_variance = height_var,
      .h = row,
  };
  innovations[kSlotRadar] = ScalarUpdate(measurement, ekf);
}

Eigen::Vector3d BodyAcceleration(const ImuSample& imu,
                                 const Eigen::Vector3d& accel_bias) noexcept {
  return Eigen::Vector3d(imu.accel_m_s2[0], imu.accel_m_s2[1],
                         imu.accel_m_s2[2]) -
         accel_bias;
}

Eigen::Vector3d BodyGyro(const ImuSample& imu,
                         const Eigen::Vector3d& gyro_bias) noexcept {
  return Eigen::Vector3d(imu.gyro_rad_s[0], imu.gyro_rad_s[1],
                         imu.gyro_rad_s[2]) -
         gyro_bias;
}

}  // namespace

std::optional<Ekf> InitEkf(const EkfConfig& config,
                           const SensorFrame& frame) noexcept {
  if (!IsUsable(frame.imu) || !IsUsable(frame.gnss) || !IsUsable(frame.baro) ||
      !IsUsable(frame.mag)) {
    return std::nullopt;
  }

  const std::optional<Eigen::Quaterniond> attitude =
      AttitudeFromAccelMag(config.specs, frame.imu, frame.mag);
  if (!attitude.has_value()) {
    return std::nullopt;
  }

  Ekf ekf{
      .time_s = frame.time_s,
      .latitude_rad = frame.gnss.latitude_rad,
      .longitude_rad = frame.gnss.longitude_rad,
      .altitude_m = IsaAltitudeM(frame.baro.pressure_pa),
      .velocity_ned_m_s = Eigen::Vector3d(
          frame.gnss.north_m_s, frame.gnss.east_m_s, frame.gnss.down_m_s),
      .attitude = *attitude,
      .gyro_bias_rad_s = Eigen::Vector3d::Zero(),
      .accel_bias_m_s2 = Eigen::Vector3d::Zero(),
      .covariance = InitialCovariance(config.specs),
  };

  const double baro_sigma = BaroAltitudeSigmaM(ekf.altitude_m);
  ekf.covariance(kPosD, kPosD) = baro_sigma * baro_sigma;
  ekf.clock_bias_m = InitialClockBiasM(frame, ekf);
  AssignChannels(frame, ekf);
  return ekf;
}

void PredictEkf(const EkfConfig& config, const ImuSample& imu,
                const double dt_s, Ekf& ekf) noexcept {
  if (!IsUsable(imu) || dt_s <= 0.0 || dt_s > kMaxPredictDtS ||
      !std::isfinite(dt_s)) {
    return;
  }

  const Eigen::Vector3d body_accel = BodyAcceleration(imu, ekf.accel_bias_m_s2);
  const Eigen::Vector3d body_gyro = BodyGyro(imu, ekf.gyro_bias_rad_s);
  const Eigen::Matrix3d rotation = BodyToNed(ekf.attitude);
  const Eigen::Vector3d specific_force_ned = rotation * body_accel;

  const Matrix27 transition = Transition(ekf, specific_force_ned, dt_s);
  const Matrix27 process_noise = ProcessNoise(config.specs, ekf, dt_s);
  const Matrix27 covariance = ekf.covariance;

  const double gravity = LocalGravity(
      {.latitude_rad = ekf.latitude_rad, .altitude_m = ekf.altitude_m});
  ekf.attitude = ekf.attitude * RotationFromVector(body_gyro * dt_s);
  ekf.attitude.normalize();

  const Eigen::Vector3d velocity_old = ekf.velocity_ned_m_s;
  const Eigen::Vector3d gravity_ned(0.0, 0.0, gravity);
  ekf.velocity_ned_m_s += (specific_force_ned + gravity_ned) * dt_s;
  const Eigen::Vector3d velocity_mean =
      0.5 * (velocity_old + ekf.velocity_ned_m_s);

  const GeoPoint origin{.latitude_rad = ekf.latitude_rad,
                        .longitude_rad = ekf.longitude_rad};
  const LocalPoint displacement{.north_m = velocity_mean.x() * dt_s,
                                .east_m = velocity_mean.y() * dt_s};
  const GeoPoint moved = ToGeo(displacement, origin);
  ekf.latitude_rad = moved.latitude_rad;
  ekf.longitude_rad = moved.longitude_rad;
  ekf.altitude_m -= velocity_mean.z() * dt_s;
  ekf.clock_bias_m += ekf.clock_drift_m_s * dt_s;
  const double channel_decay = ChannelDecay(dt_s);
  for (GnssChannel& channel : ekf.channels) {
    if (channel.slot >= 0) {
      channel.bias_m *= channel_decay;
    }
  }
  ekf.time_s += dt_s;

  ekf.covariance =
      transition * covariance * transition.transpose() + process_noise;
  ekf.covariance = Symmetrised(ekf.covariance);
}

Innovations AidEkf(const EkfConfig& config, const SensorFrame& frame,
                   const AidingUse& aiding, Ekf& ekf) noexcept {
  Innovations innovations{};
  for (Innovation& slot : innovations) {
    slot = {.residual = 0.0, .variance = 0.0};
  }

  if (aiding.gnss && frame.range_count > 0) {
    AssignChannels(frame, ekf);
    UpdateRanges(frame, aiding.exclude_channel, ekf, innovations);
  }
  if (aiding.baro && IsUsable(frame.baro)) {
    UpdateBaro(config, frame.baro, ekf, innovations);
  }
  if (aiding.magnetometer && IsUsable(frame.mag)) {
    UpdateMagnetometer(config, frame.mag, &ekf, innovations);
  }
  if (aiding.radar && IsUsable(frame.radar) &&
      frame.radar.height_m >= config.specs.radar_min_valid_height_m &&
      frame.radar.height_m <= config.specs.radar_max_valid_height_m) {
    UpdateRadar(config, frame.radar, ekf, innovations);
  }

  return innovations;
}

}  // namespace glidepath::fsw