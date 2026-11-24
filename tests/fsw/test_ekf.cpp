// Oracle guardrails for the error-state EKF: propagation, aiding signs and
// heap use. Reference formulas are independent of fsw/ekf.cpp.
#include "fsw/atmosphere.hpp"
#include "fsw/ekf.hpp"
#include "fsw/ekf_layout.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/gnss.hpp"
#include "fsw/measurements.hpp"
#include "fsw/runway.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/units.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <atomic>
#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

extern std::atomic<std::size_t> g_test_allocations;

namespace {

using glidepath::fsw::AidEkf;
using glidepath::fsw::AidingUse;
using glidepath::fsw::Ekf;
using glidepath::fsw::EkfConfig;
using glidepath::fsw::ImuSample;
using glidepath::fsw::InitEkf;
using glidepath::fsw::Innovation;
using glidepath::fsw::Innovations;
using glidepath::fsw::kAccelBiasX;
using glidepath::fsw::kAttD;
using glidepath::fsw::kAttE;
using glidepath::fsw::kAttN;
using glidepath::fsw::kControlRateHz;
using glidepath::fsw::kDefaultSpecs;
using glidepath::fsw::kGyroBiasX;
using glidepath::fsw::kMagneticFieldNedNt;
using glidepath::fsw::kPaloAlto31;
using glidepath::fsw::kPosD;
using glidepath::fsw::kSlotBaro;
using glidepath::fsw::kSlotGnssEast;
using glidepath::fsw::kSlotGnssNorth;
using glidepath::fsw::kSlotGnssVelDown;
using glidepath::fsw::kSlotGnssVelNorth;
using glidepath::fsw::kSlotMagX;
using glidepath::fsw::kSlotRadar;
using glidepath::fsw::kVelD;
using glidepath::fsw::Nis;
using glidepath::fsw::PredictEkf;
using glidepath::fsw::SensorFrame;
using glidepath::fsw::SensorSpecs;
using glidepath::fsw::ToGeo;
using glidepath::fsw::ToLocal;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kSomiglianaEquator = 9.7803253359;
std::size_t AllocationCount() {
  return g_test_allocations.load(std::memory_order_relaxed);
}

bool RelativeNear(const double measured, const double expected,
                  const double rtol) {
  const double scale = std::max(1.0, std::abs(expected));
  return std::abs(measured - expected) <= rtol * scale;
}

Eigen::Quaterniond AttitudeFromEuler(const double roll_rad,
                                     const double pitch_rad,
                                     const double yaw_rad) {
  const Eigen::AngleAxisd roll(roll_rad, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(pitch_rad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(yaw_rad, Eigen::Vector3d::UnitZ());
  return yaw * pitch * roll;
}

Eigen::Vector3d SpecificForceBody(const Eigen::Quaterniond& attitude,
                                  const double gravity_m_s2) {
  const Eigen::Matrix3d rotation = attitude.normalized().toRotationMatrix();
  const Eigen::Vector3d gravity_ned(0.0, 0.0, gravity_m_s2);
  return rotation.transpose() * (-gravity_ned);
}

Eigen::Vector3d MagBody(const Eigen::Quaterniond& attitude) {
  const Eigen::Matrix3d rotation = attitude.normalized().toRotationMatrix();
  const Eigen::Vector3d field_ned(
      kMagneticFieldNedNt[0], kMagneticFieldNedNt[1], kMagneticFieldNedNt[2]);
  return rotation.transpose() * field_ned;
}

SensorSpecs ZeroBiasSpecs(const SensorSpecs& base) {
  SensorSpecs specs = base;
  specs.gyro_bias_sigma_rad_s = 0.0;
  specs.accel_bias_sigma_m_s2 = 0.0;
  return specs;
}

EkfConfig TestConfig(const SensorSpecs& specs) {
  return {.specs = specs, .terrain_elevation_m = kPaloAlto31.elevation_m};
}

Ekf LevelEkf() {
  Ekf ekf{};
  ekf.time_s = 0.0;
  ekf.latitude_rad = 0.0;
  ekf.longitude_rad = 0.0;
  ekf.altitude_m = 0.0;
  ekf.velocity_ned_m_s.setZero();
  ekf.attitude = Eigen::Quaterniond::Identity();
  ekf.gyro_bias_rad_s.setZero();
  ekf.accel_bias_m_s2.setZero();
  ekf.covariance.setZero();
  return ekf;
}

ImuSample LevelImu(const double gravity_m_s2) {
  return {.valid = true,
          .gyro_rad_s = {0.0, 0.0, 0.0},
          .accel_m_s2 = {0.0, 0.0, -gravity_m_s2}};
}

SensorFrame NoiseFreeInitFrame(const double roll_deg, const double pitch_deg,
                               const double heading_deg) {
  const double gravity = kSomiglianaEquator;
  const Eigen::Quaterniond attitude = AttitudeFromEuler(
      roll_deg * kDegree, pitch_deg * kDegree, heading_deg * kDegree);
  const Eigen::Vector3d accel = SpecificForceBody(attitude, gravity);
  const Eigen::Vector3d mag = MagBody(attitude);
  const double altitude_m = 500.0;
  const double pressure = glidepath::fsw::IsaPressurePa(altitude_m);

  SensorFrame frame{};
  frame.time_s = 1.0;
  frame.imu = {.valid = true,
               .gyro_rad_s = {0.0, 0.0, 0.0},
               .accel_m_s2 = {accel.x(), accel.y(), accel.z()}};
  frame.gnss = {.valid = true,
                .latitude_rad = 0.5,
                .longitude_rad = -1.2,
                .north_m_s = 0.0,
                .east_m_s = 0.0,
                .down_m_s = 0.0};
  frame.baro = {.valid = true, .pressure_pa = pressure};
  frame.mag = {.valid = true, .field_nt = {mag.x(), mag.y(), mag.z()}};
  frame.pitot = {.valid = false, .differential_pressure_pa = 0.0};
  frame.radar = {.valid = false, .height_m = 0.0};
  frame.main_gear_on_ground = false;
  frame.nose_gear_on_ground = false;
  return frame;
}

SensorFrame InitializedFrame() {
  return NoiseFreeInitFrame(0.0, 0.0, 0.0);
}

std::optional<Ekf> InitReferenceFilter() {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  const SensorFrame frame = InitializedFrame();
  return InitEkf(config, frame);
}

double HeadingRad(const Eigen::Quaterniond& attitude) {
  const Eigen::Matrix3d rotation = attitude.normalized().toRotationMatrix();
  double heading = std::atan2(rotation(1, 0), rotation(0, 0));
  if (heading < 0.0) {
    heading += 2.0 * std::numbers::pi;
  }
  return heading;
}

}  // namespace

TEST_CASE("ekf: static level propagation grows P per the oracle") {
  const SensorSpecs specs = ZeroBiasSpecs(kDefaultSpecs);
  const EkfConfig config = TestConfig(specs);
  Ekf ekf = LevelEkf();
  const ImuSample imu = LevelImu(kSomiglianaEquator);
  const double dt_s = 1.0 / kControlRateHz;
  constexpr int kSteps = 40;
  for (int step = 0; step < kSteps; ++step) {
    PredictEkf(config, imu, dt_s, ekf);
  }

  const double rho_g = specs.gyro_density_rad_s_per_sqrt_hz;
  const double rho_a = specs.accel_density_m_s2_per_sqrt_hz;
  const double att_var = rho_g * rho_g * kSteps * dt_s;
  const double vel_d_var = rho_a * rho_a * kSteps * dt_s;
  const double pos_d_var = rho_a * rho_a * dt_s * dt_s * dt_s * (kSteps - 1) *
                           kSteps * (2 * kSteps - 1) / 6.0;

  CHECK(RelativeNear(ekf.covariance(kAttN, kAttN), att_var, 1e-9));
  CHECK(RelativeNear(ekf.covariance(kAttE, kAttE), att_var, 1e-9));
  CHECK(RelativeNear(ekf.covariance(kAttD, kAttD), att_var, 1e-9));
  CHECK(RelativeNear(ekf.covariance(kVelD, kVelD), vel_d_var, 1e-9));
  CHECK(RelativeNear(ekf.covariance(kPosD, kPosD), pos_d_var, 1e-9));
}

TEST_CASE("ekf: bias covariance follows sigma_b and phi") {
  const SensorSpecs specs = kDefaultSpecs;
  const EkfConfig config = TestConfig(specs);
  Ekf ekf = LevelEkf();
  ekf.covariance.setZero();
  const ImuSample imu = LevelImu(kSomiglianaEquator);
  const double dt_s = 1.0 / kControlRateHz;
  const double phi = std::exp(-dt_s / specs.bias_tau_s);
  constexpr int kSteps = 200;
  for (int step = 0; step < kSteps; ++step) {
    PredictEkf(config, imu, dt_s, ekf);
  }
  const double expected = specs.gyro_bias_sigma_rad_s *
                          specs.gyro_bias_sigma_rad_s *
                          (1.0 - std::pow(phi, 2 * kSteps));
  CHECK(RelativeNear(ekf.covariance(kGyroBiasX, kGyroBiasX), expected, 1e-9));
}

TEST_CASE("ekf: constant body acceleration integrates velocity and position") {
  constexpr double kAccelForward = 0.5;
  const SensorSpecs specs = ZeroBiasSpecs(kDefaultSpecs);
  const EkfConfig config = TestConfig(specs);
  Ekf ekf = LevelEkf();
  const double dt_s = 1.0 / kControlRateHz;
  constexpr int kAccelSteps = 80;
  ImuSample cruise = LevelImu(kSomiglianaEquator);
  cruise.accel_m_s2[0] = kAccelForward;
  for (int step = 0; step < kAccelSteps; ++step) {
    PredictEkf(config, cruise, dt_s, ekf);
  }
  const double time_s = kAccelSteps * dt_s;
  CHECK(ekf.velocity_ned_m_s.x() ==
        Catch::Approx(2.0 * kAccelForward).margin(1e-9));
  const glidepath::fsw::LocalPoint north = ToLocal(
      {.latitude_rad = ekf.latitude_rad, .longitude_rad = ekf.longitude_rad},
      {.latitude_rad = 0.0, .longitude_rad = 0.0});
  CHECK(north.north_m ==
        Catch::Approx(0.5 * kAccelForward * time_s * time_s).margin(1e-6));
}

TEST_CASE("ekf: constant yaw rate integrates heading") {
  const SensorSpecs specs = ZeroBiasSpecs(kDefaultSpecs);
  const EkfConfig config = TestConfig(specs);
  Ekf ekf = LevelEkf();
  const double dt_s = 1.0 / kControlRateHz;
  ImuSample cruise = LevelImu(kSomiglianaEquator);
  cruise.gyro_rad_s[2] = 0.1;
  constexpr int kYawSteps = 200;
  for (int step = 0; step < kYawSteps; ++step) {
    PredictEkf(config, cruise, dt_s, ekf);
  }
  CHECK(HeadingRad(ekf.attitude) == Catch::Approx(0.5).margin(1e-9));
}

TEST_CASE("ekf: InitEkf recovers attitudes from noise-free samples") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  const std::array<std::array<double, 3>, 2> kAttitudesDeg = {{
      {{0.0, 0.0, 90.0}},
      {{-5.0, 10.0, 300.0}},
  }};
  for (const std::array<double, 3>& angles : kAttitudesDeg) {
    const SensorFrame frame =
        NoiseFreeInitFrame(angles[0], angles[1], angles[2]);
    const std::optional<Ekf> ekf = InitEkf(config, frame);
    REQUIRE(ekf.has_value());
    const Eigen::Matrix3d rotation =
        ekf->attitude.normalized().toRotationMatrix();
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double pitch = -std::asin(rotation(2, 0));
    const double heading = HeadingRad(ekf->attitude);
    CHECK(roll == Catch::Approx(angles[0] * kDegree).margin(1e-9));
    CHECK(pitch == Catch::Approx(angles[1] * kDegree).margin(1e-9));
    CHECK(heading == Catch::Approx(angles[2] * kDegree).margin(1e-9));
  }
}

TEST_CASE("ekf: InitEkf rejects unusable samples") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  SensorFrame frame = InitializedFrame();
  frame.imu.valid = false;
  CHECK(!InitEkf(config, frame).has_value());
  frame = InitializedFrame();
  frame.gnss.valid = false;
  CHECK(!InitEkf(config, frame).has_value());
}

namespace {

// Tightly coupled GNSS oracles (G11c). Predicted ranges and rates are written
// here from the geometry alone; Jacobians are central finite differences, and
// posteriors are the textbook scalar Kalman update.
using glidepath::fsw::Ecef;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::kBaselineSlots;
using glidepath::fsw::kChannelBias0;
using glidepath::fsw::kClockBias;
using glidepath::fsw::kClockDrift;
using glidepath::fsw::kClockRandomWalkFmM2PerS3;
using glidepath::fsw::kClockWhiteFmM2PerS;
using glidepath::fsw::kKpaoGeoidSeparationM;
using glidepath::fsw::kPosE;
using glidepath::fsw::kPosN;
using glidepath::fsw::kPseudorangeWhiteVarianceM2;
using glidepath::fsw::kRangeRateSigmaMPerS;
using glidepath::fsw::kRunStartGpsSecondsSinceEpoch;
using glidepath::fsw::kStateDim;
using glidepath::fsw::kUreSigmaM;
using glidepath::fsw::kVelE;
using glidepath::fsw::kVelN;
using glidepath::fsw::SatelliteAt;
using glidepath::fsw::SatelliteState;

using StateMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;
using StateVector = Eigen::Matrix<double, kStateDim, 1>;

constexpr double kFdStep = 0.01;
constexpr double kPosteriorRtol = 1e-9;
constexpr double kKpaoLatDeg = 37.4611;
constexpr double kKpaoLonDeg = -122.1150;
constexpr double kKpaoMslM = 30.0;
constexpr double kPredictDtS = 0.025;

Eigen::Vector3d AsVector(const Ecef& point) {
  return {point[0], point[1], point[2]};
}

// ECEF from a north/east/down vector at the given site.
Eigen::Vector3d NedToEcef(const double lat_rad, const double lon_rad,
                          const Eigen::Vector3d& ned) {
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);
  const Eigen::Vector3d north(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
  const Eigen::Vector3d east(-sin_lon, cos_lon, 0.0);
  const Eigen::Vector3d down(-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat);
  return north * ned.x() + east * ned.y() + down * ned.z();
}

Eigen::Vector3d ReceiverEcef(const double lat_rad, const double lon_rad,
                             const double altitude_m) {
  return AsVector(glidepath::fsw::GeodeticToEcef(
      {.latitude_rad = lat_rad, .longitude_rad = lon_rad},
      altitude_m + kKpaoGeoidSeparationM));
}

double PredictedPseudorange(const SatelliteState& sat, const double lat_rad,
                            const double lon_rad, const double altitude_m,
                            const double clock_m, const double channel_m) {
  const Eigen::Vector3d receiver = ReceiverEcef(lat_rad, lon_rad, altitude_m);
  return (AsVector(sat.position_m) - receiver).norm() + clock_m + channel_m;
}

double PredictedRangeRate(const SatelliteState& sat, const double lat_rad,
                          const double lon_rad, const double altitude_m,
                          const Eigen::Vector3d& velocity_ned,
                          const double drift_m_s) {
  const Eigen::Vector3d receiver = ReceiverEcef(lat_rad, lon_rad, altitude_m);
  const Eigen::Vector3d line_of_sight = AsVector(sat.position_m) - receiver;
  const Eigen::Vector3d unit = line_of_sight / line_of_sight.norm();
  return (AsVector(sat.velocity_m_s) -
          NedToEcef(lat_rad, lon_rad, velocity_ned))
             .dot(unit) +
         drift_m_s;
}

// Pseudorange Jacobian over (N, E, D): the receiver is displaced in ECEF along
// the local axes (D positive down), so the derivative is the geometric one.
Eigen::Vector3d PseudorangePositionJacobian(const Ekf& ekf,
                                            const SatelliteState& sat) {
  const Eigen::Vector3d receiver =
      ReceiverEcef(ekf.latitude_rad, ekf.longitude_rad, ekf.altitude_m);
  const Eigen::Vector3d satellite = AsVector(sat.position_m);
  // Step 10 m: ranges are 2.6e7 m, so a 0.01 m step turns the 6e-9 m rounding
  // of each range into a 3e-7 error in the Jacobian.
  constexpr double kPositionFdStepM = 10.0;
  Eigen::Vector3d jacobian;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d ned = Eigen::Vector3d::Zero();
    ned(axis) = kPositionFdStepM;
    const Eigen::Vector3d step =
        NedToEcef(ekf.latitude_rad, ekf.longitude_rad, ned);
    jacobian(axis) = ((satellite - (receiver + step)).norm() -
                      (satellite - (receiver - step)).norm()) /
                     (2.0 * kPositionFdStepM);
  }
  return jacobian;
}

// Range-rate Jacobian over the NED velocity.
Eigen::Vector3d RangeRateVelocityJacobian(const Ekf& ekf,
                                          const SatelliteState& sat) {
  Eigen::Vector3d jacobian;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d plus = ekf.velocity_ned_m_s;
    Eigen::Vector3d minus = ekf.velocity_ned_m_s;
    plus(axis) += kFdStep;
    minus(axis) -= kFdStep;
    jacobian(axis) =
        (PredictedRangeRate(sat, ekf.latitude_rad, ekf.longitude_rad,
                            ekf.altitude_m, plus, 0.0) -
         PredictedRangeRate(sat, ekf.latitude_rad, ekf.longitude_rad,
                            ekf.altitude_m, minus, 0.0)) /
        (2.0 * kFdStep);
  }
  return jacobian;
}

StateVector PseudorangeRow(const Eigen::Vector3d& position, const int channel) {
  StateVector row = StateVector::Zero();
  row.segment<3>(kPosN) = position;
  row(kClockBias) = 1.0;
  row(kChannelBias0 + channel) = 1.0;
  return row;
}

StateVector RangeRateRow(const Eigen::Vector3d& velocity) {
  StateVector row = StateVector::Zero();
  row.segment<3>(kVelN) = velocity;
  row(kClockDrift) = 1.0;
  return row;
}

// Scalar Kalman posterior covariance for measurement row H and noise R.
StateMatrix ScalarPosterior(const StateMatrix& covariance,
                            const StateVector& row, const double noise) {
  const StateVector ph = covariance * row;
  const double innovation_variance = row.dot(ph) + noise;
  return covariance - ph * ph.transpose() / innovation_variance;
}

// Channel reset as the filter does it on assignment: zero cross terms, sigma^2.
StateMatrix ResetChannel(StateMatrix covariance, const int channel) {
  const Eigen::Index index = kChannelBias0 + channel;
  covariance.row(index).setZero();
  covariance.col(index).setZero();
  covariance(index, index) = kUreSigmaM * kUreSigmaM;
  return covariance;
}

// Zero-residual ranges: measured values equal the prediction at the current
// estimate (zero clock, zero channel bias), so the state does not move.
SensorFrame RangeFrame(const Ekf& ekf, const std::vector<int>& slots) {
  SensorFrame frame{};
  frame.time_s = ekf.time_s;
  frame.gps_seconds_since_epoch = kRunStartGpsSecondsSinceEpoch;
  frame.range_count = slots.size();
  for (std::size_t i = 0; i < slots.size(); ++i) {
    const SatelliteState sat =
        SatelliteAt(kBaselineSlots.at(static_cast<std::size_t>(slots[i])),
                    kRunStartGpsSecondsSinceEpoch);
    frame.ranges.at(i) = {
        .slot = slots[i],
        .pseudorange_m = PredictedPseudorange(
            sat, ekf.latitude_rad, ekf.longitude_rad, ekf.altitude_m, 0.0, 0.0),
        .range_rate_m_s =
            PredictedRangeRate(sat, ekf.latitude_rad, ekf.longitude_rad,
                               ekf.altitude_m, ekf.velocity_ned_m_s, 0.0)};
  }
  return frame;
}

Ekf RangeEkf() {
  Ekf ekf = LevelEkf();
  ekf.latitude_rad = kKpaoLatDeg * kDegree;
  ekf.longitude_rad = kKpaoLonDeg * kDegree;
  ekf.altitude_m = kKpaoMslM;
  ekf.velocity_ned_m_s = Eigen::Vector3d(3.0, -1.0, -0.5);
  return ekf;
}

// Gnss-only aiding for the range tests.
AidingUse GnssOnly() {
  return {.gnss = true, .baro = false, .magnetometer = false, .radar = false};
}

// Clock prior with position-clock and drift-velocity cross terms.
StateMatrix ClockPrior() {
  StateMatrix covariance = StateMatrix::Zero();
  covariance(kPosN, kPosN) = 4.0;
  covariance(kPosE, kPosE) = 4.0;
  covariance(kPosD, kPosD) = 4.0;
  covariance(kVelN, kVelN) = 0.25;
  covariance(kVelE, kVelE) = 0.25;
  covariance(kVelD, kVelD) = 0.25;
  covariance(kClockBias, kClockBias) = 1.0e4;
  covariance(kClockDrift, kClockDrift) = 0.01;
  covariance(kPosN, kClockBias) = 10.0;
  covariance(kClockBias, kPosN) = 10.0;
  covariance(kVelN, kClockDrift) = 0.05;
  covariance(kClockDrift, kVelN) = 0.05;
  covariance(kClockBias, kClockDrift) = 0.5;
  covariance(kClockDrift, kClockBias) = 0.5;
  return covariance;
}

double MaxAbs(const StateMatrix& matrix) {
  return matrix.cwiseAbs().maxCoeff();
}

}  // namespace

TEST_CASE(
    "[tight] pseudorange and range-rate rows match finite differences, "
    "and channel 0 resets on assignment",
    "[tight]") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = RangeEkf();
  ekf.covariance = ClockPrior();
  // Junk in channel 0 that the assignment must clear.
  ekf.covariance(kChannelBias0, kChannelBias0) = 1.0e6;
  ekf.covariance(kChannelBias0, kPosN) = 3.0;
  ekf.covariance(kPosN, kChannelBias0) = 3.0;
  ekf.covariance(kChannelBias0, kClockBias) = 7.0;
  ekf.covariance(kClockBias, kChannelBias0) = 7.0;

  const int slot = 0;
  const SatelliteState sat =
      SatelliteAt(kBaselineSlots.at(slot), kRunStartGpsSecondsSinceEpoch);
  const StateVector pseudorange_row =
      PseudorangeRow(PseudorangePositionJacobian(ekf, sat), 0);
  const StateVector rate_row =
      RangeRateRow(RangeRateVelocityJacobian(ekf, sat));
  const StateMatrix expected = ScalarPosterior(
      ScalarPosterior(ResetChannel(ekf.covariance, 0), pseudorange_row,
                      kPseudorangeWhiteVarianceM2),
      rate_row, kRangeRateSigmaMPerS * kRangeRateSigmaMPerS);

  const SensorFrame frame = RangeFrame(ekf, {slot});
  AidEkf(config, frame, GnssOnly(), ekf);

  INFO("max |P - expected| = "
       << (ekf.covariance - expected).cwiseAbs().maxCoeff());
  CHECK(ekf.channels.at(0).slot == slot);
  CHECK((ekf.covariance - expected).cwiseAbs().maxCoeff() <=
        kPosteriorRtol * std::max(1.0, MaxAbs(expected)));
}

TEST_CASE("[tight] a slot that leaves the frame frees its channel", "[tight]") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = RangeEkf();
  ekf.channels.at(0).slot = 3;
  ekf.channels.at(1).slot = 7;
  ekf.covariance = ClockPrior();
  for (Eigen::Index index = 0; index < kStateDim; ++index) {
    ekf.covariance(kChannelBias0, index) = 1.0 + static_cast<double>(index);
    ekf.covariance(index, kChannelBias0) = 1.0 + static_cast<double>(index);
  }

  AidEkf(config, RangeFrame(ekf, {7}), GnssOnly(), ekf);

  CHECK(ekf.channels.at(0).slot == -1);
  CHECK(ekf.channels.at(1).slot == 7);
  CHECK(ekf.covariance.row(kChannelBias0).cwiseAbs().maxCoeff() <= 1e-12);
  CHECK(ekf.covariance.col(kChannelBias0).cwiseAbs().maxCoeff() <= 1e-12);
}

TEST_CASE(
    "[tight] channel map: {3,7} then {7,12} frees 3, assigns 12 to the free "
    "channel",
    "[tight]") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = RangeEkf();
  ekf.covariance = ClockPrior();

  AidEkf(config, RangeFrame(ekf, {3, 7}), GnssOnly(), ekf);
  CHECK(ekf.channels.at(0).slot == 3);
  CHECK(ekf.channels.at(1).slot == 7);

  AidEkf(config, RangeFrame(ekf, {7, 12}), GnssOnly(), ekf);
  CHECK(ekf.channels.at(0).slot == 12);
  CHECK(ekf.channels.at(1).slot == 7);
  for (std::size_t channel = 2; channel < glidepath::fsw::kMaxChannels;
       ++channel) {
    CHECK(ekf.channels.at(channel).slot == -1);
  }
}

TEST_CASE("[tight] clock covariance after one predict equals the closed-form Q",
          "[tight]") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = LevelEkf();
  PredictEkf(config, LevelImu(kSomiglianaEquator), kPredictDtS, ekf);

  const double dt = kPredictDtS;
  const double q_bias =
      kClockWhiteFmM2PerS * dt + kClockRandomWalkFmM2PerS3 * dt * dt * dt / 3.0;
  const double q_cross = kClockRandomWalkFmM2PerS3 * dt * dt / 2.0;
  const double q_drift = kClockRandomWalkFmM2PerS3 * dt;
  CHECK(std::abs(ekf.covariance(kClockBias, kClockBias) - q_bias) <=
        1e-12 * q_bias);
  CHECK(std::abs(ekf.covariance(kClockBias, kClockDrift) - q_cross) <=
        1e-12 * q_cross);
  CHECK(std::abs(ekf.covariance(kClockDrift, kClockDrift) - q_drift) <=
        1e-12 * q_drift);
}

TEST_CASE("[tight] pseudorange residual raises the clock bias", "[tight]") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = RangeEkf();
  ekf.covariance = ClockPrior();
  SensorFrame frame = RangeFrame(ekf, {0});
  frame.ranges[0].pseudorange_m += 5.0;
  AidEkf(config, frame, GnssOnly(), ekf);
  CHECK(ekf.clock_bias_m > 0.0);
}

TEST_CASE("[tight] range-rate residual raises the clock drift", "[tight]") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = RangeEkf();
  ekf.covariance = ClockPrior();
  SensorFrame frame = RangeFrame(ekf, {0});
  frame.ranges[0].range_rate_m_s += 0.5;
  AidEkf(config, frame, GnssOnly(), ekf);
  CHECK(ekf.clock_drift_m_s > 0.0);
}

TEST_CASE(
    "[tight] InitEkf sets the clock bias from the ranges and assigns channels "
    "in frame order",
    "[tight]") {
  SensorFrame frame = InitializedFrame();
  frame.gps_seconds_since_epoch = kRunStartGpsSecondsSinceEpoch;
  frame.range_count = 2;
  frame.ranges[0] = {.slot = 5, .pseudorange_m = 0.0, .range_rate_m_s = 0.0};
  frame.ranges[1] = {.slot = 9, .pseudorange_m = 0.0, .range_rate_m_s = 0.0};
  const std::array<double, 2> bias_m{42.0, 44.0};
  constexpr double kInitAltitudeM =
      500.0;  // Baro altitude of InitializedFrame.
  for (std::size_t i = 0; i < 2; ++i) {
    const SatelliteState sat = SatelliteAt(
        kBaselineSlots.at(static_cast<std::size_t>(frame.ranges[i].slot)),
        kRunStartGpsSecondsSinceEpoch);
    frame.ranges[i].pseudorange_m =
        PredictedPseudorange(sat, 0.5, -1.2, kInitAltitudeM, 0.0, 0.0) +
        bias_m[i];
  }
  const std::optional<Ekf> ekf = InitEkf(TestConfig(kDefaultSpecs), frame);
  REQUIRE(ekf.has_value());
  CHECK(ekf->clock_bias_m == Catch::Approx(43.0).margin(1e-6));
  CHECK(ekf->clock_drift_m_s == 0.0);
  CHECK(ekf->channels.at(0).slot == 5);
  CHECK(ekf->channels.at(1).slot == 9);
}

TEST_CASE("ekf: baro high pressure residual pulls altitude up") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = InitReferenceFilter().value();
  const double before = ekf.altitude_m;
  SensorFrame frame = InitializedFrame();
  frame.time_s = ekf.time_s + 0.025;
  frame.baro.pressure_pa = glidepath::fsw::IsaPressurePa(ekf.altitude_m + 5.0);
  AidingUse aiding{
      .gnss = false, .baro = true, .magnetometer = false, .radar = false};
  const Innovations innovations = AidEkf(config, frame, aiding, ekf);
  CHECK(innovations[kSlotBaro].residual > 0.0);
  CHECK(ekf.altitude_m > before);
}

TEST_CASE("ekf: radar high height residual pulls altitude up") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = InitReferenceFilter().value();
  ekf.altitude_m = config.terrain_elevation_m + 10.0;
  const double before = ekf.altitude_m;
  SensorFrame frame = InitializedFrame();
  frame.time_s = ekf.time_s + 0.025;
  frame.radar = {.valid = true, .height_m = 10.5};
  AidingUse aiding{
      .gnss = false, .baro = false, .magnetometer = false, .radar = true};
  const Innovations innovations = AidEkf(config, frame, aiding, ekf);
  CHECK(innovations[kSlotRadar].residual > 0.0);
  CHECK(ekf.altitude_m > before);
}

TEST_CASE("ekf: magnetometer from heading ahead raises heading") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = InitReferenceFilter().value();
  const double before = HeadingRad(ekf.attitude);
  const Eigen::Quaterniond true_attitude =
      AttitudeFromEuler(0.0, 0.0, before + 2.0 * kDegree);
  const Eigen::Vector3d mag = MagBody(true_attitude);
  SensorFrame frame = InitializedFrame();
  frame.time_s = ekf.time_s + 0.025;
  frame.mag.field_nt = {mag.x(), mag.y(), mag.z()};
  AidingUse aiding{
      .gnss = false, .baro = false, .magnetometer = true, .radar = false};
  const Innovations innovations = AidEkf(config, frame, aiding, ekf);
  const double after = HeadingRad(ekf.attitude);
  CHECK(innovations[kSlotMagX].variance > 0.0);
  CHECK(after > before);
  CHECK(after < before + 2.0 * kDegree);
}

TEST_CASE("ekf: AidingUse false leaves state and slots unchanged") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = InitReferenceFilter().value();
  Ekf before = ekf;
  SensorFrame frame = InitializedFrame();
  frame.time_s = ekf.time_s + 0.025;
  const glidepath::fsw::LocalPoint offset{.north_m = 20.0, .east_m = 5.0};
  const glidepath::fsw::GeoPoint fix = ToGeo(
      offset,
      {.latitude_rad = ekf.latitude_rad, .longitude_rad = ekf.longitude_rad});
  frame.gnss.latitude_rad = fix.latitude_rad;
  frame.gnss.longitude_rad = fix.longitude_rad;
  AidingUse aiding{};
  aiding.gnss = false;
  const Innovations innovations = AidEkf(config, frame, aiding, ekf);
  CHECK(innovations[kSlotGnssNorth].residual == 0.0);
  CHECK(innovations[kSlotGnssNorth].variance == 0.0);
  CHECK(ekf.latitude_rad == Catch::Approx(before.latitude_rad).margin(0.0));
  CHECK(ekf.longitude_rad == Catch::Approx(before.longitude_rad).margin(0.0));
}

TEST_CASE("ekf: Nis matches residual squared over variance") {
  const Innovation with_signal{.residual = 2.0, .variance = 4.0};
  const Innovation zero_variance{.residual = 2.0, .variance = 0.0};
  CHECK(Nis(with_signal) == Catch::Approx(1.0).margin(0.0));
  CHECK(Nis(zero_variance) == Catch::Approx(0.0).margin(0.0));
}

TEST_CASE("ekf: PredictEkf and AidEkf allocate nothing over 4000 calls") {
  const EkfConfig config = TestConfig(kDefaultSpecs);
  Ekf ekf = InitReferenceFilter().value();
  SensorFrame frame = InitializedFrame();
  const ImuSample imu = frame.imu;
  const double dt_s = 1.0 / kControlRateHz;
  const std::size_t before = AllocationCount();
  for (int i = 0; i < 4000; ++i) {
    frame.time_s += dt_s;
    PredictEkf(config, imu, dt_s, ekf);
    (void)AidEkf(config, frame, AidingUse{}, ekf);
  }
  CHECK(AllocationCount() == before);
}