// Noise, sample rates and magnetic field for the sensor model and EKF.
#pragma once
#include "fsw/atmosphere.hpp"
#include "fsw/measurements.hpp"
#include "fsw/types.hpp"
#include "fsw/units.hpp"

#include <array>
#include <cmath>

namespace glidepath::fsw {

struct SensorSpecs {
  double gyro_density_rad_s_per_sqrt_hz;
  double accel_density_m_s2_per_sqrt_hz;
  double gyro_bias_sigma_rad_s;
  double accel_bias_sigma_m_s2;
  double bias_tau_s;
  double gnss_horizontal_sigma_m;
  double gnss_velocity_sigma_m_s;
  double baro_pressure_sigma_pa;
  double magnetometer_sigma_nt;
  double pitot_pressure_sigma_pa;
  double radar_sigma_m;
  double radar_sigma_below_1m_m;
  double radar_min_valid_height_m;
  double radar_max_valid_height_m;
  double imu_rate_hz;
  double gnss_rate_hz;
  double baro_rate_hz;
  double mag_rate_hz;
  double pitot_rate_hz;
  double radar_rate_hz;
  Vec3 magnetic_field_ned_nt;
};

// WMM2025 (NOAA/BGS, WMM_2025.COF via pygeomag 1.1.0) at 37.4611° N,
// 122.1150° W, 2 m, epoch 2026.89: N 22426.3, E 5058.2, D 41205.2 nT
// (F 47184.7 nT, declination 12.71° E, inclination 60.84°).
inline constexpr Vec3 kMagneticFieldNedNt = {22426.3, 5058.2, 41205.2};

inline constexpr double kGyroDensityDegPerSecPerRtHz = 0.0028;
inline constexpr double kGyroBiasSigmaDegPerSec = 0.010;
inline constexpr double kAccelDensityMicroGPerRtHz = 70.0;
inline constexpr double kAccelBiasSigmaMPerSec2 = 2.94e-3;
inline constexpr double kBiasTauSeconds = 300.0;
inline constexpr double kGnssHorizontalCepM = 2.0;
inline constexpr double kCepToSigma = 1.1774;
inline constexpr double kGnssVelocitySigmaMPerSec = 0.05;
inline constexpr double kBaroPressureSigmaPa = 8.0;
inline constexpr double kMagnetometerSigmaNt = 15.0;
inline constexpr double kPitotPressureSigmaPa = 1.0;
inline constexpr double kRadarSigmaM = 0.04;
inline constexpr double kRadarSigmaBelow1mM = 0.06;
inline constexpr double kRadarMinValidHeightM = 0.5;
inline constexpr double kRadarMaxValidHeightM = 50.0;
// Tachometer noise, rpm: typical digital tachometer resolution.
inline constexpr double kTachometerSigmaRpm = 10.0;

// GNSS error model for the tightly coupled filter (G11c). SPS PS = GPS SPS
// Performance Standard, 5th ed. 2020.
// Receiver clock: S_f = c^2 h0 / 2 and S_g = 2 pi^2 c^2 h-2 with a typical
// low-quality TCXO (h0 = 2e-19 s, h-2 = 2e-20 1/s); GNSS-Compare documentation,
// not peer reviewed.
inline constexpr double kClockWhiteFmM2PerS = 8.99e-3;
inline constexpr double kClockRandomWalkFmM2PerS3 = 3.55e-2;
// SIS URE 7.0 m at 95% over all AODs, SPS PS Table 3.4-1, p. 44; the 1-sigma
// value is the 95% value over 1.96.
inline constexpr double kUreSigmaM = 7.0 / 1.96;
// [U] SIS error decorrelates over hours; chosen, not sourced.
inline constexpr double kUreCorrelationTimeS = 1800.0;
// Receiver noise and multipath, SPS PS Table A.4-2 (illustrative). The white
// pseudorange variance is the sum of their squares.
inline constexpr double kPseudorangeWhiteVarianceM2 =
    (2.9 * 2.9 + 2.4 * 2.4) / (1.96 * 1.96);
// [U] Receiver Doppler noise.
inline constexpr double kRangeRateSigmaMPerS = 0.05;
// [U] Initial receiver clock bias and drift, typical TCXO (sim and filter).
inline constexpr double kInitialClockBiasSigmaM = 100.0;
inline constexpr double kInitialClockDriftSigmaMPerS = 0.1;

inline constexpr SensorSpecs kDefaultSpecs{
    .gyro_density_rad_s_per_sqrt_hz = kGyroDensityDegPerSecPerRtHz * kDegree,
    .accel_density_m_s2_per_sqrt_hz =
        kAccelDensityMicroGPerRtHz * 1.0e-6 * kGravity,
    .gyro_bias_sigma_rad_s = kGyroBiasSigmaDegPerSec * kDegree,
    .accel_bias_sigma_m_s2 = kAccelBiasSigmaMPerSec2,
    .bias_tau_s = kBiasTauSeconds,
    .gnss_horizontal_sigma_m = kGnssHorizontalCepM / kCepToSigma,
    .gnss_velocity_sigma_m_s = kGnssVelocitySigmaMPerSec,
    .baro_pressure_sigma_pa = kBaroPressureSigmaPa,
    .magnetometer_sigma_nt = kMagnetometerSigmaNt,
    .pitot_pressure_sigma_pa = kPitotPressureSigmaPa,
    .radar_sigma_m = kRadarSigmaM,
    .radar_sigma_below_1m_m = kRadarSigmaBelow1mM,
    .radar_min_valid_height_m = kRadarMinValidHeightM,
    .radar_max_valid_height_m = kRadarMaxValidHeightM,
    .imu_rate_hz = kControlRateHz,
    .gnss_rate_hz = 10.0,
    .baro_rate_hz = 20.0,
    .mag_rate_hz = 20.0,
    .pitot_rate_hz = 20.0,
    .radar_rate_hz = 40.0,
    .magnetic_field_ned_nt = kMagneticFieldNedNt,
};

inline double ImuSampleIntervalS() noexcept {
  return 1.0 / kControlRateHz;
}

inline double GyroSampleSigmaRadS(const SensorSpecs& specs) noexcept {
  return specs.gyro_density_rad_s_per_sqrt_hz / std::sqrt(kControlRateHz);
}

inline double AccelSampleSigmaMPerSec2(const SensorSpecs& specs) noexcept {
  return specs.accel_density_m_s2_per_sqrt_hz / std::sqrt(kControlRateHz);
}

inline double BaroAltitudeSigmaM(const double altitude_m) noexcept {
  const double gravity = 9.80665;
  const double density = IsaDensityKgM3(altitude_m);
  return kBaroPressureSigmaPa / (density * gravity);
}

}  // namespace glidepath::fsw