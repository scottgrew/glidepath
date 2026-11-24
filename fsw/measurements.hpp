// Sensor sample types for the error-state EKF and the sensor model.
#pragma once
#include <array>
#include <cmath>
#include <cstddef>

namespace glidepath::fsw {

using Vec3 = std::array<double, 3>;

struct ImuSample {
  bool valid;
  Vec3 gyro_rad_s;
  Vec3 accel_m_s2;  // Specific force, body axes.
};

struct GnssSample {
  bool valid;
  double latitude_rad;
  double longitude_rad;
  double north_m_s;
  double east_m_s;
  double down_m_s;  // No vertical position measurement.
};

struct BaroSample {
  bool valid;
  double pressure_pa;
};

struct MagSample {
  bool valid;
  Vec3 field_nt;
};

struct PitotSample {
  bool valid;
  double differential_pressure_pa;
};

struct RadarSample {
  bool valid;
  double height_m;  // Vertical height over the runway plane.
};

struct TachSample {
  bool valid;
  double rpm;
};

// Number of GNSS channels, the measured visible maximum at KPAO over a sidereal
// day.
inline constexpr std::size_t kMaxChannels = 10;

// One satellite's pseudorange and range rate for one channel.
struct RangeSample {
  int slot;
  double pseudorange_m;
  double range_rate_m_s;
};

struct SensorFrame {
  double time_s{};
  ImuSample imu{};
  GnssSample gnss{};
  BaroSample baro{};
  MagSample mag{};
  PitotSample pitot{};
  RadarSample radar{};
  TachSample tach{};
  bool main_gear_on_ground{};
  bool nose_gear_on_ground{};  // Squat switches: no noise.
  std::array<RangeSample, kMaxChannels> ranges{};
  std::size_t range_count{};
  double gps_seconds_since_epoch{};
};

inline bool IsFinite(const double value) noexcept {
  return std::isfinite(value);
}

inline bool IsUsable(const Vec3& vector) noexcept {
  return IsFinite(vector[0]) && IsFinite(vector[1]) && IsFinite(vector[2]);
}

inline bool IsUsable(const ImuSample& sample) noexcept {
  return sample.valid && IsUsable(sample.gyro_rad_s) &&
         IsUsable(sample.accel_m_s2);
}

inline bool IsUsable(const GnssSample& sample) noexcept {
  return sample.valid && IsFinite(sample.latitude_rad) &&
         IsFinite(sample.longitude_rad) && IsFinite(sample.north_m_s) &&
         IsFinite(sample.east_m_s) && IsFinite(sample.down_m_s);
}

inline bool IsUsable(const BaroSample& sample) noexcept {
  return sample.valid && IsFinite(sample.pressure_pa);
}

inline bool IsUsable(const MagSample& sample) noexcept {
  return sample.valid && IsUsable(sample.field_nt);
}

inline bool IsUsable(const PitotSample& sample) noexcept {
  return sample.valid && IsFinite(sample.differential_pressure_pa);
}

inline bool IsUsable(const RadarSample& sample) noexcept {
  return sample.valid && IsFinite(sample.height_m);
}

inline bool IsUsable(const TachSample& sample) noexcept {
  return sample.valid && IsFinite(sample.rpm);
}

}  // namespace glidepath::fsw