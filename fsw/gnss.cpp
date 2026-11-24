#include "fsw/gnss.hpp"

#include "fsw/geodesy.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace glidepath::fsw {
namespace {

constexpr double kSecondsPerDay = 86400.0;
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84EccentricitySq =
    kWgs84Flattening * (2.0 - kWgs84Flattening);
constexpr double kHalfTurnDeg = 180.0;

constexpr double DegToRad(double degrees) noexcept {
  return degrees * std::numbers::pi / kHalfTurnDeg;
}

// Rotation about z by -angle (ECI to ECEF) applied to a vector.
Ecef RotateToEcef(const Ecef& v, double angle_rad) noexcept {
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  return {c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]};
}

}  // namespace

SatelliteState SatelliteAt(const GpsSlot& slot,
                           const double seconds_since_epoch) noexcept {
  const double a = kGpsSemiMajorAxisM;
  const double mean_motion = std::sqrt(kGpsMu / (a * a * a));
  const double inclination = DegToRad(kGpsInclinationDeg);
  const double cos_i = std::cos(inclination);
  const double sin_i = std::sin(inclination);
  const double u = DegToRad(slot.argument_of_latitude_deg) +
                   mean_motion * seconds_since_epoch;
  const double raan =
      DegToRad(slot.raan_deg) +
      DegToRad(kGpsNodalRateDegPerDay) * (seconds_since_epoch / kSecondsPerDay);
  const double raan_rate =
      DegToRad(kGpsNodalRateDegPerDay) / kSecondsPerDay;  // rad/s.
  const double cos_o = std::cos(raan);
  const double sin_o = std::sin(raan);
  const double cos_u = std::cos(u);
  const double sin_u = std::sin(u);

  // ECI position and its time derivative (u advances at n, RAAN at raan_rate).
  const Ecef eci_position = {a * (cos_o * cos_u - sin_o * sin_u * cos_i),
                             a * (sin_o * cos_u + cos_o * sin_u * cos_i),
                             a * sin_u * sin_i};
  const Ecef d_du = {a * (-cos_o * sin_u - sin_o * cos_u * cos_i),
                     a * (-sin_o * sin_u + cos_o * cos_u * cos_i),
                     a * cos_u * sin_i};
  const Ecef d_draan = {a * (-sin_o * cos_u - cos_o * sin_u * cos_i),
                        a * (cos_o * cos_u - sin_o * sin_u * cos_i), 0.0};
  const Ecef eci_velocity = {mean_motion * d_du[0] + raan_rate * d_draan[0],
                             mean_motion * d_du[1] + raan_rate * d_draan[1],
                             mean_motion * d_du[2]};

  const double theta = DegToRad(kEpochGreenwichHourAngleDeg) +
                       kEarthRateRadS * seconds_since_epoch;
  const Ecef ecef_position = RotateToEcef(eci_position, theta);
  // ECEF velocity = Rz(-theta) v_eci - w x r_ecef, w = (0, 0, w_e).
  const Ecef rotated_velocity = RotateToEcef(eci_velocity, theta);
  const Ecef ecef_velocity = {
      rotated_velocity[0] + kEarthRateRadS * ecef_position[1],
      rotated_velocity[1] - kEarthRateRadS * ecef_position[0],
      rotated_velocity[2]};
  return {.position_m = ecef_position, .velocity_m_s = ecef_velocity};
}

Ecef GeodeticToEcef(const GeoPoint& site,
                    const double ellipsoid_height_m) noexcept {
  const double latitude_rad = site.latitude_rad;
  const double longitude_rad = site.longitude_rad;
  const double sin_lat = std::sin(latitude_rad);
  const double cos_lat = std::cos(latitude_rad);
  const double prime_vertical =
      kWgs84SemiMajorAxisM /
      std::sqrt(1.0 - kWgs84EccentricitySq * sin_lat * sin_lat);
  return {
      (prime_vertical + ellipsoid_height_m) * cos_lat * std::cos(longitude_rad),
      (prime_vertical + ellipsoid_height_m) * cos_lat * std::sin(longitude_rad),
      (prime_vertical * (1.0 - kWgs84EccentricitySq) + ellipsoid_height_m) *
          sin_lat};
}

LookAngles Look(const Ecef& receiver, const GeoPoint& site,
                const Ecef& satellite) noexcept {
  const double latitude_rad = site.latitude_rad;
  const double longitude_rad = site.longitude_rad;
  const double dx = satellite[0] - receiver[0];
  const double dy = satellite[1] - receiver[1];
  const double dz = satellite[2] - receiver[2];
  const double sin_lat = std::sin(latitude_rad);
  const double cos_lat = std::cos(latitude_rad);
  const double sin_lon = std::sin(longitude_rad);
  const double cos_lon = std::cos(longitude_rad);

  const double east = -sin_lon * dx + cos_lon * dy;
  const double north =
      -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
  const double up =
      cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

  const double horizontal = std::hypot(east, north);
  double azimuth = std::atan2(east, north);
  if (azimuth < 0.0) {
    azimuth += 2.0 * std::numbers::pi;
  }
  return {.elevation_rad = std::atan2(up, horizontal),
          .azimuth_rad = azimuth,
          .range_m = std::sqrt(dx * dx + dy * dy + dz * dz)};
}

std::size_t VisibleSlots(const Ecef& receiver, const GeoPoint& site,
                         const double seconds_since_epoch,
                         std::array<int, 24>& out) {
  const double mask_rad = DegToRad(kElevationMaskDeg);
  std::size_t count = 0;
  for (std::size_t i = 0; i < kBaselineSlots.size(); ++i) {
    const Ecef satellite =
        SatelliteAt(kBaselineSlots.at(i), seconds_since_epoch).position_m;
    const LookAngles look = Look(receiver, site, satellite);
    if (look.elevation_rad >= mask_rad) {
      out.at(count) = static_cast<int>(i);
      ++count;
    }
  }
  return count;
}

}  // namespace glidepath::fsw
