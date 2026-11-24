#include "fsw/geodesy.hpp"

#include <cmath>

namespace glidepath::fsw {
namespace {

struct Radii {
  double meridian_m;
  double prime_vertical_m;
};

Radii RadiiAt(const double latitude_rad) noexcept {
  constexpr double kSemiMajorAxisM = 6378137.0;
  constexpr double kEccentricitySquared = 6.69437999014e-3;
  const double sin_lat = std::sin(latitude_rad);
  const double w = std::sqrt(1.0 - kEccentricitySquared * sin_lat * sin_lat);
  return {.meridian_m =
              kSemiMajorAxisM * (1.0 - kEccentricitySquared) / (w * w * w),
          .prime_vertical_m = kSemiMajorAxisM / w};
}

}  // namespace

LocalPoint ToLocal(const GeoPoint& point, const GeoPoint& origin) noexcept {
  const Radii radii = RadiiAt(origin.latitude_rad);
  const double cos_lat0 = std::cos(origin.latitude_rad);
  return {
      .north_m = (point.latitude_rad - origin.latitude_rad) * radii.meridian_m,
      .east_m = (point.longitude_rad - origin.longitude_rad) *
                radii.prime_vertical_m * cos_lat0};
}

GeoPoint ToGeo(const LocalPoint& point, const GeoPoint& origin) noexcept {
  const Radii radii = RadiiAt(origin.latitude_rad);
  const double cos_lat0 = std::cos(origin.latitude_rad);
  return {
      .latitude_rad = origin.latitude_rad + point.north_m / radii.meridian_m,
      .longitude_rad = origin.longitude_rad +
                       point.east_m / (radii.prime_vertical_m * cos_lat0)};
}

}  // namespace glidepath::fsw