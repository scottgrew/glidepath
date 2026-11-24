// Positions on the Earth and a flat local north/east frame for guidance. The
// frame uses the WGS84 radii at its origin, so it is most accurate near the
// origin; guidance puts the origin at the runway threshold.
#pragma once

namespace glidepath::fsw {

struct GeoPoint {
  double latitude_rad;  // Geodetic.
  double longitude_rad;
};

struct LocalPoint {
  double north_m;
  double east_m;
};

LocalPoint ToLocal(const GeoPoint& point, const GeoPoint& origin) noexcept;
GeoPoint ToGeo(const LocalPoint& point, const GeoPoint& origin) noexcept;

}  // namespace glidepath::fsw