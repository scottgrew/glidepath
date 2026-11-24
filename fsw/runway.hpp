// Runway geometry from published survey data.
#pragma once
#include "fsw/geodesy.hpp"
#include "fsw/units.hpp"

namespace glidepath::fsw {

struct Runway {
  GeoPoint threshold;  // Where the landing direction begins.
  GeoPoint far_end;
  double elevation_m;  // Threshold elevation above mean sea level.
};

// Palo Alto (KPAO) runway 31, from the FAA's surveyed runway ends (effective
// 29 October 2026): threshold 37-27.509528N 122-06.746287W at 6.7 ft, far
// end (runway 13 threshold) 37-27.825037N 122-07.059308W.
inline constexpr Runway kPaloAlto31{
    .threshold = {.latitude_rad = (37.0 + 27.509528 / 60.0) * kDegree,
                  .longitude_rad = -(122.0 + 6.746287 / 60.0) * kDegree},
    .far_end = {.latitude_rad = (37.0 + 27.825037 / 60.0) * kDegree,
                .longitude_rad = -(122.0 + 7.059308 / 60.0) * kDegree},
    .elevation_m = 6.7 * kFoot};

}  // namespace glidepath::fsw