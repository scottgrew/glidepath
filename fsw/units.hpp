// Unit conversions and constants, defined once for all of glidepath.
#pragma once
#include <numbers>

namespace glidepath::fsw {

inline constexpr double kFoot = 0.3048;                      // m per ft.
inline constexpr double kKnot = 1852.0 / 3600.0;             // m/s per kt.
inline constexpr double kNauticalMile = 1852.0;              // m per nm.
inline constexpr double kDegree = std::numbers::pi / 180.0;  // rad per deg.
inline constexpr double kGravity = 9.80665;                  // m/s^2.

}  // namespace glidepath::fsw