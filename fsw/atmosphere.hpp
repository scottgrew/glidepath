// International Standard Atmosphere (troposphere) for baro aiding.
#pragma once

namespace glidepath::fsw {

inline constexpr double kSeaLevelDensityKgM3 = 1.225;

double IsaPressurePa(double altitude_m) noexcept;
double IsaAltitudeM(double pressure_pa) noexcept;
double IsaDensityKgM3(double altitude_m) noexcept;

}  // namespace glidepath::fsw