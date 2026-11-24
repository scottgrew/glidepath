#include "fsw/atmosphere.hpp"

#include <cmath>

namespace glidepath::fsw {
namespace {

constexpr double kSeaLevelTemperatureK = 288.15;
constexpr double kLapseRateKPerM = 0.0065;
constexpr double kSeaLevelPressurePa = 101325.0;
constexpr double kTemperatureExponent = 5.255876;
constexpr double kGasConstant = 287.05287;

double TroposphereTemperatureK(const double altitude_m) noexcept {
  return kSeaLevelTemperatureK - kLapseRateKPerM * altitude_m;
}

}  // namespace

double IsaPressurePa(const double altitude_m) noexcept {
  const double temperature_k = TroposphereTemperatureK(altitude_m);
  return kSeaLevelPressurePa *
         std::pow(temperature_k / kSeaLevelTemperatureK, kTemperatureExponent);
}

double IsaAltitudeM(const double pressure_pa) noexcept {
  const double temperature_k =
      kSeaLevelTemperatureK *
      std::pow(pressure_pa / kSeaLevelPressurePa, 1.0 / kTemperatureExponent);
  return (kSeaLevelTemperatureK - temperature_k) / kLapseRateKPerM;
}

double IsaDensityKgM3(const double altitude_m) noexcept {
  const double temperature_k = TroposphereTemperatureK(altitude_m);
  const double pressure_pa = IsaPressurePa(altitude_m);
  return pressure_pa / (kGasConstant * temperature_k);
}

}  // namespace glidepath::fsw