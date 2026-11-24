// Error-state layout, innovation slots and snapshot storage (no Eigen).
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace glidepath::fsw {

inline constexpr std::ptrdiff_t kStateDim = 27;
inline constexpr std::ptrdiff_t kPosN = 0;
inline constexpr std::ptrdiff_t kPosE = 1;
inline constexpr std::ptrdiff_t kPosD = 2;
inline constexpr std::ptrdiff_t kVelN = 3;
inline constexpr std::ptrdiff_t kVelE = 4;
inline constexpr std::ptrdiff_t kVelD = 5;
inline constexpr std::ptrdiff_t kAttN = 6;
inline constexpr std::ptrdiff_t kAttE = 7;
inline constexpr std::ptrdiff_t kAttD = 8;
inline constexpr std::ptrdiff_t kGyroBiasX = 9;
inline constexpr std::ptrdiff_t kGyroBiasY = 10;
inline constexpr std::ptrdiff_t kGyroBiasZ = 11;
inline constexpr std::ptrdiff_t kAccelBiasX = 12;
inline constexpr std::ptrdiff_t kAccelBiasY = 13;
inline constexpr std::ptrdiff_t kAccelBiasZ = 14;
// Receiver clock (m, m/s) and one range-bias state per GNSS channel (m).
inline constexpr std::ptrdiff_t kClockBias = 15;
inline constexpr std::ptrdiff_t kClockDrift = 16;
inline constexpr std::ptrdiff_t kChannelBias0 = 17;

inline constexpr std::size_t kCovarianceSize = 729;
static_assert(kCovarianceSize ==
              static_cast<std::size_t>(kStateDim * kStateDim));

inline constexpr std::size_t kSlotGnssNorth = 0;
inline constexpr std::size_t kSlotGnssEast = 1;
inline constexpr std::size_t kSlotGnssVelNorth = 2;
inline constexpr std::size_t kSlotGnssVelEast = 3;
inline constexpr std::size_t kSlotGnssVelDown = 4;
inline constexpr std::size_t kSlotBaro = 5;
inline constexpr std::size_t kSlotMagX = 6;
inline constexpr std::size_t kSlotMagY = 7;
inline constexpr std::size_t kSlotMagZ = 8;
inline constexpr std::size_t kSlotRadar = 9;
inline constexpr std::size_t kInnovationSlots = 10;

struct Innovation {
  double residual;
  double variance;  // S = H P H' + R; zero means not applied.
};

using Innovations = std::array<Innovation, kInnovationSlots>;

inline double Nis(const Innovation& innovation) noexcept {
  return innovation.variance > 0.0
             ? innovation.residual * innovation.residual / innovation.variance
             : 0.0;
}

struct EkfSnapshot {
  bool valid;
  std::array<double, kCovarianceSize> covariance;  // Row-major P.
  Innovations innovations;
};

}  // namespace glidepath::fsw