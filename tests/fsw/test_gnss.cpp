// Oracles for GPS constellation geometry. Expected values come from the SPS
// PS (Tables 3.2-1 and 3.2-3, GPS SPS PS 5th ed. 2020), IS-GPS-200 constants
// and WGS84, written out here independently of the implementation.
#include "fsw/geodesy.hpp"
#include "fsw/gnss.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <utility>

namespace {

using glidepath::fsw::Ecef;
using glidepath::fsw::GeodeticToEcef;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::kBaselineSlots;
using glidepath::fsw::Look;
using glidepath::fsw::LookAngles;
using glidepath::fsw::SatelliteAt;
using glidepath::fsw::SatelliteState;
using glidepath::fsw::VisibleSlots;

constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr double kSemiMajorAxisM = 26'559'800.0;  // Table 3.2-3, 26,559.8 km.
constexpr double kNodalRateDegPerDay = -0.0402;   // Table 3.2-3.
constexpr double kInclinationRad = 55.0 * kDegToRad;  // Table 3.2-3.
constexpr double kGha0Rad = 100.765 * kDegToRad;      // Table 3.2-1 notes.
constexpr double kEarthRateRadS = 7.2921151467e-5;    // IS-GPS-200.
constexpr double kGpsMu = 3.986005e14;                // IS-GPS-200.
constexpr double kHalfWidthS = 0.05;  // Finite-difference half step.
constexpr double kSiderealDayS = 86164.0;
constexpr double kVisibleStepS = 30.0;
constexpr double kKpaoLatitudeRad = 37.4611 * kDegToRad;
constexpr double kKpaoLongitudeRad = -122.1150 * kDegToRad;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

double Norm(const Ecef& v) noexcept {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Ecef Minus(const Ecef& a, const Ecef& b) noexcept {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

}  // namespace

TEST_CASE("every slot orbits at the semi-major axis with the circular speed",
          "[gnss]") {
  for (const double t : {0.0, 3600.0, 43082.0}) {
    for (const auto& slot : kBaselineSlots) {
      const SatelliteState sat = SatelliteAt(slot, t);
      REQUIRE(std::abs(Norm(sat.position_m) - kSemiMajorAxisM) < 1e-6);
      // ECI velocity = ECEF velocity + w x r, with w = (0, 0, w_e).
      const Ecef& r = sat.position_m;
      const Ecef& v = sat.velocity_m_s;
      const Ecef eci = {v[0] - kEarthRateRadS * r[1],
                        v[1] + kEarthRateRadS * r[0], v[2]};
      // In-plane speed is sqrt(mu / a); the nodal regression (Table 3.2-3)
      // adds at most a * |RAAN rate| (0.216 m/s), so the speed sits within
      // that of the circular speed. The finite-difference test checks the
      // exact velocity.
      const double circular_speed = std::sqrt(kGpsMu / kSemiMajorAxisM);
      const double nodal_bound = kSemiMajorAxisM *
                                 std::abs(kNodalRateDegPerDay) *
                                 (std::numbers::pi / 180.0) / 86400.0;
      REQUIRE(std::abs(Norm(eci) - circular_speed) <= nodal_bound + 1e-6);
    }
  }
}

TEST_CASE("slot A1 at epoch matches the independent ECI-to-ECEF formula",
          "[gnss]") {
  const double raan = 288.85 * kDegToRad;
  const double u = 239.54 * kDegToRad;
  const double x = std::cos(raan) * std::cos(u) -
                   std::sin(raan) * std::sin(u) * std::cos(kInclinationRad);
  const double y = std::sin(raan) * std::cos(u) +
                   std::cos(raan) * std::sin(u) * std::cos(kInclinationRad);
  const double z = std::sin(u) * std::sin(kInclinationRad);
  const double c = std::cos(kGha0Rad);
  const double s = std::sin(kGha0Rad);
  const Ecef expected = {kSemiMajorAxisM * (c * x + s * y),
                         kSemiMajorAxisM * (-s * x + c * y),
                         kSemiMajorAxisM * z};

  const Ecef actual = SatelliteAt(kBaselineSlots[0], 0.0).position_m;

  for (std::size_t i = 0; i < 3; ++i) {
    REQUIRE(std::abs(actual[i] - expected[i]) < 1e-6);
  }
}

TEST_CASE("ECEF velocity matches the central difference of position",
          "[gnss]") {
  constexpr double t = 1000.0;
  const std::array<std::size_t, 4> indices = {0, 5, 12, 23};
  for (const std::size_t index : indices) {
    const auto& slot = kBaselineSlots[index];
    const Ecef velocity = SatelliteAt(slot, t).velocity_m_s;
    const Ecef plus = SatelliteAt(slot, t + kHalfWidthS).position_m;
    const Ecef minus = SatelliteAt(slot, t - kHalfWidthS).position_m;
    const Ecef difference = Minus(plus, minus);
    for (std::size_t i = 0; i < 3; ++i) {
      const double central = difference[i] / (2.0 * kHalfWidthS);
      REQUIRE(std::abs(velocity[i] - central) < 1e-3);
    }
  }
}

TEST_CASE("geodetic to ECEF matches WGS84 at the axes and the poles",
          "[gnss]") {
  const Ecef origin = GeodeticToEcef(GeoPoint{0.0, 0.0}, 0.0);
  REQUIRE(std::abs(origin[0] - 6378137.0) < 1e-6);
  REQUIRE(std::abs(origin[1]) < 1e-6);
  REQUIRE(std::abs(origin[2]) < 1e-6);

  const Ecef pole = GeodeticToEcef(GeoPoint{std::numbers::pi / 2.0, 0.0}, 0.0);
  REQUIRE(std::abs(pole[0]) < 1e-6);
  REQUIRE(std::abs(pole[1]) < 1e-6);
  REQUIRE(std::abs(pole[2] - 6356752.314245) < 1e-6);

  const Ecef raised =
      GeodeticToEcef(GeoPoint{0.0, std::numbers::pi / 2.0}, 100.0);
  REQUIRE(std::abs(raised[0]) < 1e-6);
  REQUIRE(std::abs(raised[1] - 6378237.0) < 1e-6);
  REQUIRE(std::abs(raised[2]) < 1e-6);
}

TEST_CASE("look angles give zenith straight up and azimuth zero due north",
          "[gnss]") {
  const double latitude = 0.6;
  const double longitude = -2.1;
  const Ecef receiver = GeodeticToEcef(GeoPoint{latitude, longitude}, 0.0);

  const Ecef up = {std::cos(latitude) * std::cos(longitude),
                   std::cos(latitude) * std::sin(longitude),
                   std::sin(latitude)};
  const Ecef overhead = {receiver[0] + 20.2e6 * up[0],
                         receiver[1] + 20.2e6 * up[1],
                         receiver[2] + 20.2e6 * up[2]};
  const LookAngles zenith =
      Look(receiver, GeoPoint{latitude, longitude}, overhead);
  REQUIRE(std::abs(zenith.elevation_rad - std::numbers::pi / 2.0) < 1e-9);

  const Ecef north = {-std::sin(latitude) * std::cos(longitude),
                      -std::sin(latitude) * std::sin(longitude),
                      std::cos(latitude)};
  const Ecef due_north = {receiver[0] + 1.0e6 * north[0],
                          receiver[1] + 1.0e6 * north[1],
                          receiver[2] + 1.0e6 * north[2]};
  const LookAngles ahead =
      Look(receiver, GeoPoint{latitude, longitude}, due_north);
  // Azimuth wraps at 2 pi, so accept a value just below it.
  const double wrap_distance =
      std::min(ahead.azimuth_rad, kTwoPi - ahead.azimuth_rad);
  REQUIRE(wrap_distance < 1e-9);
}

TEST_CASE("KPAO sees 6 to 10 slots with the sidereal-day fractions", "[gnss]") {
  const Ecef receiver =
      GeodeticToEcef(GeoPoint{kKpaoLatitudeRad, kKpaoLongitudeRad}, 0.0);
  std::array<std::size_t, kBaselineSlots.size() + 1> counts{};
  std::array<int, kBaselineSlots.size()> visible{};
  std::size_t total = 0;
  std::size_t fewest = kBaselineSlots.size();
  std::size_t most = 0;

  for (double t = 0.0; t <= kSiderealDayS; t += kVisibleStepS) {
    const std::size_t count = VisibleSlots(
        receiver, GeoPoint{kKpaoLatitudeRad, kKpaoLongitudeRad}, t, visible);
    ++counts[count];
    ++total;
    fewest = std::min(fewest, count);
    most = std::max(most, count);
  }

  REQUIRE(fewest == 6);
  REQUIRE(most == 10);
  const std::array<std::pair<std::size_t, double>, 5> expected = {
      {{6, 0.102}, {7, 0.399}, {8, 0.355}, {9, 0.118}, {10, 0.025}}};
  for (const auto& [count, fraction] : expected) {
    const double observed =
        static_cast<double>(counts[count]) / static_cast<double>(total);
    REQUIRE(std::abs(observed - fraction) <= 0.01);
  }
}
