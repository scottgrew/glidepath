// M0 guardrail for the simulator wrapper. Every threshold comes from the
// director's direct JSBSim v1.3.3 runs (raw FGFDMExec calls, no glidepath
// code) at the cruise below; the measured value sits beside each threshold.
#include "sim.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <filesystem>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace {

using glidepath::Controls;
using glidepath::InitialCondition;
using glidepath::Sim;
using glidepath::TruthState;

constexpr double kFoot = 0.3048;
constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kDegree = std::numbers::pi / 180.0;

// Level cruise at 4000 ft and 100 kt calibrated, heading east.
constexpr InitialCondition kCruise{.latitude_rad = 37.4 * kDegree,
                                   .longitude_rad = -122.0 * kDegree,
                                   .altitude_m = 4000.0 * kFoot,
                                   .calibrated_airspeed_m_s = 100.0 * kKnot,
                                   .heading_rad = 90.0 * kDegree};

const std::filesystem::path kRoot{GLIDEPATH_JSBSIM_ROOT};

// Holds the same controls for the given time and returns the final state.
TruthState Fly(Sim& sim, const Controls& controls, double seconds) {
  const long steps = std::lround(seconds * glidepath::kSimRateHz);
  for (long i = 0; i < steps; ++i) sim.step(controls);
  return sim.truth();
}

bool AllFinite(const TruthState& s) {
  for (const double v :
       {s.time_s, s.latitude_rad, s.longitude_rad, s.altitude_m, s.roll_rad,
        s.pitch_rad, s.heading_rad, s.roll_rate_rad_s, s.pitch_rate_rad_s,
        s.yaw_rate_rad_s, s.calibrated_airspeed_m_s, s.climb_rate_m_s}) {
    if (!std::isfinite(v)) return false;
  }
  return true;
}

// State after flying from trim with one control offset from its trim value.
TruthState Offset(double Controls::* field, double delta, double seconds) {
  Sim sim(kRoot, kCruise);
  Controls controls = sim.trim_controls();
  controls.*field += delta;
  return Fly(sim, controls, seconds);
}

}  // namespace

TEST_CASE("the trimmed state starts at the initial condition at time zero") {
  const Sim sim(kRoot, kCruise);
  const TruthState s = sim.truth();
  CHECK(s.time_s == 0.0);
  CHECK(std::abs(s.latitude_rad - kCruise.latitude_rad) < 1e-9);
  CHECK(std::abs(s.longitude_rad - kCruise.longitude_rad) < 1e-9);
  CHECK(std::abs(s.altitude_m - kCruise.altitude_m) < 1e-6);
  CHECK(std::abs(s.heading_rad - kCruise.heading_rad) < 1e-6);
}

TEST_CASE("trim controls hold level cruise for 60 s") {
  Sim sim(kRoot, kCruise);
  const Controls trim = sim.trim_controls();
  for (int i = 0; i < 60 * 120; ++i) {
    sim.step(trim);
    REQUIRE(AllFinite(sim.truth()));
  }
  const TruthState s = sim.truth();
  CHECK(std::abs(s.time_s - 60.0) < 1e-9);
  CHECK(std::abs(s.altitude_m - kCruise.altitude_m) < 0.6);  // measured 0.095
  CHECK(std::abs(s.calibrated_airspeed_m_s - kCruise.calibrated_airspeed_m_s) <
        0.25);  // measured 0.004
  CHECK(std::abs(s.heading_rad - kCruise.heading_rad) < 0.01);  // 4e-5
  CHECK(std::abs(s.roll_rad) < 0.01);                           // 2e-5
  CHECK(std::abs(s.climb_rate_m_s) < 0.1);
}

TEST_CASE("identical runs give identical states") {
  Sim a(kRoot, kCruise);
  Sim b(kRoot, kCruise);
  Controls controls = a.trim_controls();
  controls.elevator -= 0.1;
  CHECK(Fly(a, controls, 10.0) == Fly(b, controls, 10.0));
}

TEST_CASE("each control moves the aircraft in the JSBSim sign convention") {
  const double level_pitch = Offset(&Controls::elevator, 0.0, 2.0).pitch_rad;
  // Negative elevator pulls the nose up: measured +23.6 deg in 2 s.
  CHECK(Offset(&Controls::elevator, -0.3, 2.0).pitch_rad >
        level_pitch + 10.0 * kDegree);
  // Positive elevator pushes the nose down: measured -22.4 deg.
  CHECK(Offset(&Controls::elevator, 0.3, 2.0).pitch_rad <
        level_pitch - 10.0 * kDegree);
  // Positive aileron rolls right: measured +27.9 deg.
  CHECK(Offset(&Controls::aileron, 0.3, 2.0).roll_rad > 15.0 * kDegree);
  // Positive rudder yaws the nose left: measured -5.8 deg of heading.
  CHECK(Offset(&Controls::rudder, 0.3, 2.0).heading_rad <
        kCruise.heading_rad - 2.0 * kDegree);
  // Closing the throttle loses height: measured -119 m in 20 s.
  CHECK(Offset(&Controls::throttle, -1.0, 20.0).altitude_m <
        kCruise.altitude_m - 60.0);
}

TEST_CASE("controls outside their range are clamped") {
  // JSBSim itself does not clamp: elevator -5 and -1 give different pitch
  // there. Each pair below lands beyond the same limit, so it must match.
  CHECK(Offset(&Controls::elevator, -5.0, 2.0) ==
        Offset(&Controls::elevator, -1.0, 2.0));
  CHECK(Offset(&Controls::aileron, 5.0, 2.0) ==
        Offset(&Controls::aileron, 1.0, 2.0));
  // Trim throttle is 0.72, so both offsets fall below the 0 limit.
  CHECK(Offset(&Controls::throttle, -5.0, 2.0) ==
        Offset(&Controls::throttle, -1.0, 2.0));
}

TEST_CASE("a non-finite control is rejected") {
  Sim sim(kRoot, kCruise);
  Controls controls = sim.trim_controls();
  controls.aileron = std::numeric_limits<double>::quiet_NaN();
  CHECK_THROWS_AS(sim.step(controls), std::invalid_argument);
}

TEST_CASE("a missing JSBSim root names the path") {
  CHECK_THROWS_WITH(Sim("/nonexistent/jsbsim", kCruise),
                    Catch::Matchers::ContainsSubstring("/nonexistent/jsbsim"));
}

TEST_CASE("an untrimmable airspeed reports a trim failure") {
  // Measured: JSBSim's full trim throws at 10 kt and at 30 kt.
  InitialCondition slow = kCruise;
  slow.calibrated_airspeed_m_s = 10.0 * kKnot;
  CHECK_THROWS_WITH(Sim(kRoot, slow),
                    Catch::Matchers::ContainsSubstring("trim"));
}
