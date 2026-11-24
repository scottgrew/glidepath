// Guardrail for the JSBSim plant. Every threshold comes from the director's
// direct JSBSim v1.3.3 runs (raw FGFDMExec calls, no glidepath code) at the
// cruise below; the measured value sits beside each threshold.
#include "fsw/types.hpp"
#include "sim/plant.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <filesystem>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace {

using glidepath::fsw::ActuatorCommands;
using glidepath::fsw::VehicleState;
using glidepath::sim::InitialCondition;
using glidepath::sim::Plant;

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

// Holds the same commands for the given time and returns the final state.
VehicleState Fly(Plant& plant, const ActuatorCommands& commands,
                 double seconds) {
  const long steps = std::lround(seconds * glidepath::sim::kRateHz);
  for (long i = 0; i < steps; ++i) plant.step(commands);
  return plant.truth();
}

bool AllFinite(const VehicleState& s) {
  for (const double v :
       {s.time_s, s.latitude_rad, s.longitude_rad, s.altitude_m, s.roll_rad,
        s.pitch_rad, s.heading_rad, s.roll_rate_rad_s, s.pitch_rate_rad_s,
        s.yaw_rate_rad_s, s.calibrated_airspeed_m_s, s.climb_rate_m_s}) {
    if (!std::isfinite(v)) return false;
  }
  return true;
}

// State after flying from trim with one command offset from its trim value.
VehicleState Offset(double ActuatorCommands::* field, double delta,
                    double seconds) {
  Plant plant(kRoot, kCruise);
  ActuatorCommands commands = plant.trim_commands();
  commands.*field += delta;
  return Fly(plant, commands, seconds);
}

}  // namespace

TEST_CASE("the trimmed state starts at the initial condition at time zero") {
  const Plant plant(kRoot, kCruise);
  const VehicleState s = plant.truth();
  CHECK(s.time_s == 0.0);
  CHECK(std::abs(s.latitude_rad - kCruise.latitude_rad) < 1e-9);
  CHECK(std::abs(s.longitude_rad - kCruise.longitude_rad) < 1e-9);
  CHECK(std::abs(s.altitude_m - kCruise.altitude_m) < 1e-6);
  CHECK(std::abs(s.heading_rad - kCruise.heading_rad) < 1e-6);
}

TEST_CASE("trim commands hold level cruise for 60 s") {
  Plant plant(kRoot, kCruise);
  const ActuatorCommands trim = plant.trim_commands();
  for (int i = 0; i < 60 * 120; ++i) {
    plant.step(trim);
    REQUIRE(AllFinite(plant.truth()));
  }
  const VehicleState s = plant.truth();
  CHECK(std::abs(s.time_s - 60.0) < 1e-9);
  CHECK(std::abs(s.altitude_m - kCruise.altitude_m) < 0.6);  // measured 0.095
  CHECK(std::abs(s.calibrated_airspeed_m_s - kCruise.calibrated_airspeed_m_s) <
        0.25);  // measured 0.004
  CHECK(std::abs(s.heading_rad - kCruise.heading_rad) < 0.01);  // 4e-5
  CHECK(std::abs(s.roll_rad) < 0.01);                           // 2e-5
  CHECK(std::abs(s.climb_rate_m_s) < 0.1);
}

TEST_CASE("identical runs give identical states") {
  Plant a(kRoot, kCruise);
  Plant b(kRoot, kCruise);
  ActuatorCommands commands = a.trim_commands();
  commands.elevator -= 0.1;
  CHECK(Fly(a, commands, 10.0) == Fly(b, commands, 10.0));
}

TEST_CASE("each command moves the aircraft in the JSBSim sign convention") {
  const double level_pitch =
      Offset(&ActuatorCommands::elevator, 0.0, 2.0).pitch_rad;
  // Negative elevator pulls the nose up: measured +23.6 deg in 2 s.
  CHECK(Offset(&ActuatorCommands::elevator, -0.3, 2.0).pitch_rad >
        level_pitch + 10.0 * kDegree);
  // Positive elevator pushes the nose down: measured -22.4 deg.
  CHECK(Offset(&ActuatorCommands::elevator, 0.3, 2.0).pitch_rad <
        level_pitch - 10.0 * kDegree);
  // Positive aileron rolls right: measured +27.9 deg.
  CHECK(Offset(&ActuatorCommands::aileron, 0.3, 2.0).roll_rad > 15.0 * kDegree);
  // Positive rudder yaws the nose left: measured -5.8 deg of heading.
  CHECK(Offset(&ActuatorCommands::rudder, 0.3, 2.0).heading_rad <
        kCruise.heading_rad - 2.0 * kDegree);
  // Closing the throttle loses height: measured -119 m in 20 s.
  CHECK(Offset(&ActuatorCommands::throttle, -1.0, 20.0).altitude_m <
        kCruise.altitude_m - 60.0);
}

TEST_CASE("commands outside their range are clamped") {
  // JSBSim itself does not clamp: elevator -5 and -1 give different pitch
  // there. Each pair below lands beyond the same limit, so it must match.
  CHECK(Offset(&ActuatorCommands::elevator, -5.0, 2.0) ==
        Offset(&ActuatorCommands::elevator, -1.0, 2.0));
  CHECK(Offset(&ActuatorCommands::aileron, 5.0, 2.0) ==
        Offset(&ActuatorCommands::aileron, 1.0, 2.0));
  // Trim throttle is 0.72, so both offsets fall below the 0 limit.
  CHECK(Offset(&ActuatorCommands::throttle, -5.0, 2.0) ==
        Offset(&ActuatorCommands::throttle, -1.0, 2.0));
}

TEST_CASE("a non-finite command is rejected") {
  Plant plant(kRoot, kCruise);
  ActuatorCommands commands = plant.trim_commands();
  commands.aileron = std::numeric_limits<double>::quiet_NaN();
  CHECK_THROWS_AS(plant.step(commands), std::invalid_argument);
}

TEST_CASE("a missing JSBSim root names the path") {
  CHECK_THROWS_WITH(Plant("/nonexistent/jsbsim", kCruise),
                    Catch::Matchers::ContainsSubstring("/nonexistent/jsbsim"));
}

TEST_CASE("an untrimmable airspeed reports a trim failure") {
  // Measured: JSBSim's full trim throws at 10 kt and at 30 kt.
  InitialCondition slow = kCruise;
  slow.calibrated_airspeed_m_s = 10.0 * kKnot;
  CHECK_THROWS_WITH(Plant(kRoot, slow),
                    Catch::Matchers::ContainsSubstring("trim"));
}
