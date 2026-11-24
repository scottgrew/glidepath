// Unit guardrail for waypoint guidance as a pure function: no simulator.
// Geometry oracles are outside facts (the FAA's surveyed runway ends and
// published length for Palo Alto, KPAO); steering cases are hand-built.
#include "fsw/autopilot.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/guidance.hpp"
#include "fsw/types.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace {

using glidepath::fsw::FlightPlan;
using glidepath::fsw::GeoPoint;
using glidepath::fsw::GuidanceMemory;
using glidepath::fsw::kDefaultGuidance;
using glidepath::fsw::LateralMode;
using glidepath::fsw::LocalPoint;
using glidepath::fsw::StartPlan;
using glidepath::fsw::Targets;
using glidepath::fsw::ToLocal;
using glidepath::fsw::UpdateGuidance;
using glidepath::fsw::VehicleState;
using glidepath::fsw::Waypoint;

constexpr double kDegree = std::numbers::pi / 180.0;

// FAA runway ends for KPAO 13/31 (AirNav, effective 29 October 2026).
constexpr GeoPoint kThreshold31{
    .latitude_rad = (37.0 + 27.509528 / 60) * kDegree,
    .longitude_rad = -(122.0 + 6.746287 / 60) * kDegree};
constexpr GeoPoint kThreshold13{
    .latitude_rad = (37.0 + 27.825037 / 60) * kDegree,
    .longitude_rad = -(122.0 + 7.059308 / 60) * kDegree};

// A state at local point `p` (from kThreshold31) flying `course` at 40 m/s.
// The inverse map uses rounded Earth radii, independent of the fsw code;
// tests only compare points mapped the same way.
VehicleState At(LocalPoint p, double course_rad) {
  constexpr double kNorthRadius = 6359000.0;  // Meridian radius near 37.5 N.
  constexpr double kEastRadius = 6386000.0;   // Prime-vertical radius there.
  const double lat = kThreshold31.latitude_rad + p.north_m / kNorthRadius;
  const double lon =
      kThreshold31.longitude_rad +
      p.east_m / (kEastRadius * std::cos(kThreshold31.latitude_rad));
  return VehicleState{.time_s = 0.0,
                      .latitude_rad = lat,
                      .longitude_rad = lon,
                      .altitude_m = 300.0,
                      .roll_rad = 0.0,
                      .pitch_rad = 0.0,
                      .heading_rad = course_rad,
                      .roll_rate_rad_s = 0.0,
                      .pitch_rate_rad_s = 0.0,
                      .yaw_rate_rad_s = 0.0,
                      .calibrated_airspeed_m_s = 40.0,
                      .climb_rate_m_s = 0.0,
                      .north_velocity_m_s = 40.0 * std::cos(course_rad),
                      .east_velocity_m_s = 40.0 * std::sin(course_rad)};
}

GeoPoint GeoAt(LocalPoint p) {
  const VehicleState s = At(p, 0.0);
  return {.latitude_rad = s.latitude_rad, .longitude_rad = s.longitude_rad};
}

// Two legs due north: start, then (5000, 0) at 300 m and 40 m/s, then
// (10000, 0) at 400 m and 45 m/s, in local metres.
FlightPlan NorthPlan() {
  FlightPlan plan{};
  plan.origin = kThreshold31;
  plan.waypoints[0] =
      Waypoint{.position = GeoAt({.north_m = 5000.0, .east_m = 0.0}),
               .altitude_m = 300.0,
               .calibrated_airspeed_m_s = 40.0};
  plan.waypoints[1] =
      Waypoint{.position = GeoAt({.north_m = 10000.0, .east_m = 0.0}),
               .altitude_m = 400.0,
               .calibrated_airspeed_m_s = 45.0};
  plan.count = 2;
  return plan;
}

}  // namespace

TEST_CASE("the local frame reproduces KPAO's surveyed runway") {
  const LocalPoint far_end = ToLocal(kThreshold13, kThreshold31);
  const LocalPoint origin = ToLocal(kThreshold31, kThreshold31);
  CHECK(std::abs(origin.north_m) < 1e-9);
  CHECK(std::abs(origin.east_m) < 1e-9);
  // FAA: 2441 ft = 744.0 m between the runway ends.
  CHECK(std::abs(std::hypot(far_end.north_m, far_end.east_m) - 744.0) < 1.0);
  // Runway 31 points north-west.
  CHECK(far_end.north_m > 0.0);
  CHECK(far_end.east_m < 0.0);
}

TEST_CASE("on the path and along it, guidance commands wings level") {
  const FlightPlan plan = NorthPlan();
  GuidanceMemory memory =
      StartPlan(plan, At({.north_m = 0.0, .east_m = 0.0}, 0.0));
  const Targets t =
      UpdateGuidance(kDefaultGuidance, plan,
                     At({.north_m = 1000.0, .east_m = 0.0}, 0.0), memory);
  CHECK(t.lateral_mode == LateralMode::kBank);
  CHECK(std::abs(t.bank_rad) < 0.05 * kDegree);
  CHECK(t.altitude_m == 300.0);
  CHECK(t.calibrated_airspeed_m_s == 40.0);
}

TEST_CASE("off the path, guidance banks back toward it") {
  const FlightPlan plan = NorthPlan();
  const VehicleState start = At({.north_m = 0.0, .east_m = 0.0}, 0.0);
  SECTION("left (west) of a northbound path banks right") {
    GuidanceMemory memory = StartPlan(plan, start);
    const Targets t =
        UpdateGuidance(kDefaultGuidance, plan,
                       At({.north_m = 1000.0, .east_m = -100.0}, 0.0), memory);
    CHECK(t.bank_rad > 1.0 * kDegree);
  }
  SECTION("right (east) of a northbound path banks left") {
    GuidanceMemory memory = StartPlan(plan, start);
    const Targets t =
        UpdateGuidance(kDefaultGuidance, plan,
                       At({.north_m = 1000.0, .east_m = 100.0}, 0.0), memory);
    CHECK(t.bank_rad < -1.0 * kDegree);
  }
  SECTION("on the path but heading away to the east banks left") {
    GuidanceMemory memory = StartPlan(plan, start);
    const Targets t = UpdateGuidance(
        kDefaultGuidance, plan,
        At({.north_m = 1000.0, .east_m = 0.0}, 30.0 * kDegree), memory);
    CHECK(t.bank_rad < -1.0 * kDegree);
  }
}

TEST_CASE(
    "a waypoint is passed when the aircraft crosses the line through "
    "it square to the leg") {
  const FlightPlan plan = NorthPlan();
  GuidanceMemory memory =
      StartPlan(plan, At({.north_m = 0.0, .east_m = 0.0}, 0.0));
  // 300 m east of the path and 1 m short of the first waypoint: not passed.
  UpdateGuidance(kDefaultGuidance, plan,
                 At({.north_m = 4999.0, .east_m = 300.0}, 0.0), memory);
  CHECK(memory.active == 0);
  // 1 m beyond it: passed, and the second waypoint's targets apply.
  const Targets t =
      UpdateGuidance(kDefaultGuidance, plan,
                     At({.north_m = 5001.0, .east_m = 300.0}, 0.0), memory);
  CHECK(memory.active == 1);
  CHECK(t.altitude_m == 400.0);
  CHECK(t.calibrated_airspeed_m_s == 45.0);
}

TEST_CASE(
    "after the last waypoint the plan is complete and the last leg "
    "continues") {
  const FlightPlan plan = NorthPlan();
  GuidanceMemory memory =
      StartPlan(plan, At({.north_m = 0.0, .east_m = 0.0}, 0.0));
  UpdateGuidance(kDefaultGuidance, plan,
                 At({.north_m = 6000.0, .east_m = 0.0}, 0.0), memory);
  UpdateGuidance(kDefaultGuidance, plan,
                 At({.north_m = 11000.0, .east_m = 0.0}, 0.0), memory);
  CHECK(memory.complete);
  // Beyond the end, 100 m west of the extended last leg: still steers back.
  const Targets t =
      UpdateGuidance(kDefaultGuidance, plan,
                     At({.north_m = 12000.0, .east_m = -100.0}, 0.0), memory);
  CHECK(t.bank_rad > 1.0 * kDegree);
  CHECK(t.altitude_m == 400.0);
}
