// Named flight scenarios and the loop that flies them: the JSBSim plant at
// sim::kRateHz and the flight software at fsw::kControlRateHz.
#pragma once
#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <span>
#include <string_view>

#include "fsw/autopilot.hpp"
#include "fsw/ekf_layout.hpp"
#include "fsw/guidance.hpp"
#include "fsw/integrity.hpp"
#include "sim/gnss_errors.hpp"
#include "fsw/types.hpp"
#include "fsw/units.hpp"
#include "sim/plant.hpp"

namespace glidepath::app {

enum class StateSource : std::uint8_t { kTruth, kEstimate };

// A change of one or more autopilot targets at a scenario time.
struct TargetChange {
  double at_s;
  std::optional<double> altitude_m;
  std::optional<double> heading_rad;
  std::optional<double> calibrated_airspeed_m_s;
};

struct Scenario {
  std::string_view name;
  std::string_view description;  // One line, shown by --help.
  sim::InitialCondition start;   // The first targets hold this state.
  double duration_s;
  std::span<const TargetChange> changes;  // Sorted by at_s.
  std::optional<fsw::FlightPlan> (*make_plan)() = nullptr;
  bool land = false;
  bool take_off = false;
  StateSource state_source = StateSource::kTruth;
  // A pseudorange fault injected in the simulator (G11d).
  std::optional<sim::GnssFault> gnss_fault;
  // The simulator stops the engine at this scenario time.
  std::optional<double> engine_failure_s;
};

// What one flight-software cycle saw and did.
struct CycleRecord {
  fsw::VehicleState truth{};
  fsw::VehicleState estimate{};
  fsw::EkfSnapshot ekf{};
  fsw::Targets targets;
  fsw::ActuatorCommands commands{};
  fsw::IntegrityStatus integrity{};
  // Slot of the injected GNSS fault, -1 when none has started.
  int injected_gnss_slot = -1;
  bool engine_out{};
};

inline constexpr std::uint64_t kDefaultSensorSeed = 20261121;

struct SensorOptions {
  std::uint64_t seed = kDefaultSensorSeed;
  std::optional<StateSource> source;
};

// Level cruise at 4000 ft and 100 kt heading east, over the San Francisco
// peninsula (FlightGear's default scenery).
inline constexpr sim::InitialCondition kCruise{
    .latitude_rad = 37.4 * fsw::kDegree,
    .longitude_rad = -122.0 * fsw::kDegree,
    .altitude_m = 4000.0 * fsw::kFoot,
    .calibrated_airspeed_m_s = 100.0 * fsw::kKnot,
    .heading_rad = 90.0 * fsw::kDegree};

inline constexpr std::array<TargetChange, 1> kClimb{{
    {.at_s = 5.0,
     .altitude_m = 4500.0 * fsw::kFoot,
     .heading_rad = std::nullopt,
     .calibrated_airspeed_m_s = std::nullopt},
}};
inline constexpr std::array<TargetChange, 1> kTurn{{
    {.at_s = 5.0,
     .altitude_m = std::nullopt,
     .heading_rad = 180.0 * fsw::kDegree,
     .calibrated_airspeed_m_s = std::nullopt},
}};
inline constexpr std::array<TargetChange, 1> kSlow{{
    {.at_s = 5.0,
     .altitude_m = std::nullopt,
     .heading_rad = std::nullopt,
     .calibrated_airspeed_m_s = 90.0 * fsw::kKnot},
}};
inline constexpr std::array<TargetChange, 5> kTour{{
    {.at_s = 10.0,
     .altitude_m = 4500.0 * fsw::kFoot,
     .heading_rad = std::nullopt,
     .calibrated_airspeed_m_s = std::nullopt},
    {.at_s = 70.0,
     .altitude_m = std::nullopt,
     .heading_rad = 180.0 * fsw::kDegree,
     .calibrated_airspeed_m_s = std::nullopt},
    {.at_s = 110.0,
     .altitude_m = std::nullopt,
     .heading_rad = std::nullopt,
     .calibrated_airspeed_m_s = 85.0 * fsw::kKnot},
    {.at_s = 150.0,
     .altitude_m = 4000.0 * fsw::kFoot,
     .heading_rad = 300.0 * fsw::kDegree,
     .calibrated_airspeed_m_s = std::nullopt},
    {.at_s = 230.0,
     .altitude_m = std::nullopt,
     .heading_rad = std::nullopt,
     .calibrated_airspeed_m_s = 100.0 * fsw::kKnot},
}};
std::optional<fsw::FlightPlan> PaloAltoApproach() noexcept;
std::optional<fsw::FlightPlan> PaloAltoAutolandPlan() noexcept;
std::optional<fsw::FlightPlan> PaloAltoTakeoffPlan() noexcept;

inline constexpr sim::InitialCondition kAutolandStart{
    .latitude_rad = 0.0,
    .longitude_rad = 0.0,
    .altitude_m = 1300.0 * fsw::kFoot,
    .calibrated_airspeed_m_s = 80.0 * fsw::kKnot,
    .heading_rad = 0.0,
    .terrain_elevation_m = 6.7 * fsw::kFoot};

const std::array<Scenario, 15>& Scenarios() noexcept;

// The scenario named `name`, or nullptr.
const Scenario* FindScenario(std::string_view name) noexcept;

// Trims at scenario.start, engages the autopilot and flies the scenario.
// Calls `record` once for the trimmed state (commands = trim) and once after
// every cycle, so a scenario gives duration_s * kControlRateHz + 1 records.
// Throws what sim::Plant throws.
void Fly(const Scenario& scenario, const std::filesystem::path& jsbsim_root,
         const sim::PlantOptions& plant_options,
         const SensorOptions& sensor_options,
         const std::function<void(const CycleRecord&)>& record);

inline void Fly(const Scenario& scenario,
                const std::filesystem::path& jsbsim_root,
                const sim::PlantOptions& plant_options,
                const std::function<void(const CycleRecord&)>& record) {
  Fly(scenario, jsbsim_root, plant_options, {}, record);
}

inline void Fly(const Scenario& scenario,
                const std::filesystem::path& jsbsim_root,
                const std::function<void(const CycleRecord&)>& record) {
  Fly(scenario, jsbsim_root, {}, {}, record);
}

}  // namespace glidepath::app