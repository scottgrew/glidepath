#include "app/flight.hpp"
#include "fsw/autopilot.hpp"
#include "fsw/geodesy.hpp"
#include "fsw/guidance.hpp"
#include "fsw/ground.hpp"
#include "fsw/landing.hpp"
#include "fsw/runway.hpp"
#include "fsw/estimator.hpp"
#include "fsw/measurements.hpp"
#include "fsw/sensor_specs.hpp"
#include "fsw/takeoff.hpp"
#include "fsw/types.hpp"
#include "fsw/units.hpp"
#include "sim/plant.hpp"
#include "sim/sensors.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <functional>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string_view>

namespace glidepath::app {
namespace {

double RunwayCourseRad() noexcept {
  const fsw::LocalPoint far =
      fsw::ToLocal(fsw::kPaloAlto31.far_end, fsw::kPaloAlto31.threshold);
  return std::atan2(far.east_m, far.north_m);
}

double PaloAltoRunway31HeadingRad() noexcept {
  return RunwayCourseRad();
}

sim::InitialCondition AutolandXwindStart(sim::Wind wind) noexcept {
  return sim::InitialCondition{.latitude_rad = 0.0,
                               .longitude_rad = 0.0,
                               .altitude_m = 1300.0 * fsw::kFoot,
                               .calibrated_airspeed_m_s = 80.0 * fsw::kKnot,
                               .heading_rad = 0.0,
                               .terrain_elevation_m = 6.7 * fsw::kFoot,
                               .wind = wind};
}

// The final approach fix, 4 nm out on the runway 31 course, at altitude_m.
sim::InitialCondition AutolandInitialCondition(
    const sim::Wind& wind = {},
    const double altitude_m = 1300.0 * fsw::kFoot) noexcept {
  const fsw::LocalPoint far =
      fsw::ToLocal(fsw::kPaloAlto31.far_end, fsw::kPaloAlto31.threshold);
  const double length = std::hypot(far.north_m, far.east_m);
  const fsw::LocalPoint u_hat{.north_m = far.north_m / length,
                              .east_m = far.east_m / length};
  const fsw::LocalPoint faf{
      .north_m = -4.0 * fsw::kNauticalMile * u_hat.north_m,
      .east_m = -4.0 * fsw::kNauticalMile * u_hat.east_m};
  const fsw::GeoPoint position = fsw::ToGeo(faf, fsw::kPaloAlto31.threshold);
  return sim::InitialCondition{
      .latitude_rad = position.latitude_rad,
      .longitude_rad = position.longitude_rad,
      .altitude_m = altitude_m,
      .calibrated_airspeed_m_s = 80.0 * fsw::kKnot,
      .heading_rad = std::atan2(u_hat.east_m, u_hat.north_m),
      .terrain_elevation_m = 6.7 * fsw::kFoot,
      .wind = wind};
}

}  // namespace

const std::array<Scenario, 12>& Scenarios() noexcept {
  static const std::array<Scenario, 12> kScenarios{{
      {.name = "hold",
       .description = "Holds level cruise at 4000 ft and 100 kt, heading east",
       .start = kCruise,
       .duration_s = 60.0,
       .changes = {}},
      {.name = "climb",
       .description = "Climbs 500 ft",
       .start = kCruise,
       .duration_s = 90.0,
       .changes = kClimb},
      {.name = "turn",
       .description = "Turns right from heading 090 to 180",
       .start = kCruise,
       .duration_s = 60.0,
       .changes = kTurn},
      {.name = "slow",
       .description = "Slows from 100 to 90 kt",
       .start = kCruise,
       .duration_s = 60.0,
       .changes = kSlow},
      {.name = "tour",
       .description =
           "Climbs, turns, slows, then descends and turns back up to "
           "speed over five minutes",
       .start = kCruise,
       .duration_s = 300.0,
       .changes = kTour},
      {.name = "approach",
       .description = "Flies a flight plan onto final for Palo Alto runway 31 "
                      "and makes a low pass",
       .start = kCruise,
       .duration_s = 700.0,
       .changes = {},
       .make_plan = &PaloAltoApproach},
      {.name = "autoland",
       .description =
           "Lands on Palo Alto runway 31 from the final approach fix",
       .start = kAutolandStart,
       .duration_s = 400.0,
       .changes = {},
       .make_plan = &PaloAltoAutolandPlan,
       .land = true,
       .state_source = StateSource::kEstimate},
      {.name = "autoland_xwind_left_10",
       .description = "Autoland with 10 kt crosswind from the left of the "
                      "landing direction",
       .start = AutolandXwindStart(
           {.speed_m_s = 10.0 * fsw::kKnot,
            .from_rad = PaloAltoRunway31HeadingRad() - 90.0 * fsw::kDegree}),
       .duration_s = 400.0,
       .changes = {},
       .make_plan = &PaloAltoAutolandPlan,
       .land = true,
       .state_source = StateSource::kEstimate},
      {.name = "autoland_xwind_right_10",
       .description =
           "Autoland with 10 kt crosswind from the right of the landing "
           "direction",
       .start = AutolandXwindStart(
           {.speed_m_s = 10.0 * fsw::kKnot,
            .from_rad = PaloAltoRunway31HeadingRad() + 90.0 * fsw::kDegree}),
       .duration_s = 400.0,
       .changes = {},
       .make_plan = &PaloAltoAutolandPlan,
       .land = true,
       .state_source = StateSource::kEstimate},
      {.name = "autoland_xwind_left_5",
       .description = "Autoland with 5 kt crosswind from the left of the "
                      "landing direction",
       .start = AutolandXwindStart(
           {.speed_m_s = 5.0 * fsw::kKnot,
            .from_rad = PaloAltoRunway31HeadingRad() - 90.0 * fsw::kDegree}),
       .duration_s = 400.0,
       .changes = {},
       .make_plan = &PaloAltoAutolandPlan,
       .land = true,
       .state_source = StateSource::kEstimate},
      {.name = "autoland_xwind_right_5",
       .description =
           "Autoland with 5 kt crosswind from the right of the landing "
           "direction",
       .start = AutolandXwindStart(
           {.speed_m_s = 5.0 * fsw::kKnot,
            .from_rad = PaloAltoRunway31HeadingRad() + 90.0 * fsw::kDegree}),
       .duration_s = 400.0,
       .changes = {},
       .make_plan = &PaloAltoAutolandPlan,
       .land = true,
       .state_source = StateSource::kEstimate},
      {.name = "takeoff",
       .description =
           "Takes off from Palo Alto runway 31 and climbs to 1000 ft",
       .start = kAutolandStart,
       .duration_s = 120.0,
       .changes = {},
       .make_plan = &PaloAltoTakeoffPlan,
       .take_off = true},
  }};
  return kScenarios;
}

namespace {

constexpr int kStepsPerCycle =
    static_cast<int>(sim::kRateHz / fsw::kControlRateHz);
static_assert(
    kStepsPerCycle * fsw::kControlRateHz == sim::kRateHz,
    "the simulator rate must be a whole multiple of the control rate");

void ApplyChange(const TargetChange& change, fsw::Targets& targets) {
  if (change.altitude_m) {
    targets.altitude_m = *change.altitude_m;
  }
  if (change.heading_rad) {
    targets.heading_rad = *change.heading_rad;
  }
  if (change.calibrated_airspeed_m_s) {
    targets.calibrated_airspeed_m_s = *change.calibrated_airspeed_m_s;
  }
}

sim::InitialCondition TakeoffInitialCondition() noexcept {
  const fsw::LocalPoint far =
      fsw::ToLocal(fsw::kPaloAlto31.far_end, fsw::kPaloAlto31.threshold);
  const double length = std::hypot(far.north_m, far.east_m);
  const fsw::LocalPoint u_hat{.north_m = far.north_m / length,
                              .east_m = far.east_m / length};
  return sim::InitialCondition{
      .latitude_rad = fsw::kPaloAlto31.threshold.latitude_rad,
      .longitude_rad = fsw::kPaloAlto31.threshold.longitude_rad,
      .altitude_m = fsw::kPaloAlto31.elevation_m,
      .calibrated_airspeed_m_s = 0.0,
      .heading_rad = std::atan2(u_hat.east_m, u_hat.north_m),
      .terrain_elevation_m = fsw::kPaloAlto31.elevation_m,
      .on_ground = true};
}

fsw::ActuatorCommands ControlCycle(
    const fsw::VehicleState& state, const fsw::AutopilotConfig& config,
    fsw::AutopilotMemory& memory, fsw::Targets& targets,
    const std::optional<fsw::FlightPlan>& plan,
    std::optional<fsw::GuidanceMemory>& guidance_memory,
    const std::optional<fsw::LandingConfig>& landing_config,
    fsw::LandingMemory* landing_memory,
    const std::optional<fsw::TakeoffConfig>& takeoff_config,
    fsw::TakeoffMemory* takeoff_memory) {
  if (landing_config && landing_memory != nullptr) {
    const fsw::LandingOutput landing =
        fsw::UpdateLanding(*landing_config, state, *landing_memory);
    if (landing.on_ground) {
      return landing.ground_commands;
    }
    targets = landing.targets;
    const fsw::ActuatorCommands commands =
        fsw::UpdateAutopilot(config, targets, state, memory);
    landing_memory->throttle = commands.throttle;
    return commands;
  }
  if (takeoff_config && takeoff_memory != nullptr) {
    const fsw::LandingOutput takeoff =
        fsw::UpdateTakeoff(*takeoff_config, state, *takeoff_memory);
    if (takeoff.on_ground) {
      return takeoff.ground_commands;
    }
    targets = takeoff.targets;
    fsw::ActuatorCommands commands =
        fsw::UpdateAutopilot(config, targets, state, memory);
    if (takeoff_memory != nullptr &&
        takeoff_memory->phase == fsw::TakeoffPhase::kRotate &&
        state.main_gear_on_ground) {
      const fsw::RunwayFrame runway_frame =
          fsw::RunwayFrameOf(takeoff_config->runway, state);
      const double steer = fsw::CenterlineSteer(
          takeoff_config->gains.centerline, runway_frame, state);
      commands.rudder = -steer;
    }
    return commands;
  }
  if (guidance_memory && plan.has_value()) {
    targets = fsw::UpdateGuidance(fsw::kDefaultGuidance, plan.value(), state,
                                  *guidance_memory);
  }
  return fsw::UpdateAutopilot(config, targets, state, memory);
}

struct Observer {
  StateSource source = StateSource::kTruth;
  fsw::EstimatorConfig config{};
  sim::SensorModel sensor_model = sim::StartSensors(fsw::kDefaultSpecs, 0);
  fsw::EstimatorMemory estimator_memory = fsw::StartEstimator();
};

fsw::SensorFrame Observe(Observer& observer, const sim::Plant& plant) noexcept {
  if (observer.source == StateSource::kTruth) {
    return fsw::SensorFrame{};
  }
  return sim::SampleSensors(observer.config.specs, observer.sensor_model,
                            plant.truth(), plant.specific_force());
}

CycleRecord MakeRecord(const fsw::VehicleState& truth,
                       const fsw::VehicleState& flown,
                       const fsw::NavSnapshot& snapshot,
                       const fsw::Targets& targets,
                       const fsw::ActuatorCommands& commands) {
  CycleRecord record{};
  record.truth = truth;
  record.estimate = flown;
  record.nav = snapshot;
  record.targets = targets;
  record.commands = commands;
  return record;
}

struct CycleContext {
  sim::Plant* plant;
  StateSource flown_source;
  Observer* observer;
  const fsw::AutopilotConfig* config;
  fsw::AutopilotMemory* memory;
  fsw::Targets* targets;
  const std::optional<fsw::FlightPlan>* plan;
  std::optional<fsw::GuidanceMemory>* guidance_memory;
  const std::optional<fsw::LandingConfig>* landing_config;
  std::optional<fsw::LandingMemory>* landing_memory;
  const std::optional<fsw::TakeoffConfig>* takeoff_config;
  std::optional<fsw::TakeoffMemory>* takeoff_memory;
  fsw::ActuatorCommands* last_commands;
  bool* landed_extra_cycle;
  const std::function<void(const CycleRecord&)>* record;
};

bool RunOneCycle(const CycleContext& ctx, const double time_s,
                 std::span<const TargetChange>::iterator* change_it,
                 const std::span<const TargetChange>::iterator& change_end) {
  while (*change_it != change_end && (*change_it)->at_s <= time_s + 1e-9) {
    ApplyChange(**change_it, *ctx.targets);
    ++*change_it;
  }

  const fsw::SensorFrame frame = Observe(*ctx.observer, *ctx.plant);
  fsw::VehicleState flown = ctx.plant->truth();
  fsw::NavSnapshot snapshot{};
  bool estimate_valid = true;
  if (ctx.flown_source == StateSource::kEstimate) {
    const fsw::Estimate estimate = fsw::UpdateEstimator(
        ctx.observer->config, frame, &ctx.observer->estimator_memory);
    estimate_valid = estimate.valid;
    flown = estimate.state;
    snapshot = estimate.nav;
  }

  fsw::ActuatorCommands commands{};
  if (ctx.flown_source == StateSource::kEstimate && !estimate_valid) {
    commands = *ctx.last_commands;
  } else {
    commands = ControlCycle(
        flown, *ctx.config, *ctx.memory, *ctx.targets, *ctx.plan,
        *ctx.guidance_memory, *ctx.landing_config,
        ctx.landing_memory->has_value() ? &ctx.landing_memory->value()
                                        : nullptr,
        *ctx.takeoff_config,
        ctx.takeoff_memory->has_value() ? &ctx.takeoff_memory->value()
                                        : nullptr);
    *ctx.last_commands = commands;
  }

  for (int step = 0; step < kStepsPerCycle; ++step) {
    ctx.plant->step(commands);
  }
  const CycleRecord record =
      MakeRecord(ctx.plant->truth(), flown, snapshot, *ctx.targets, commands);
  (*ctx.record)(record);

  if (ctx.landing_memory->has_value() &&
      ctx.landing_memory->value().phase == fsw::LandingPhase::kStopped) {
    if (*ctx.landed_extra_cycle) {
      return true;
    }
    *ctx.landed_extra_cycle = true;
  }
  return false;
}

sim::InitialCondition ScenarioInitialCondition(
    const Scenario& scenario) noexcept {
  if (scenario.land) {
    return AutolandInitialCondition(scenario.start.wind);
  }
  if (scenario.take_off) {
    return TakeoffInitialCondition();
  }
  return scenario.start;
}

void SettleGearOnGround(sim::Plant& plant) {
  const fsw::ActuatorCommands settle{};
  for (int step = 0; step < static_cast<int>(2.0 * sim::kRateHz); ++step) {
    plant.step(settle);
  }
}

fsw::FlightPlan RequireScenarioPlan(const Scenario& scenario) {
  if (scenario.make_plan == nullptr) {
    throw std::logic_error(
        std::format("scenario {} has no flight plan factory", scenario.name));
  }
  std::optional<fsw::FlightPlan> plan = scenario.make_plan();
  if (!plan) {
    throw std::logic_error(
        std::format("scenario {} has no valid flight plan", scenario.name));
  }
  return *plan;
}

}  // namespace

std::optional<fsw::FlightPlan> PaloAltoTakeoffPlan() noexcept {
  const fsw::LocalPoint far =
      fsw::ToLocal(fsw::kPaloAlto31.far_end, fsw::kPaloAlto31.threshold);
  const double len = std::hypot(far.north_m, far.east_m);
  const fsw::LocalPoint u_hat{.north_m = far.north_m / len,
                              .east_m = far.east_m / len};
  const fsw::LocalPoint past_far{.north_m = (len + 5000.0) * u_hat.north_m,
                                 .east_m = (len + 5000.0) * u_hat.east_m};
  fsw::FlightPlan plan{};
  plan.origin = fsw::kPaloAlto31.threshold;
  plan.waypoints[0] = {
      .position = fsw::ToGeo(past_far, fsw::kPaloAlto31.threshold),
      .altitude_m = fsw::kPaloAlto31.elevation_m + 1000.0 * fsw::kFoot,
      .calibrated_airspeed_m_s = 74.0 * fsw::kKnot,
  };
  plan.count = 1;
  return plan;
}

std::optional<fsw::FlightPlan> PaloAltoAutolandPlan() noexcept {
  constexpr fsw::ApproachProfile kProfile{
      .initial_fix_distance_m = 8.0 * fsw::kNauticalMile,
      .initial_fix_altitude_m = 2000.0 * fsw::kFoot,
      .final_fix_distance_m = 4.0 * fsw::kNauticalMile,
      .final_fix_altitude_m = 1300.0 * fsw::kFoot,
      .airspeed_m_s = 80.0 * fsw::kKnot,
      .pass_altitude_m = 1000.0 * fsw::kFoot};
  std::optional<fsw::FlightPlan> plan =
      fsw::BuildApproach({}, fsw::kPaloAlto31, kProfile);
  if (!plan) {
    return plan;
  }
  // The scenario starts at the final approach fix, past the initial fix, so
  // the plan starts at the final approach fix too: guidance would otherwise
  // turn back to fly to the initial fix first.
  const std::span<fsw::Waypoint> active =
      std::span(plan->waypoints).first(plan->count);
  std::ranges::copy(std::views::drop(active, 1), active.begin());
  --plan->count;
  return plan;
}

std::optional<fsw::FlightPlan> PaloAltoApproach() noexcept {
  constexpr std::array<fsw::Waypoint, 1> kEnroute{{
      {.position = {.latitude_rad = 37.330 * fsw::kDegree,
                    .longitude_rad = -121.970 * fsw::kDegree},
       .altitude_m = 3000.0 * fsw::kFoot,
       .calibrated_airspeed_m_s = 100.0 * fsw::kKnot},
  }};
  constexpr fsw::ApproachProfile kProfile{
      .initial_fix_distance_m = 8.0 * fsw::kNauticalMile,
      .initial_fix_altitude_m = 2000.0 * fsw::kFoot,
      .final_fix_distance_m = 4.0 * fsw::kNauticalMile,
      .final_fix_altitude_m = 1300.0 * fsw::kFoot,
      .airspeed_m_s = 80.0 * fsw::kKnot,
      .pass_altitude_m = 1000.0 * fsw::kFoot};
  return fsw::BuildApproach(kEnroute, fsw::kPaloAlto31, kProfile);
}

const Scenario* FindScenario(const std::string_view name) noexcept {
  const std::array<Scenario, 12>& scenarios = Scenarios();
  const auto* const it = std::ranges::find(scenarios, name, &Scenario::name);
  return it == scenarios.end() ? nullptr : &*it;
}

void Fly(const Scenario& scenario, const std::filesystem::path& jsbsim_root,
         const sim::PlantOptions& plant_options,
         const SensorOptions& sensor_options,
         const std::function<void(const CycleRecord&)>& record) {
  const sim::InitialCondition start = ScenarioInitialCondition(scenario);
  sim::Plant plant(jsbsim_root, start, plant_options);

  if (start.on_ground) {
    SettleGearOnGround(plant);
  }
  const fsw::Trim trim{
      .commands = plant.trim_commands(),
      .pitch_rad = plant.truth().pitch_rad,
      .calibrated_airspeed_m_s = plant.truth().calibrated_airspeed_m_s};
  const fsw::AutopilotConfig config{.gains = fsw::kDefaultGains, .trim = trim};
  fsw::AutopilotMemory memory = fsw::Engage(trim);

  fsw::Targets targets{
      .altitude_m = start.altitude_m,
      .heading_rad = start.heading_rad,
      .calibrated_airspeed_m_s = start.calibrated_airspeed_m_s};

  const StateSource flown_source =
      sensor_options.source.value_or(scenario.state_source);
  Observer observer{
      .source = flown_source,
      .config{.specs = fsw::kDefaultSpecs,
              .terrain_elevation_m = start.terrain_elevation_m},
      .sensor_model =
          sim::StartSensors(fsw::kDefaultSpecs, sensor_options.seed),
  };

  if (flown_source == StateSource::kEstimate) {
    Observe(observer, plant);
  }
  record(MakeRecord(plant.truth(), plant.truth(), {}, targets, trim.commands));

  std::optional<fsw::FlightPlan> plan;
  std::optional<fsw::GuidanceMemory> guidance_memory;
  if (scenario.make_plan != nullptr && !scenario.land && !scenario.take_off) {
    plan = RequireScenarioPlan(scenario);
    guidance_memory = fsw::StartPlan(*plan, plant.truth());
  }

  std::optional<fsw::FlightPlan> landing_plan;
  std::optional<fsw::LandingMemory> landing_memory;
  std::optional<fsw::LandingConfig> landing_config;
  std::optional<fsw::FlightPlan> takeoff_plan;
  std::optional<fsw::TakeoffMemory> takeoff_memory;
  std::optional<fsw::TakeoffConfig> takeoff_config;
  if (scenario.take_off) {
    takeoff_plan = RequireScenarioPlan(scenario);
    takeoff_config = fsw::TakeoffConfig{.gains = fsw::kDefaultTakeoff,
                                        .runway = fsw::kPaloAlto31,
                                        .plan = &takeoff_plan.value()};
    takeoff_memory = fsw::StartTakeoff(*takeoff_config, plant.truth());
  }

  if (scenario.land) {
    landing_plan = RequireScenarioPlan(scenario);
    landing_config = fsw::LandingConfig{.gains = fsw::kDefaultLanding,
                                        .runway = fsw::kPaloAlto31,
                                        .plan = &landing_plan.value()};
    landing_memory = fsw::StartLanding(*landing_config, plant.truth());
  }

  const std::int64_t cycles =
      std::llround(scenario.duration_s * fsw::kControlRateHz);
  auto change_it = scenario.changes.begin();
  const auto change_end = scenario.changes.end();
  bool landed_extra_cycle = false;
  fsw::ActuatorCommands last_commands = trim.commands;
  const CycleContext cycle_ctx{
      .plant = &plant,
      .flown_source = flown_source,
      .observer = &observer,
      .config = &config,
      .memory = &memory,
      .targets = &targets,
      .plan = &plan,
      .guidance_memory = &guidance_memory,
      .landing_config = &landing_config,
      .landing_memory = &landing_memory,
      .takeoff_config = &takeoff_config,
      .takeoff_memory = &takeoff_memory,
      .last_commands = &last_commands,
      .landed_extra_cycle = &landed_extra_cycle,
      .record = &record,
  };

  for (std::int64_t cycle = 0; cycle < cycles; ++cycle) {
    const double t = static_cast<double>(cycle) / fsw::kControlRateHz;
    if (RunOneCycle(cycle_ctx, t, &change_it, change_end)) {
      break;
    }
  }
}

}  // namespace glidepath::app