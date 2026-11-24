#include "sim/plant.hpp"

#include "fsw/types.hpp"
#include "fsw/units.hpp"

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <initialization/FGTrim.h>
#include <simgear/props/props.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using glidepath::fsw::ActuatorCommands;
using glidepath::fsw::VehicleState;

constexpr double kFootPerMetre = 1.0 / glidepath::fsw::kFoot;
constexpr double kKnotPerMetrePerSecond = 1.0 / glidepath::fsw::kKnot;

struct CommandSpec {
  const char* property;
  double ActuatorCommands::* member;
  const char* name;
  double low;
};

struct StateSpec {
  const char* property;
  double VehicleState::* member;
  double scale = 1.0;
};

constexpr std::array<CommandSpec, 6> kCommandSpecs{{
    {.property = "fcs/elevator-cmd-norm",
     .member = &ActuatorCommands::elevator,
     .name = "elevator",
     .low = -1.0},
    {.property = "fcs/aileron-cmd-norm",
     .member = &ActuatorCommands::aileron,
     .name = "aileron",
     .low = -1.0},
    {.property = "fcs/rudder-cmd-norm",
     .member = &ActuatorCommands::rudder,
     .name = "rudder",
     .low = -1.0},
    {.property = "fcs/throttle-cmd-norm",
     .member = &ActuatorCommands::throttle,
     .name = "throttle",
     .low = 0.0},
    {.property = "fcs/flap-cmd-norm",
     .member = &ActuatorCommands::flaps,
     .name = "flaps",
     .low = 0.0},
    {.property = "fcs/left-brake-cmd-norm",
     .member = &ActuatorCommands::brakes,
     .name = "brakes",
     .low = 0.0},
}};

constexpr std::array<StateSpec, 15> kStateSpecs{{
    {.property = "simulation/sim-time-sec", .member = &VehicleState::time_s},
    {.property = "position/lat-geod-rad",
     .member = &VehicleState::latitude_rad},
    {.property = "position/long-gc-rad",
     .member = &VehicleState::longitude_rad},
    {.property = "position/h-sl-meters", .member = &VehicleState::altitude_m},
    {.property = "attitude/phi-rad", .member = &VehicleState::roll_rad},
    {.property = "attitude/theta-rad", .member = &VehicleState::pitch_rad},
    {.property = "attitude/psi-rad", .member = &VehicleState::heading_rad},
    {.property = "velocities/p-rad_sec",
     .member = &VehicleState::roll_rate_rad_s},
    {.property = "velocities/q-rad_sec",
     .member = &VehicleState::pitch_rate_rad_s},
    {.property = "velocities/r-rad_sec",
     .member = &VehicleState::yaw_rate_rad_s},
    {.property = "velocities/vc-fps",
     .member = &VehicleState::calibrated_airspeed_m_s,
     .scale = glidepath::fsw::kFoot},
    {.property = "velocities/h-dot-fps",
     .member = &VehicleState::climb_rate_m_s,
     .scale = glidepath::fsw::kFoot},
    {.property = "velocities/v-north-fps",
     .member = &VehicleState::north_velocity_m_s,
     .scale = glidepath::fsw::kFoot},
    {.property = "velocities/v-east-fps",
     .member = &VehicleState::east_velocity_m_s,
     .scale = glidepath::fsw::kFoot},
    {.property = "position/h-agl-ft",
     .member = &VehicleState::height_above_ground_m,
     .scale = glidepath::fsw::kFoot},
}};

SGPropertyNode* RequireNode(const JSBSim::FGFDMExec& fdm, const char* path) {
  SGPropertyNode* const node = fdm.GetPropertyManager()->GetNode(path);
  if (node == nullptr) {
    throw std::runtime_error(std::format("JSBSim has no property {}", path));
  }
  return node;
}

}  // namespace

namespace glidepath::sim {

Plant::Plant(const std::filesystem::path& jsbsim_root,
             const InitialCondition& ic, const PlantOptions& options) {
  // JSBSim prints a banner and trim progress unless its debug level is 0.
  // A JSBSIM_DEBUG environment variable set by the user still wins.
  if (std::getenv("JSBSIM_DEBUG") == nullptr) JSBSim::FGJSBBase::debug_lvl = 0;

  const std::filesystem::path aircraft =
      jsbsim_root / "aircraft" / "c172p" / "c172p.xml";
  if (!std::filesystem::exists(aircraft)) {
    throw std::runtime_error(std::format(
        "no JSBSim aircraft data at {}: expected aircraft/c172p/c172p.xml "
        "under it",
        jsbsim_root.string()));
  }

  fdm_ = std::make_unique<JSBSim::FGFDMExec>();
  fdm_->SetRootDir(SGPath(jsbsim_root.string()));
  fdm_->SetAircraftPath(SGPath("aircraft"));
  fdm_->SetEnginePath(SGPath("engine"));
  fdm_->SetSystemsPath(SGPath("systems"));

  if (!fdm_->LoadModel("c172p")) {
    throw std::runtime_error(std::format("JSBSim failed to load c172p from {}",
                                         jsbsim_root.string()));
  }

  if (options.stream_to_flightgear &&
      !fdm_->SetOutputDirectives(SGPath("data_output/flightgear.xml"))) {
    throw std::runtime_error(
        "JSBSim could not load data_output/flightgear.xml");
  }

  fdm_->Setdt(1.0 / kRateHz);

  const std::shared_ptr<JSBSim::FGInitialCondition> init = fdm_->GetIC();
  init->SetGeodLatitudeRadIC(ic.latitude_rad);
  init->SetLongitudeRadIC(ic.longitude_rad);
  init->SetTerrainElevationFtIC(ic.terrain_elevation_m * kFootPerMetre);
  init->SetAltitudeASLFtIC(ic.altitude_m * kFootPerMetre);
  init->SetVcalibratedKtsIC(ic.calibrated_airspeed_m_s *
                            kKnotPerMetrePerSecond);
  init->SetPsiRadIC(ic.heading_rad);
  init->SetFlightPathAngleRadIC(0.0);
  // Wind goes in the initial condition: trim re-initialises the atmosphere
  // from it, so wind set as a property before trim is cleared.
  init->SetWindNEDFpsIC(
      -ic.wind.speed_m_s * kFootPerMetre * std::cos(ic.wind.from_rad),
      -ic.wind.speed_m_s * kFootPerMetre * std::sin(ic.wind.from_rad), 0.0);

  fdm_->SetPropertyValue("fcs/mixture-cmd-norm", 1.0);
  fdm_->SetPropertyValue("propulsion/magneto_cmd", 3.0);

  if (!fdm_->RunIC()) {
    throw std::runtime_error("JSBSim rejected the initial condition");
  }

  fdm_->SetPropertyValue("propulsion/set-running", -1.0);

  if (ic.on_ground) {
    for (const CommandSpec& spec : kCommandSpecs) {
      fdm_->SetPropertyValue(spec.property, 0.0);
    }
  } else {
    try {
      fdm_->DoTrim(JSBSim::tFull);
    } catch (const JSBSim::BaseException& e) {
      throw std::runtime_error(
          std::format("trim failed at {:.1f} m and {:.1f} m/s: {}",
                      ic.altitude_m, ic.calibrated_airspeed_m_s, e.what()));
    }
  }

  std::ranges::transform(kCommandSpecs, commands_.begin(),
                         [this](const CommandSpec& spec) {
                           return BoundCommand{
                               .node = RequireNode(*fdm_, spec.property),
                               .member = spec.member,
                               .name = spec.name,
                               .low = spec.low,
                           };
                         });

  std::ranges::transform(kStateSpecs, truth_.begin(),
                         [this](const StateSpec& spec) {
                           return BoundTruth{
                               .node = RequireNode(*fdm_, spec.property),
                               .member = spec.member,
                               .scale = spec.scale,
                           };
                         });

  steer_node_ = RequireNode(*fdm_, "fcs/steer-cmd-norm");
  main_wow_node_ = RequireNode(*fdm_, "gear/unit[1]/WOW");
  nose_wow_node_ = RequireNode(*fdm_, "gear/unit[0]/WOW");
  specific_force_nodes_[0] =
      RequireNode(*fdm_, "accelerations/a-pilot-x-ft_sec2");
  specific_force_nodes_[1] =
      RequireNode(*fdm_, "accelerations/a-pilot-y-ft_sec2");
  specific_force_nodes_[2] =
      RequireNode(*fdm_, "accelerations/a-pilot-z-ft_sec2");
  engine_rpm_node_ = RequireNode(*fdm_, "propulsion/engine/engine-rpm");
}

Plant::~Plant() = default;

fsw::ActuatorCommands Plant::trim_commands() const {
  // Trim also sets fcs/pitch-trim-cmd-norm; elevator 0 means "at trim".
  fsw::ActuatorCommands controls{};
  for (const BoundCommand& command : commands_) {
    controls.*command.member = command.node->getDoubleValue();
  }
  return controls;
}

void Plant::step(const fsw::ActuatorCommands& controls) {
  for (const BoundCommand& command : commands_) {
    const double value = controls.*command.member;
    if (!std::isfinite(value)) {
      throw std::invalid_argument(
          std::format("control {} is not finite", command.name));
    }
  }

  for (const BoundCommand& command : commands_) {
    const double value = controls.*command.member;
    command.node->setDoubleValue(std::clamp(value, command.low, 1.0));
  }
  steer_node_->setDoubleValue(std::clamp(-controls.rudder, -1.0, 1.0));
  fdm_->SetPropertyValue("fcs/right-brake-cmd-norm",
                         std::clamp(controls.brakes, 0.0, 1.0));

  if (!fdm_->Run()) {
    throw std::runtime_error(
        std::format("JSBSim stopped at t = {} s", fdm_->GetSimTime()));
  }
}

fsw::VehicleState Plant::truth() const {
  fsw::VehicleState state{};
  for (const BoundTruth& field : truth_) {
    state.*field.member = field.node->getDoubleValue() * field.scale;
  }
  const bool main_one = main_wow_node_->getDoubleValue() > 0.5;
  SGPropertyNode* const main_two =
      fdm_->GetPropertyManager()->GetNode("gear/unit[2]/WOW");
  const bool main_two_wow =
      main_two != nullptr && main_two->getDoubleValue() > 0.5;
  state.main_gear_on_ground = main_one || main_two_wow;
  state.nose_gear_on_ground = nose_wow_node_->getDoubleValue() > 0.5;
  return state;
}

std::array<double, 3> Plant::specific_force() const {
  return {specific_force_nodes_[0]->getDoubleValue() * glidepath::fsw::kFoot,
          specific_force_nodes_[1]->getDoubleValue() * glidepath::fsw::kFoot,
          specific_force_nodes_[2]->getDoubleValue() * glidepath::fsw::kFoot};
}

void Plant::StopEngine() {
  fdm_->SetPropertyValue("fcs/mixture-cmd-norm", 0.0);
}

double Plant::engine_rpm() const {
  return engine_rpm_node_->getDoubleValue();
}

}  // namespace glidepath::sim