#include "sim.hpp"

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

constexpr double kFoot = 0.3048;
constexpr double kFootPerMetre = 1.0 / kFoot;
constexpr double kKnotPerMetrePerSecond = 3600.0 / 1852.0;

struct CommandSpec {
  const char* property;
  double glidepath::Controls::* member;
  const char* name;
  double low;
};

struct TruthSpec {
  const char* property;
  double glidepath::TruthState::* member;
  double scale;
};

constexpr std::array<CommandSpec, 4> kCommandSpecs{{
    {.property = "fcs/elevator-cmd-norm",
     .member = &glidepath::Controls::elevator,
     .name = "elevator",
     .low = -1.0},
    {.property = "fcs/aileron-cmd-norm",
     .member = &glidepath::Controls::aileron,
     .name = "aileron",
     .low = -1.0},
    {.property = "fcs/rudder-cmd-norm",
     .member = &glidepath::Controls::rudder,
     .name = "rudder",
     .low = -1.0},
    {.property = "fcs/throttle-cmd-norm",
     .member = &glidepath::Controls::throttle,
     .name = "throttle",
     .low = 0.0},
}};

constexpr std::array<TruthSpec, 12> kTruthSpecs{{
    {.property = "simulation/sim-time-sec",
     .member = &glidepath::TruthState::time_s,
     .scale = 1.0},
    {.property = "position/lat-geod-rad",
     .member = &glidepath::TruthState::latitude_rad,
     .scale = 1.0},
    {.property = "position/long-gc-rad",
     .member = &glidepath::TruthState::longitude_rad,
     .scale = 1.0},
    {.property = "position/h-sl-meters",
     .member = &glidepath::TruthState::altitude_m,
     .scale = 1.0},
    {.property = "attitude/phi-rad",
     .member = &glidepath::TruthState::roll_rad,
     .scale = 1.0},
    {.property = "attitude/theta-rad",
     .member = &glidepath::TruthState::pitch_rad,
     .scale = 1.0},
    {.property = "attitude/psi-rad",
     .member = &glidepath::TruthState::heading_rad,
     .scale = 1.0},
    {.property = "velocities/p-rad_sec",
     .member = &glidepath::TruthState::roll_rate_rad_s,
     .scale = 1.0},
    {.property = "velocities/q-rad_sec",
     .member = &glidepath::TruthState::pitch_rate_rad_s,
     .scale = 1.0},
    {.property = "velocities/r-rad_sec",
     .member = &glidepath::TruthState::yaw_rate_rad_s,
     .scale = 1.0},
    {.property = "velocities/vc-fps",
     .member = &glidepath::TruthState::calibrated_airspeed_m_s,
     .scale = kFoot},
    {.property = "velocities/h-dot-fps",
     .member = &glidepath::TruthState::climb_rate_m_s,
     .scale = kFoot},
}};

SGPropertyNode* RequireNode(const JSBSim::FGFDMExec& fdm, const char* path) {
  SGPropertyNode* const node = fdm.GetPropertyManager()->GetNode(path);
  if (node == nullptr) {
    throw std::runtime_error(std::format("JSBSim has no property {}", path));
  }
  return node;
}

}  // namespace

namespace glidepath {

Sim::Sim(const std::filesystem::path& jsbsim_root, const InitialCondition& ic) {
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

  fdm_->Setdt(1.0 / kSimRateHz);

  const std::shared_ptr<JSBSim::FGInitialCondition> init = fdm_->GetIC();
  init->SetGeodLatitudeRadIC(ic.latitude_rad);
  init->SetLongitudeRadIC(ic.longitude_rad);
  init->SetTerrainElevationFtIC(0.0);
  init->SetAltitudeASLFtIC(ic.altitude_m * kFootPerMetre);
  init->SetVcalibratedKtsIC(ic.calibrated_airspeed_m_s *
                            kKnotPerMetrePerSecond);
  init->SetPsiRadIC(ic.heading_rad);
  init->SetFlightPathAngleRadIC(0.0);

  fdm_->SetPropertyValue("fcs/mixture-cmd-norm", 1.0);
  fdm_->SetPropertyValue("propulsion/magneto_cmd", 3.0);

  if (!fdm_->RunIC()) {
    throw std::runtime_error("JSBSim rejected the initial condition");
  }

  fdm_->SetPropertyValue("propulsion/set-running", -1.0);

  try {
    fdm_->DoTrim(JSBSim::tFull);
  } catch (const JSBSim::BaseException& e) {
    throw std::runtime_error(
        std::format("trim failed at {:.1f} m and {:.1f} m/s: {}", ic.altitude_m,
                    ic.calibrated_airspeed_m_s, e.what()));
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

  std::ranges::transform(kTruthSpecs, truth_.begin(),
                         [this](const TruthSpec& spec) {
                           return BoundTruth{
                               .node = RequireNode(*fdm_, spec.property),
                               .member = spec.member,
                               .scale = spec.scale,
                           };
                         });
}

Sim::~Sim() = default;

Controls Sim::trim_controls() const {
  // Trim also sets fcs/pitch-trim-cmd-norm; elevator 0 means "at trim".
  Controls controls{};
  for (const BoundCommand& command : commands_) {
    controls.*command.member = command.node->getDoubleValue();
  }
  return controls;
}

void Sim::step(const Controls& controls) {
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

  if (!fdm_->Run()) {
    throw std::runtime_error(
        std::format("JSBSim stopped at t = {} s", fdm_->GetSimTime()));
  }
}

TruthState Sim::truth() const {
  TruthState state{};
  for (const BoundTruth& field : truth_) {
    state.*field.member = field.node->getDoubleValue() * field.scale;
  }
  return state;
}

}  // namespace glidepath