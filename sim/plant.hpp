// The JSBSim Cessna 172P plant. Only sim/plant.cpp includes JSBSim; feet,
// knots and degrees stay there.
#pragma once
#include "fsw/types.hpp"

#include <array>
#include <filesystem>
#include <memory>

namespace JSBSim {
class FGFDMExec;
}
class SGPropertyNode;

namespace glidepath::sim {

inline constexpr double kRateHz = 120.0;  // JSBSim steps per second.

// A steady wind: speed over the ground and the true direction it blows from.
struct Wind {
  double speed_m_s = 0.0;
  double from_rad = 0.0;
};

struct InitialCondition {
  double latitude_rad{};  // Geodetic.
  double longitude_rad{};
  double altitude_m{};  // Above mean sea level; terrain is at sea level.
  double calibrated_airspeed_m_s{};
  double heading_rad{};  // True heading, 0 north, pi/2 east.
  double terrain_elevation_m = 0.0;
  // Start on the runway at rest; JSBSim cannot trim at 0 kt (it throws at
  // 10 kt), so the plant skips trim and the caller settles the gear first.
  bool on_ground = false;
  Wind wind{};
};

struct PlantOptions {
  // Streams the state to FlightGear over UDP on localhost:5550 at 60 Hz of
  // simulation time, using JSBSim's data_output/flightgear.xml.
  bool stream_to_flightgear = false;
};

class Plant {
 public:
  Plant(const std::filesystem::path& jsbsim_root, const InitialCondition& ic,
        const PlantOptions& options = {});
  ~Plant();
  Plant(const Plant&) = delete;
  Plant& operator=(const Plant&) = delete;
  Plant(Plant&&) = delete;
  Plant& operator=(Plant&&) = delete;

  // Returns the normalised control positions JSBSim found at trim.
  fsw::ActuatorCommands trim_commands() const;
  // Applies controls for one simulation step at kRateHz.
  void step(const fsw::ActuatorCommands& controls);
  // Returns the current truth state in SI units.
  fsw::VehicleState truth() const;
  // Body specific force without gravity, m/s^2.
  std::array<double, 3> specific_force() const;

 private:
  struct BoundCommand {
    SGPropertyNode* node;
    double fsw::ActuatorCommands::* member;
    const char* name;  // For error messages.
    double low;        // Lower limit; the upper limit is 1 for every command.
  };
  struct BoundTruth {
    SGPropertyNode* node;
    double fsw::VehicleState::* member;
    double scale;  // Multiplies JSBSim's value into SI units.
  };

  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  std::array<BoundCommand, 6> commands_{};
  std::array<BoundTruth, 15> truth_{};
  SGPropertyNode* steer_node_ = nullptr;
  SGPropertyNode* main_wow_node_ = nullptr;
  SGPropertyNode* nose_wow_node_ = nullptr;
  std::array<SGPropertyNode*, 3> specific_force_nodes_{};
};

}  // namespace glidepath::sim