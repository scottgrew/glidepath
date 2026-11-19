// Owns one JSBSim instance flying the Cessna 172P. The rest of glidepath
// sees only these SI-unit structs; feet, knots and degrees stay in sim.cpp.
#pragma once
#include <array>
#include <filesystem>
#include <memory>

namespace JSBSim {
class FGFDMExec;
}
class SGPropertyNode;

namespace glidepath {

inline constexpr double kSimRateHz = 120.0;  // JSBSim steps per second.
inline constexpr int kStepsPerCycle = 3;     // Autopilot runs at 40 Hz.

struct InitialCondition {
  double latitude_rad;  // Geodetic.
  double longitude_rad;
  double altitude_m;  // Above mean sea level; terrain is at sea level.
  double calibrated_airspeed_m_s;
  double heading_rad;  // True heading, 0 north, pi/2 east.
};

// Normalised commands in JSBSim's sign convention. elevator, aileron and
// rudder span [-1, 1]; throttle spans [0, 1]. Negative elevator raises the
// nose, positive aileron rolls right, positive rudder yaws the nose left.
struct Controls {
  double elevator;
  double aileron;
  double rudder;
  double throttle;
};

struct TruthState {
  double time_s;
  double latitude_rad;  // Geodetic.
  double longitude_rad;
  double altitude_m;  // Above mean sea level.
  double roll_rad;
  double pitch_rad;
  double heading_rad;      // [0, 2*pi).
  double roll_rate_rad_s;  // Body axes.
  double pitch_rate_rad_s;
  double yaw_rate_rad_s;
  double calibrated_airspeed_m_s;
  double climb_rate_m_s;  // Positive up.
  bool operator==(const TruthState&) const = default;
};

// One entry per TruthState field, in declaration order; the CSV header and
// rows are written from this table.
struct TruthField {
  const char* name;
  double TruthState::* member;
};

inline constexpr std::array<TruthField, 12> kTruthFields{{
    {.name = "time_s", .member = &TruthState::time_s},
    {.name = "latitude_rad", .member = &TruthState::latitude_rad},
    {.name = "longitude_rad", .member = &TruthState::longitude_rad},
    {.name = "altitude_m", .member = &TruthState::altitude_m},
    {.name = "roll_rad", .member = &TruthState::roll_rad},
    {.name = "pitch_rad", .member = &TruthState::pitch_rad},
    {.name = "heading_rad", .member = &TruthState::heading_rad},
    {.name = "roll_rate_rad_s", .member = &TruthState::roll_rate_rad_s},
    {.name = "pitch_rate_rad_s", .member = &TruthState::pitch_rate_rad_s},
    {.name = "yaw_rate_rad_s", .member = &TruthState::yaw_rate_rad_s},
    {.name = "calibrated_airspeed_m_s",
     .member = &TruthState::calibrated_airspeed_m_s},
    {.name = "climb_rate_m_s", .member = &TruthState::climb_rate_m_s},
}};

class Sim {
 public:
  Sim(const std::filesystem::path& jsbsim_root, const InitialCondition& ic);
  ~Sim();
  Sim(const Sim&) = delete;
  Sim& operator=(const Sim&) = delete;
  Sim(Sim&&) = delete;
  Sim& operator=(Sim&&) = delete;

  // Returns the normalised control positions JSBSim found at trim.
  Controls trim_controls() const;
  // Applies controls for one simulation step at kSimRateHz.
  void step(const Controls& controls);
  // Returns the current truth state in SI units.
  TruthState truth() const;

 private:
  struct BoundCommand {
    SGPropertyNode* node;
    double Controls::* member;
    const char* name;  // For error messages.
    double low;        // Lower limit; the upper limit is 1 for every command.
  };
  struct BoundTruth {
    SGPropertyNode* node;
    double TruthState::* member;
    double scale;  // Multiplies JSBSim's value into SI units.
  };

  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  std::array<BoundCommand, 4> commands_{};
  std::array<BoundTruth, 12> truth_{};
};

}  // namespace glidepath