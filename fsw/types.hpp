// Interface types of the flight software. Nothing under fsw/ includes the
// simulator or JSBSim; the simulator adapts to these types.
#pragma once
#include <array>
#include <cstdint>

namespace glidepath::fsw {

// How the rudder is commanded in the air. kCoordinated: yaw damper against
// the coordinated-turn yaw rate (today's law). kHeading: rudder holds the
// nose on yaw_heading_rad (the de-crab before touchdown in a crosswind).
enum class YawMode : std::uint8_t { kCoordinated, kHeading };

inline constexpr double kControlRateHz = 40.0;  // Flight software cycle.

// Normalised surface and throttle commands. elevator, aileron and rudder span
// [-1, 1]; throttle spans [0, 1]. Negative elevator raises the nose, positive
// aileron rolls right, positive rudder yaws the nose left.
struct ActuatorCommands {
  double elevator;
  double aileron;
  double rudder;
  double throttle;
  double flaps;   // [0, 1]
  double brakes;  // [0, 1], both main wheels
};

// Aircraft state in SI units. The simulator reports the true state in this
// form; the flight software's estimate uses the same form.
struct VehicleState {
  double time_s;
  double latitude_rad;  // Geodetic.
  double longitude_rad;
  double altitude_m;  // Above mean sea level.
  double roll_rad;
  double pitch_rad;
  double heading_rad;      // True, [0, 2*pi), 0 north, pi/2 east.
  double roll_rate_rad_s;  // Body axes.
  double pitch_rate_rad_s;
  double yaw_rate_rad_s;
  double calibrated_airspeed_m_s;
  double climb_rate_m_s;      // Positive up.
  double north_velocity_m_s;  // Over the ground.
  double east_velocity_m_s;
  double height_above_ground_m;  // CG above the terrain below it.
  bool main_gear_on_ground;      // Either main wheel (squat switch).
  bool nose_gear_on_ground;
  bool operator==(const VehicleState&) const = default;
};

// Names one double member of a record, for logs and telemetry.
template <typename Record>
struct Field {
  const char* name;
  double Record::* member;
};

inline constexpr std::array<Field<VehicleState>, 15> kStateFields{{
    {.name = "time_s", .member = &VehicleState::time_s},
    {.name = "latitude_rad", .member = &VehicleState::latitude_rad},
    {.name = "longitude_rad", .member = &VehicleState::longitude_rad},
    {.name = "altitude_m", .member = &VehicleState::altitude_m},
    {.name = "roll_rad", .member = &VehicleState::roll_rad},
    {.name = "pitch_rad", .member = &VehicleState::pitch_rad},
    {.name = "heading_rad", .member = &VehicleState::heading_rad},
    {.name = "roll_rate_rad_s", .member = &VehicleState::roll_rate_rad_s},
    {.name = "pitch_rate_rad_s", .member = &VehicleState::pitch_rate_rad_s},
    {.name = "yaw_rate_rad_s", .member = &VehicleState::yaw_rate_rad_s},
    {.name = "calibrated_airspeed_m_s",
     .member = &VehicleState::calibrated_airspeed_m_s},
    {.name = "climb_rate_m_s", .member = &VehicleState::climb_rate_m_s},
    {.name = "north_velocity_m_s", .member = &VehicleState::north_velocity_m_s},
    {.name = "east_velocity_m_s", .member = &VehicleState::east_velocity_m_s},
    {.name = "height_above_ground_m",
     .member = &VehicleState::height_above_ground_m},
}};

inline constexpr std::array<Field<ActuatorCommands>, 6> kCommandFields{{
    {.name = "elevator", .member = &ActuatorCommands::elevator},
    {.name = "aileron", .member = &ActuatorCommands::aileron},
    {.name = "rudder", .member = &ActuatorCommands::rudder},
    {.name = "throttle", .member = &ActuatorCommands::throttle},
    {.name = "flaps", .member = &ActuatorCommands::flaps},
    {.name = "brakes", .member = &ActuatorCommands::brakes},
}};

}  // namespace glidepath::fsw