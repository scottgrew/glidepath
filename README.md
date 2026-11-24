# glidepath

C++ flight software for a Cessna 172 simulated in JSBSim. It takes off from Palo Alto (KPAO) runway 31, flies approaches, and lands on runway 31 in calm air and in 5 and 10 kt crosswinds. The autopilot flies on its own navigation estimate, computed from simulated sensors, not on the simulator's true state. FlightGear draws the aircraft; Lichtblick shows the telemetry.

## Scenarios

| name | what it flies |
|---|---|
| `hold`, `climb`, `turn`, `slow`, `tour` | cruise at 4000 ft with altitude, heading and airspeed changes |
| `approach` | a flight plan onto final for runway 31 and a low pass |
| `autoland` | final approach fix to a stop on runway 31 |
| `autoland_xwind_{left,right}_{5,10}` | the same with a 5 or 10 kt crosswind |
| `takeoff` | takeoff from runway 31 and climb to 1000 ft |

## Method

The flight software runs at 40 Hz. JSBSim steps the aircraft at 120 Hz.

**Control** (`fsw/autopilot`, `fsw/guidance`)
- Cascaded PID loops on elevator, aileron, rudder and throttle hold altitude, heading and airspeed targets.
- Flight plans are flown leg by leg with L1 lateral guidance (Park, Deyst and How, 2004).

**Landing and takeoff** (`fsw/landing`, `fsw/takeoff`, `fsw/ground`)
- Approach on the runway centerline and a 3° glideslope, flare at 6 m, touchdown, idle and brakes to a stop.
- In a crosswind the aircraft crabs into the wind on final, then holds the runway heading with rudder before touchdown and keeps the wings level on the rollout.
- Takeoff: ground roll on the centerline, rotation, climb-out and level-off at 1000 ft.

**Navigation** (`fsw/attitude`, `fsw/nav_filter`, `fsw/estimator`)
- Attitude: a Mahony complementary filter. The gyro is integrated and pulled toward gravity (accelerometer) and magnetic north (magnetometer). The aircraft's own acceleration, from successive GNSS velocities, is removed from the accelerometer first.
- Position and velocity: a linear Kalman filter over north, east, down position and velocity. IMU acceleration, rotated by the attitude, drives the prediction; GNSS position and velocity, barometric altitude and radar altitude (0.5 to 50 m) correct it.
- Airspeed from the pitot tube.

**Sensors** (`sim/sensors`): IMU at 40 Hz, GNSS at 10 Hz, barometer, magnetometer and pitot at 20 Hz, radar altimeter at 40 Hz. Each has white noise; the IMU also has a constant bias drawn at the start of the run.

## Build

macOS on Intel with Homebrew `cmake`, `zstd` and `lz4`. CMake fetches JSBSim, Eigen, Catch2, MCAP and the Foxglove SDK.

```
make build
make test      # unit tests and closed-loop flights
make check     # clang-format, clang-tidy
```

## Run

```
make flightgear                     # FlightGear 2024.1 as the 3D view
make fly                            # in a second terminal; default autoland_xwind_left_10
make fly SCENARIO=takeoff
make live SCENARIO=autoland         # telemetry for Lichtblick on ws://localhost:8765
make record SCENARIO=autoland       # telemetry to autoland.mcap
```

Open `docs/lichtblick_layout.json` in Lichtblick for the instrument layout. Every run also writes each 40 Hz cycle (true state, targets, commands) to `flight.csv`.

## Layout

```
fsw/    flight software; never includes simulator code
sim/    JSBSim plant and sensor model
app/    scenarios, the flight loop, CSV, Lichtblick and FlightGear output
tests/  unit tests and closed-loop flight tests (Catch2)
```
