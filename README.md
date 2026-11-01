# fps-shooter

C++23 starter project using AppleClang, CMake, Ninja, and raylib.

## Requirements

- Xcode command line tools or Xcode
- CMake 3.28+
- Ninja
- raylib 5.5+

On macOS with Homebrew:

```sh
brew install cmake ninja raylib
```

## Build

```sh
cmake --preset debug
cmake --build --preset debug
```

The presets explicitly use `/usr/bin/clang` and `/usr/bin/clang++`, so builds use AppleClang even if another LLVM install appears earlier on `PATH`.

## Run

```sh
./build/debug/fps-shooter
```

## Controls

SKIFF is a single-engine tilt-jet: one engine with a vectoring nozzle (straight
down to hover, straight back to fly) and a fixed wing. There is no mode switch;
the same controls mean the same thing at every speed, and the craft blends from
jet-borne to wing-borne as it speeds up (25 to 45 m/s).

| Input | Does |
|---|---|
| Mouse | moves the aim point: the craft goes where you look |
| W / S | raise / lower the speed (S from a hover backs up) |
| let go of W / S | hold the current speed; below 125 km/h, come to a hover |
| A / D | strafe when slow, roll when fast |
| Space / Shift | climb / descend when slow |
| Left click | fire the cannon |
| Tab | live tuning panel |

Parked on the ground with hands off, the engine throttles back and shuts off
after 6 s. The craft is indestructible.

24 target drones float around the spawn point (red brackets on the HUD). Three hits
break one; it comes back after 8 s.

## How it flies

The pilot law (`game/player/pilot.cpp`) turns the inputs into a wanted
acceleration, then into the force the airframe must make (gravity included):

- **Hover:** a velocity along the aim (plus strafe and climb). The body tilts so
  the thrust lines up with what is needed (30 deg limit) and turns to face the aim.
  The view looks along the aim, like a free look.
- **Flight:** the flight path is turned onto the aim, up to a 6 g limit, measured
  as heading and elevation against the horizon, so a held mouse flies a level
  circle and a turn toward a point behind you doesn't climb. Near the vertical
  the shortest way round is used instead, so loops are continuous. The craft
  banks so the lift points along the needed force and raises the nose by the
  angle of attack that makes it. A point below is reached by pushing (2 g max).
  Upside down with nothing to pull toward, it rolls upright. The cockpit is
  bolted to the airframe and the aim point is a cyan circle.
- **Engine:** whatever the wing and hull don't provide, the engine does through
  the nozzle, so the flight path lands on the aim point without trim. At forward
  speed the nozzle makes the fore/aft push. Jet-borne it tips forward to brake;
  wing-borne an airbrake opens instead.

Tuning: the "Airframe", "Attitude loop", "Speed", "Hover" and "Flight" blocks of
`game/core/constants.hpp`. Physics checks: `./build/release/flight-sim`.

Above 250 km/h the grass only draws near the craft (fully at 400 km/h; see
`game/render/grass.cpp`). The HUD shows a compass tape,
artificial horizon with a flight-path marker, speed and altitude panels, the
speed set point, an RPM gauge and a top-down schematic with the nozzle angle.

## Audio

All sound is synthesised at runtime (`game/audio/audio.cpp`): a resonant saw-stack drive hum, rising whine, ignition thump and speed-dependent wind. No sample files.
