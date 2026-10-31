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

SKIFF is a hover craft: it drives like a car on the ground, glides on rigid
wings, and flies out on a jet.

| Input | Hover (ground) | Air |
|---|---|---|
| W / S | throttle / brake, then reverse | W jet (spools to turbo), S reverse-thrust brake |
| A / D | steer | roll by hand |
| Shift | handbrake (drift) | lift jet |
| Space | thruster hop (wings open if you are fast enough) | open wings |
| Mouse | look (yaw and pitch) | moves the aim circle; the craft rolls and pulls its nose onto it |

Hop or launch off a crest above 18 m/s and the wings open on their own. Landing
faster than 10 m/s into the ground is a hull breach and respawns you on the
platform. The cockpit HUD shows a compass tape, artificial horizon with a
flight-path marker, speed and altitude panels, an RPM gauge with the turbo zone,
a fuel cell, and a top-down schematic of the skiff (wings, hover pads, exhaust).

## Mouse-aim flight

In the air the mouse moves a cyan circle (the aim point) around the cockpit and
the craft flies to it, War Thunder style. Far from the circle it rolls its lift
side toward it and pulls; near it, it levels out and trims so the flight path
(the amber marker), not just the nose, lands on the circle. The pull fades before
the wing stalls, so a full pull is the hardest turn the wing allows. The circle
may lead the nose by up to 100 degrees, so holding the mouse right flies a level
circle and holding it up flies a loop. Mouse X turns the circle about the world's
vertical (horizon-referenced), mouse Y pitches it about the wing axis. Tuning:
the `kAim*` / `kFlight*` constants in `src/core/constants.hpp`, the loop in
`src/player/flight.cpp`.

Above 250 km/h the scene gets a zoom blur (full at 400 km/h), and the grass only
draws near the craft (see `src/render/motion.hpp`).

## Engine and turbo

The jet engine has an RPM. Any jet input (W, Shift, S) lights it and it idles at
2,500 rpm; with no input it shuts off after 6 s. Holding W spools it up at
3,000 rpm/s. Thrust is 1x until 6,000 rpm, then the turbo ramps it linearly to
2x at 8,000 rpm, and it eases toward 3x at the 12,000 rpm redline. Thrust follows
RPM, so letting go of W spools the power back down (5,000 rpm/s). Turbo burns up
to twice the fuel. Shift and S hold the engine at 5,500 rpm, just under the
turbo. Tuning: the RPM constants in `src/core/constants.hpp`.

## Audio

All sound is synthesised at runtime (`src/audio/audio.cpp`): a resonant saw-stack drive hum, rising whine, ignition thump and speed-dependent wind. No sample files.
