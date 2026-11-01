# SKIFF agent wiki

Orientation for an agent (or human) picking up this repo. Written from the working
session that built it; verify against the code before trusting a detail, the
constants especially (`game/core/constants.hpp` is the source of truth).

## What it is

A C++23 / raylib 5.5 game, originally an FPS prototype (dir `fps-shooter`, target
`fps-shooter`, window title "SKIFF"). The player is now a **hover skiff**:
it hovers like a game character and, after an explicit conversion (F), flies like a
jet. The world is a huge procedural, painterly terrain. Everything is procedural: no
art, audio or model assets exist. Git repo; the current controller lives on the
`new-flight-controller` branch (2026-10-31).

- Platform: macOS (Apple M2, AppleClang), OpenGL 4.1 via raylib, GLSL `#version 330`.
- Units: **1 unit = 1 m**, SI throughout. Gravity 9.81. Mass 80 kg.
- Window 1280x720 (`kScreenWidth/Height`); render target is the same size (HiDPI
  screenshots come out 2560x1440).
- World axes: Y up. **North is -Z**, east +X. Yaw 0 looks along +X, yaw -90 looks
  along -Z (north, the spawn heading). Increasing yaw turns right.

## Build, run, test

```
./build.sh debug | release | run        # or: cmake --build --preset debug|release [--target flight-sim]
./build/release/fps-shooter
./build/release/flight-sim              # headless physics checks, prints PASS/FAIL, "0 check(s) failed" when green
```

- Targets: `fps-shooter` (game) and `flight-sim` (`tools/flight_sim.cpp` linked
  against `skiff-sim`; no rendering). See the source map for `engine` / `skiff-sim`.
- raylib comes from Homebrew (`/opt/homebrew`), found via `find_package`, with a
  FetchContent fallback. Threads are linked (terrain workers).
- Release build has one known, harmless warning: unused parameter in
  `drawCollisionDebug`.
- Run `flight-sim` after any change to `game/player/*`, `collision/*`, `world/terrain.*`
  or physics constants. It is the regression suite; extend it when adding mechanics.
- Visual checks: temporarily hack a screenshot into `main.cpp` (raylib's
  `TakeScreenshot("x.png")` saves into the **cwd by basename**), run from the
  scratchpad dir, read the PNG, then **remove the hook** and verify with
  `grep -c getenv game/main.cpp`.
- Shell is zsh with fish login. zsh does not word-split `$var`; macOS `sed -i ''`
  is fiddly; prefer Python for multi-line edits.

## Source map

The code is split into an **engine** (`engine/`, a static library, namespace
`engine`) and the **game** (`game/`). Hard rule: nothing in `engine/` includes a
game header; the `engine` target only has `engine/include` on its include path, so
the compiler enforces it. Move code into the engine once it is clearly not about
SKIFF, or a second user needs it.

CMake targets: `engine` (generic), `skiff-sim` (SKIFF's simulation: no window or
GPU), `fps-shooter` (the game: rendering, audio, camera, input, `main`) and
`flight-sim` (headless checks). Both executables link `skiff-sim`.

| Path | Role |
|---|---|
| `engine/include/engine/app.hpp`, `engine/src/app.cpp` | `engine::run(config, game)`: window, audio device, fixed-timestep loop with interpolation alpha. The game implements `engine::Game` (`init`, `input`, `fixedUpdate`, `update`, `render`, `shutdown`) |
| `engine/.../input/actions.{hpp,cpp}` | `ActionMap<Action>`: named actions bound to keys / mouse buttons; `down` / `pressed` / `released`, `update()` once per frame |
| `engine/.../math/attitude.{hpp,cpp}` | `Attitude` frame, bank / pitch / heading, `smoothstep`, `wrapAngle` |
| `engine/.../math/noise.hpp` | `engine::noise`: hashes, gradient noise, fbm, erosion |
| `engine/.../collision/capsule.{hpp,cpp}` | `Capsule`, capsule vs AABB penetration / resolution / grounding |
| `engine/.../tuning/tuning.{hpp,cpp}` | `TuningParam` table (pointers to the game's floats), save / load / reset |
| `engine/.../tuning/panel.{hpp,cpp}` | `TuningPanel`: slider panel for any `TuningParam` table, one tab per group |
| `game/main.cpp` | `Skiff : engine::Game`: glue between input, `stepPlayer`, camera, render, audio, panel |
| `game/core/controls.{hpp,cpp}` | **Controls**: the `Action` enum and the default key table. Game code reads actions, never raw keys |
| `game/core/constants.hpp` | Fixed constants (screen, ground, engine gauge, landing) |
| `game/core/tuning.{hpp,cpp}` | **Live flight tuning** (hover, conversion, jet, g): `FLIGHT_TUNING` X-macro list, `tuning` global, `kFlightTuningParams` (the engine table), `jetDrag` |
| `game/player/player.{hpp,cpp}` | `Player` state, `PlayerInput`, `stepPlayer` |
| `game/player/hover.{hpp,cpp}` | Hover: kinematic game controller (free look, WASD velocity ramps) |
| `game/player/jet.{hpp,cpp}` | Jet: kinematic, spring-loaded stick, auto-bank, energy |
| `game/player/gload.{hpp,cpp}` | Felt g, the g dose, greyout / blackout / redout levels |
| `game/player/input.{hpp,cpp}` | Copies flight actions into `PlayerInput` (the sim's input, which `flight-sim` scripts directly) |
| `game/collision/` | Player capsule, player vs level boxes + terrain contact, `queryGround`, `snapToGround` |
| `game/camera/camera.{hpp,cpp}` | `computeView` (hover look / jet cockpit blend, g shake, fov) |
| `game/render/render.{hpp,cpp}` | `renderWorld` and the whole cockpit HUD |
| `game/render/scenery.{hpp,cpp}` | Streaming terrain/tree LOD system, post-processing |
| `game/render/shaders.hpp` | Embedded GLSL: world, sky, Kuwahara post |
| `game/world/terrain.{hpp,cpp}` | `terrainHeight`, `terrainNormal`, `surfaceFriction`, trees |
| `game/world/level.cpp`, `colliders.hpp` | Spawn platform and box obstacles |
| `game/audio/audio.{hpp,cpp}` | Fully synthesised audio |
| `tools/flight_sim.cpp` | Headless checks |

## Main loop (`engine::run` in `engine/src/app.cpp`, driving `Skiff` in `game/main.cpp`)

The engine owns the loop; the steps below are which `Skiff` hook does each.

1. `init`: `initAudio`, `initScenery`, create `Player`, `preloadScenery(camera)` ("Generating world...").
2. `input` (per frame): `controls.update()`; Tab (`Action::TuningPanel`) toggles the panel; mouse delta accumulated into `input.mouseDelta`, F presses latched into
   `input.convert`; both are consumed (cleared) by the next physics step. `samplePlayerInput`.
3. `fixedUpdate`: the engine's fixed-timestep accumulator (`AppConfig::fixedDt`, set to
   `kPhysicsDt = 1/120`) calls it to run `stepPlayer`; frame
   time clamped to 0.1 s. `previous`/`current` states are blended for rendering
   (`computeView(prev, cur, alpha)`).
4. `update`: `updateAudio(player)`, `syncCamera`. `render`: `renderWorld` + `drawHud(player, camera)`
   + the tuning panel.
5. `AppConfig::targetFps = 60`. This was lowered from 144 to cut heat (the GPU was saturated).

## Player physics (`stepPlayer`)

**Rebuilt from scratch (2026-10-31, branch `new-flight-controller`).** User brief:
"on hover mode it should feel like a game and on flight mode it should feel like a
jet". The previous force-based single blended law (vectoring engine, wing, FBW
attitude loop, mouse-aim point) is gone (git history on `main`). Two kinematic
controllers and an explicit conversion instead; no aero simulation. `stepPlayer` is
pure (no raylib IO) so the sim can script it. `Player.position` is the **eye**
position; the craft is indestructible.

Per step: `stepConversion` (mode machine) -> `stepJet` or `stepHover` (with a hand-over
authority from `convert`) -> integrate position -> `resolvePlayerCollisions` (impact >
`kCrashSpeed` shows HULL BREACH) -> touching the ground in jet mode drops straight back
to hover -> Landed/Hover label -> felt g into `stepGLoad` -> engine rpm.

### Modes and conversion (`player.cpp`)
`FlightMode`: Landed, Hover, ToJet, Jet, ToHover. **F arms** the next conversion (press
again to disarm); the **speed gate** starts it: hover -> jet once forward speed >=
`kConvertSpeed` 30 m/s (hover top speed is 40, so arm and hold W), jet -> hover once
speed <= `kConvertBackSpeed` 70 m/s (S to idle opens the airbrake). `convert` ramps
0 <-> 1 over `kConvertTime` 1.5 s (nozzle swing: HUD, camera blend, audio), and the new
controller's authority ramps with it. `enterJet` points the nose along the look and sets
the throttle that holds the speed; `enterHover` points the look along the nose.

### Hover (`hover.cpp`): a game
Mouse = free look (`lookHeading`, `lookPitch`, +-85 deg). W/S/A/D ask for a velocity
relative to the look heading (40 fwd, 12 back, 18 strafe m/s), Space/Shift climb 12 /
descend 10. The velocity ramps to it at `kHoverAccel` 22 up, `kHoverBrake` 32 down (letting
go stops in ~1.2 s), vertical 18 m/s^2. No gravity, no forces: the engine just holds you.
The body faces the look heading and leans into the acceleration (cosmetic, max 18 deg).

### Jet (`jet.cpp`): kinematic with momentum (direction "C")
- **Stick:** the mouse deflects a spring-loaded stick (`kStickSensitivity` 0.004/count,
  springs back with `kStickReturnTau` 0.25 s, |stick| <= 1). Turning takes continuous
  movement; a flick turns ~15 deg and stops (user liked this spring feel on the old
  build).
- **Rates:** stick X = heading rate (about world up), stick Y = pitch about the wing.
  Full stick = `kJetMaxRateDeg` 80 deg/s, capped by the g allowed at this speed
  (`kJetMaxG` 10 -> at 125 m/s ~45 deg/s; corner speed ~70 m/s), softened below the stall.
- **Auto-bank:** rolls to the coordinated bank of the heading turn (atan(v w / g), max
  85 deg); fades near the vertical. **A/D roll by hand** (3.5 rad/s); letting go holds
  that bank, which then eases back over `kRollTrimReturnTau` 2 s.
- **Flight path** follows the nose (`kJetAlign` 6/s): a little lag / drift.
- **Speed** is one number: `kJetThrustAccel` 18 m/s^2 x throttle (W/S, 0.7/s; it holds when
  let go) minus drag (top level speed 170 m/s), minus g sin(climb), minus turn bleed
  `kJetTurnBleed` x g^2 (10 g costs 14 m/s^2). S at idle = airbrake (3x drag).
- **Stall:** below `kJetStallSpeed` 45 m/s the wing carries (v/vs)^2 of the weight; the path
  and the nose sag with the rest.
- **Soft loss of control (blackout):** max g eases from 10 to `kGLocMaxG` 6 and stick
  authority drops 40% as `blackout` goes 0 -> 1. Held full pull settles ~5-6 g, blind.

### g (`gload.cpp`)
Felt g = (acceleration + 1 g up) . body up / g, smoothed 0.08 s (collision jolts are
skipped). **Dose model** (user: "exceeds 10g or sustains 6g+ for 2-3 s" -> start
blacking out): above `kGOnset` 5 g the dose grows at (g - 5)^1.2 per s; below 4 g it
drains 3/s. greyout = smoothstep(2, 5, dose), blackout (tunnel then black) =
smoothstep(5, 12, dose). So 6 g greys at ~2.4 s, tunnels at ~5 s, black at ~12 s; 10 g
greys at ~0.4 s, black at ~1.8 s; black to clear ~4 s. Negative g below -2 builds a
redout dose (faster). `shake` = smoothstep(4, 11, g). A full-stick 180 at 450 km/h greys
out and starts the tunnel (5.4 s); near corner speed (290 km/h) it is 2.7 s.

Test footing: `flight-sim` air tests run at 1500 m.

### Live tuning (Tab)
Every hover / conversion / jet / g number is a runtime field of `tuning` (`game/core/tuning.hpp`),
declared once in the `FLIGHT_TUNING(X)` list as (group, name, default, min, max, label,
unit); the struct, the engine table (`kFlightTuningParams`), and so the panel rows and
`tuning.txt`, all come from that list, so **add a
tunable by adding one line there**. The jet drag is derived (`jetDrag()` from thrust and
top speed). Tab opens the panel (cursor freed, mouse stops flying, sim keeps running):
drag / click = set, Shift = fine, wheel = nudge, right click = default, changed values
are amber with a white default tick. Save writes `tuning.txt` in the working directory
and `main` loads it at startup; the sim always uses the defaults. To ship a tuned feel,
copy the values from `tuning.txt` into the defaults in the list.

### Not yet done
Hand-tuning (all numbers are from the sim, not hand-play), a craft model, landing gear /
proper collider, the conversion's own sound, breathing / g-strain audio, a proper
desaturation post-pass (greyout is an overlay for now).

## Collision (`engine/.../collision/` shapes, `game/collision/` player)
Capsule vs level boxes (`world/level.cpp` platform and obstacles) and vs terrain via
the analytic `terrainHeight`/`terrainNormal`. `queryGround(player, tolerance)` returns
a `GroundContact` (grounded, normal, friction); `snapToGround` keeps the car on the
surface while grounded. Trees have **no collision**.

## Terrain and scenery

`terrainHeight(x, z)` (`world/terrain.cpp`) is analytic: gradient noise with analytic
derivatives, erosion-style fBm, domain warp and a continent mask; mountains up to
~1100 m. Spawn is on a flat platform at the origin; mountains start 200-900 m away.
Because it is analytic, physics and rendering agree, and there is no heightmap to
stream.

Rendering (`render/scenery.cpp`):
- Quadtree LOD terrain tiles: 64 m base tile, 32x32 cells, root level 6 (4096 m),
  `kViewDistance` 7000 m, `kSplitFactor` 2.2, with skirts to hide cracks.
- Worker threads (`std::jthread`) generate meshes from a priority queue (nearest
  first). Workers poll with an 8 ms sleep when idle (was 1 ms; it burned CPU).
- Tree chunks on a 128 m grid with two detail tiers (full under ~320 m, simple to
  ~1200 m).
- **Cache rules learned the hard way** (do not regress): hysteresis `kHysteresis`
  1.3 on split/detail decisions; children beyond view distance must be skipped when
  deciding to split; coarse parents are "touched" while their children draw and
  `drawResident()` is a fallback to avoid holes; eviction is **LRU against a vertex
  budget** (`kVertexBudget` 14M), not time-based; CPU mesh copies freed after upload.
  Known limitation: LOD swaps visibly pop (no morphing).
- Clip planes 0.2..12000, fog density 0.00035, sky sphere radius 8000, drawn first
  with depth writes off.
- Float precision degrades far from the origin.

Look (`render/shaders.hpp`): world shader = banded toon lighting + brush-stroke
noise (fine strokes skipped beyond ~500 m) + cool shadows + haze; sky shader =
gradient + cel-shaded clouds + sun; post = Kuwahara oil-paint filter (R=2 taps with
2-texel stride, 36 samples) + paper grain + vignette. `beginScene/drawLandscape/
endScene` render into a `RenderTexture` and apply the post shader to the screen quad;
the **HUD is drawn after the post pass**, so it is not painterly-filtered.

### Performance notes
Measured on M2 at 1280x720: render was 14.6 ms (post filter ~7 ms, landscape ~6 ms);
after the Kuwahara, shader and sky trims it is ~7.5 ms, and the frame is capped at
60 fps. To measure GPU time honestly, flush and read a pixel after the render
(`rlDrawRenderBatchActive` + `rlReadScreenPixels`); vsync otherwise hides it.
Further options if heat remains: render the world at 75% scale, simpler tree shader,
a quality toggle.

## Camera (`camera.cpp`)
`computeView` returns position/forward/up/fov. Hovering it looks along the look angles
with 35% of the body's lean; in the jet it is bolted to the body; `convert` blends the
two. g effects: shake (incommensurate sines, ~0.9 deg and 4 cm at full `load.shake`)
and a 6 deg FOV squeeze from 3 to 9 g. FOV widens with speed.

## HUD (`render.cpp`, bottom half of the file)
All 2D, drawn with raylib primitives and the default font. Pieces: compass tape
(heading from camera forward, north = -Z), pitch ladder rotated by camera roll
(`roll = atan2(-right.y, up.y)`, drawn rotated by -roll), boresight reticle,
flight-path marker (`GetWorldToScreen` of the velocity direction), speed panel
(km/h, m/s, "Mach" = v/340), altitude panel (AGL, ASL, V/S, L/D capped at 99.9),
mode panel (mode, F-armed gate, throttle / airbrake, nozzle-swing bar), RPM arc gauge,
ship schematic (nozzle angle = convert x 90 deg), g readout + dose bar left of the
reticle, HULL BREACH overlay, per-mode control hints top-left. **g effects are drawn
last, over the HUD** (`drawGEffects`): grey wash, then a radial-gradient vignette
texture that closes in (tunnel), then black; redout is a red wash + red tunnel. Palette:
cyan/amber/red on translucent dark panels with corner brackets.

## Audio (`audio.cpp`)
No samples. One mono 32-bit float `AudioStream` (44.1 kHz, buffer 2048) filled by a
callback on the audio thread (`synth`). The game thread publishes atomics once a
frame in `updateAudio`: `gThrottle` (0 off, 1 wing jet, 2 heavy), `gRpm`, `gTurbo`,
`gHover`, `gWind` (airspeed), `gMuffle` (grounded), `gDim` (g effects: the whole mix goes
through a two-pole low-pass closing 12 kHz -> 300 Hz and ducks 65%), `gHeart` (a lub-dub
heartbeat at 52 Hz, 1.1 -> 2.3 beats/s, from the g dose).
Layers: drive hum (detuned saws + sub through a resonant state-variable filter, with
a slow throb), rising sine whine, soft air hiss, ignition thump, and turbo layers
(octave-up saw growl, sweeping siren, rushing air, a wastegate "pssh" + kick when the
turbo engages, a chopped compressor surge when it cuts out), hover hum, and speed
wind. A `tanh` limiter closes the chain. The design goal was "quiet, industrial,
sci-fi", explicitly not a fan/roar; keep levels low. To sanity-check without ears,
`#include "audio/audio.cpp"` in a scratch program, call `synth` on a buffer and check
peak/RMS and NaNs.

## Headless sim (`tools/flight_sim.cpp`)
Sections: hover W / let go, strafe / climb / look, landing and idle-down; conversion
(armed at rest stays hover, gate, 1.5 s swing) and back (only below the gate, ends
stopped); jet hands-off cruise, flick and release, swiped 90s, full-stick 180 at 450 and
at 290 km/h, pitch swipe and release, climb bleed, throttle / idle / airbrake, stall sag,
hand roll; g dose timings (6 g, 10 g, 4.5 g, -4 g, recovery), soft loss under a held pull,
hover never builds a dose. Ranges encode the intended feel; when you change tuning,
update the expectations deliberately, not to silence a failure.

## Controls (current)
Hover: mouse = look, WASD = move (let go: stop), Space/Shift = climb/descend.
Jet: mouse = spring-loaded stick (X turns with auto-bank, Y pitches), W/S = throttle
(S at idle: airbrake), A/D = roll. F = arm the conversion either way (speed-gated).

## History and design decisions (why things are the way they are)
1. Painterly procedural world (no assets). 2. Streaming huge terrain. 3. Fixed the
   "terrain keeps regenerating" bug (see cache rules). 4. SI physics rework: removed
   the hover jetpack, wingsuit aero, fixed timestep. 5. Ground launch: jump + W with
   a 3000 N launch burn; later wings deploy during the burn. 6. Jet tuning: +50%
   thrust, 2x fuel, brake on S, lift on Shift, fuel regen anywhere. 7. Wingsuit
   replaced by rigid wings with a stability assist, then by the path-hold pitch loop.
8. Turbo first by speed, then reworked to engine RPM. 9. Person -> hover skiff,
   car-like ground handling, cockpit HUD, hover audio. 10. Car and jetpack model replaced by a
   rigid-body jet craft with a quad-lift hover controller. 11. Flight law: wing, aero moments,
   control surfaces, throttle lever, lift jets fading out with airspeed, F to toggle.
12. (2026-10-29) All of 10-11 replaced: one vectoring engine, fly-by-wire attitude, one
   blended control law, no mode switch (user: the controls had become "an accumulation of
   a bunch of jets from previous builds").
13. (2026-10-31) Mouse-aim fixes on 12 (no snap roll on pitch, no wrong-way bank after a
   dive, spring-back aim point), then the user asked to rethink from the ground up:
   hover as a game, jet as a jet. Chosen: kinematic jet ("C") + g dose / blackout,
   mouse turns with auto-bank, F + speed gate, soft loss of control. Built on branch
   `new-flight-controller`.
The user's preferences seen so far: wants "feel" and character (sound, turbo),
physical plausibility with SI units, quiet/industrial sci-fi tone, low laptop heat.

## Known gaps and ideas
- No launch ridge near spawn: starting from high ground needs a jet or driving up.
- LOD pop; float precision far from origin; no tree collision.
- Turbo top speed (~470 km/h) outruns terrain streaming.
- The craft has no external model; the capsule collider is person-sized, not craft-sized.
- The new hover / jet / g feel, HUD effects and audio are unverified by hand.
- Release-build unused-parameter warning in `drawCollisionDebug`.
