# SKIFF agent wiki

Orientation for an agent (or human) picking up this repo. Written from the working
session that built it; verify against the code before trusting a detail, the
constants especially (`src/core/constants.hpp` is the source of truth).

## What it is

A C++23 / raylib 5.5 game, originally an FPS prototype (dir `fps-shooter`, target
`fps-shooter`, window title "SKIFF"). The player is now a **hover skiff**:
it is a jet-driven hover craft (rigid body, quad lift jets, drone-style
hovering); wings and forward flight are being rebuilt on top of that. The world is a
huge procedural, painterly terrain. Everything is procedural: no art, audio or
model assets exist. Not a git repo.

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

- Targets: `fps-shooter` (game) and `flight-sim` (`tools/flight_sim.cpp` + player,
  collision and world sources; no rendering).
- raylib comes from Homebrew (`/opt/homebrew`), found via `find_package`, with a
  FetchContent fallback. Threads are linked (terrain workers).
- Release build has one known, harmless warning: unused parameter in
  `drawCollisionDebug`.
- Run `flight-sim` after any change to `src/player/*`, `collision/*`, `world/terrain.*`
  or physics constants. It is the regression suite; extend it when adding mechanics.
- Visual checks: temporarily hack a screenshot into `main.cpp` (raylib's
  `TakeScreenshot("x.png")` saves into the **cwd by basename**), run from the
  scratchpad dir, read the PNG, then **remove the hook** and verify with
  `grep -c getenv src/main.cpp`.
- Shell is zsh with fish login. zsh does not word-split `$var`; macOS `sed -i ''`
  is fiddly; prefer Python for multi-line edits.

## Source map

| Path | Role |
|---|---|
| `src/main.cpp` | Window, fixed-timestep loop, glue |
| `src/core/constants.hpp` | **All tuning constants** (ground, wing, engine, controls, landing) |
| `src/player/player.{hpp,cpp}` | `Player` state, `PlayerInput`, `stepPlayer` |
| `src/player/airframe.{hpp,cpp}` | Rigid body, jet table, lift-jet mixer |
| `src/player/hover.{hpp,cpp}` | Hover flight controller, `shapedRate` |
| `src/player/flight.{hpp,cpp}` | Stabilised-stick flight law |
| `src/player/aero.{hpp,cpp}` | Atmosphere, `Attitude`, wing model (unused until wings return) |
| `src/player/input.{hpp,cpp}` | Keyboard sampling into `PlayerInput` |
| `src/collision/` | Capsule vs level boxes + terrain contact, `queryGround`, `snapToGround` |
| `src/camera/camera.{hpp,cpp}` | `computeView` (interpolated view, bob, lean, wing attitude) |
| `src/render/render.{hpp,cpp}` | `renderWorld` and the whole cockpit HUD |
| `src/render/scenery.{hpp,cpp}` | Streaming terrain/tree LOD system, post-processing |
| `src/render/shaders.hpp` | Embedded GLSL: world, sky, Kuwahara post |
| `src/world/noise.hpp`, `terrain.{hpp,cpp}` | Noise, `terrainHeight`, `terrainNormal`, `surfaceFriction` |
| `src/world/level.cpp`, `colliders.hpp` | Spawn platform and box obstacles |
| `src/audio/audio.{hpp,cpp}` | Fully synthesised audio |
| `tools/flight_sim.cpp` | Headless checks |

## Main loop (`main.cpp`)

1. `initAudio`, `initScenery`, create `Player`, `preloadScenery(camera)` ("Generating world...").
2. Per frame: mouse delta -> `applyMouseLook` (on ground/falling, applied every frame
   for responsiveness) **or** accumulated into `input.mouseDelta` (in wing mode,
   consumed by the physics step). `samplePlayerInput`.
3. Fixed-timestep accumulator at `kPhysicsDt = 1/120` calling `stepPlayer`; frame
   time clamped to 0.1 s. `previous`/`current` states are blended for rendering
   (`computeView(prev, cur, accumulator/dt)`).
4. `updateAudio(player)`, `syncCamera`, then `renderWorld` + `drawHud(player, camera)`.
5. `SetTargetFPS(60)`. This was lowered from 144 to cut heat (the GPU was saturated).

## Player physics (`stepPlayer`)

**Rewritten around a rigid-body hover craft.** The old car / wing / jetpack model is gone
(`aero.cpp` is the wing model, in use again). `stepPlayer` is pure
(no raylib IO) so the sim can script it. `Player.position` is the **eye** position
(`kEyeHeight` above the capsule base; the capsule is still the collision shape).
`FlightMode` is `Landed` / `Hover` / `Flight` (HUD label; `Player.law` is the actual control law). The craft is **indestructible**:
the impact speed from `resolvePlayerCollisions` is ignored (`crashTimer` is unused).

Per step: `hoverControl` (controller) -> `stepAirframe` (physics) -> integrate position ->
`resolvePlayerCollisions` -> engine rpm for HUD/audio -> Landed/Hover label.

### Airframe (`airframe.{hpp,cpp}`)
- Body frame: x right, y up, z forward, origin at the centre of mass. Orientation is
  `Player.body` (an `Attitude`), angular velocity `Player.angularVelocity` is a **world**
  vector. Axes are rotated by `omega*dt` (Rodrigues) and re-orthonormalised each step.
- Mass 600 kg, diagonal inertia (roll 1900, pitch 2600, yaw 4300), gyroscopic terms
  neglected, aero rate damping 0.6/s, hull drag CdA 1.5 m^2.
- **Jet table** `kJets` (position, thrust direction, max thrust, yaw reaction): four
  vertical lift jets (front, rear, left, right; 4000 N each, hover at about 37% throttle)
  and two forward main jets on the wings (2800 N each; zero in hover, throttle lever in flight).
  Every jet spools toward its command with `kJetSpoolTau` 0.12 s. Force is applied at the
  mounting point, so torque is `r x F`. Lift-jet nozzles are "canted": `yawReaction`
  gives yaw torque in proportion to thrust (+ fore/aft, - left/right).
- `mixLiftJets(collective, BodyTorque)` solves the symmetric quad mixer from the table.
  Commands clamp to 0..1.

### Hover controller (`hover.{hpp,cpp}`)
Cascade: pilot velocity wish (WASD relative to heading `player.yaw`, Space climb 6 m/s,
Shift descend 5 m/s, max 20 m/s horizontal) -> acceleration (P on velocity error) ->
thrust vector (adds weight and hull drag feed-forward, tilt limited to 35 deg) -> wanted
up axis -> tilt/yaw rate command (`shapedRate`: proportional but never faster than the
torque limit can brake, which is what removed the overshoot) -> rate loop torque
(clamped to `kMaxTiltTorque` / `kMaxYawTorque` so a hard turn cannot starve the
collective) -> mixer. No input holds position and altitude (that is the idle hover).
Mouse X/Y go to `player.yaw` / `player.pitch` every frame (`applyMouseLook`); the craft
turns toward `yaw`, `pitch` is head tilt only.
Gains and limits live in the "Hover controller" block of `constants.hpp`.

### Ground
There is no ground mode any more. The craft rests on terrain/platform through the capsule
collision and takes off by climbing. `resolvePlayerCollisions` now measures box
penetration **before** the push-out so a craft pressing down loses its velocity.

### Wing and flight law (`flight.{hpp,cpp}`, `Player.law`)
**F toggles `ControlLaw::Hover` / `Flight`.** The physics is the same rigid body in both;
only the controller changes. The wing (`computeWingsuitAero`, now 10 m^2 / 10 m span) is
always on, plus aerodynamic moments in `stepAirframe` (pitch stiffness `kCmAlpha`,
weathervane `kCnBeta`, rate damping `kCmq/kCnr/kClp`) so the craft flies nose-first.
Hull drag is anisotropic (0.4 forward, 2.0 side, 3.0 belly m^2). `aeroForce()` (hull plus
wing) is what the hover controller feeds forward, so hover still works at speed.

Flight law ("instructor"), mouse deltas arrive through `PlayerInput.mouseDelta` (not
`applyMouseLook`). All of it is `flightControl` in `src/player/flight.cpp`:
- **Aim point** (`Player.aimDir`, drawn as a cyan circle by `drawAimCircle`): mouse X turns
  it about the *world* up axis (horizon-referenced, so a held mouse flies a level circle);
  mouse Y pitches it about the wing axis (levelled while near-horizontal, continuous over
  the top of a loop). It stays fixed in the world while the craft chases it, clamped to
  `kAimMaxAngleDeg` (100) from the nose. Toggling to flight aims along the velocity.
- **Chase:** error to the target (aim plus trim) as a rotation vector. Roll rate = roll the
  lift side onto the target (`rollToTarget`) when far (`far`), wings-level hold when near;
  pull = `err * cos(rollToTarget)`; yaw handles the rest (it is the world-vertical channel
  in a banked turn, so it must keep full weight). A/D override the roll by hand.
- **Alpha limiter:** pitch rate fades to zero over `kAlphaMargin` before `kMaxAlphaDeg`
  (17, stall is 20), so full pull is the max-g turn, not a stall.
- **Trim:** `pitchTrim` integrates "circle above flight path" near the circle so the
  *path* lands on it (the nose sits about alpha above the circle; the amber flight-path
  marker sits on it). In a hard turn `sag` aims higher by how far the path sags below the
  circle's elevation, which is what keeps a banked turn at the horizon level.
- **Engine, no throttle lever:** `stepEngine` spools `Player.engineRpm` (W spools, S holds
  5,500 and applies `kBrakeReverse` reverse thrust, none = idle then off after 6 s).
  `engineThrust(rpm)`: 1x at 6,000, 2x at 8,000, 3x at the 12,000 redline; drives both
  main jets (their command can exceed 1). Hover law still derives `engineRpm` from the jets.
- **Surfaces / jets:** control surfaces take torque first (authority = q x `kSurface*`), the
  lift jets make up the rest through the mixer.
- **Lift-jet fade:** the wing carries `smoothstep(18, 34 m/s)` of the weight; lift jets
  carry the rest plus a vertical-speed hold (Space / Shift climb). Closing the engine
  bleeds speed to about 15 m/s, then the lift jets take over and it hovers forward-flight style.
- Toggling to hover at speed hands over to the hover controller, which flares and brakes
  (tilt limit 35 deg).
- On the ground in flight law `kGroundFriction` (2.5 m/s^2) drags the skids.
- Speeds: about 160-200 m/s with W held (turbo). Takeoff from the platform works with
  W + Space and the mouse nudged up.
- **Motion blur / grass at speed:** `render/motion.hpp` `motionBlurAmount` is 0 below
  250 km/h, 1 at 400. `setMotionBlur` adds a zoom-blur pass (`kBlurFrag`, scenery.cpp
  `endScene`) and `drawGrass` shrinks its range to 30% and skips tiles the craft has passed.

Test footing: `flight-sim` flight tests run at 1500 m, because the terrain a few hundred
metres from spawn rises past 25 m and bounces a low flier (collisions are indestructible).

### Not yet done
Main-jet differential/vectoring, a craft model, per-jet audio, landing gear / proper
collider, terrain-aware autopilot (none), hand-tuned feel.

## Collision (`src/collision/`)
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
`computeView` returns position/forward/up/fov. Ground/falling: free-look from yaw
and pitch; grounded adds bob, banking lean and a speed FOV boost. Wings: uses the
body frame (forward/up), easing in from free-look over `kDeployBlendTime` (0.5 s),
FOV widens with airspeed. The view follows the wing's actual attitude, so it lags the
commanded path by roughly half a second (`kAlphaTau`, `kPathGain` are the knobs).

## HUD (`render.cpp`, bottom half of the file)
All 2D, drawn with raylib primitives and the default font. Pieces: compass tape
(heading from camera forward, north = -Z), pitch ladder rotated by camera roll
(`roll = atan2(-right.y, up.y)`, drawn rotated by -roll), boresight reticle,
flight-path marker (`GetWorldToScreen` of the velocity direction), speed panel
(km/h, m/s, "Mach" = v/340), altitude panel (AGL, ASL, V/S, L/D capped at 99.9),
mode/engine panel, RPM arc gauge with the turbo zone, fuel bar, top-down ship
schematic (wings stow/deploy, hover pads, exhaust flame by RPM, blue in turbo), STALL
and LOW FUEL flashes, HULL BREACH overlay, control hints top-left. Palette:
cyan/amber/red on translucent dark panels with corner brackets.

## Audio (`audio.cpp`)
No samples. One mono 32-bit float `AudioStream` (44.1 kHz, buffer 2048) filled by a
callback on the audio thread (`synth`). The game thread publishes atomics once a
frame in `updateAudio`: `gThrottle` (0 off, 1 wing jet, 2 heavy), `gRpm`, `gTurbo`,
`gHover`, `gWind` (airspeed), `gMuffle` (grounded).
Layers: drive hum (detuned saws + sub through a resonant state-variable filter, with
a slow throb), rising sine whine, soft air hiss, ignition thump, and turbo layers
(octave-up saw growl, sweeping siren, rushing air, a wastegate "pssh" + kick when the
turbo engages, a chopped compressor surge when it cuts out), hover hum, and speed
wind. A `tanh` limiter closes the chain. The design goal was "quiet, industrial,
sci-fi", explicitly not a fan/roar; keep levels low. To sanity-check without ears,
`#include "audio/audio.cpp"` in a scratch program, call `synth` on a buffer and check
peak/RMS and NaNs.

## Headless sim (`tools/flight_sim.cpp`)
Sections: idle hover, cold start, translation (cruise speed, overshoot, tilt, jet
headroom, release), strafe, climb/descent, heading turn, disturbance recovery, landing
and lift-off, flying into terrain; flight: hover-to-flight transition, level cruise, bank
engine curve, mouse-aim turns (90/100/slow), held-mouse turn past 180, held-mouse loop, hand roll, engine-off (no stall), flight-to-hover, takeoff from the platform. Ranges encode the intended feel; when you
change tuning, update the expectations deliberately, not to silence a failure.

## Controls (current)
Hover: W/S/A/D move relative to heading, Space climb, Shift descend, mouse X turns the
craft, mouse Y tilts the head. F switches to flight.
Flight: mouse moves the aim circle (craft chases it), W engine / turbo, S brake, A/D roll
by hand, Space/Shift slow vertical. F switches back to hover.

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
The user's preferences seen so far: wants "feel" and character (sound, turbo),
physical plausibility with SI units, quiet/industrial sci-fi tone, low laptop heat.

## Known gaps and ideas
- No launch ridge near spawn: starting from high ground needs a jet or driving up.
- LOD pop; float precision far from origin; no tree collision.
- Turbo top speed (~470 km/h) outruns terrain streaming.
- The craft has no external model; the capsule collider is person-sized, not craft-sized.
- Hover and flight feel, HUD and audio are unverified by hand; fuel and turbo are gone.
- Not yet hand-played end to end: controls feel and audio mix are unverified by ear.
- Release-build unused-parameter warning in `drawCollisionDebug`.
