# SKIFF agent wiki

Orientation for an agent (or human) picking up this repo. Written from the working
session that built it; verify against the code before trusting a detail, the
constants especially (`src/core/constants.hpp` is the source of truth).

## What it is

A C++23 / raylib 5.5 game, originally an FPS prototype (dir `fps-shooter`, target
`fps-shooter`, window title "SKIFF"). The player is now a **hover skiff**:
it is a single-engine tilt-jet (one vectoring nozzle, a fixed wing) that hovers and
flies on one control law. The world is a
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
| `src/player/airframe.{hpp,cpp}` | Rigid body: one vectoring engine, wing, drag, fly-by-wire rate loop |
| `src/player/pilot.{hpp,cpp}` | The one control law (aim, speed command, attitude, nozzle), `shapedRate` |
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
2. Per frame: mouse delta accumulated into `input.mouseDelta` (consumed by the next
   physics step, moves the aim point). `samplePlayerInput`.
3. Fixed-timestep accumulator at `kPhysicsDt = 1/120` calling `stepPlayer`; frame
   time clamped to 0.1 s. `previous`/`current` states are blended for rendering
   (`computeView(prev, cur, accumulator/dt)`).
4. `updateAudio(player)`, `syncCamera`, then `renderWorld` + `drawHud(player, camera)`.
5. `SetTargetFPS(60)`. This was lowered from 144 to cut heat (the GPU was saturated).

## Player physics (`stepPlayer`)

**Rewritten (2026-10-29) as one engine and one control law.** The previous build
(six jets, quad mixer, control surfaces, aero moments, separate hover and flight laws
on an F toggle, trim/sag patches) is kept in `backup/flight-before-rework/`.
`stepPlayer` is pure (no raylib IO) so the sim can script it. `Player.position` is the
**eye** position (`kEyeHeight` above the capsule base; the capsule is still the
collision shape). `FlightMode` (Landed / Hover / Flight) is only a label: `Player.flight`
(0..1, airspeed 25 to 45 m/s) blends the law. The craft is **indestructible**.

Per step: `pilotControl` (law) -> `stepAirframe` (physics) -> integrate position ->
`resolvePlayerCollisions` -> skid friction (scaled by the weight the engine is not
carrying) -> mode label.

### Airframe (`airframe.{hpp,cpp}`)
- Body frame: x right, y up, z forward (right = forward x up, so the basis is
  left-handed; `attitudeQuat` in pilot.cpp uses -forward as its z column).
- Mass 600 kg. One engine, `kEngineMaxThrust` 14 kN (hover at 42%), spool tau 0.15 s.
  The nozzle angle is in the body up/forward plane: 0 = thrust along body up, 90 deg =
  along forward, negative = tipped forward (braking). It cannot push sideways.
- Wing (`computeWingsuitAero`, 10 m^2, stall 20 deg), anisotropic hull drag (0.4 fwd,
  2 side, 3 belly m^2; the belly acts as lift at angle of attack), airbrake (20 m^2, arcade-big).
- **Fly-by-wire attitude:** no torques or inertia. `AirframeCommand.rate` is a world
  angular velocity; the body rate follows it with `kAttitudeTau` 0.08 s.

### Pilot law (`pilot.{hpp,cpp}`)
1. `steerAim`: mouse X turns `aimDir` about world up, mouse Y about its level right axis
   (flying: aligned with the wing near the vertical so loops are continuous). Hovering
   it is clamped to +-85 deg elevation. **Flying, the aim is a spring-loaded stick**
   (user, 2026-10-31: the sticky War Thunder-style point was too sensitive, "goes anywhere
   and very far"; turning at speed should take constant movement): it leads the path by at
   most `kAimMaxAngleDeg` 60 (30 felt "a little too restrictive") and eases back toward it with `kAimReturnTau` 0.3 s, so a
   flick turns ~12 deg and stops, and a 180 needs a sustained swipe (~150 deg/s of aim,
   3 s at 125 m/s). The easing is done per axis against the horizon (great circle only
   near the vertical) and stops short of the path: `kAimHoldDeg` 1 sideways,
   `kAimHoldElevationDeg` 3 vertically. **Both matter:** easing along the great circle
   lifts a point leading a level turn (the arc bulges up) and spirals the turn into a
   climb; easing all the way onto the path soaks the path's own lag/wander into the point,
   so released cruise drifts and turns climb away ~18 m.
2. `stepSpeedCommand`: W ramps `speedCommand` (10 m/s^2 hover, 25 flying), at most
   `kSpeedLead` 25 m/s above the real speed. **S is a hard brake:** it drops the command at
   `kBrakeRate` 250 m/s^2 to `kBrakeLead` 60 below the speed, so the law asks for the full
   `kFlightMaxBrake` 40 m/s^2 (hover: `kHoverMaxBrake` 20); 100 -> 20 m/s takes ~2.7 s. On release it latches the current speed, or 0
   below `kCruiseLatch` 35 m/s. Speed is along the aim when hovering, airspeed when flying.
3. Wanted acceleration: hover = P on velocity error (aim * speed + strafe + climb);
   flight = `turnDirection` (heading/elevation error against the horizon, great circle when
   the arc goes near the vertical) * shaped turn rate + P on speed. **Arcade turning:** no
   g limit; the rate cap is what the wing pulls at `kTurnAlphaDeg` 12 (less the weight),
   up to `kMaxTurnRateDeg` 100 deg/s, so a 180 at 125 m/s takes ~2.9 s (~13 g). Blended by
   `flight`. Force = m (a + g up).
4. Attitude target: hover tilts body up along `need` = force - aero (with the fore/aft
   part removed as forward speed builds, since the nozzle does that), faces the aim's
   heading. Flight splits the force's part across the path into sideways and vertical
   (horizon frame; canopy frame near the vertical) and picks one of three ways to fly it:
   **turn** = bank into the turn and pull, tan(bank) = sideways / vertical with the vertical
   floored (1 W when it is small or points down, 0.5 W when it clearly points up);
   **push** = a mostly-down demand, wings level, negative lift (capped at 2 g), sideways by
   skid; **invert** = pull past knife edge (loop top) only when the wing is already
   `kPushPreferDeg` 45 nearer that. Pull/push memory = current alpha sign (hysteresis).
   **Don'ts, both learned 2026-10-31:** don't bank along the force's direction (on a
   pushover the vertical is ~0 and any sideways crumb sent the wing to knife edge: "pitch
   at speed snaps fully left/right"); don't meet a sideways demand by banking away and
   pushing (turns the right way, rolls the wrong way: "point down, then left, it turns
   right"). Sideways aim error below `kFlatTurnDeg` 15 is flown wings level: the bank
   *angle* fades in over it (MouseFlight-style wings-level vs. aggressive-roll blend) and
   the skid (nose yawed up to `kMaxSkidDeg` 6) makes up whatever sideways push the wing is
   not making yet. Lift = weighted least squares (vertical 2x) for a bank halfway between
   the current and target one, so the pull comes as the bank arrives (roll, then pull):
   turns stay within ~4-6 m of altitude, at ~0.15 s on the arcade 180.
   alpha = lift / (q (S CLa + belly CdA)), clamped to -8..15 deg. Blended by `flight`.
5. Rates: quaternion attitude error -> `shapedRate` per body axis + **feed-forward of the
   path's rotation** (v x a / v^2 of the planned lift). Without the feed-forward a steady
   turn runs 7 deg short on alpha and the engine ends up doing the wing's job.
   A/D override the roll rate at speed.
6. Engine: airbrake opens when the engine would need to push backwards (at any speed).
   Nozzle = direction of `need` in the body plane, clamped to +-12 deg hovering, opening
   to 90 deg aft with forward speed; forward braking travel only while jet-borne (a tipped
   nozzle also lifts, which made braking from cruise balloon). Thrust = need . nozzle.
   Parked + no keys = thrust 0. `engineRpm` = idle + throttle^1.5 scale; off after 6 s at rest.

Test footing: `flight-sim` flight tests run at 1500 m.

### Not yet done
A craft model, per-engine audio, landing gear / proper collider, terrain-aware
autopilot, hand-tuned feel (all tuning so far is from the sim, not from hand-play).

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
`computeView` returns position/forward/up/fov. Blended by `Player.flight`: hovering it
looks along `aimDir` (free look) with 35% of the body's tilt; flying it is bolted to the
body frame. FOV widens with speed.

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
Sections: idle hover, cold start, W and release, strafe, reverse, climb/descent, look-down
descent, turning to the aim, disturbance recovery, landing / parking / lift-off, flying into
terrain; hover-to-flight on W, top speed, level cruise, flick-and-release, swiped 90 deg turns, held-mouse level circle,
loop, Immelmann (stop at the top), inverted recovery, hand roll, push-over (negative-g
limit), pitch with sideways drift (no snap roll), dive then aim sideways (banks toward the aim), pitch swipe then release, S from cruise to a hover, takeoff from the platform. Ranges encode the intended feel; when you
change tuning, update the expectations deliberately, not to silence a failure.

## Controls (current)
One scheme at every speed, no mode switch: mouse = aim point (go where you look;
flying, a spring-loaded stick that recentres on release),
W/S = faster/slower (let go: hold speed, or stop below 125 km/h), A/D = strafe when
slow / roll when fast, Space/Shift = climb/descend when slow.

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
