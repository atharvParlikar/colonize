// Headless checks of the hover model against its design targets.
// Build target: flight-sim. Exit code is the number of failed checks.

#include "collision/collision.hpp"
#include "core/constants.hpp"
#include "player/airframe.hpp"
#include "player/flight.hpp"
#include "player/player.hpp"
#include "world/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

void check(const char *name, float value, float lo, float hi, const char *unit) {
  const bool ok = value >= lo && value <= hi;
  failures += ok ? 0 : 1;
  std::printf("  [%s] %-44s %8.2f %-5s (target %.2f..%.2f)\n", ok ? "PASS" : "FAIL",
              name, value, unit, lo, hi);
}

Player airborne(float height = 80.0f) {
  Player p;
  p.position = {0.0f, height, 0.0f};
  for (int i = 0; i < LiftRight + 1; ++i) p.jetThrottle[i] = kAirframeWeight / 4.0f / kLiftJetMaxThrust;
  return p;
}

void run(Player &p, const PlayerInput &in, float seconds) {
  for (float t = 0.0f; t < seconds; t += kPhysicsDt) stepPlayer(p, in, kPhysicsDt);
}

float tiltDeg(const Player &p) {
  return RAD2DEG * std::acos(std::clamp(p.body.up.y, -1.0f, 1.0f));
}

float headingDeg(const Player &p) {
  return RAD2DEG * std::atan2(p.body.forward.z, p.body.forward.x);
}

float wrap(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

float maxLiftThrottle(const Player &p) {
  float m = 0.0f;
  for (int i = 0; i <= LiftRight; ++i) m = std::max(m, p.jetThrottle[i]);
  return m;
}

void testIdleHover() {
  std::printf("Idle hover (no input, 80 m up)\n");
  Player p = airborne();
  run(p, {}, 15.0f);
  check("altitude drift", p.position.y - 80.0f, -0.5f, 0.5f, "m");
  check("horizontal drift", std::hypot(p.position.x, p.position.z), 0.0f, 0.3f, "m");
  check("speed", Vector3Length(p.velocity), 0.0f, 0.1f, "m/s");
  check("tilt", tiltDeg(p), 0.0f, 0.5f, "deg");
  check("mean lift-jet throttle (idle rpm)", (p.jetThrottle[0] + p.jetThrottle[1] + p.jetThrottle[2] + p.jetThrottle[3]) / 4.0f,
        0.33f, 0.41f, "");
}

void testColdStart() {
  std::printf("Cold start in the air (jets at zero)\n");
  Player p;
  p.position = {0.0f, 200.0f, 0.0f};
  run(p, {}, 10.0f);
  check("falls less than", 200.0f - p.position.y, 0.0f, 12.0f, "m");
  check("speed after settling", Vector3Length(p.velocity), 0.0f, 1.0f, "m/s");
  check("tilt", tiltDeg(p), 0.0f, 1.0f, "deg");
}

void testTranslation() {
  std::printf("Translation\n");
  Player p = airborne();
  PlayerInput in;
  in.forward = true;
  float maxTilt = 0.0f, maxSpeed = 0.0f, maxThr = 0.0f, minY = 1e9f, maxY = -1e9f;
  for (float t = 0.0f; t < 12.0f; t += kPhysicsDt) {
    stepPlayer(p, in, kPhysicsDt);
    maxTilt = std::max(maxTilt, tiltDeg(p));
    maxSpeed = std::max(maxSpeed, Vector3Length(p.velocity));
    maxThr = std::max(maxThr, maxLiftThrottle(p));
    minY = std::min(minY, p.position.y);
    maxY = std::max(maxY, p.position.y);
  }
  check("cruise speed (W)", std::hypot(p.velocity.x, p.velocity.z), 19.0f, 20.5f, "m/s");
  check("speed overshoot", maxSpeed - kHoverMaxSpeed, -2.0f, 1.5f, "m/s");
  check("peak tilt", maxTilt, 5.0f, kHoverMaxTiltDeg + 2.0f, "deg");
  check("heading stays north", wrap(headingDeg(p) + 90.0f), -2.0f, 2.0f, "deg");
  check("altitude held while accelerating", maxY - minY, 0.0f, 2.0f, "m");
  check("peak lift-jet throttle", maxThr, 0.0f, 0.95f, "");
  in = {};
  run(p, in, 10.0f);
  check("speed 10 s after release", Vector3Length(p.velocity), 0.0f, 0.3f, "m/s");
  check("level again", tiltDeg(p), 0.0f, 1.0f, "deg");

  Player q = airborne();
  PlayerInput strafe;
  strafe.right = true;
  run(q, strafe, 10.0f);
  check("strafe right speed (+x when facing north)", q.velocity.x, 16.5f, 21.0f, "m/s");
  check("strafe keeps heading", wrap(headingDeg(q) + 90.0f), -2.0f, 2.0f, "deg");
}

void testVertical() {
  std::printf("Climb and descent\n");
  Player p = airborne(100.0f);
  PlayerInput up;
  up.ascend = true;
  run(p, up, 6.0f);
  check("climb rate", p.velocity.y, 5.5f, 6.3f, "m/s");
  check("tilt while climbing", tiltDeg(p), 0.0f, 1.0f, "deg");
  PlayerInput down;
  down.descend = true;
  run(p, down, 6.0f);
  check("descent rate", p.velocity.y, -5.3f, -4.5f, "m/s");
  run(p, {}, 6.0f);
  check("vertical speed after release", p.velocity.y, -0.2f, 0.2f, "m/s");
}

void testYaw() {
  std::printf("Heading\n");
  Player p = airborne();
  p.yaw = 0.0f; // 90 deg to the right of north
  float peak = -999.0f;
  float settled = -1.0f;
  for (float t = 0.0f; t < 8.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    peak = std::max(peak, wrap(headingDeg(p)));
    if (settled < 0.0f && std::fabs(wrap(headingDeg(p))) < 2.0f) settled = t;
  }
  check("reaches heading within", settled, 0.0f, 4.0f, "s");
  check("overshoot", peak, 0.0f, 8.0f, "deg");
  check("final heading error", wrap(headingDeg(p)), -1.0f, 1.0f, "deg");
  check("tilt after turning", tiltDeg(p), 0.0f, 0.5f, "deg");
  check("turn costs altitude", p.position.y - 80.0f, -1.0f, 1.0f, "m");
}

void testDisturbance() {
  std::printf("Disturbance recovery\n");
  Player p = airborne();
  p.body = makeAttitude(Vector3RotateByAxisAngle(p.body.forward, p.body.right, 0.35f),
                        Vector3RotateByAxisAngle(p.body.up, p.body.right, 0.35f));
  p.body = makeAttitude(p.body.forward, Vector3RotateByAxisAngle(p.body.up, p.body.forward, 0.4f));
  p.angularVelocity = {0.6f, 0.3f, -0.8f};
  check("starting tilt", tiltDeg(p), 20.0f, 32.0f, "deg");
  float worstY = 0.0f;
  for (float t = 0.0f; t < 10.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    worstY = std::max(worstY, 80.0f - p.position.y);
  }
  check("tilt after 10 s", tiltDeg(p), 0.0f, 0.5f, "deg");
  check("angular rate", Vector3Length(p.angularVelocity), 0.0f, 0.02f, "rad/s");
  check("altitude lost in recovery", worstY, 0.0f, 12.0f, "m");
  check("residual speed", Vector3Length(p.velocity), 0.0f, 0.2f, "m/s");
}

void testLanding() {
  std::printf("Landing on the spawn platform\n");
  Player p = airborne(30.0f);
  p.position = {0.0f, 30.0f, 6.0f};
  PlayerInput down;
  down.descend = true;
  run(p, down, 12.0f);
  check("rests at eye height above the platform", p.position.y, kEyeHeight - 0.05f, kEyeHeight + 0.3f, "m");
  check("mode is Landed", p.mode == FlightMode::Landed ? 1.0f : 0.0f, 1.0f, 1.0f, "");
  check("not crashed", p.crashTimer, 0.0f, 0.0f, "s");
  check("tilt on the ground", tiltDeg(p), 0.0f, 1.0f, "deg");
  PlayerInput up;
  up.ascend = true;
  run(p, up, 4.0f);
  check("lifts off again", p.position.y, 10.0f, 40.0f, "m");
}

void testSteepTerrain() {
  std::printf("Flying into a mountain (indestructible)\n");
  Player p = airborne(60.0f);
  p.position = {0.0f, 60.0f, 0.0f};
  PlayerInput in;
  in.forward = true;
  in.descend = true;
  run(p, in, 20.0f);
  check("still alive", p.crashTimer, 0.0f, 0.0f, "s");
  check("not under the terrain", p.position.y - kEyeHeight - terrainHeight(p.position.x, p.position.z), -0.2f,
        1e6f, "m");
}

// Flight tests run high up, over nothing, so terrain never interferes.
constexpr float kCruiseAlt = 1500.0f;

// Level flight along +X at `speed` with the engine at `rpm` and the aim circle
// dead ahead.
Player cruising(float speed, float rpm) {
  Player p;
  p.position = {0.0f, kCruiseAlt, 0.0f};
  p.body = makeAttitude({1.0f, 0.0f, 0.0f}, kWorldUp);
  p.velocity = {speed, 0.0f, 0.0f};
  p.law = ControlLaw::Flight;
  p.engineRpm = rpm;
  p.aimDir = {1.0f, 0.0f, 0.0f};
  p.yaw = 0.0f;
  for (int i = 0; i <= LiftRight; ++i) p.jetThrottle[i] = 0.0f;
  return p;
}

float bankDeg(const Player &p) { return RAD2DEG * std::atan2(-p.body.right.y, p.body.up.y); }
float pathDeg(const Player &p) {
  return RAD2DEG * std::asin(std::clamp(p.velocity.y / std::max(Vector3Length(p.velocity), 1.0f), -1.0f, 1.0f));
}
float headingOfPath(const Player &p) { return RAD2DEG * std::atan2(p.velocity.z, p.velocity.x); }
float angleBetweenDeg(Vector3 a, Vector3 b) {
  return RAD2DEG * std::atan2(Vector3Length(Vector3CrossProduct(a, b)), Vector3DotProduct(a, b));
}

void testEngine() {
  std::printf("Engine thrust curve and spool\n");
  check("thrust at idle", engineThrust(kIdleRpm), 0.3f, 0.5f, "x");
  check("thrust at the turbo threshold", engineThrust(kTurboStartRpm), 0.99f, 1.01f, "x");
  check("thrust at the turbo kick", engineThrust(kTurboKickRpm), 1.99f, 2.01f, "x");
  check("thrust at the redline", engineThrust(kRedlineRpm), 2.99f, 3.01f, "x");

  Player p = cruising(40.0f, 0.0f);
  PlayerInput w;
  w.forward = true;
  run(p, w, 1.0f);
  check("W lights the engine and spools up", p.engineRpm, kIdleRpm + 0.5f * kSpoolUpRate, kTurboStartRpm, "rpm");
  run(p, w, 5.0f);
  check("held W reaches the turbo", p.engineRpm, kTurboKickRpm, kRedlineRpm, "rpm");
  PlayerInput none;
  run(p, none, 2.0f);
  check("released W spools down toward idle", p.engineRpm, kIdleRpm, 6000.0f, "rpm");
  run(p, none, 8.0f);
  check("engine shuts off after idling", p.engineRpm, 0.0f, 0.5f, "rpm");
}

void testTransition() {
  std::printf("Hover to flight transition (F, then W)\n");
  Player p = airborne(kCruiseAlt);
  p.yaw = 0.0f;
  run(p, {}, 3.0f);
  PlayerInput toggle;
  toggle.toggleFlight = true;
  stepPlayer(p, toggle, kPhysicsDt);
  PlayerInput jet;
  jet.forward = true;
  float minY = 1e9f, maxY = -1e9f;
  for (float t = 0.0f; t < 14.0f; t += kPhysicsDt) {
    stepPlayer(p, jet, kPhysicsDt);
    minY = std::min(minY, p.position.y);
    maxY = std::max(maxY, p.position.y);
  }
  check("mode is Flight", p.mode == FlightMode::Flight ? 1.0f : 0.0f, 1.0f, 1.0f, "");
  check("airspeed after 14 s of W", Vector3Length(p.velocity), 90.0f, 400.0f, "m/s");
  check("altitude held through the transition", maxY - minY, 0.0f, 25.0f, "m");
  check("lift jets handed over to the wing", p.jetThrottle[0], 0.0f, 0.02f, "");
  check("heading kept", wrap(headingOfPath(p)), -3.0f, 3.0f, "deg");
}

void testCruise() {
  std::printf("Level cruise (60 m/s, circle dead ahead)\n");
  Player p = cruising(60.0f, 6000.0f);
  PlayerInput hold; // keep the engine lit so it does not time out
  hold.ascend = false;
  float peakRate = 0.0f;
  float minY = kCruiseAlt, maxY = kCruiseAlt;
  for (float t = 0.0f; t < 25.0f; t += kPhysicsDt) {
    stepPlayer(p, hold, kPhysicsDt);
    peakRate = std::max(peakRate, Vector3Length(p.angularVelocity));
    minY = std::min(minY, p.position.y);
    maxY = std::max(maxY, p.position.y);
  }
  check("altitude drift", p.position.y - kCruiseAlt, -15.0f, 15.0f, "m");
  check("flight path angle", pathDeg(p), -2.0f, 2.0f, "deg");
  // The nose sits above the circle by the angle of attack, so the flight path lands on it.
  check("flight path is on the circle", angleBetweenDeg(p.velocity, p.aimDir), 0.0f, 3.0f, "deg");
  check("wings level", std::fabs(bankDeg(p)), 0.0f, 2.0f, "deg");
  check("sideslip", RAD2DEG * flowAngles(p).beta, -1.0f, 1.0f, "deg");
  check("peak angular rate", peakRate, 0.0f, 0.5f, "rad/s");
}

// Circle `degrees` to the right of the heading (negative = left), level.
Player turnTest(float degrees, float speed, float rpm, float seconds, float &peakAlpha, float &minY,
                float &timeToPoint) {
  Player p = cruising(speed, rpm);
  const float a = DEG2RAD * degrees;
  p.aimDir = {std::cos(a), 0.0f, std::sin(a)};
  peakAlpha = 0.0f;
  minY = kCruiseAlt;
  timeToPoint = seconds;
  PlayerInput in;
  in.forward = true;
  for (float t = 0.0f; t < seconds; t += kPhysicsDt) {
    stepPlayer(p, in, kPhysicsDt);
    peakAlpha = std::max(peakAlpha, std::fabs(RAD2DEG * flowAngles(p).alpha));
    minY = std::min(minY, p.position.y);
    if (timeToPoint == seconds && angleBetweenDeg(p.body.forward, p.aimDir) < 8.0f) timeToPoint = t;
  }
  return p;
}

void testSharpTurns() {
  std::printf("Mouse aim: hard turns (70 m/s, W held)\n");
  float alpha, minY, t;
  Player p = turnTest(90.0f, 70.0f, 6000.0f, 8.0f, alpha, minY, t);
  check("90 right: nose on the circle in", t, 0.0f, 3.0f, "s");
  check("90 right: flight path follows", wrap(headingOfPath(p) - 90.0f), -10.0f, 10.0f, "deg");
  check("90 right: never stalls", alpha, 0.0f, 22.0f, "deg");
  check("90 right: altitude lost", kCruiseAlt - minY, 0.0f, 120.0f, "m");

  p = turnTest(-90.0f, 70.0f, 6000.0f, 8.0f, alpha, minY, t);
  check("90 left: nose on the circle in", t, 0.0f, 3.0f, "s");
  check("90 left: flight path follows", wrap(headingOfPath(p) + 90.0f), -10.0f, 10.0f, "deg");

  p = turnTest(100.0f, 70.0f, 6000.0f, 8.0f, alpha, minY, t);
  check("100 right (the circle's limit): nose on it in", t, 0.0f, 3.5f, "s");
  check("100 right: never stalls", alpha, 0.0f, 22.0f, "deg");

  p = turnTest(150.0f, 40.0f, kTurboStartRpm, 12.0f, alpha, minY, t);
  check("slow 150 turn: nose on the circle in", t, 0.0f, 8.0f, "s");
  check("slow 150 turn: never stalls", alpha, 0.0f, 22.0f, "deg");
}

void testNoseDown() {
  std::printf("Mouse aim: nose down (70 m/s, W held)\n");
  for (float dive : {20.0f, 45.0f, 80.0f}) {
    Player p = cruising(70.0f, 6000.0f);
    const float a = DEG2RAD * dive;
    p.aimDir = {std::cos(a), -std::sin(a), 0.0f};
    PlayerInput in;
    in.forward = true;
    float peakBank = 0.0f, peakAlpha = 0.0f, t80 = 99.0f, minAlpha = 0.0f;
    for (float t = 0.0f; t < 6.0f; t += kPhysicsDt) {
      stepPlayer(p, in, kPhysicsDt);
      peakBank = std::max(peakBank, std::fabs(bankDeg(p)));
      peakAlpha = std::max(peakAlpha, RAD2DEG * flowAngles(p).alpha);
      minAlpha = std::min(minAlpha, RAD2DEG * flowAngles(p).alpha);
      if (t80 == 99.0f && angleBetweenDeg(p.body.forward, p.aimDir) < 6.0f) t80 = t;
    }
    std::printf(" dive %.0f deg:\n", dive);
    check("nose gets onto the circle in", t80, 0.0f, 4.0f, "s");
    check("peak bank on the way", peakBank, 0.0f, 25.0f, "deg");
    check("heading drift", std::fabs(wrap(headingDeg(p))), 0.0f, 8.0f, "deg");
    check("alpha stays in the envelope", minAlpha, kMinAlphaDeg - 3.0f, 0.0f, "deg");
  }
  // Held mouse down for 1.5 s, then released.
  Player p = cruising(70.0f, 6000.0f);
  PlayerInput in;
  in.forward = true;
  float peakBank = 0.0f;
  for (float t = 0.0f; t < 5.0f; t += kPhysicsDt) {
    in.mouseDelta = t < 1.5f ? Vector2{0.0f, 2.0f} : Vector2{};
    stepPlayer(p, in, kPhysicsDt);
    peakBank = std::max(peakBank, std::fabs(bankDeg(p)));
  }
  check("held mouse down: path angle", pathDeg(p), -90.0f, -30.0f, "deg");
  check("held mouse down: peak bank", peakBank, 0.0f, 25.0f, "deg");
  check("held mouse down: heading drift", std::fabs(wrap(headingDeg(p))), 0.0f, 10.0f, "deg");
}

void testHeldMouseTurn() {
  std::printf("Mouse aim: held mouse right keeps turning past 180 deg\n");
  Player p = cruising(80.0f, 7000.0f);
  PlayerInput in;
  in.forward = true;
  in.mouseDelta = {5.0f, 0.0f};
  float travelled = 0.0f, last = 0.0f, peakAlpha = 0.0f, minY = kCruiseAlt;
  float at180 = 99.0f;
  for (float t = 0.0f; t < 8.0f; t += kPhysicsDt) {
    stepPlayer(p, in, kPhysicsDt);
    const float heading = std::atan2(p.body.forward.z, p.body.forward.x);
    float step = heading - last;
    if (step > PI) step -= 2.0f * PI;
    if (step < -PI) step += 2.0f * PI;
    travelled += step;
    last = heading;
    if (at180 == 99.0f && RAD2DEG * travelled > 180.0f) at180 = t;
    peakAlpha = std::max(peakAlpha, std::fabs(RAD2DEG * flowAngles(p).alpha));
    minY = std::min(minY, p.position.y);
  }
  check("time to turn through 180 deg", at180, 0.0f, 4.0f, "s");
  check("heading travelled in 8 s", RAD2DEG * travelled, 360.0f, 5000.0f, "deg");
  check("never stalls", peakAlpha, 0.0f, 22.0f, "deg");
  check("altitude lost", kCruiseAlt - minY, -50.0f, 250.0f, "m");
}

void testLoop() {
  std::printf("Mouse aim: held mouse up is a loop\n");
  Player p = cruising(80.0f, 7000.0f);
  PlayerInput in;
  in.forward = true;
  in.mouseDelta = {0.0f, -6.0f}; // ~1.8 rad/s of aim motion
  float travelled = 0.0f, last = 0.0f, minSpeed = 1e9f, peakAlpha = 0.0f;
  for (float t = 0.0f; t < 14.0f; t += kPhysicsDt) {
    stepPlayer(p, in, kPhysicsDt);
    // Pitch in the plane of the loop, unwrapped.
    const float pitch = std::atan2(p.body.forward.y, p.body.forward.x);
    float step = pitch - last;
    if (step > PI) step -= 2.0f * PI;
    if (step < -PI) step += 2.0f * PI;
    travelled += step;
    last = pitch;
    minSpeed = std::min(minSpeed, Vector3Length(p.velocity));
    peakAlpha = std::max(peakAlpha, std::fabs(RAD2DEG * flowAngles(p).alpha));
  }
  check("pitch travelled (loops, no 180 limit)", RAD2DEG * travelled, 540.0f, 5000.0f, "deg");
  check("slowest point of the loop", minSpeed, 12.0f, 400.0f, "m/s");
  check("never stalls", peakAlpha, 0.0f, 22.0f, "deg");
  check("no NaNs", std::isfinite(p.position.x + p.position.y + p.position.z) ? 1.0f : 0.0f, 1.0f, 1.0f, "");
}

void testRollOverride() {
  std::printf("Hand roll (D held for 1 s)\n");
  Player p = cruising(70.0f, 6000.0f);
  PlayerInput in;
  in.right = true;
  float peakBank = 0.0f;
  float rolled = 0.0f, last = 0.0f;
  for (float t = 0.0f; t < 1.0f; t += kPhysicsDt) {
    stepPlayer(p, in, kPhysicsDt);
    const float b = std::atan2(-p.body.right.y, p.body.up.y);
    float d = b - last;
    if (d > PI) d -= 2.0f * PI;
    if (d < -PI) d += 2.0f * PI;
    rolled += d;
    last = b;
    peakBank = std::max(peakBank, std::fabs(RAD2DEG * b));
  }
  check("rolls right by hand (deg in 1 s)", RAD2DEG * rolled, 100.0f, 400.0f, "deg");
}

void testPowerOff() {
  std::printf("Engine off in cruise\n");
  Player p = cruising(45.0f, 0.0f);
  float maxAlpha = 0.0f, minSpeed = 1e9f, minY = kCruiseAlt, maxTilt = 0.0f;
  for (float t = 0.0f; t < 40.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    maxAlpha = std::max(maxAlpha, RAD2DEG * flowAngles(p).alpha);
    minSpeed = std::min(minSpeed, Vector3Length(p.velocity));
    minY = std::min(minY, p.position.y);
    maxTilt = std::max(maxTilt, tiltDeg(p));
  }
  check("never stalls", maxAlpha, 0.0f, 20.0f, "deg");
  check("lowest airspeed", minSpeed, 12.0f, 45.0f, "m/s");
  check("altitude lost", kCruiseAlt - minY, 0.0f, 60.0f, "m");
  check("peak tilt", maxTilt, 0.0f, 25.0f, "deg");
}

void testBackToHover() {
  std::printf("Flight to hover (F at 60 m/s)\n");
  Player p = cruising(60.0f, 6000.0f);
  run(p, {}, 2.0f);
  PlayerInput toggle;
  toggle.toggleFlight = true;
  stepPlayer(p, toggle, kPhysicsDt);
  float minY = kCruiseAlt, maxTilt = 0.0f;
  for (float t = 0.0f; t < 30.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    minY = std::min(minY, p.position.y);
    maxTilt = std::max(maxTilt, tiltDeg(p));
  }
  check("comes to rest", Vector3Length(p.velocity), 0.0f, 0.5f, "m/s");
  check("altitude lost braking", kCruiseAlt - minY, 0.0f, 80.0f, "m");
  check("peak tilt", maxTilt, 0.0f, 50.0f, "deg");
  check("level", tiltDeg(p), 0.0f, 1.0f, "deg");
}

void testTakeoffRoll() {
  std::printf("Takeoff from the platform in flight mode\n");
  Player p;
  p.position = {-14.0f, kEyeHeight, 10.0f};
  p.body = makeAttitude({1.0f, 0.0f, 0.0f}, kWorldUp); // along the platform, clear of the box
  p.yaw = 0.0f;
  PlayerInput toggle;
  toggle.toggleFlight = true;
  stepPlayer(p, toggle, kPhysicsDt);
  p.aimDir = Vector3Normalize({std::cos(DEG2RAD * 12.0f), std::sin(DEG2RAD * 12.0f), 0.0f}); // mouse up a touch
  PlayerInput in;
  in.forward = true;
  in.ascend = true;
  run(p, in, 5.0f);
  in.ascend = false;
  run(p, in, 3.0f); // short: the terrain ahead rises into the flight path
  check("altitude after the climb-out", p.position.y - kEyeHeight - terrainHeight(p.position.x, p.position.z),
        15.0f, 1e6f, "m");
  check("airspeed", Vector3Length(p.velocity), 40.0f, 200.0f, "m/s");
  check("climbing on the circle (path angle)", pathDeg(p), 6.0f, 16.0f, "deg");
}

} // namespace

int main() {
  testIdleHover();
  testColdStart();
  testTranslation();
  testVertical();
  testYaw();
  testDisturbance();
  testLanding();
  testSteepTerrain();
  testEngine();
  testTransition();
  testCruise();
  testSharpTurns();
  testNoseDown();
  testHeldMouseTurn();
  testLoop();
  testRollOverride();
  testPowerOff();
  testBackToHover();
  testTakeoffRoll();
  std::printf("\n%d check(s) failed\n", failures);
  return failures;
}
