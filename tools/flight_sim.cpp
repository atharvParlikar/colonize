// Headless checks of the flight model against its design targets.
// Build target: flight-sim. Exit code is the number of failed checks.

#include "collision/collision.hpp"
#include "core/constants.hpp"
#include "player/airframe.hpp"
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
  std::printf("  [%s] %-46s %8.2f %-5s (target %.2f..%.2f)\n", ok ? "PASS" : "FAIL", name, value, unit, lo, hi);
}

constexpr float kHigh = 1500.0f; // clear of every mountain near spawn

Player airborne(float height = 80.0f) {
  Player p;
  p.position = {0.0f, height, 0.0f};
  p.mode = FlightMode::Hover;
  p.throttle = kAirframeWeight / kEngineMaxThrust;
  p.engineIdleTime = 0.0f;
  return p;
}

void run(Player &p, const PlayerInput &in, float seconds) {
  for (float t = 0.0f; t < seconds; t += kPhysicsDt) stepPlayer(p, in, kPhysicsDt);
}

float tiltDeg(const Player &p) { return RAD2DEG * std::acos(std::clamp(p.body.up.y, -1.0f, 1.0f)); }
float headingDeg(const Player &p) { return RAD2DEG * std::atan2(p.body.forward.z, p.body.forward.x); }
float headingOfPath(const Player &p) { return RAD2DEG * std::atan2(p.velocity.z, p.velocity.x); }
float pathDeg(const Player &p) {
  return RAD2DEG * std::asin(std::clamp(p.velocity.y / std::max(Vector3Length(p.velocity), 1e-3f), -1.0f, 1.0f));
}
float bankDeg(const Player &p) { return RAD2DEG * std::atan2(-p.body.right.y, p.body.up.y); }
// Bank about the flight path against the horizon (right wing down > 0): what a
// roll looks like, without the tilt a yawed nose shows in a steep dive.
float pathBankDeg(const Player &p) {
  const Vector3 w = Vector3Normalize(p.velocity);
  const Vector3 r = Vector3Normalize(Vector3CrossProduct(w, kWorldUp));
  const Vector3 u = Vector3CrossProduct(r, w);
  return RAD2DEG * std::atan2(Vector3DotProduct(p.body.up, r), Vector3DotProduct(p.body.up, u));
}
float angleDeg(Vector3 a, Vector3 b) {
  return RAD2DEG * std::atan2(Vector3Length(Vector3CrossProduct(a, b)), Vector3DotProduct(a, b));
}
float wrap(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}
// Mouse counts that turn the aim point by deg.
float counts(float deg) { return deg * DEG2RAD / kAimSensitivity; }

// Straight and level at cruise, high up, facing north.
Player cruising(float speed = 100.0f) {
  Player p = airborne(kHigh);
  p.velocity = {0.0f, 0.0f, -speed};
  p.speedCommand = speed;
  p.flight = 1.0f;
  p.nozzle = kNozzleAftDeg * DEG2RAD;
  p.throttle = 0.4f;
  p.mode = FlightMode::Flight;
  p.body = makeAttitude({0.0f, 0.05f, -1.0f}, kWorldUp);
  run(p, {}, 4.0f);
  // Settling moves the path a little, and the released stick holds whatever
  // path it has: aim it level again and let the path come onto it.
  p.aimDir = {0.0f, 0.0f, -1.0f};
  run(p, {}, 4.0f);
  return p;
}

void testIdleHover() {
  std::printf("Idle hover (no input, 80 m up)\n");
  Player p = airborne();
  run(p, {}, 15.0f);
  check("altitude drift", p.position.y - 80.0f, -0.5f, 0.5f, "m");
  check("horizontal drift", std::hypot(p.position.x, p.position.z), 0.0f, 0.3f, "m");
  check("speed", Vector3Length(p.velocity), 0.0f, 0.1f, "m/s");
  check("tilt", tiltDeg(p), 0.0f, 0.5f, "deg");
  check("throttle", p.throttle, 0.38f, 0.46f, "");
  check("nozzle near vertical", std::fabs(p.nozzle * RAD2DEG), 0.0f, 1.0f, "deg");
  check("engine running", p.engineRpm, kIdleRpm, kTurboStartRpm, "rpm");
}

void testColdStart() {
  std::printf("Cold start in the air (engine at zero)\n");
  Player p = airborne(200.0f);
  p.throttle = 0.0f;
  run(p, {}, 10.0f);
  check("falls less than", 200.0f - p.position.y, 0.0f, 6.0f, "m");
  check("speed after settling", Vector3Length(p.velocity), 0.0f, 0.5f, "m/s");
  check("tilt", tiltDeg(p), 0.0f, 1.0f, "deg");
}

void testTranslation() {
  std::printf("Hover: W, then let go\n");
  Player p = airborne();
  PlayerInput w;
  w.forward = true;
  float maxTilt = 0.0f, minY = 1e9f, maxY = -1e9f;
  for (float t = 0.0f; t < 2.0f; t += kPhysicsDt) {
    stepPlayer(p, w, kPhysicsDt);
    maxTilt = std::max(maxTilt, tiltDeg(p));
    minY = std::min(minY, p.position.y);
    maxY = std::max(maxY, p.position.y);
  }
  check("speed after 2 s of W", Vector3Length(p.velocity), 10.0f, 20.0f, "m/s");
  check("moves north", -p.velocity.z, 10.0f, 20.0f, "m/s");
  check("peak tilt", maxTilt, 3.0f, kHoverMaxTiltDeg + 2.0f, "deg");
  check("heading stays north", wrap(headingDeg(p) + 90.0f), -2.0f, 2.0f, "deg");
  float stopped = -1.0f;
  for (float t = 0.0f; t < 8.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    minY = std::min(minY, p.position.y);
    maxY = std::max(maxY, p.position.y);
    if (stopped < 0.0f && Vector3Length(p.velocity) < 0.5f) stopped = t;
  }
  check("stops after letting go within", stopped, 0.0f, 4.0f, "s");
  check("speed command back to zero", p.speedCommand, 0.0f, 0.0f, "m/s");
  check("altitude held throughout", maxY - minY, 0.0f, 1.5f, "m");
  check("level again", tiltDeg(p), 0.0f, 1.0f, "deg");

  Player q = airborne();
  PlayerInput strafe;
  strafe.right = true;
  float strafeTilt = 0.0f;
  for (float t = 0.0f; t < 6.0f; t += kPhysicsDt) {
    stepPlayer(q, strafe, kPhysicsDt);
    strafeTilt = std::max(strafeTilt, tiltDeg(q));
  }
  check("strafe right speed (+x facing north)", q.velocity.x, kStrafeSpeed - 0.5f, kStrafeSpeed + 0.5f, "m/s");
  check("strafe banks into it", strafeTilt, 5.0f, kHoverMaxTiltDeg + 2.0f, "deg");
  check("strafe keeps heading", wrap(headingDeg(q) + 90.0f), -2.0f, 2.0f, "deg");

  Player r = airborne();
  PlayerInput back;
  back.back = true;
  run(r, back, 4.0f);
  check("S from a hover reverses", r.velocity.z, kReverseSpeed - 0.5f, kReverseSpeed + 0.5f, "m/s");
}

void testVertical() {
  std::printf("Hover: climb and descent\n");
  Player p = airborne(100.0f);
  PlayerInput up;
  up.ascend = true;
  run(p, up, 5.0f);
  check("climb rate", p.velocity.y, kClimbSpeed - 0.3f, kClimbSpeed + 0.3f, "m/s");
  check("tilt while climbing", tiltDeg(p), 0.0f, 1.0f, "deg");
  PlayerInput down;
  down.descend = true;
  run(p, down, 5.0f);
  check("descent rate", p.velocity.y, -kDescendSpeed - 0.3f, -kDescendSpeed + 0.3f, "m/s");
  run(p, {}, 4.0f);
  check("vertical speed after release", p.velocity.y, -0.1f, 0.1f, "m/s");

  Player q = airborne(200.0f);
  PlayerInput lookDown;
  lookDown.mouseDelta = {0.0f, counts(30.0f)};
  stepPlayer(q, lookDown, kPhysicsDt);
  PlayerInput w;
  w.forward = true;
  run(q, w, 2.0f);
  check("W while looking down descends", q.velocity.y, -12.0f, -3.0f, "m/s");
}

void testYaw() {
  std::printf("Hover: turning to the aim\n");
  Player p = airborne();
  PlayerInput turn;
  turn.mouseDelta = {counts(90.0f), 0.0f}; // north -> east
  stepPlayer(p, turn, kPhysicsDt);
  check("aim swung to east", wrap(RAD2DEG * std::atan2(p.aimDir.z, p.aimDir.x)), -1.0f, 1.0f, "deg");
  float peak = -999.0f, settled = -1.0f;
  for (float t = 0.0f; t < 5.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    peak = std::max(peak, wrap(headingDeg(p)));
    if (settled < 0.0f && std::fabs(wrap(headingDeg(p))) < 2.0f) settled = t;
  }
  check("body reaches the heading within", settled, 0.0f, 1.8f, "s");
  check("overshoot", peak, 0.0f, 3.0f, "deg");
  check("tilt after turning", tiltDeg(p), 0.0f, 0.5f, "deg");
  check("turn costs altitude", p.position.y - 80.0f, -0.5f, 0.5f, "m");
}

void testDisturbance() {
  std::printf("Hover: disturbance recovery\n");
  Player p = airborne(150.0f);
  p.body = makeAttitude({0.3f, 0.2f, -1.0f}, {0.4f, 1.0f, 0.1f});
  p.angularVelocity = {0.8f, 0.4f, -0.6f};
  check("starting tilt", tiltDeg(p), 15.0f, 40.0f, "deg");
  float worstY = 0.0f;
  for (float t = 0.0f; t < 8.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    worstY = std::max(worstY, 150.0f - p.position.y);
  }
  check("tilt after 8 s", tiltDeg(p), 0.0f, 0.5f, "deg");
  check("angular rate", Vector3Length(p.angularVelocity), 0.0f, 0.02f, "rad/s");
  check("altitude lost in recovery", worstY, 0.0f, 3.0f, "m");
  check("residual speed", Vector3Length(p.velocity), 0.0f, 0.2f, "m/s");
}

void testLanding() {
  std::printf("Landing on the platform and lifting off\n");
  Player p = airborne(12.0f);
  p.position.z = 6.0f; // clear of the red block in the middle
  PlayerInput down;
  down.descend = true;
  run(p, down, 5.0f);
  run(p, {}, 2.0f);
  check("rests at eye height above the platform", p.position.y, kEyeHeight - 0.05f, kEyeHeight + 0.3f, "m");
  check("mode is Landed", p.mode == FlightMode::Landed ? 1.0f : 0.0f, 1.0f, 1.0f, "");
  check("engine throttled back on the skids", p.throttle, 0.0f, 0.05f, "");
  check("tilt on the ground", tiltDeg(p), 0.0f, 1.0f, "deg");
  run(p, {}, kIdleTimeout + 1.0f);
  check("engine shuts off when parked", p.engineRpm, 0.0f, 0.0f, "rpm");
  PlayerInput up;
  up.ascend = true;
  run(p, up, 3.0f);
  check("lifts off again", p.position.y, 10.0f, 40.0f, "m");
}

void testSteepTerrain() {
  std::printf("Flying into terrain\n");
  Player p = cruising(60.0f);
  p.position = {0.0f, terrainHeight(0.0f, -150.0f) + 20.0f, -150.0f};
  PlayerInput dive;
  dive.mouseDelta = {0.0f, counts(40.0f)};
  stepPlayer(p, dive, kPhysicsDt);
  float lowest = 1e9f;
  for (float t = 0.0f; t < 5.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    lowest = std::min(lowest, p.position.y - kEyeHeight - terrainHeight(p.position.x, p.position.z));
  }
  check("never under the terrain", lowest, -0.2f, 1000.0f, "m");
  check("no NaN", std::isfinite(p.position.y) && std::isfinite(Vector3Length(p.velocity)) ? 1.0f : 0.0f, 1.0f,
        1.0f, "");
}

void testTransition() {
  std::printf("Hover to flight on W\n");
  Player p = airborne(kHigh);
  PlayerInput w;
  w.forward = true;
  float minY = 1e9f, maxY = -1e9f, peakAlpha = -99.0f;
  for (float t = 0.0f; t < 12.0f; t += kPhysicsDt) {
    stepPlayer(p, w, kPhysicsDt);
    minY = std::min(minY, p.position.y);
    maxY = std::max(maxY, p.position.y);
    if (p.flight > 0.5f) peakAlpha = std::max(peakAlpha, RAD2DEG * flowAngles(p).alpha);
  }
  check("mode is Flight", p.mode == FlightMode::Flight ? 1.0f : 0.0f, 1.0f, 1.0f, "");
  check("airspeed after 12 s of W", Vector3Length(p.velocity), 110.0f, kMaxSpeed + 1.0f, "m/s");
  check("altitude held through the transition", maxY - minY, 0.0f, 10.0f, "m");
  check("nozzle swung aft", p.nozzle * RAD2DEG, 70.0f, kNozzleAftDeg, "deg");
  check("heading kept", wrap(headingOfPath(p) + 90.0f), -2.0f, 2.0f, "deg");
  check("angle of attack stays clear of stall", peakAlpha, -5.0f, kStallAngleDeg - 3.0f, "deg");
  run(p, w, 6.0f);
  check("top speed", Vector3Length(p.velocity), kMaxSpeed - 3.0f, kMaxSpeed + 1.0f, "m/s");
}

void testCruise() {
  std::printf("Level cruise, hands off (100 m/s)\n");
  Player p = cruising();
  const float y0 = p.position.y;
  float peakRate = 0.0f;
  for (float t = 0.0f; t < 10.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    peakRate = std::max(peakRate, Vector3Length(p.angularVelocity));
  }
  check("speed holds without W", Vector3Length(p.velocity), 98.0f, 102.0f, "m/s");
  check("altitude drift", p.position.y - y0, -2.0f, 2.0f, "m");
  check("flight path is on the aim point", angleDeg(p.velocity, p.aimDir), 0.0f, 0.5f, "deg");
  check("wings level", std::fabs(bankDeg(p)), 0.0f, 1.0f, "deg");
  check("sideslip", RAD2DEG * flowAngles(p).beta, -0.5f, 0.5f, "deg");
  check("angle of attack (wing carries 1 g)", RAD2DEG * flowAngles(p).alpha, 0.5f, 4.0f, "deg");
  check("peak angular rate", peakRate, 0.0f, 0.1f, "rad/s");
}

// Mouse counts per physics step for an aim swipe of degPerSec.
float swipeCounts(float degPerSec) { return counts(degPerSec * kPhysicsDt); }

void testTurns() {
  std::printf("Mouse-aim turns at 100 m/s (the aim is a spring-loaded stick)\n");
  {
    // One big flick, then hands off: the point is capped, comes straight back,
    // and the craft turns only part of the way and stops.
    Player p = cruising();
    const float y0 = p.position.y, h0 = headingOfPath(p);
    PlayerInput flick;
    flick.mouseDelta = {counts(90.0f), 0.0f};
    stepPlayer(p, flick, kPhysicsDt);
    float peakLead = 0.0f, back = -1.0f, minY = 1e9f, maxY = -1e9f, h3 = 0.0f;
    for (float t = 0.0f; t < 6.0f; t += kPhysicsDt) {
      stepPlayer(p, {}, kPhysicsDt);
      peakLead = std::max(peakLead, angleDeg(p.velocity, p.aimDir));
      if (back < 0.0f && angleDeg(p.velocity, p.aimDir) < kAimHoldDeg + 0.5f) back = t;
      if (std::fabs(t - 3.0f) < 0.5f * kPhysicsDt) h3 = headingOfPath(p);
      minY = std::min(minY, p.position.y);
      maxY = std::max(maxY, p.position.y);
    }
    check("flick: aim lead never past the cap", peakLead, 0.0f, kAimMaxAngleDeg + 0.5f, "deg");
    check("flick: aim back on the path within", back, 0.0f, 1.5f, "s");
    check("flick: turns part of the way", wrap(headingOfPath(p) - h0), 5.0f, 45.0f, "deg");
    check("flick: then stops turning (3..6 s)", std::fabs(wrap(headingOfPath(p) - h3)), 0.0f, 1.0f, "deg");
    check("flick: altitude band", maxY - minY, 0.0f, 10.0f, "m");
    check("flick: wings level after", std::fabs(bankDeg(p)), 0.0f, 2.0f, "deg");
  }
  for (float side : {1.0f, -1.0f}) {
    // Keep swiping until the path is nearly round, then let go.
    Player p = cruising();
    const float y0 = p.position.y, h0 = headingOfPath(p);
    PlayerInput swipe;
    swipe.mouseDelta = {side * swipeCounts(150.0f), 0.0f};
    float released = -1.0f, peakAlpha = 0.0f, peakBank = 0.0f, minY = 1e9f, maxY = -1e9f;
    for (float t = 0.0f; t < 10.0f; t += kPhysicsDt) {
      const bool swiping = released < 0.0f && side * wrap(headingOfPath(p) - h0) < 75.0f;
      if (!swiping && released < 0.0f) released = t;
      stepPlayer(p, swiping ? swipe : PlayerInput{}, kPhysicsDt);
      peakAlpha = std::max(peakAlpha, RAD2DEG * flowAngles(p).alpha);
      peakBank = std::max(peakBank, side * bankDeg(p));
      minY = std::min(minY, p.position.y);
      maxY = std::max(maxY, p.position.y);
    }
    const bool right = side > 0.0f;
    check(right ? "swipe right: swiping for" : "swipe left: swiping for", released, 0.0f, 2.0f, "s");
    check(right ? "swipe right: ends near 90" : "swipe left: ends near 90", side * wrap(headingOfPath(p) - h0), 78.0f,
          102.0f, "deg");
    check("banks into the turn", peakBank, 60.0f, 95.0f, "deg");
    check("never stalls", peakAlpha, 0.0f, kStallAngleDeg - 3.0f, "deg");
    check("altitude band in the turn", maxY - minY, 0.0f, 25.0f, "m");
    check("altitude after", p.position.y - y0, -15.0f, 15.0f, "m");
    check("wings level after", std::fabs(bankDeg(p)), 0.0f, 2.0f, "deg");
  }

  Player p = cruising();
  const float y0 = p.position.y;
  PlayerInput held;
  held.mouseDelta = {counts(1.0f), 0.0f}; // 120 deg/s of aim
  float turned = 0.0f, last = headingOfPath(p), minY = 1e9f, maxY = -1e9f;
  for (float t = 0.0f; t < 20.0f; t += kPhysicsDt) {
    stepPlayer(p, held, kPhysicsDt);
    const float h = headingOfPath(p);
    turned += wrap(h - last);
    last = h;
    if (t > 3.0f) {
      minY = std::min(minY, p.position.y);
      maxY = std::max(maxY, p.position.y);
    }
  }
  check("held mouse: keeps turning", turned, 360.0f, 3600.0f, "deg");
  check("held mouse: level circle", maxY - minY, 0.0f, 25.0f, "m");
  check("held mouse: altitude", p.position.y - y0, -30.0f, 30.0f, "m");
  check("held mouse: banked hard (deg)", std::fabs(bankDeg(p)), 70.0f, 89.5f, "deg");
}

void testReversal() {
  std::printf("Arcade 180 at 450 km/h: keep swiping\n");
  Player p = cruising(125.0f);
  const float y0 = p.position.y;
  PlayerInput swipe;
  swipe.mouseDelta = {swipeCounts(150.0f), 0.0f};
  float reversed = -1.0f, peakAlpha = 0.0f, minSpeed = 1e9f, minY = 1e9f, maxY = -1e9f;
  for (float t = 0.0f; t < 8.0f; t += kPhysicsDt) {
    const bool swiping = wrap(headingOfPath(p) + 90.0f) < 165.0f && wrap(headingOfPath(p) + 90.0f) > -5.0f;
    stepPlayer(p, swiping ? swipe : PlayerInput{}, kPhysicsDt);
    peakAlpha = std::max(peakAlpha, RAD2DEG * flowAngles(p).alpha);
    minSpeed = std::min(minSpeed, Vector3Length(p.velocity));
    minY = std::min(minY, p.position.y);
    maxY = std::max(maxY, p.position.y);
    if (reversed < 0.0f && p.velocity.z > 0.0f && std::fabs(wrap(headingOfPath(p) - 90.0f)) < 5.0f) reversed = t;
  }
  check("heading reversed within", reversed, 0.0f, 3.2f, "s");
  check("heading stays reversed", std::fabs(wrap(headingOfPath(p) - 90.0f)), 0.0f, 10.0f, "deg");
  check("never stalls", peakAlpha, 0.0f, kStallAngleDeg - 3.0f, "deg");
  check("keeps its speed", minSpeed, 90.0f, 200.0f, "m/s");
  check("altitude band", maxY - minY, 0.0f, 40.0f, "m");
  check("altitude after", p.position.y - y0, -30.0f, 30.0f, "m");
}

void testLoop() {
  std::printf("Held mouse up: a loop\n");
  Player p = cruising(120.0f);
  const float y0 = p.position.y;
  PlayerInput held;
  held.mouseDelta = {0.0f, -counts(0.6f)}; // 72 deg/s of aim
  float maxPath = -90.0f, peakAlpha = 0.0f, inverted = 0.0f, minSpeed = 1e9f;
  float pitched = 0.0f, last = pathDeg(p);
  for (float t = 0.0f; t < 25.0f && pitched < 360.0f; t += kPhysicsDt) {
    stepPlayer(p, held, kPhysicsDt);
    maxPath = std::max(maxPath, pathDeg(p));
    peakAlpha = std::max(peakAlpha, RAD2DEG * flowAngles(p).alpha);
    if (p.body.up.y < -0.8f) inverted = 1.0f;
    minSpeed = std::min(minSpeed, Vector3Length(p.velocity));
    // Path angle in the vertical plane of the loop (north is -z).
    const float angle = RAD2DEG * std::atan2(p.velocity.y, -p.velocity.z);
    float d = angle - last;
    if (d < -180.0f) d += 360.0f;
    if (d > 180.0f) d -= 360.0f;
    pitched += d;
    last = angle;
  }
  check("goes vertical", maxPath, 85.0f, 90.0f, "deg");
  check("goes over the top inverted", inverted, 1.0f, 1.0f, "");
  check("comes all the way round", pitched, 300.0f, 400.0f, "deg");
  check("never stalls", peakAlpha, 0.0f, kStallAngleDeg - 3.0f, "deg");
  check("keeps flying speed", minSpeed, kWingSpeedHi, 200.0f, "m/s");
  check("loop height", p.position.y - y0, -100.0f, 100.0f, "m");
}

void testStopLoopAtTop() {
  std::printf("Stopping at the top of a loop: rolls upright (Immelmann)\n");
  Player p = cruising(120.0f);
  PlayerInput held;
  held.mouseDelta = {0.0f, -counts(0.6f)};
  // Keep pulling over the top, and let go as the nose comes level, heading back.
  for (int i = 0; i < 2400 && !(p.body.forward.y < 0.05f && p.body.forward.z > 0.5f); ++i) stepPlayer(p, held, kPhysicsDt);
  float upright = -1.0f;
  for (float t = 0.0f; t < 8.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    if (upright < 0.0f && t > 0.5f && p.body.up.y > 0.95f) upright = t;
  }
  check("upright within", upright, 0.0f, 7.0f, "s");
  check("heading reversed (south)", p.velocity.z / std::max(Vector3Length(p.velocity), 1.0f), 0.97f, 1.0f, "");
  check("level", pathDeg(p), -3.0f, 3.0f, "deg");
}

void testInvertedRecovery() {
  std::printf("Inverted, hands off\n");
  Player p = cruising();
  p.body = makeAttitude(p.body.forward, Vector3Negate(p.body.up));
  float upright = -1.0f;
  float lost = 0.0f;
  const float y0 = p.position.y;
  for (float t = 0.0f; t < 6.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    lost = std::max(lost, y0 - p.position.y);
    if (upright < 0.0f && p.body.up.y > 0.95f) upright = t;
  }
  check("rolls upright within", upright, 0.0f, 2.0f, "s");
  check("altitude lost", lost, 0.0f, 15.0f, "m");
}

void testManualRoll() {
  std::printf("A / D at speed: roll by hand\n");
  Player p = cruising();
  PlayerInput roll;
  roll.right = true;
  run(p, roll, 0.6f);
  check("right roll after 0.6 s", bankDeg(p), 70.0f, 120.0f, "deg");
  float level = -1.0f;
  for (float t = 0.0f; t < 4.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    if (level < 0.0f && std::fabs(bankDeg(p)) < 3.0f) level = t;
  }
  check("levels on release within", level, 0.0f, 1.5f, "s");
}

void testDive() {
  std::printf("Aim 30 deg below: pushes over, never past the negative-g limit\n");
  Player p = cruising();
  PlayerInput dive;
  dive.mouseDelta = {0.0f, counts(30.0f)};
  stepPlayer(p, dive, kPhysicsDt);
  float minG = 99.0f, onPath = -1.0f;
  Vector3 lastV = p.velocity;
  for (float t = 0.0f; t < 6.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    const Vector3 a = Vector3Scale(Vector3Subtract(p.velocity, lastV), 1.0f / kPhysicsDt);
    lastV = p.velocity;
    minG = std::min(minG, Vector3DotProduct(Vector3Add(a, Vector3{0.0f, kGravity, 0.0f}), p.body.up) / kGravity);
    if (onPath < 0.0f && angleDeg(p.velocity, p.aimDir) < 2.0f) onPath = t;
  }
  check("path on the aim within", onPath, 0.0f, 3.0f, "s");
  check("most negative g", minG, -kMaxPushG - 0.3f, 1.0f, "g");
  check("stays upright", p.body.up.y, 0.5f, 1.0f, "");
}

// Real mouse pitches are never perfectly vertical. A pitch with a little
// sideways drift must not snap the wings over: the bank should stay a small,
// proportional correction, for a pull up and for a push down.
void testPitchWithDrift() {
  std::printf("Pitch with a little sideways drift at speed: no snap roll\n");
  float worstBank = 0.0f, worstOnPath = 0.0f;
  for (float speed : {100.0f, 130.0f}) {
    for (float pitchDeg : {20.0f, -20.0f, 8.0f, -8.0f}) {
      for (float drift : {3.0f, -3.0f, 8.0f, -8.0f}) {
        Player p = cruising(speed);
        PlayerInput flick;
        flick.mouseDelta = {counts(drift), counts(-pitchDeg)};
        stepPlayer(p, flick, kPhysicsDt);
        float peakBank = 0.0f, onPath = 99.0f;
        for (float t = 0.0f; t < 6.0f; t += kPhysicsDt) {
          stepPlayer(p, {}, kPhysicsDt);
          peakBank = std::max(peakBank, std::fabs(bankDeg(p)));
          if (onPath > 98.0f && angleDeg(p.velocity, p.aimDir) < 1.0f) onPath = t;
        }
        worstBank = std::max(worstBank, peakBank);
        worstOnPath = std::max(worstOnPath, onPath);
      }
    }
  }
  // 3 deg of drift barely rocks the wings (~3 deg); the worst case, 8 deg sideways while pushing
  // 8 deg down, is a real diagonal move and banks a measured ~25 deg (it used to roll near knife edge).
  check("worst peak bank (3 and 8 deg drift, +-8/20 pitch)", worstBank, 0.0f, 30.0f, "deg");
  check("slowest path onto the aim", worstOnPath, 0.0f, 4.0f, "s");
}

// Diving, then swinging the aim sideways: the path must turn the way the aim
// went (left is left), and the wings must bank that way, not roll the other way
// and push.
void testDiveThenTurn() {
  std::printf("Dive, then aim to the side: turns toward the aim\n");
  for (float diveDeg : {15.0f, 30.0f, 60.0f, 80.0f}) {
    for (float side : {-1.0f, 1.0f}) {
      Player p = cruising(130.0f);
      PlayerInput down;
      down.mouseDelta = {0.0f, counts(diveDeg)};
      stepPlayer(p, down, kPhysicsDt);
      run(p, {}, 3.0f);
      const float h0 = headingOfPath(p);
      PlayerInput turn;
      turn.mouseDelta = {side * counts(30.0f), 0.0f};
      stepPlayer(p, turn, kPhysicsDt);
      float wrongBank = 0.0f, wrongTurn = 0.0f;
      for (float t = 0.0f; t < 3.0f; t += kPhysicsDt) {
        stepPlayer(p, {}, kPhysicsDt);
        // bankDeg > 0 is right wing down; headingOfPath grows turning right.
        wrongBank = std::max(wrongBank, -side * pathBankDeg(p));
        wrongTurn = std::max(wrongTurn, -side * wrap(headingOfPath(p) - h0));
      }
      char name[64];
      std::snprintf(name, sizeof name, "dive %.0f, aim %s: bank the wrong way", diveDeg, side < 0 ? "left" : "right");
      check(name, wrongBank, 0.0f, 6.0f, "deg"); // was 88 deg at a 30 deg dive; ~5 near vertical from the skid
      std::snprintf(name, sizeof name, "dive %.0f, aim %s: path turns the wrong way", diveDeg, side < 0 ? "left" : "right");
      check(name, wrongTurn, 0.0f, 1.0f, "deg");
    }
  }
}

// Swipe the nose up, let go: the pitch stops soon after, not where the point
// was flung to.
void testPitchSwipeRelease() {
  std::printf("Pitch swipe at speed, then let go: the climb angle stops\n");
  for (float dir : {1.0f, -1.0f}) {
    Player p = cruising(120.0f);
    PlayerInput swipe;
    swipe.mouseDelta = {0.0f, -dir * swipeCounts(60.0f)};
    run(p, swipe, 0.4f); // 24 deg of aim
    const float atRelease = pathDeg(p);
    float settled = -1.0f, last = atRelease;
    for (float t = 0.0f; t < 4.0f; t += kPhysicsDt) {
      stepPlayer(p, {}, kPhysicsDt);
      const float now = pathDeg(p);
      if (settled < 0.0f && std::fabs(now - last) < 0.5f * kPhysicsDt) settled = t; // < 0.5 deg/s
      last = now;
    }
    check(dir > 0.0f ? "up: climb angle keeps growing after release by" : "down: dive angle keeps growing after release by",
          dir * (pathDeg(p) - atRelease), 0.0f, 12.0f, "deg");
    check(dir > 0.0f ? "up: path stops pitching within" : "down: path stops pitching within", settled, 0.0f, 2.0f, "s");
    check(dir > 0.0f ? "up: then holds it" : "down: then holds it", dir * pathDeg(p), 3.0f, 30.0f, "deg");
  }
}

void testSlowToHover() {
  std::printf("S from cruise: slows down and comes to a hover\n");
  Player p = cruising();
  const float y0 = p.position.y;
  PlayerInput s;
  s.back = true;
  float minY = 1e9f, maxY = -1e9f;
  float t = 0.0f;
  for (; t < 20.0f && Vector3Length(p.velocity) > 20.0f; t += kPhysicsDt) {
    stepPlayer(p, s, kPhysicsDt);
    minY = std::min(minY, p.position.y);
    maxY = std::max(maxY, p.position.y);
  }
  check("100 -> 20 m/s on S in", t, 1.5f, 3.5f, "s");
  float stopped = -1.0f;
  for (float u = 0.0f; u < 10.0f; u += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    minY = std::min(minY, p.position.y);
    maxY = std::max(maxY, p.position.y);
    if (stopped < 0.0f && Vector3Length(p.velocity) < 0.5f) stopped = u;
  }
  check("then hovers within", stopped, 0.0f, 4.0f, "s");
  check("mode is Hover", p.mode == FlightMode::Hover ? 1.0f : 0.0f, 1.0f, 1.0f, "");
  check("altitude band", maxY - minY, 0.0f, 20.0f, "m");
  check("altitude change", p.position.y - y0, -20.0f, 20.0f, "m");
  check("level", tiltDeg(p), 0.0f, 1.0f, "deg");
}

void testTakeoff() {
  std::printf("Takeoff from the platform: W with the aim a little up\n");
  Player p;
  PlayerInput look;
  look.mouseDelta = {0.0f, -counts(12.0f)};
  stepPlayer(p, look, kPhysicsDt);
  PlayerInput w;
  w.forward = true;
  run(p, w, 10.0f);
  check("flying", p.mode == FlightMode::Flight ? 1.0f : 0.0f, 1.0f, 1.0f, "");
  check("speed", Vector3Length(p.velocity), 80.0f, kMaxSpeed + 1.0f, "m/s");
  check("height above ground", playerAltitudeAboveGround(p), 35.0f, 1000.0f, "m");
  check("climb angle follows the aim", pathDeg(p), 9.0f, 15.0f, "deg");
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
  testTransition();
  testCruise();
  testTurns();
  testReversal();
  testLoop();
  testStopLoopAtTop();
  testInvertedRecovery();
  testManualRoll();
  testDive();
  testPitchWithDrift();
  testDiveThenTurn();
  testPitchSwipeRelease();
  testSlowToHover();
  testTakeoff();
  std::printf("\n%d check(s) failed\n", failures);
  return failures;
}
