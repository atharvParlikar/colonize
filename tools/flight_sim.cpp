// Headless checks of the flight controllers against their design targets.
// Build target: flight-sim. Exit code is the number of failed checks.
// The ranges encode the intended feel; when tuning changes, update them
// deliberately, not to silence a failure.

#include "core/constants.hpp"
#include "player/attitude.hpp"
#include "player/gload.hpp"
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
  std::printf("  [%s] %-48s %8.2f %-5s (target %.2f..%.2f)\n", ok ? "PASS" : "FAIL", name, value, unit, lo, hi);
}

constexpr float kHigh = 1500.0f; // clear of every mountain near spawn

void run(Player &p, const PlayerInput &in, float seconds) {
  for (float t = 0.0f; t < seconds; t += kPhysicsDt) stepPlayer(p, in, kPhysicsDt);
}

float deg(float rad) { return rad * RAD2DEG; }
float headingDeg(Vector3 v) { return deg(headingOf(v)); }
float pathDeg(const Player &p) {
  return deg(std::asin(std::clamp(p.velocity.y / std::max(Vector3Length(p.velocity), 1e-3f), -1.0f, 1.0f)));
}
float wrapDeg(float d) { return deg(wrapAngle(d * DEG2RAD)); }
float speedOf(const Player &p) { return Vector3Length(p.velocity); }
// Mouse counts per physics step for a stick / look swipe of countsPerSec.
Vector2 swipe(float xPerSec, float yPerSec) { return {xPerSec * kPhysicsDt, yPerSec * kPhysicsDt}; }

Player hovering(float height = kHigh) {
  Player p;
  p.position = {0.0f, height, 0.0f};
  p.mode = FlightMode::Hover;
  p.engineIdleTime = 0.0f;
  return p;
}

// Straight and level in the jet, high up, heading north.
Player jetting(float speed = 120.0f) {
  Player p = hovering();
  p.mode = FlightMode::Jet;
  p.convert = 1.0f;
  p.velocity = {0.0f, 0.0f, -speed};
  p.body = makeAttitude({0.0f, 0.0f, -1.0f}, kWorldUp);
  p.throttle = jetDrag() * speed * speed / tuning.jetThrustAccel;
  run(p, {}, 0.5f);
  return p;
}

// ---- Hover ----------------------------------------------------------------

void testHoverMove() {
  std::printf("Hover: W, let go\n");
  Player p = hovering();
  const float y0 = p.position.y;
  PlayerInput w;
  w.forward = true;
  float reached = -1.0f;
  for (float t = 0.0f; t < 4.0f; t += kPhysicsDt) {
    stepPlayer(p, w, kPhysicsDt);
    if (reached < 0.0f && speedOf(p) > tuning.hoverMaxSpeed - 0.5f) reached = t;
  }
  check("top speed reached within", reached, 0.0f, 2.5f, "s");
  check("heading kept (north)", headingDeg(p.velocity), -0.5f, 0.5f, "deg");
  float stopped = -1.0f;
  for (float t = 0.0f; t < 4.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    if (stopped < 0.0f && speedOf(p) < 0.3f) stopped = t;
  }
  check("let go: stopped within", stopped, 0.0f, 1.6f, "s");
  check("altitude held", p.position.y - y0, -0.3f, 0.3f, "m");
  check("upright again", deg(std::acos(std::clamp(p.body.up.y, -1.0f, 1.0f))), 0.0f, 1.0f, "deg");
}

void testHoverStrafeClimbLook() {
  std::printf("Hover: strafe, climb, look\n");
  Player p = hovering();
  PlayerInput d;
  d.right = true;
  run(p, d, 2.0f);
  check("strafe speed", p.velocity.x, tuning.hoverStrafeSpeed - 0.5f, tuning.hoverStrafeSpeed + 0.1f, "m/s");
  check("strafing leans into it, then levels", deg(bankOf(p.body)), -1.0f, 1.0f, "deg");

  p = hovering();
  PlayerInput up;
  up.ascend = true;
  run(p, up, 2.0f);
  check("climb speed", p.velocity.y, tuning.hoverClimbSpeed - 0.5f, tuning.hoverClimbSpeed + 0.1f, "m/s");

  p = hovering();
  PlayerInput look;
  look.mouseDelta = {90.0f * DEG2RAD / tuning.lookSensitivity, 0.0f};
  stepPlayer(p, look, kPhysicsDt);
  check("look turns at once", deg(p.lookHeading), 89.0f, 91.0f, "deg");
  run(p, {}, 0.6f);
  check("body follows the look within 0.6 s", headingDeg(p.body.forward), 87.0f, 93.0f, "deg");
}

void testLanding() {
  std::printf("Hover: land, idle down\n");
  Player p = hovering(terrainHeight(0.0f, 0.0f) + kEyeHeight + 6.0f);
  PlayerInput down;
  down.descend = true;
  run(p, down, 3.0f);
  run(p, {}, 0.5f);
  check("landed", p.mode == FlightMode::Landed ? 1.0f : 0.0f, 1.0f, 1.0f, "");
  run(p, {}, kIdleTimeout + 0.5f);
  check("engine off at rest", p.engineRpm, 0.0f, 0.0f, "rpm");
}

// ---- Conversion -------------------------------------------------------------

void testConversion() {
  std::printf("Conversion: F arms, the speed gate starts it\n");
  Player p = hovering();
  PlayerInput f;
  f.convert = true;
  stepPlayer(p, f, kPhysicsDt);
  run(p, {}, 1.0f);
  check("armed at rest: still hovering", p.mode == FlightMode::Hover && p.convertArmed ? 1.0f : 0.0f, 1.0f, 1.0f, "");
  PlayerInput w;
  w.forward = true;
  float started = -1.0f, jet = -1.0f;
  for (float t = 0.0f; t < 6.0f; t += kPhysicsDt) {
    stepPlayer(p, w, kPhysicsDt);
    if (started < 0.0f && p.mode == FlightMode::ToJet) started = t;
    if (jet < 0.0f && p.mode == FlightMode::Jet) jet = t;
  }
  check("converts once past the gate (s of W)", started, 1.0f, 2.5f, "s");
  check("conversion takes", jet - started, tuning.convertTime - 0.05f, tuning.convertTime + 0.05f, "s");
  check("flying as a jet", p.mode == FlightMode::Jet ? 1.0f : 0.0f, 1.0f, 1.0f, "");

  std::printf("Conversion back: only below the gate\n");
  p = jetting(120.0f);
  stepPlayer(p, f, kPhysicsDt);
  run(p, {}, 0.5f);
  check("armed at 430 km/h: still a jet", p.mode == FlightMode::Jet && p.convertArmed ? 1.0f : 0.0f, 1.0f, 1.0f, "");
  PlayerInput s;
  s.back = true;
  float back = -1.0f, hover = -1.0f;
  for (float t = 0.0f; t < 15.0f; t += kPhysicsDt) {
    stepPlayer(p, back < 0.0f ? s : PlayerInput{}, kPhysicsDt);
    if (back < 0.0f && p.mode == FlightMode::ToHover) back = t;
    if (hover < 0.0f && p.mode == FlightMode::Hover) hover = t;
  }
  check("slowing with S: converts back within", back, 0.0f, 6.0f, "s");
  check("hovering again", p.mode == FlightMode::Hover ? 1.0f : 0.0f, 1.0f, 1.0f, "");
  check("and brought to a stop", speedOf(p), 0.0f, 0.5f, "m/s");
}

// ---- Jet ----------------------------------------------------------------------

void testJetCruise() {
  std::printf("Jet: hands off at 120 m/s\n");
  Player p = jetting(120.0f);
  const float y0 = p.position.y;
  run(p, {}, 10.0f);
  check("speed held by the throttle", speedOf(p), 118.0f, 122.0f, "m/s");
  check("altitude held", p.position.y - y0, -3.0f, 3.0f, "m");
  check("heading held", headingDeg(p.velocity), -0.5f, 0.5f, "deg");
  check("wings level", deg(bankOf(p.body)), -1.0f, 1.0f, "deg");
  check("1 g", p.load.g, 0.95f, 1.05f, "g");
}

void testJetFlick() {
  std::printf("Jet: one flick, let go (the stick springs back)\n");
  Player p = jetting(120.0f);
  const float h0 = headingDeg(p.velocity);
  PlayerInput flick;
  flick.mouseDelta = {400.0f, 0.0f}; // past full stick
  stepPlayer(p, flick, kPhysicsDt);
  float peakBank = 0.0f, h2 = 0.0f;
  for (float t = 0.0f; t < 5.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    peakBank = std::max(peakBank, deg(bankOf(p.body)));
    if (std::fabs(t - 2.0f) < 0.5f * kPhysicsDt) h2 = headingDeg(p.velocity);
  }
  check("turns right, part of the way", wrapDeg(headingDeg(p.velocity) - h0), 5.0f, 40.0f, "deg");
  check("then stops turning (2..5 s)", std::fabs(wrapDeg(headingDeg(p.velocity) - h2)), 0.0f, 1.0f, "deg");
  check("banked right while turning", peakBank, 20.0f, 90.0f, "deg");
  check("wings level after", deg(bankOf(p.body)), -2.0f, 2.0f, "deg");
}

void testJetTurns() {
  std::printf("Jet: keep swiping to turn 90, let go\n");
  for (float side : {1.0f, -1.0f}) {
    Player p = jetting(120.0f);
    const float h0 = headingDeg(p.velocity), y0 = p.position.y;
    float released = -1.0f, minY = 1e9f, maxY = -1e9f;
    for (float t = 0.0f; t < 8.0f; t += kPhysicsDt) {
      const bool swiping = released < 0.0f && side * wrapDeg(headingDeg(p.velocity) - h0) < 80.0f;
      if (!swiping && released < 0.0f) released = t;
      PlayerInput in;
      in.mouseDelta = swiping ? swipe(side * 1500.0f, 0.0f) : Vector2{};
      stepPlayer(p, in, kPhysicsDt);
      minY = std::min(minY, p.position.y);
      maxY = std::max(maxY, p.position.y);
    }
    check(side > 0 ? "right: swiping for" : "left: swiping for", released, 0.5f, 3.0f, "s");
    check(side > 0 ? "right: ends near 90" : "left: ends near 90", side * wrapDeg(headingDeg(p.velocity) - h0),
          82.0f, 100.0f, "deg");
    check("altitude band", maxY - minY, 0.0f, 15.0f, "m");
    check("altitude after", p.position.y - y0, -10.0f, 10.0f, "m");
    check("wings level after", deg(bankOf(p.body)), -2.0f, 2.0f, "deg");
  }

  std::printf("Jet: 180 at 450 km/h, swiping hard (blacks out on the way round)\n");
  Player p = jetting(125.0f);
  float reversed = -1.0f, peakG = 0.0f, peakBank = 0.0f, peakGrey = 0.0f, minSpeed = 1e9f;
  for (float t = 0.0f; t < 8.0f; t += kPhysicsDt) {
    const float turned = wrapDeg(headingDeg(p.velocity) - 0.0f);
    const bool swiping = reversed < 0.0f && turned > -5.0f && turned < 170.0f;
    PlayerInput in;
    in.mouseDelta = swiping ? swipe(3000.0f, 0.0f) : Vector2{};
    stepPlayer(p, in, kPhysicsDt);
    peakG = std::max(peakG, p.load.g);
    peakBank = std::max(peakBank, deg(bankOf(p.body)));
    peakGrey = std::max(peakGrey, p.load.greyout);
    minSpeed = std::min(minSpeed, speedOf(p));
    if (reversed < 0.0f && std::fabs(wrapDeg(headingDeg(p.velocity) - 180.0f)) < 5.0f) reversed = t;
  }
  check("reversed within", reversed, 0.0f, 6.0f, "s");
  check("pulls hard (g)", peakG, 8.5f, tuning.jetMaxG + 1.5f, "g");
  check("banked hard", peakBank, 70.0f, 90.0f, "deg");
  check("the pull costs speed (slowest)", minSpeed, 50.0f, 110.0f, "m/s");
  check("and the pilot feels it (peak greyout)", peakGrey, 0.5f, 1.0f, "");
  check("and has recovered 2.5 s after", p.load.greyout, 0.0f, 0.0f, "");

  std::printf("Jet: 180 near corner speed (290 km/h): tighter, quicker\n");
  p = jetting(80.0f);
  reversed = -1.0f;
  for (float t = 0.0f; t < 8.0f; t += kPhysicsDt) {
    const float turned = wrapDeg(headingDeg(p.velocity));
    const bool swiping = reversed < 0.0f && turned > -5.0f && turned < 170.0f;
    PlayerInput in;
    in.mouseDelta = swiping ? swipe(3000.0f, 0.0f) : Vector2{};
    stepPlayer(p, in, kPhysicsDt);
    if (reversed < 0.0f && std::fabs(wrapDeg(headingDeg(p.velocity) - 180.0f)) < 5.0f) reversed = t;
  }
  check("reversed within", reversed, 0.0f, 3.5f, "s");
}

void testJetPitch() {
  std::printf("Jet: swipe the nose up, let go\n");
  Player p = jetting(120.0f);
  PlayerInput up;
  up.mouseDelta = swipe(0.0f, -1500.0f);
  run(p, up, 0.4f);
  float settled = -1.0f, last = pathDeg(p);
  for (float t = 0.0f; t < 3.0f; t += kPhysicsDt) {
    stepPlayer(p, {}, kPhysicsDt);
    if (settled < 0.0f && std::fabs(pathDeg(p) - last) < 1.0f * kPhysicsDt) settled = t;
    last = pathDeg(p);
  }
  check("climbing", pathDeg(p), 8.0f, 45.0f, "deg");
  check("pitch stops within", settled, 0.0f, 1.5f, "s");
  const float v0 = speedOf(p);
  run(p, {}, 3.0f);
  check("the climb bleeds speed (3 s)", v0 - speedOf(p), 5.0f, 60.0f, "m/s");
  check("climb angle held", pathDeg(p), 8.0f, 45.0f, "deg");
}

void testJetThrottle() {
  std::printf("Jet: throttle, airbrake, stall\n");
  Player p = jetting(120.0f);
  PlayerInput w;
  w.forward = true;
  run(p, w, 8.0f);
  check("full throttle accelerates (8 s)", speedOf(p), 140.0f, 171.0f, "m/s");
  p = jetting(120.0f);
  p.throttle = 0.0f;
  run(p, {}, 3.0f);
  const float idle = speedOf(p);
  p = jetting(120.0f);
  PlayerInput s;
  s.back = true;
  run(p, s, 3.0f);
  check("idle slows (3 s)", idle, 90.0f, 110.0f, "m/s");
  check("S to idle + airbrake slows harder", speedOf(p), 40.0f, idle - 10.0f, "m/s");

  p = jetting(35.0f);
  p.throttle = 0.0f;
  run(p, {}, 2.0f);
  check("below the stall the nose drops", pathDeg(p), -90.0f, -10.0f, "deg");
}

void testJetRoll() {
  std::printf("Jet: roll by hand\n");
  Player p = jetting(120.0f);
  PlayerInput d;
  d.right = true;
  run(p, d, 0.3f);
  const float set = deg(bankOf(p.body));
  check("D rolls right", set, 45.0f, 75.0f, "deg");
  run(p, {}, 0.5f);
  check("let go: the bank holds", deg(bankOf(p.body)), set - 15.0f, set + 2.0f, "deg");
  run(p, {}, 8.0f);
  check("then eases back level", std::fabs(deg(bankOf(p.body))), 0.0f, 3.0f, "deg");
  run(p, d, 1.0f);
  check("held D: rolls on past inverted", std::fabs(deg(bankOf(p.body))), 90.0f, 180.0f, "deg");
}

// ---- g -------------------------------------------------------------------------

float firstTime(float g, float seconds, float GLoad::*field, float above) {
  GLoad load;
  for (float t = 0.0f; t < seconds; t += kPhysicsDt) {
    stepGLoad(load, g, kPhysicsDt);
    if (load.*field > above) return t;
  }
  return -1.0f;
}

void testGLoad() {
  std::printf("g: dose, greyout, blackout, redout\n");
  check("6 g: greyout starts after", firstTime(6.0f, 10.0f, &GLoad::greyout, 0.01f), 1.8f, 3.0f, "s");
  check("6 g: tunnel starts after", firstTime(6.0f, 10.0f, &GLoad::blackout, 0.01f), 4.5f, 7.0f, "s");
  check("10 g: greyout starts after", firstTime(10.0f, 5.0f, &GLoad::greyout, 0.01f), 0.0f, 0.5f, "s");
  check("10 g: tunnel starts after", firstTime(10.0f, 5.0f, &GLoad::blackout, 0.01f), 0.5f, 1.2f, "s");
  check("10 g: black after", firstTime(10.0f, 5.0f, &GLoad::blackout, 0.99f), 1.4f, 2.5f, "s");
  check("4.5 g: never greys", firstTime(4.5f, 20.0f, &GLoad::greyout, 0.0f), -1.0f, -1.0f, "s");
  check("-4 g: redout starts after", firstTime(-4.0f, 5.0f, &GLoad::redout, 0.01f), 0.0f, 1.5f, "s");

  GLoad load;
  for (float t = 0.0f; t < 3.0f; t += kPhysicsDt) stepGLoad(load, 10.0f, kPhysicsDt);
  float clear = -1.0f;
  for (float t = 0.0f; t < 10.0f; t += kPhysicsDt) {
    stepGLoad(load, 1.0f, kPhysicsDt);
    if (clear < 0.0f && load.greyout <= 0.0f) clear = t;
  }
  check("recovers from black at 1 g within", clear, 2.0f, 5.0f, "s");

  std::printf("g: blacked out, the jet eases off (soft loss of control)\n");
  Player p = jetting(150.0f);
  p.throttle = 1.0f;
  float peakBlack = 0.0f, lateG = 0.0f;
  int lateN = 0;
  for (float t = 0.0f; t < 10.0f; t += kPhysicsDt) {
    PlayerInput in;
    in.mouseDelta = swipe(0.0f, -3000.0f); // hold the stick fully back
    stepPlayer(p, in, kPhysicsDt);
    peakBlack = std::max(peakBlack, p.load.blackout);
    if (t > 6.0f) {
      lateG += p.load.g;
      ++lateN;
    }
  }
  check("blacks out", peakBlack, 0.3f, 1.0f, "");
  check("held pull settles to (g, over a loop)", lateG / std::max(lateN, 1), 4.5f, 7.5f, "g");

  std::printf("g: hovering never builds a dose\n");
  p = hovering();
  float peakDose = 0.0f;
  PlayerInput w;
  w.forward = true;
  for (float t = 0.0f; t < 6.0f; t += kPhysicsDt) {
    stepPlayer(p, t < 3.0f ? w : PlayerInput{}, kPhysicsDt);
    peakDose = std::max(peakDose, p.load.dose);
  }
  check("dose", peakDose, 0.0f, 0.0f, "");
}

} // namespace

int main() {
  testHoverMove();
  testHoverStrafeClimbLook();
  testLanding();
  testConversion();
  testJetCruise();
  testJetFlick();
  testJetTurns();
  testJetPitch();
  testJetThrottle();
  testJetRoll();
  testGLoad();
  std::printf("\n%d check(s) failed\n", failures);
  return failures;
}
