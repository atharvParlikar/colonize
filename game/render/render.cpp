#include "render/render.hpp"

#include "collision/capsule.hpp"
#include "core/constants.hpp"
#include "render/combat.hpp"
#include "render/grass.hpp"
#include "render/scenery.hpp"
#include "engine/math/attitude.hpp"
#include "world/colliders.hpp"
#include "world/level.hpp"
#include "raylib.h"

#include <algorithm>
#include <cmath>

using namespace engine;

namespace {

void drawObstacles() {
  for (const BoxObstacle &obstacle : levelObstacles()) {
    DrawCube(obstacle.center, obstacle.size.x, obstacle.size.y, obstacle.size.z,
             obstacle.color);
    if (obstacle.size.y > 0.5f) {
      DrawCubeWires(obstacle.center, obstacle.size.x, obstacle.size.y,
                    obstacle.size.z, MAROON);
    }
  }
}

} // namespace

void renderWorld(const Camera3D &camera, const Player &player, const Combat &combat, float alpha) {
  const float time = static_cast<float>(GetTime());
  updateGrassWind(camera, time);
  beginScene(camera);

  BeginShaderMode(worldShader());
  drawLandscape();
  drawObstacles();
  EndShaderMode();
  drawCombat(combat, camera, alpha);
  drawGrass(camera, player.position, player.velocity, time);
  // drawCollisionDebug(player);

  endScene();
}

void drawCollisionDebug(const Player &player) {
#if !defined(NDEBUG)
  const Capsule capsule = makePlayerCapsule(player);
  DrawCapsuleWires(capsule.start, capsule.end, capsule.radius, 12, 24,
                   ORANGE);

  for (const BoundingBox &collider : getLevelColliders()) {
    DrawBoundingBox(collider, GREEN);
  }
#endif
}

namespace {

// --- Cockpit HUD -----------------------------------------------------------

constexpr Color kCyan{90, 230, 255, 255};
constexpr Color kCyanDim{40, 120, 140, 255};
constexpr Color kAmber{255, 176, 60, 255};
constexpr Color kRed{255, 84, 72, 255};
constexpr Color kPanel{4, 16, 24, 150};
constexpr Color kWhite{225, 245, 250, 255};

Color alpha(Color c, float a) {
  c.a = static_cast<unsigned char>(std::clamp(a, 0.0f, 1.0f) * 255.0f);
  return c;
}

void text(const char *str, float x, float y, float size, Color color, float spacing = 1.5f) {
  DrawTextEx(GetFontDefault(), str, {x, y}, size, spacing, color);
}

void textCentered(const char *str, float cx, float y, float size, Color color, float spacing = 1.5f) {
  const Vector2 m = MeasureTextEx(GetFontDefault(), str, size, spacing);
  text(str, cx - m.x * 0.5f, y, size, color, spacing);
}

void textRight(const char *str, float rx, float y, float size, Color color, float spacing = 1.5f) {
  const Vector2 m = MeasureTextEx(GetFontDefault(), str, size, spacing);
  text(str, rx - m.x, y, size, color, spacing);
}

// A translucent panel with corner brackets.
void panel(float x, float y, float w, float h, Color edge = kCyanDim) {
  DrawRectangle(static_cast<int>(x), static_cast<int>(y), static_cast<int>(w), static_cast<int>(h), kPanel);
  const float k = 12.0f;
  auto corner = [&](float cx, float cy, float dx, float dy) {
    DrawLineEx({cx, cy}, {cx + dx * k, cy}, 2.0f, edge);
    DrawLineEx({cx, cy}, {cx, cy + dy * k}, 2.0f, edge);
  };
  corner(x, y, 1, 1);
  corner(x + w, y, -1, 1);
  corner(x, y + h, 1, -1);
  corner(x + w, y + h, -1, -1);
}

void fillTriangle(Vector2 a, Vector2 b, Vector2 c, Color color) {
  const float cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  if (cross < 0.0f) {
    DrawTriangle(a, b, c, color);
  } else {
    DrawTriangle(a, c, b, color);
  }
}

float wrap360(float degrees) {
  degrees = std::fmod(degrees, 360.0f);
  return degrees < 0.0f ? degrees + 360.0f : degrees;
}

struct Attitude2D {
  float headingDeg, pitchDeg, rollRad;
};

Attitude2D cameraAttitude(const Camera3D &camera) {
  const Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
  const Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera.up));
  Attitude2D a;
  a.headingDeg = wrap360(std::atan2(forward.x, -forward.z) * RAD2DEG); // north is -Z
  a.pitchDeg = std::asin(std::clamp(forward.y, -1.0f, 1.0f)) * RAD2DEG;
  a.rollRad = std::atan2(-right.y, camera.up.y);
  return a;
}

void drawCompass(const Attitude2D &att) {
  constexpr float cx = kScreenWidth * 0.5f;
  constexpr float y = 20.0f;
  constexpr float halfWidth = 250.0f;
  constexpr float pxPerDeg = 4.2f;
  panel(cx - halfWidth - 6, y - 6, halfWidth * 2 + 12, 52);

  BeginScissorMode(static_cast<int>(cx - halfWidth), static_cast<int>(y - 6), static_cast<int>(halfWidth * 2), 52);
  const int first = static_cast<int>(std::floor((att.headingDeg - halfWidth / pxPerDeg) / 5.0f)) * 5;
  for (int deg = first; deg <= att.headingDeg + halfWidth / pxPerDeg + 5; deg += 5) {
    const float x = cx + (deg - att.headingDeg) * pxPerDeg;
    const int h = static_cast<int>(wrap360(static_cast<float>(deg)) + 0.5f) % 360;
    const bool major = h % 15 == 0;
    DrawLineEx({x, y + 24}, {x, y + (major ? 10.0f : 17.0f)}, major ? 2.0f : 1.0f, kCyanDim);
    if (h % 45 == 0) {
      const char *label = h == 0 ? "N" : h == 90 ? "E" : h == 180 ? "S" : h == 270 ? "W"
                          : h == 45 ? "NE" : h == 135 ? "SE" : h == 225 ? "SW" : "NW";
      const bool cardinal = h % 90 == 0;
      textCentered(label, x, y - 4, cardinal ? 20 : 14, cardinal ? kCyan : kCyanDim);
    } else if (h % 30 == 0) {
      textCentered(TextFormat("%02d", h / 10), x, y + 26, 12, kCyanDim);
    }
  }
  EndScissorMode();

  fillTriangle({cx - 7, y + 38}, {cx + 7, y + 38}, {cx, y + 28}, kAmber);
  textCentered(TextFormat("%03d", static_cast<int>(att.headingDeg + 0.5f) % 360), cx, y + 46, 18, kAmber);
}

// Artificial horizon and pitch ladder, rotated by the camera roll.
void drawHorizon(const Attitude2D &att, float fovy) {
  constexpr Vector2 c{kScreenWidth * 0.5f, kScreenHeight * 0.5f};
  const float pxPerDeg = (kScreenHeight * 0.5f) / std::tan(fovy * 0.5f * DEG2RAD) * DEG2RAD;
  const float theta = -att.rollRad;
  const float cs = std::cos(theta), sn = std::sin(theta);
  auto rot = [&](float x, float y) {
    return Vector2{c.x + x * cs - y * sn, c.y + x * sn + y * cs};
  };

  for (int a = -90; a <= 90; a += 5) {
    const float dy = (att.pitchDeg - a) * pxPerDeg;
    if (std::fabs(dy) > 270.0f) continue;
    const bool horizon = a == 0;
    const bool major = a % 10 == 0;
    const float half = horizon ? 150.0f : (major ? 56.0f : 28.0f);
    const float gap = horizon ? 70.0f : 0.0f;
    const Color col = alpha(horizon ? kCyan : kCyanDim, 1.0f - std::fabs(dy) / 330.0f);
    DrawLineEx(rot(-half, dy), rot(-gap, dy), horizon ? 2.0f : 1.5f, col);
    DrawLineEx(rot(gap, dy), rot(half, dy), horizon ? 2.0f : 1.5f, col);
    if (major && !horizon) {
      const float tick = a > 0 ? 8.0f : -8.0f;
      DrawLineEx(rot(-half, dy), rot(-half, dy + tick), 1.5f, col);
      DrawLineEx(rot(half, dy), rot(half, dy + tick), 1.5f, col);
      const char *label = TextFormat("%d", std::abs(a));
      text(label, rot(half + 6, dy).x, rot(half + 6, dy).y - 7, 14, col);
    }
  }

  // Fixed boresight reticle.
  DrawCircleLinesV(c, 7.0f, alpha(kWhite, 0.8f));
  DrawLineEx({c.x - 26, c.y}, {c.x - 12, c.y}, 2.0f, alpha(kWhite, 0.8f));
  DrawLineEx({c.x + 12, c.y}, {c.x + 26, c.y}, 2.0f, alpha(kWhite, 0.8f));
}

void drawFlightPathMarker(const Player &player, const Camera3D &camera) {
  const float speed = Vector3Length(player.velocity);
  if (speed < 3.0f) return;
  const Vector3 dir = Vector3Scale(player.velocity, 1.0f / speed);
  const Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
  if (Vector3DotProduct(dir, forward) < 0.2f) return;
  const Vector2 p = GetWorldToScreen(Vector3Add(camera.position, Vector3Scale(dir, 200.0f)), camera);
  DrawCircleLinesV(p, 9.0f, kAmber);
  DrawLineEx({p.x - 22, p.y}, {p.x - 9, p.y}, 2.0f, kAmber);
  DrawLineEx({p.x + 9, p.y}, {p.x + 22, p.y}, 2.0f, kAmber);
  DrawLineEx({p.x, p.y - 9}, {p.x, p.y - 20}, 2.0f, kAmber);
}

void drawSpeedPanel(const Player &player) {
  constexpr float x = 36.0f, y = 250.0f, w = 210.0f, h = 190.0f;
  const float speed = Vector3Length(player.velocity);
  const float kmh = speed * 3.6f;
  panel(x, y, w, h);
  text("AIRSPEED", x + 14, y + 10, 14, kCyanDim);
  text(TextFormat("%3.0f", kmh), x + 14, y + 28, 64, kWhite, 2.0f);
  text("km/h", x + 138, y + 68, 18, kCyan);
  text(TextFormat("%5.1f m/s", speed), x + 14, y + 100, 18, kCyan);
  text(TextFormat("MACH %.2f", speed / 340.0f), x + 14, y + 124, 18, kCyan);

  constexpr int segments = 22;
  const float fraction = std::clamp(kmh / 500.0f, 0.0f, 1.0f);
  for (int i = 0; i < segments; ++i) {
    const bool lit = (i + 0.5f) / segments <= fraction;
    const float t = static_cast<float>(i) / segments;
    const Color on = t > 0.8f ? kRed : (t > 0.55f ? kAmber : kCyan);
    DrawRectangle(static_cast<int>(x + 14 + i * 8.6f), static_cast<int>(y + 156), 6, 18,
                  lit ? on : alpha(kCyanDim, 0.35f));
  }
}

void drawAltitudePanel(const Player &player) {
  constexpr float w = 210.0f, h = 190.0f;
  constexpr float x = kScreenWidth - 36.0f - w, y = 250.0f;
  const float agl = playerAltitudeAboveGround(player);
  panel(x, y, w, h);
  text("ALTITUDE AGL", x + 14, y + 10, 14, kCyanDim);
  text(TextFormat("%4.0f", std::max(agl, 0.0f)), x + 14, y + 28, 64, kWhite, 2.0f);
  text("m", x + 160, y + 68, 18, kCyan);
  text(TextFormat("ASL  %5.0f m", player.position.y - kEyeHeight), x + 14, y + 100, 18, kCyan);

  const float vs = player.velocity.y;
  const Color vsColor = vs < -9.0f ? kRed : (vs < -4.0f ? kAmber : kCyan);
  text(TextFormat("V/S %+6.1f m/s", vs), x + 14, y + 124, 18, vsColor);
  text(player.mode == FlightMode::Landed ? "LANDED" : (isJetMode(player.mode) ? "WING-BORNE" : "JET-BORNE"), x + 14,
       y + 152, 14, kCyanDim);
  // Tiny vertical-speed bar: up above the centre line, down below.
  const float mid = y + 176.0f;
  const float len = std::clamp(vs / 20.0f, -1.0f, 1.0f) * 60.0f;
  DrawLineEx({x + w - 24, mid}, {x + w - 24, mid - len}, 6.0f, vsColor);
  DrawLineEx({x + w - 34, mid}, {x + w - 14, mid}, 1.5f, kCyanDim);
}

void drawEngineGauge(const Player &player, float time) {
  (void)time;
  constexpr Vector2 c{kScreenWidth * 0.5f, 612.0f};
  constexpr float radius = 78.0f;
  constexpr float start = 150.0f, sweep = 240.0f;
  const float fraction = std::clamp(player.engineRpm / kRedlineRpm, 0.0f, 1.0f);
  const float turboFrac = kTurboStartRpm / kRedlineRpm;

  DrawCircleV(c, radius + 10.0f, kPanel);
  DrawRing(c, radius - 7.0f, radius, start, start + sweep, 64, alpha(kCyanDim, 0.35f));
  DrawRing(c, radius - 7.0f, radius, start + sweep * turboFrac, start + sweep, 40, alpha(kAmber, 0.25f));
  if (fraction > 0.002f) {
    const float split = std::min(fraction, turboFrac);
    DrawRing(c, radius - 7.0f, radius, start, start + sweep * split, 48, kCyan);
    if (fraction > turboFrac) {
      const Color hot = fraction > 0.85f ? kRed : kAmber;
      DrawRing(c, radius - 7.0f, radius, start + sweep * turboFrac, start + sweep * fraction, 48, hot);
    }
  }
  for (int k = 0; k <= 12; ++k) {
    const float a = (start + sweep * k / 12.0f) * DEG2RAD;
    const float r0 = radius + 2.0f, r1 = radius + (k % 2 == 0 ? 10.0f : 6.0f);
    DrawLineEx({c.x + std::cos(a) * r0, c.y + std::sin(a) * r0},
               {c.x + std::cos(a) * r1, c.y + std::sin(a) * r1}, 1.5f, kCyanDim);
  }
  const float needle = (start + sweep * fraction) * DEG2RAD;
  DrawLineEx({c.x + std::cos(needle) * 30.0f, c.y + std::sin(needle) * 30.0f},
             {c.x + std::cos(needle) * (radius - 10.0f), c.y + std::sin(needle) * (radius - 10.0f)}, 3.0f, kWhite);
  DrawCircleV(c, 5.0f, kWhite);

  textCentered(TextFormat("%.0f", player.engineRpm), c.x, c.y + 14, 22, kWhite);
  textCentered("RPM", c.x, c.y + 38, 12, kCyanDim);
  if (player.engineRpm > 1.0f) {
    textCentered("IDLE", c.x, c.y - radius - 26, 14, kCyanDim);
  }
}

// Top-down schematic: hull, wings and the engine filled by its throttle, plus
// a side view of the nozzle with the exhaust plume.
void drawShipDiagram(const Player &player, float time) {
  constexpr float w = 230.0f, h = 200.0f;
  constexpr float px = kScreenWidth - 36.0f - w, py = kScreenHeight - 36.0f - h;
  panel(px, py, w, h);
  text("SKIFF // ENGINE", px + 12, py + 8, 13, kCyanDim);

  const Vector2 c{px + w * 0.5f + 20.0f, py + 108.0f};
  const float u = 20.0f; // pixels per metre
  auto P = [&](float x, float z) { return Vector2{c.x + x * u, c.y - z * u}; };

  const Vector2 hull[] = {P(0, 4.2f), P(0.7f, 1.6f), P(0.8f, -2.4f), P(0.4f, -3.4f),
                          P(-0.4f, -3.4f), P(-0.8f, -2.4f), P(-0.7f, 1.6f)};
  for (int i = 1; i < 6; ++i) fillTriangle(hull[0], hull[i], hull[i + 1], alpha(kCyan, 0.18f));
  for (int i = 0; i < 7; ++i) DrawLineEx(hull[i], hull[(i + 1) % 7], 2.0f, kCyan);
  for (float side : {-1.0f, 1.0f}) {
    fillTriangle(P(side * 0.7f, 1.0f), P(side * 3.4f, -1.8f), P(side * 0.7f, -1.6f), alpha(kCyan, 0.12f));
    DrawLineEx(P(side * 0.7f, 1.0f), P(side * 3.4f, -1.8f), 1.5f, kCyan);
    DrawLineEx(P(side * 3.4f, -1.8f), P(side * 0.7f, -1.6f), 1.5f, kCyan);
  }
  DrawCircleV(P(0, 2.2f), 5.0f, alpha(kAmber, 0.8f)); // cockpit

  const float level = std::clamp(player.engineRpm / kRedlineRpm, 0.0f, 1.0f);
  const float nozzle = player.convert * 90.0f * DEG2RAD; // straight down hovering, aft in the jet
  const Vector2 engine = P(0, -1.0f);
  DrawCircleLinesV(engine, 12.0f, kCyan);
  DrawCircleV(engine, 11.0f * level, alpha(level > 0.7f ? kAmber : kCyan, 0.75f));
  textCentered(TextFormat("%.0f", level * 100.0f), engine.x, engine.y - 6, 12, kWhite);

  // Side view, nose to the right: the plume leaves opposite to the thrust.
  const Vector2 pivot{px + 34.0f, py + 70.0f};
  DrawLineEx({pivot.x - 16, pivot.y}, {pivot.x + 16, pivot.y}, 3.0f, kCyan);
  const Vector2 plume{-std::sin(nozzle), std::cos(nozzle)};
  const float flicker = 1.0f + 0.08f * std::sin(time * 40.0f);
  const float length = 8.0f + 34.0f * level * (player.engineRpm > 1.0f ? flicker : 0.0f);
  DrawLineEx(pivot, {pivot.x + plume.x * length, pivot.y + plume.y * length}, 5.0f,
             alpha(level > 0.7f ? kAmber : kCyan, 0.8f));
  textCentered(TextFormat("%+.0f", nozzle * RAD2DEG), pivot.x, pivot.y + 50, 12, kWhite);
  textCentered("NOZZLE", pivot.x, pivot.y + 64, 11, kCyanDim);

  const float roll = std::atan2(-player.body.right.y, player.body.up.y) * RAD2DEG;
  const float pitch = std::asin(std::clamp(player.body.forward.y, -1.0f, 1.0f)) * RAD2DEG;
  text(TextFormat("ROLL %+3.0f", roll), px + 12, py + h - 20, 13, kCyan);
  textRight(TextFormat("PITCH %+3.0f", pitch), px + w - 12, py + h - 20, 13, kCyan);
}

const char *modeName(FlightMode mode) {
  switch (mode) {
  case FlightMode::Landed: return "LANDED";
  case FlightMode::Hover: return "HOVER";
  case FlightMode::ToJet: return "CONVERT";
  case FlightMode::Jet: return "JET";
  case FlightMode::ToHover: return "CONVERT";
  }
  return "";
}

void drawStatus(const Player &player, float time) {
  constexpr float x = 36.0f, y = 462.0f, w = 210.0f, h = 100.0f;
  panel(x, y, w, h);
  text("FLIGHT MODE", x + 14, y + 10, 13, kCyanDim);
  text(modeName(player.mode), x + 14, y + 26, 34, kWhite, 2.0f);
  const bool blink = std::fmod(time, 0.8f) < 0.5f;
  const float speed = Vector3Length(player.velocity);
  if (player.engineRpm <= 1.0f) {
    text("ENGINE  COLD", x + 14, y + 68, 16, kCyanDim);
  } else if (player.convertArmed && !isJetMode(player.mode)) {
    if (blink) text(TextFormat("F ARMED  >%3.0f km/h", tuning.convertSpeed * 3.6f), x + 14, y + 68, 16, kAmber);
  } else if (player.convertArmed) {
    if (blink) text(TextFormat("F ARMED  <%3.0f km/h", tuning.convertBackSpeed * 3.6f), x + 14, y + 68, 16, kAmber);
  } else if (player.mode == FlightMode::ToJet || player.mode == FlightMode::ToHover) {
    text("NOZZLES SWINGING", x + 14, y + 68, 16, kAmber);
  } else if (player.mode == FlightMode::Jet) {
    text(player.airbrake ? "AIRBRAKE" : TextFormat("THROTTLE  %3.0f%%", player.throttle * 100.0f), x + 14, y + 68,
         16, player.airbrake ? kAmber : kCyan);
  } else {
    text(speed >= tuning.convertSpeed ? "F: CONVERT TO JET" : "F: ARM CONVERSION", x + 14, y + 68, 16, kCyanDim);
  }
  DrawRectangle(static_cast<int>(x + 14), static_cast<int>(y + 88), static_cast<int>(w - 28), 5,
                alpha(kCyanDim, 0.35f));
  DrawRectangle(static_cast<int>(x + 14), static_cast<int>(y + 88),
                static_cast<int>((w - 28) * std::clamp(player.convert, 0.0f, 1.0f)), 5, kAmber); // nozzle swing
}

// Felt load beside the reticle: amber as the dose builds, red once the
// greyout starts.
void drawGMeter(const Player &player) {
  const GLoad &g = player.load;
  const Color c = g.greyout > 0.0f || g.redout > 0.0f ? kRed : (g.dose > 0.0f || g.g > tuning.gOnset ? kAmber : kCyan);
  constexpr float x = kScreenWidth * 0.5f - 190.0f, y = kScreenHeight * 0.5f + 34.0f;
  textRight(TextFormat("%+.1f G", g.g), x + 70.0f, y, 22, c);
  const float stress = std::clamp(std::max(g.dose / tuning.blackoutDose, g.negDose / tuning.redoutDose), 0.0f, 1.0f);
  DrawRectangle(static_cast<int>(x), static_cast<int>(y + 26), 70, 4, alpha(kCyanDim, 0.35f));
  DrawRectangle(static_cast<int>(x), static_cast<int>(y + 26), static_cast<int>(70 * stress), 4, c);
}

// Crosshair at the aim point (the view looks where the gun aims), with hit
// ticks after a hit.
void drawCrosshair(const Combat &combat) {
  constexpr Vector2 c{kScreenWidth * 0.5f, kScreenHeight * 0.5f};
  DrawCircleLinesV(c, 9.0f, alpha(kCyan, 0.8f));
  DrawCircleV(c, 1.5f, kCyan);
  if (combat.hitMarker > 0.0f) {
    for (int i = 0; i < 4; ++i) {
      const float sx = (i & 1) ? 1.0f : -1.0f, sy = (i & 2) ? 1.0f : -1.0f;
      DrawLineEx({c.x + sx * 13.0f, c.y + sy * 13.0f}, {c.x + sx * 21.0f, c.y + sy * 21.0f}, 2.5f, kWhite);
    }
  }
}

// A bracket on every drone in range and in front of the view: at a few hundred
// metres a drone is only a few pixels.
void drawDroneMarkers(const Combat &combat, const Camera3D &camera) {
  constexpr float kMarkerRange = 900.0f; // m
  const Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
  for (auto [handle, drone] : combat.drones) {
    const Vector3 to = Vector3Subtract(drone.position, camera.position);
    const float distance = Vector3Length(to);
    if (distance > kMarkerRange || Vector3DotProduct(to, forward) <= 0.0f) continue;
    const Vector2 s = GetWorldToScreenEx(drone.position, camera, kScreenWidth, kScreenHeight);
    const bool hit = handle == combat.lastHit;
    const Color c = alpha(hit ? kAmber : kRed, 0.85f);
    const float k = 7.0f, r = 11.0f;
    for (int i = 0; i < 4; ++i) {
      const float sx = (i & 1) ? 1.0f : -1.0f, sy = (i & 2) ? 1.0f : -1.0f;
      DrawLineEx({s.x + sx * r, s.y + sy * r}, {s.x + sx * (r - k), s.y + sy * r}, 2.0f, c);
      DrawLineEx({s.x + sx * r, s.y + sy * r}, {s.x + sx * r, s.y + sy * (r - k)}, 2.0f, c);
    }
    text(TextFormat("%.0f", distance), s.x + r + 3, s.y + r - 4, 11, c, 1.0f);
  }
}

// Under the flight-mode panel: drones left, kills, and the health of the
// drone last hit. lastHit is a handle, so once that drone breaks get() is
// null and the row clears by itself.
void drawWeaponsPanel(const Combat &combat) {
  constexpr float x = 36.0f, y = 584.0f, w = 210.0f, h = 100.0f;
  panel(x, y, w, h);
  text("CANNON", x + 14, y + 10, 13, kCyanDim);
  text(TextFormat("TARGETS %2d/%d", static_cast<int>(combat.drones.size()), kDroneCount), x + 14, y + 30, 18, kCyan);
  text(TextFormat("KILLS %d", combat.kills), x + 14, y + 54, 16, kWhite);
  if (const Drone *drone = combat.drones.get(combat.lastHit)) {
    text("LAST HIT", x + 110, y + 56, 12, kCyanDim);
    for (int i = 0; i < kDroneHealth; ++i) {
      const Rectangle pip{x + 110 + i * 22.0f, y + 74, 18, 8};
      DrawRectangleRec(pip, i < drone->health ? kAmber : alpha(kCyanDim, 0.35f));
    }
  }
}

// Greyout, blackout tunnel and redout, drawn over everything (the HUD is in
// the pilot's eyes too).
void drawGEffects(const Player &player) {
  static Texture2D vignette{};
  if (vignette.id == 0) {
    Image image = GenImageGradientRadial(512, 512, 0.25f, Color{0, 0, 0, 0}, Color{0, 0, 0, 255});
    vignette = LoadTextureFromImage(image);
    UnloadImage(image);
  }
  const GLoad &g = player.load;
  const float w = kScreenWidth, h = kScreenHeight;
  if (g.greyout > 0.0f) {
    // Colour drains first: a flat grey wash, slightly dark.
    DrawRectangle(0, 0, kScreenWidth, kScreenHeight, Color{70, 72, 74, static_cast<unsigned char>(150 * g.greyout)});
  }
  auto tunnel = [&](float closing, Color tint) {
    // The vignette starts well outside the screen and closes in.
    const float size = std::hypot(w, h) * (2.4f - 1.9f * closing);
    const Rectangle dest{(w - size) * 0.5f, (h - size) * 0.5f, size, size};
    DrawTexturePro(vignette, {0, 0, 512, 512}, dest, {0, 0}, 0.0f, tint);
    if (dest.x > 0.0f) { // fill outside the texture
      DrawRectangle(0, 0, static_cast<int>(dest.x) + 1, kScreenHeight, tint);
      DrawRectangle(static_cast<int>(dest.x + size) - 1, 0, static_cast<int>(dest.x) + 2, kScreenHeight, tint);
    }
    if (dest.y > 0.0f) {
      DrawRectangle(0, 0, kScreenWidth, static_cast<int>(dest.y) + 1, tint);
      DrawRectangle(0, static_cast<int>(dest.y + size) - 1, kScreenWidth, static_cast<int>(dest.y) + 2, tint);
    }
  };
  const float closing = std::max(0.35f * g.greyout, g.blackout);
  if (closing > 0.0f) tunnel(closing, Color{0, 0, 0, 255});
  if (g.blackout > 0.75f) {
    DrawRectangle(0, 0, kScreenWidth, kScreenHeight,
                  Color{0, 0, 0, static_cast<unsigned char>(255 * smoothstep(0.75f, 1.0f, g.blackout))});
  }
  if (g.redout > 0.0f) {
    DrawRectangle(0, 0, kScreenWidth, kScreenHeight, Color{150, 0, 0, static_cast<unsigned char>(130 * g.redout)});
    tunnel(0.6f * g.redout, Color{90, 0, 0, 255});
  }
}

void drawCrash(const Player &player) {
  if (player.crashTimer <= 0.0f) return;
  DrawRectangle(0, 0, kScreenWidth, kScreenHeight, alpha(kRed, 0.12f * std::min(1.0f, player.crashTimer)));
  textCentered("HULL BREACH", kScreenWidth * 0.5f, kScreenHeight * 0.3f, 56, kRed, 6.0f);
  textCentered(TextFormat("IMPACT %.0f m/s  //  REBOOTING SYSTEMS", player.crashImpact), kScreenWidth * 0.5f,
               kScreenHeight * 0.3f + 66, 20, kWhite, 2.0f);
}

} // namespace

void drawHud(const Player &player, const Combat &combat, const Camera3D &camera) {
  const float time = static_cast<float>(GetTime());
  const Attitude2D att = cameraAttitude(camera);

  drawHorizon(att, camera.fovy);
  drawFlightPathMarker(player, camera);
  drawCompass(att);
  drawSpeedPanel(player);
  drawAltitudePanel(player);
  drawStatus(player, time);
  drawEngineGauge(player, time);
  drawShipDiagram(player, time);
  drawGMeter(player);
  drawCrash(player);
  drawDroneMarkers(combat, camera);
  drawCrosshair(combat);
  drawWeaponsPanel(combat);

  constexpr const char *kHoverHints[][2] = {
      {"MOUSE", "look"},
      {"W A S D", "move (let go: stop)"},
      {"SPACE / SHIFT", "climb / descend"},
      {"F", "arm conversion to jet"},
      {"LEFT CLICK", "fire"},
      {"", ""},
  };
  constexpr const char *kJetHints[][2] = {
      {"MOUSE", "stick: X turns, Y pitches"},
      {"", "springs back: keep moving to keep turning"},
      {"W / S", "throttle (S at idle: airbrake)"},
      {"A / D", "roll"},
      {"F", "arm conversion to hover"},
      {"LEFT CLICK", "fire"},
  };
  const auto &hints = isJetMode(player.mode) ? kJetHints : kHoverHints;
  for (int pass = 0; pass < 2; ++pass) {
    const float o = pass == 0 ? 1.0f : 0.0f;
    const Color c = pass == 0 ? Color{0, 10, 16, 200} : alpha(kWhite, 0.75f);
    for (int i = 0; i < 6; ++i) {
      text(hints[i][0], 36 + o, 20 + 15 * i + o, 12, c, 1.0f);
      text(hints[i][1], 150 + o, 20 + 15 * i + o, 12, c, 1.0f);
    }
  }
  text(TextFormat("%d FPS", GetFPS()), kScreenWidth - 90, 20, 12, kCyanDim, 1.0f);
  drawGEffects(player);
}
