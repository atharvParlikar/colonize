#include "render/combat.hpp"

#include "render/scenery.hpp"
#include "raymath.h"

#include <algorithm>

namespace {

constexpr Color kDroneBody{58, 62, 70, 255};
constexpr Color kDroneRing{232, 120, 40, 255};
constexpr Color kDroneEye{255, 60, 50, 255};
constexpr Color kFlash{255, 255, 255, 255};
constexpr Color kTracer{255, 214, 120, 255};
constexpr Color kSpark{255, 150, 60, 255};
constexpr float kTracerLength = 45.0f; // m drawn behind each round
// Tracers keep at least this angular width (rad), or far ones vanish into a
// sub-pixel line that the painterly post-filter wipes out.
constexpr float kTracerMinAngle = 0.0016f;
// Nothing is drawn in the first metres after the muzzle, which sits half a
// metre from the eye and would fill the screen.
constexpr float kTracerStart = 12.0f; // m

void drawDrones(const Combat &combat) {
  BeginShaderMode(worldShader());
  for (auto [handle, drone] : combat.drones) {
    const bool flash = drone.hitFlash > 0.0f;
    const Vector3 p = drone.position;
    DrawSphereEx(p, kDroneRadius * 0.75f, 10, 14, flash ? kFlash : kDroneBody);
    // Rotor ring and a red eye.
    DrawCylinder({p.x, p.y + 0.15f, p.z}, kDroneRadius, kDroneRadius, 0.18f, 18, flash ? kFlash : kDroneRing);
    DrawSphereEx({p.x, p.y - 0.4f, p.z}, 0.35f, 6, 8, kDroneEye);
  }
  EndShaderMode();
}

void drawRounds(const Combat &combat, const Camera3D &camera, float alpha) {
  for (auto [handle, round] : combat.rounds) {
    const Vector3 head = Vector3Lerp(round.previous, round.position, alpha);
    const float speed = Vector3Length(round.velocity);
    if (speed < 1e-3f) continue;
    // The tail never reaches back past kTracerStart from the muzzle.
    const float flown = round.age * speed - Vector3Distance(head, round.position);
    const float length = std::clamp(flown - kTracerStart, 0.0f, kTracerLength);
    if (length <= 0.0f) continue;
    const Vector3 tail = Vector3Subtract(head, Vector3Scale(round.velocity, length / speed));
    const auto width = [&](Vector3 p) {
      return std::max(0.05f, kTracerMinAngle * Vector3Distance(p, camera.position));
    };
    DrawCylinderEx(tail, head, width(tail) * 0.5f, width(head), 5, kTracer);
  }
}

void drawSparks(const Combat &combat, float alpha) {
  for (auto [handle, spark] : combat.sparks) {
    const Vector3 p = Vector3Lerp(spark.previous, spark.position, alpha);
    const float life = 1.0f - spark.age / kSparkLifetime;
    DrawCube(p, 0.35f * life + 0.05f, 0.35f * life + 0.05f, 0.35f * life + 0.05f, kSpark);
  }
}

} // namespace

void drawCombat(const Combat &combat, const Camera3D &camera, float alpha) {
  drawDrones(combat);
  drawRounds(combat, camera, alpha);
  drawSparks(combat, alpha);
}
