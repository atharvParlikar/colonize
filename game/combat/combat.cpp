#include "combat/combat.hpp"

#include "engine/collision/queries.hpp"
#include "engine/math/noise.hpp"
#include "world/colliders.hpp"
#include "world/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <raymath.h>

using namespace engine;

namespace {

constexpr float kBobHeight = 1.5f; // m
constexpr float kBobRate = 0.8f;   // rad/s
constexpr int kSparksPerBreak = 14;
constexpr float kHitFlashTime = 0.12f;
constexpr float kHitMarkerTime = 0.15f;

Vector3 bobbed(const Drone &drone, float time) {
  const float t = time * kBobRate + drone.phase;
  return Vector3Add(drone.home,
                    {0.6f * std::sin(t * 0.7f), kBobHeight * std::sin(t),
                     0.6f * std::cos(t * 0.5f)});
}

// The gun sits under and right of the pilot and is aimed so its rounds cross
// the aim line at kGunConvergence.
void fireRound(Combat &combat, const Player &player) {
  const Vector3 aim = aimDirection(player);
  Vector3 right = Vector3CrossProduct(aim, kWorldUp);
  right = Vector3LengthSqr(right) > 1e-6f ? Vector3Normalize(right)
                                          : player.body.right;
  const Vector3 up = Vector3CrossProduct(right, aim);
  const Vector3 muzzle = Vector3Add(
      player.position,
      Vector3Add(Vector3Scale(up, -0.5f), Vector3Add(Vector3Scale(right, 0.4f),
                                                     Vector3Scale(aim, 1.5f))));
  const Vector3 target =
      Vector3Add(player.position, Vector3Scale(aim, kGunConvergence));
  const Vector3 dir = Vector3Normalize(Vector3Subtract(target, muzzle));
  const Vector3 velocity =
      Vector3Add(player.velocity, Vector3Scale(dir, kMuzzleSpeed));
  combat.rounds.create(Round{muzzle, muzzle, velocity});
}

void breakDrone(Combat &combat, Handle<Drone> handle) {
  const Drone drone =
      *combat.drones.get(handle); // copy: destroy clears the slot
  combat.drones.destroy(handle);  // lastHit, if it was this one, is stale now
  combat.respawns.push_back({drone.home, kDroneRespawnTime});
  ++combat.kills;

  for (int i = 0; i < kSparksPerBreak; ++i) {
    const auto h = [&](std::uint32_t salt) {
      return noise::hash2(combat.kills, i, salt) * 2.0f - 1.0f;
    };
    const Vector3 dir = Vector3Normalize({h(1), h(2) + 0.6f, h(3)});
    const Vector3 v = Vector3Scale(dir, 18.0f + 14.0f * (h(4) + 1.0f));
    combat.sparks.create(Spark{drone.position, drone.position, v});
  }
}

// Does the round, moving from p0 to p1 this step, hit anything? Drones take
// damage; terrain and level boxes just stop it.
bool resolveRound(Combat &combat, Vector3 p0, Vector3 p1) {
  Handle<Drone> nearestDrone{};
  float nearest = 2.0f; // t along the step; > 1 means nothing
  for (auto [handle, drone] : combat.drones) {
    if (const auto t = segmentSphere(p0, p1, drone.position, kDroneRadius);
        t && *t < nearest) {
      nearest = *t;
      nearestDrone = handle;
    }
  }
  bool solidFirst = false;
  for (const BoundingBox &box : getLevelColliders()) {
    if (const auto t = segmentAabb(p0, p1, box); t && *t < nearest) {
      nearest = *t;
      solidFirst = true;
    }
  }
  if (solidFirst)
    return true;
  if (Drone *drone = combat.drones.get(nearestDrone)) {
    drone->hitFlash = kHitFlashTime;
    combat.lastHit = nearestDrone;
    combat.hitMarker = kHitMarkerTime;
    if (--drone->health <= 0)
      breakDrone(combat, nearestDrone);
    return true;
  }
  return p1.y < terrainHeight(p1.x, p1.z);
}

} // namespace

Drone makeDrone(Vector3 home, float phase) {
  Drone drone{};
  drone.home = home;
  drone.position = home;
  drone.phase = phase;
  return drone;
}

void spawnDroneField(Combat &combat) {
  // Deterministic: the same field every run. Rings out from the platform,
  // floating 15..110 m above the ground.
  for (int i = 0; i < kDroneCount; ++i) {
    const float angle = noise::hash2(i, 0, 11) * 2.0f * PI;
    const float distance = 60.0f + 440.0f * noise::hash2(i, 0, 12);
    const float x = std::sin(angle) * distance;
    const float z = -std::cos(angle) * distance;
    const float y =
        terrainHeight(x, z) + 15.0f + 95.0f * noise::hash2(i, 0, 13);
    combat.drones.create(
        makeDrone({x, y, z}, noise::hash2(i, 0, 14) * 2.0f * PI));
  }
}

void stepCombat(Combat &combat, const Player &player, const PlayerInput &input,
                float dt) {
  combat.time += dt;
  combat.hitMarker = std::max(combat.hitMarker - dt, 0.0f);

  // Cannon: a steady rate while held; the first round fires on the press.
  combat.cooldown -= dt;
  if (input.fire) {
    while (combat.cooldown <= 0.0f) {
      fireRound(combat, player);
      combat.cooldown += 1.0f / kCannonRate;
    }
  } else {
    combat.cooldown = std::max(combat.cooldown, 0.0f);
  }

  for (auto [handle, drone] : combat.drones) {
    drone.position = bobbed(drone, combat.time);
    drone.hitFlash = std::max(drone.hitFlash - dt, 0.0f);
  }

  for (auto [handle, round] : combat.rounds) {
    // round.previous = round.position;
    // round.position = Vector3Add(round.position, Vector3Scale(round.velocity,
    // dt)); round.age += dt;

    round.previous = round.position;
    round.position =
        Vector3Add(round.position, Vector3Scale(round.velocity, dt));

    const Vector3 from = round.previous, to = round.position;
    const bool expired = round.age >= kRoundLifetime;
    // resolveRound may create sparks; `round` is not used after it.
    if (resolveRound(combat, from, to) || expired)
      combat.rounds.destroy(handle);
  }

  for (auto [handle, spark] : combat.sparks) {
    spark.previous = spark.position;
    spark.velocity.y -= kGravity * dt;
    spark.velocity = Vector3Scale(spark.velocity, 1.0f - 0.8f * dt); // air drag
    spark.position =
        Vector3Add(spark.position, Vector3Scale(spark.velocity, dt));
    spark.age += dt;
    if (spark.age >= kSparkLifetime)
      combat.sparks.destroy(handle);
  }

  for (Combat::Respawn &r : combat.respawns)
    r.timer -= dt;
  std::erase_if(combat.respawns, [&](const Combat::Respawn &r) {
    if (r.timer > 0.0f)
      return false;
    combat.drones.create(makeDrone(r.home, r.home.x * 0.01f));
    return true;
  });
}
