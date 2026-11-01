#pragma once

#include "engine/entity/pool.hpp"
#include "player/player.hpp"
#include "raylib.h"

#include <vector>

// The cannon and its targets. Every kind of thing is a plain struct in its own
// engine::Pool; things refer to each other through engine::Handle, which goes
// stale (get() returns null) when the thing it names is destroyed.

struct Round {
  Vector3 position;
  Vector3 previous; // position one step ago, for drawing between steps
  Vector3 velocity;
  float age = 0.0f;
};

struct Drone {
  Vector3 home;     // it bobs around this point
  Vector3 position;
  float phase = 0.0f; // its own bob timing
  int health = kDroneHealth;
  float hitFlash = 0.0f; // s left of the white flash after a hit
};

// Debris thrown out when a drone breaks.
struct Spark {
  Vector3 position;
  Vector3 previous;
  Vector3 velocity;
  float age = 0.0f;
};

struct Combat {
  engine::Pool<Round> rounds;
  engine::Pool<Drone> drones;
  engine::Pool<Spark> sparks;

  struct Respawn {
    Vector3 home;
    float timer;
  };
  std::vector<Respawn> respawns; // broken drones waiting to come back

  engine::Handle<Drone> lastHit; // the HUD shows its health until it breaks
  float cooldown = 0.0f;         // s until the cannon can fire again
  float hitMarker = 0.0f;        // s left to show the hit marker
  int kills = 0;
  float time = 0.0f;             // s of combat simulated, drives the bobbing
};

// Fills the field around spawn with drones.
void spawnDroneField(Combat &combat);
Drone makeDrone(Vector3 home, float phase);
// One fixed step: fire if the trigger is held, move everything, resolve hits.
void stepCombat(Combat &combat, const Player &player, const PlayerInput &input, float dt);
