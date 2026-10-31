#pragma once

#include "core/constants.hpp"
#include "player/aero.hpp"
#include "player/airframe.hpp"
#include "raylib.h"
#include "raymath.h"

enum class FlightMode {
  Landed, // resting on a surface
  Hover,  // airborne on the lift jets
  Flight, // flying on the wing
};

// Which control law is flying the airframe; F toggles it.
enum class ControlLaw { Hover, Flight };

struct Player {
  Vector3 position{0.0f, kEyeHeight, 6.0f}; // eye position
  Vector3 velocity{0.0f, 0.0f, 0.0f};
  FlightMode mode = FlightMode::Landed;

  // Pilot look: yaw is the commanded heading the craft turns toward, pitch is
  // the head tilt relative to the airframe.
  float yaw = -90.0f;
  float pitch = 0.0f;

  // Rigid body: orientation, angular velocity (world frame, rad/s) and the
  // spooled throttle (0..1) of every jet.
  Attitude body{}; // faces north, level
  Vector3 angularVelocity{};
  JetCommand jetThrottle{};

  ControlLaw law = ControlLaw::Hover;
  // Flight law: the pilot's aim point (a world direction the mouse moves around
  // the cockpit), and the nose trim that makes the flight path, not just the
  // nose, end up on it.
  Vector3 aimDir{0.0f, 0.0f, -1.0f};
  float pitchTrim = 0.0f; // rad

  // Hover: mean jet throttle mapped onto the rpm scale. Flight: the spooled
  // engine rpm, which sets the main-jet thrust (idle, then turbo above 6000).
  float engineRpm = 0.0f;
  float engineIdleTime = 0.0f; // s without input; the engine shuts off after kIdleTimeout
  float crashTimer = 0.0f;
  float crashImpact = 0.0f;
};

// Everything the physics step needs from the outside world; filled from the
// keyboard and mouse in the game, or scripted in the flight simulator.
struct PlayerInput {
  bool forward = false;
  bool back = false;
  bool left = false;
  bool right = false;
  bool ascend = false;
  bool descend = false;
  bool toggleFlight = false; // edge, consumed by the next physics step
  Vector2 mouseDelta{};      // accumulated since the last physics step (flight stick)
};

void applyMouseLook(Player &player, Vector2 mouseDelta);
void stepPlayer(Player &player, const PlayerInput &input, float dt);
float playerAltitudeAboveGround(const Player &player);
