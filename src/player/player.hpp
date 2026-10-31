#pragma once

#include "core/constants.hpp"
#include "player/aero.hpp"
#include "raylib.h"
#include "raymath.h"

// What the craft is doing (HUD label); the control law itself blends
// continuously with airspeed, there is no mode switch.
enum class FlightMode {
  Landed, // resting on a surface
  Hover,  // jet-borne
  Flight, // wing-borne
};

struct Player {
  Vector3 position{0.0f, kEyeHeight, 6.0f}; // eye position
  Vector3 velocity{0.0f, 0.0f, 0.0f};
  FlightMode mode = FlightMode::Landed;

  // Rigid body: orientation and angular velocity (world frame, rad/s).
  Attitude body{}; // faces north, level
  Vector3 angularVelocity{};

  // Engine: spooled thrust (0..1 of kEngineMaxThrust) and nozzle angle (rad,
  // 0 = thrust along body up, pi/2 = along body forward).
  float throttle = 0.0f;
  float nozzle = 0.0f;
  float airbrake = 0.0f; // 0..1 deployed

  // Pilot: the aim point (a world direction the mouse moves; the craft goes
  // where it points), the speed command along it (W / S), and how wing-borne
  // the control law currently is (0 hover .. 1 flight).
  Vector3 aimDir{0.0f, 0.0f, -1.0f};
  float speedCommand = 0.0f; // m/s
  bool speedKeyHeld = false;  // W or S last step, so letting go can latch the speed
  float flight = 0.0f;

  // Gauge and sound: rpm follows the spooled thrust; 0 is off.
  float engineRpm = 0.0f;
  float engineIdleTime = kIdleTimeout; // s at rest (starts cold); the engine shuts off after kIdleTimeout
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
  Vector2 mouseDelta{}; // accumulated since the last physics step (moves the aim point)
};

void stepPlayer(Player &player, const PlayerInput &input, float dt);
float playerAltitudeAboveGround(const Player &player);
