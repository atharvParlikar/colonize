#pragma once

#include "core/constants.hpp"
#include "engine/math/attitude.hpp"
#include "player/gload.hpp"
#include "raylib.h"
#include "raymath.h"

// Two controllers and a deliberate change between them:
//   Hover   - a game: velocity commands, free look (player/hover.cpp);
//   Jet     - a jet: spring-loaded stick, auto-bank, momentum (player/jet.cpp);
//   ToJet / ToHover - the conversion, tuning.convertTime long, F to arm it.
enum class FlightMode {
  Landed,  // resting on a surface, hover controller
  Hover,
  ToJet,
  Jet,
  ToHover,
};

struct Player {
  Vector3 position{0.0f, kEyeHeight, 6.0f}; // eye position
  Vector3 velocity{0.0f, 0.0f, 0.0f};
  FlightMode mode = FlightMode::Landed;
  engine::Attitude body{}; // faces north, level

  // Hover: where the pilot looks (rad).
  float lookHeading = 0.0f;
  float lookPitch = 0.0f;

  // Conversion: 0 hover .. 1 jet (the nozzle swing), and whether F asked for
  // the next one and it is waiting for the speed gate.
  float convert = 0.0f;
  bool convertArmed = false;

  // Jet.
  float throttle = 0.0f;     // 0..1
  bool airbrake = false;     // S held at idle
  Vector2 stick{};           // spring-loaded stick, x right, y nose up, |stick| <= 1
  float rollTrim = 0.0f;     // rad of bank set by hand (A / D) on top of the auto-bank

  GLoad load;

  // Gauge and sound: 0 is off.
  float engineRpm = 0.0f;
  float engineIdleTime = kIdleTimeout; // s at rest (starts cold)
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
  bool convert = false; // F pressed since the last physics step
  bool fire = false;    // cannon trigger held
  Vector2 mouseDelta{}; // accumulated since the last physics step
};

bool isJetMode(FlightMode mode);
// Where the pilot is aiming, which is where the view looks: the free look
// while hovering, the nose in the jet, blended through the conversion.
Vector3 aimDirection(Vector3 look, Vector3 bodyForward, float convert);
Vector3 aimDirection(const Player &player);
void stepPlayer(Player &player, const PlayerInput &input, float dt);
float playerAltitudeAboveGround(const Player &player);
