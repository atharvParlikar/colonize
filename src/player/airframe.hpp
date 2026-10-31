#pragma once

#include "core/constants.hpp"
#include "raylib.h"

struct Player;

// Body frame: x right, y up, z forward, origin at the centre of mass.
//
// The craft has one engine with a vectoring nozzle in the body's up/forward
// plane: at 0 the thrust pushes straight along body up (hover), at 90 deg
// straight along body forward (flight), and a little below 0 it tips forward
// to brake. It cannot push sideways, so turning or strafing means banking.
//
// What the pilot law asks of the airframe each step.
struct AirframeCommand {
  Vector3 rate{};      // wanted angular velocity, world frame, rad/s (the fly-by-wire rate loop)
  float thrust = 0.0f; // N, wanted engine thrust (spools toward it)
  float nozzle = 0.0f; // rad, nozzle angle this step (the pilot law slews it)
  float airbrake = 0.0f; // 0..1, extra drag against the flight path
};

// Spools the engine, applies thrust, wing lift, hull drag and gravity, and
// integrates velocity and orientation. Position is the caller's job so
// collisions can run in between.
void stepAirframe(Player &player, const AirframeCommand &command, float dt);

// Unit thrust direction (world) for a nozzle angle.
Vector3 nozzleDirection(const Player &player, float nozzle);

// Angle of attack (rad, nose above the flow) and sideslip (rad, flow from the
// right is positive) of the airframe against its own velocity.
struct FlowAngles {
  float airspeed = 0.0f;
  float alpha = 0.0f;
  float beta = 0.0f;
};
FlowAngles flowAngles(const Player &player);

// Dynamic pressure (Pa) at the craft's altitude.
float dynamicPressure(const Player &player);

// Force the hull drag puts on the craft (N).
Vector3 hullDrag(const Player &player);

// Everything aerodynamic on the craft (hull drag plus wing lift and drag), in N.
Vector3 aeroForce(const Player &player);
