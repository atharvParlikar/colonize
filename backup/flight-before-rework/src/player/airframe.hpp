#pragma once

#include "core/constants.hpp"
#include "raylib.h"

#include <array>

struct Player;

// Body frame: x right, y up, z forward, origin at the centre of mass.
enum JetId { LiftFront, LiftRear, LiftLeft, LiftRight, MainLeft, MainRight, kJetCount };

struct Jet {
  Vector3 position;   // m
  Vector3 thrustDir;  // direction the thrust pushes the craft
  float maxThrust;    // N
  float yawReaction;  // N m of yaw torque per N of thrust (canted nozzle)
};

// Four lift jets point their thrust straight up (exhaust down) and give the
// hover its attitude control by differential throttle, like a quadcopter. Their
// reaction signs alternate fore-aft versus left-right so yaw can be mixed too.
// The two main jets sit out on the wings and push forward.
inline constexpr std::array<Jet, kJetCount> kJets{{
    {{0.0f, 0.0f, kLiftJetArmZ}, {0.0f, 1.0f, 0.0f}, kLiftJetMaxThrust, kYawReaction},
    {{0.0f, 0.0f, -kLiftJetArmZ}, {0.0f, 1.0f, 0.0f}, kLiftJetMaxThrust, kYawReaction},
    {{-kLiftJetArmX, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, kLiftJetMaxThrust, -kYawReaction},
    {{kLiftJetArmX, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, kLiftJetMaxThrust, -kYawReaction},
    {{-kMainJetArmX, 0.0f, -0.5f}, {0.0f, 0.0f, 1.0f}, kMainJetMaxThrust, 0.0f},
    {{kMainJetArmX, 0.0f, -0.5f}, {0.0f, 0.0f, 1.0f}, kMainJetMaxThrust, 0.0f},
}};

// Wanted torque about the body axes (N m): roll about forward, pitch about the
// right axis (positive = nose up), yaw about up.
struct BodyTorque {
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
};

using JetCommand = std::array<float, kJetCount>; // 0..1 of each jet's max thrust

// Splits a total lift (N along body up) and a torque over the four lift jets.
// The main jets are left at zero.
JetCommand mixLiftJets(float collective, BodyTorque torque);

// What a control law asks of the airframe: jet throttles plus the torque the
// aerodynamic control surfaces (elevator, ailerons, rudder) should produce.
struct Controls {
  JetCommand jets{};
  BodyTorque surfaces{};
};

// Spools the jets toward the command, applies thrust, wing lift, drag and
// gravity, and integrates velocity and orientation. Position is the caller's
// job so collisions can run in between.
void stepAirframe(Player &player, const Controls &controls, float dt);

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

Vector3 bodyToWorld(const Player &player, Vector3 v);

// Force the hull drag puts on the craft (N).
Vector3 hullDrag(const Player &player);

// Everything aerodynamic on the craft (hull drag plus wing lift and drag), in N.
Vector3 aeroForce(const Player &player);
