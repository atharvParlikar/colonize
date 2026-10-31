#include "player/airframe.hpp"

#include "player/aero.hpp"
#include "player/player.hpp"

#include <algorithm>
#include <cmath>

Vector3 bodyToWorld(const Player &player, Vector3 v) {
  const Attitude &b = player.body;
  return Vector3Add(Vector3Add(Vector3Scale(b.right, v.x), Vector3Scale(b.up, v.y)),
                    Vector3Scale(b.forward, v.z));
}

Vector3 hullDrag(const Player &player) {
  // Streamlined along the nose, a flat plate across the belly and flanks.
  const Attitude &b = player.body;
  const float rho = airDensity(player.position.y);
  auto axis = [&](Vector3 dir, float cda) {
    const float v = Vector3DotProduct(player.velocity, dir);
    return Vector3Scale(dir, -0.5f * rho * cda * std::fabs(v) * v);
  };
  return Vector3Add(Vector3Add(axis(b.forward, kHullDragForward), axis(b.right, kHullDragSide)),
                    axis(b.up, kHullDragUp));
}

Vector3 aeroForce(const Player &player) {
  return Vector3Add(hullDrag(player),
                    computeWingsuitAero(player.velocity, player.body, player.position.y).force);
}

float dynamicPressure(const Player &player) {
  const float speed = Vector3Length(player.velocity);
  return 0.5f * airDensity(player.position.y) * speed * speed;
}

FlowAngles flowAngles(const Player &player) {
  FlowAngles out;
  const float speed = Vector3Length(player.velocity);
  out.airspeed = speed;
  if (speed < 0.5f) return out;
  const Attitude &b = player.body;
  out.alpha = std::atan2(-Vector3DotProduct(player.velocity, b.up),
                         Vector3DotProduct(player.velocity, b.forward));
  out.beta = std::asin(std::clamp(Vector3DotProduct(player.velocity, b.right) / speed, -1.0f, 1.0f));
  return out;
}

JetCommand mixLiftJets(float collective, BodyTorque torque) {
  // Torque about forward is -sum(x F), about right sum(z F), about up sum(k F).
  // The four jets are symmetric, so each axis solves independently.
  const Jet &front = kJets[LiftFront];
  const Jet &right = kJets[LiftRight];
  const float cPitch = torque.pitch / (2.0f * front.position.z * front.position.z);
  const float cRoll = torque.roll / (2.0f * right.position.x * right.position.x);
  const float k = front.yawReaction;
  const float cYaw = torque.yaw / (4.0f * k * k);

  JetCommand command{};
  for (int i = LiftFront; i <= LiftRight; ++i) {
    const Jet &jet = kJets[i];
    const float force = 0.25f * collective + cPitch * jet.position.z - cRoll * jet.position.x +
                        cYaw * jet.yawReaction;
    command[i] = std::clamp(force / jet.maxThrust, 0.0f, 1.0f);
  }
  return command;
}

void stepAirframe(Player &player, const Controls &controls, float dt) {
  Vector3 force{};
  Vector3 torque{};
  for (int i = 0; i < kJetCount; ++i) {
    const Jet &jet = kJets[i];
    float &throttle = player.jetThrottle[i];
    throttle += (controls.jets[i] - throttle) * std::min(1.0f, dt / kJetSpoolTau);

    const float thrust = throttle * jet.maxThrust;
    const Vector3 f = Vector3Scale(bodyToWorld(player, jet.thrustDir), thrust);
    force = Vector3Add(force, f);
    torque = Vector3Add(torque, Vector3CrossProduct(bodyToWorld(player, jet.position), f));
    torque = Vector3Add(torque, Vector3Scale(player.body.up, jet.yawReaction * thrust));
  }
  force = Vector3Add(force, hullDrag(player));

  // Wing: lift and drag through the centre of mass, plus the moments that make
  // the airframe fly nose-first (stiffness) and settle (rate damping).
  const Attitude &b = player.body;
  const AeroResult aero = computeWingsuitAero(player.velocity, b, player.position.y);
  force = Vector3Add(force, aero.force);
  const float q = dynamicPressure(player);
  const float v = std::max(aero.airspeed, 10.0f);
  const float pitchRate = Vector3DotProduct(player.angularVelocity, b.right);
  const float yawRate = Vector3DotProduct(player.angularVelocity, b.up);
  const float rollRate = Vector3DotProduct(player.angularVelocity, b.forward);
  const float pitchMoment = q * kWingArea * kWingChord *
                            (-kCmAlpha * aero.alpha - kCmq * kWingChord / (2.0f * v) * pitchRate);
  const float yawMoment = q * kWingArea * kWingSpan *
                          (-kCnBeta * aero.beta - kCnr * kWingSpan / (2.0f * v) * yawRate);
  const float rollMoment = q * kWingArea * kWingSpan * (-kClp * kWingSpan / (2.0f * v) * rollRate);
  torque = Vector3Add(torque, Vector3Scale(b.right, pitchMoment + controls.surfaces.pitch));
  torque = Vector3Add(torque, Vector3Scale(b.up, yawMoment + controls.surfaces.yaw));
  torque = Vector3Add(torque, Vector3Scale(b.forward, rollMoment + controls.surfaces.roll));

  const Vector3 accel =
      Vector3Add(Vector3{0.0f, -kGravity, 0.0f}, Vector3Scale(force, 1.0f / kAirframeMass));
  player.velocity = Vector3Add(player.velocity, Vector3Scale(accel, dt));

  // Angular acceleration per body axis (diagonal inertia, gyroscopic terms
  // neglected), kept as a world vector.
  const Vector3 angAccel = Vector3Add(
      Vector3Add(Vector3Scale(b.forward, Vector3DotProduct(torque, b.forward) / kInertiaRoll),
                 Vector3Scale(b.right, Vector3DotProduct(torque, b.right) / kInertiaPitch)),
      Vector3Scale(b.up, Vector3DotProduct(torque, b.up) / kInertiaYaw));
  player.angularVelocity = Vector3Add(player.angularVelocity, Vector3Scale(angAccel, dt));
  player.angularVelocity =
      Vector3Scale(player.angularVelocity, std::exp(-kAngularDamping * dt));

  const float rate = Vector3Length(player.angularVelocity);
  if (rate > 1e-6f) {
    const Vector3 axis = Vector3Scale(player.angularVelocity, 1.0f / rate);
    const float angle = rate * dt;
    const Vector3 forward = Vector3RotateByAxisAngle(b.forward, axis, angle);
    const Vector3 up = Vector3RotateByAxisAngle(b.up, axis, angle);
    player.body = makeAttitude(forward, up); // re-orthonormalise
  }
}
