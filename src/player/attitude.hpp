#pragma once

#include "raylib.h"
#include "raymath.h"

// Orthonormal body frame: forward is the nose, up the canopy, right = forward x up.
struct Attitude {
  Vector3 forward{0.0f, 0.0f, -1.0f};
  Vector3 up{0.0f, 1.0f, 0.0f};
  Vector3 right{1.0f, 0.0f, 0.0f};
};

Attitude makeAttitude(Vector3 forward, Vector3 upHint);
// Turns the frame by a world-space angular velocity (rad/s) for dt.
Attitude rotateAttitude(const Attitude &a, Vector3 omega, float dt);
// Eases the frame toward target by the fraction k (0..1).
Attitude blendAttitude(const Attitude &from, const Attitude &to, float k);

// Angles against the horizon, in radians.
float bankOf(const Attitude &a);   // right wing down is positive
float pitchOf(const Attitude &a);  // nose up is positive
float headingOf(Vector3 dir);      // 0 = north (-Z), east is positive
Vector3 directionFrom(float heading, float pitch);

float smoothstep(float a, float b, float x);
float wrapAngle(float a); // to -pi..pi
