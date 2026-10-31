#pragma once

#include "raylib.h"

// Orthonormal body frame of a prone flyer. forward points at the head, up is
// the back (the wing's lift side), right = forward x up.
struct Attitude {
  Vector3 forward{0.0f, 0.0f, -1.0f};
  Vector3 up{0.0f, 1.0f, 0.0f};
  Vector3 right{1.0f, 0.0f, 0.0f};
};

Attitude makeAttitude(Vector3 forward, Vector3 upHint);
// Body frame for a flyer moving along velocity with the given angle of attack
// (nose above the flight path) and bank (positive = right wing low), no
// sideslip. Falls back to previous when velocity is too small or vertical.
Attitude windAttitude(Vector3 velocity, float alpha, float bank, const Attitude &previous);

struct AeroResult {
  Vector3 force{};  // N, world space
  float airspeed = 0.0f;
  float alpha = 0.0f; // rad, angle of attack (positive = flow hits the belly)
  float beta = 0.0f;  // rad, sideslip
  float cl = 0.0f;
  float cd = 0.0f;
  bool stalled = false;
};

float airDensity(float altitude);
void wingCoefficients(float alpha, float &cl, float &cd);
// Lift, drag and side force on an extended wingsuit moving through still air.
AeroResult computeWingsuitAero(Vector3 velocity, const Attitude &attitude, float altitude);
// Quadratic drag on an unfolded, free-falling body.
Vector3 computeBluffBodyDrag(Vector3 velocity, float altitude);
