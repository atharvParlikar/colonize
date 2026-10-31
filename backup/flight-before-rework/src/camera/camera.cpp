#include "camera/camera.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kBaseFov = 75.0f;
constexpr float kMaxFovBoost = 14.0f;

float smoothstep(float a, float b, float x) {
  const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

} // namespace

View computeView(const Player &previous, const Player &current, float alpha) {
  View view{};
  view.position = Vector3Lerp(previous.position, current.position, alpha);
  view.fovy = kBaseFov;

  // The cockpit is bolted to the airframe, so the view tilts with it; the
  // pilot's head pitch is added on top.
  const Vector3 forward = Vector3Normalize(Vector3Lerp(previous.body.forward, current.body.forward, alpha));
  const Vector3 up = Vector3Normalize(Vector3Lerp(previous.body.up, current.body.up, alpha));
  const Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, up));
  const float head = DEG2RAD * current.pitch;
  view.forward = Vector3RotateByAxisAngle(forward, right, head);
  view.up = Vector3RotateByAxisAngle(up, right, head);

  view.fovy += kMaxFovBoost * smoothstep(5.0f, 40.0f, Vector3Length(current.velocity));
  return view;
}

Camera3D makeCamera(const View &view) {
  Camera3D camera{};
  camera.projection = CAMERA_PERSPECTIVE;
  syncCamera(camera, view);
  return camera;
}

void syncCamera(Camera3D &camera, const View &view) {
  camera.position = view.position;
  camera.target = Vector3Add(view.position, view.forward);
  camera.up = view.up;
  camera.fovy = view.fovy;
}
