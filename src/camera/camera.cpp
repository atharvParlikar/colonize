#include "camera/camera.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kBaseFov = 75.0f;
constexpr float kMaxFovBoost = 14.0f;
constexpr float kHoverViewTilt = 0.35f; // share of the body's tilt the hover view leans with

float smoothstep(float a, float b, float x) {
  const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

} // namespace

View computeView(const Player &previous, const Player &current, float alpha) {
  View view{};
  view.position = Vector3Lerp(previous.position, current.position, alpha);
  view.fovy = kBaseFov;

  // Hovering, the pilot looks along the aim point (free look; the craft turns
  // to follow) with the horizon mostly level. Flying, the cockpit is bolted to
  // the airframe and the aim point is a circle the nose chases.
  const Vector3 bodyForward = Vector3Normalize(Vector3Lerp(previous.body.forward, current.body.forward, alpha));
  const Vector3 bodyUp = Vector3Normalize(Vector3Lerp(previous.body.up, current.body.up, alpha));
  const Vector3 hoverUp = Vector3Normalize(Vector3Lerp(kWorldUp, bodyUp, kHoverViewTilt));
  const float flight = current.flight;
  const Vector3 forward = Vector3Normalize(Vector3Lerp(current.aimDir, bodyForward, flight));
  const Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, Vector3Lerp(hoverUp, bodyUp, flight)));
  view.forward = forward;
  view.up = Vector3CrossProduct(right, forward);

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
