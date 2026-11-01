#include "camera/camera.hpp"

#include <cmath>

namespace {

constexpr float kBaseFov = 75.0f;
constexpr float kMaxFovBoost = 14.0f;
constexpr float kGFovSqueeze = 6.0f;     // deg the view narrows under load
constexpr float kHoverViewTilt = 0.35f;  // share of the body's lean the hover view leans with
constexpr float kShakeAngleDeg = 0.9f;   // at full shake
constexpr float kShakeOffset = 0.04f;    // m

} // namespace

View computeView(const Player &previous, const Player &current, float alpha) {
  View view{};
  view.position = Vector3Lerp(previous.position, current.position, alpha);

  // Hovering, the pilot looks freely with the horizon mostly level. In the jet
  // the cockpit is bolted to the airframe. The conversion blends the two.
  const Vector3 bodyForward = Vector3Normalize(Vector3Lerp(previous.body.forward, current.body.forward, alpha));
  const Vector3 bodyUp = Vector3Normalize(Vector3Lerp(previous.body.up, current.body.up, alpha));
  const float heading = previous.lookHeading + wrapAngle(current.lookHeading - previous.lookHeading) * alpha;
  const float pitch = previous.lookPitch + (current.lookPitch - previous.lookPitch) * alpha;
  const Vector3 look = directionFrom(heading, pitch);
  const Vector3 hoverUp = Vector3Normalize(Vector3Lerp(kWorldUp, bodyUp, kHoverViewTilt));
  const float jet = smoothstep(0.0f, 1.0f, current.convert);
  Vector3 forward = Vector3Normalize(Vector3Lerp(look, bodyForward, jet));
  Vector3 up = Vector3Lerp(hoverUp, bodyUp, jet);
  Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, up));
  up = Vector3CrossProduct(right, forward);

  // Load shake: a few incommensurate wobbles, stronger the harder you pull.
  const float shake = current.load.shake;
  if (shake > 0.001f) {
    const float t = static_cast<float>(GetTime());
    const float yaw = (std::sin(t * 23.0f) + 0.6f * std::sin(t * 37.0f + 1.3f)) * 0.6f;
    const float nod = (std::sin(t * 29.0f + 0.7f) + 0.6f * std::sin(t * 17.0f + 2.1f)) * 0.6f;
    const float amount = shake * shake * kShakeAngleDeg * DEG2RAD;
    forward = Vector3Normalize(Vector3Add(forward, Vector3Add(Vector3Scale(right, yaw * amount),
                                                              Vector3Scale(up, nod * amount))));
    right = Vector3Normalize(Vector3CrossProduct(forward, up));
    up = Vector3CrossProduct(right, forward);
    view.position = Vector3Add(view.position, Vector3Add(Vector3Scale(right, yaw * shake * kShakeOffset),
                                                         Vector3Scale(up, nod * shake * kShakeOffset)));
  }
  view.forward = forward;
  view.up = up;

  view.fovy = kBaseFov + kMaxFovBoost * smoothstep(5.0f, 40.0f, Vector3Length(current.velocity)) -
              kGFovSqueeze * smoothstep(3.0f, 9.0f, current.load.g);
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
