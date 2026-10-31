#pragma once

#include <algorithm>

// Above this speed the scene gets a zoom blur, and grass is only drawn near the
// craft (the blur hides distant blades, and the skiff outruns the streaming).
constexpr float kBlurStartKmh = 250.0f;
constexpr float kBlurFullKmh = 400.0f;

// 0 below kBlurStartKmh, easing up to 1 at kBlurFullKmh.
inline float motionBlurAmount(const float speedMetersPerSecond) {
  const float kmh = speedMetersPerSecond * 3.6f;
  const float t = std::clamp((kmh - kBlurStartKmh) / (kBlurFullKmh - kBlurStartKmh), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}
