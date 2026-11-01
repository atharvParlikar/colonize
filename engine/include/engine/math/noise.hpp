#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine::noise {

inline std::uint32_t hashBits(int x, int y, std::uint32_t seed = 0) {
  std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393u +
                    static_cast<std::uint32_t>(y) * 668265263u +
                    seed * 2246822519u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return h ^ (h >> 16);
}

inline float hash2(int x, int y, std::uint32_t seed = 0) {
  return static_cast<float>(hashBits(x, y, seed) & 0xFFFFFFu) / 16777215.0f;
}

inline float smooth(float a, float b, float x) {
  float t = (x - a) / (b - a);
  t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
  return t * t * (3.0f - 2.0f * t);
}

struct Sample {
  float v;  // value, roughly [-0.7, 0.7]
  float dx; // analytic partial derivatives
  float dz;
};

// Perlin-style gradient noise with analytic derivatives and a quintic fade.
inline Sample gradientNoise(float x, float z) {
  static constexpr float kDirs[16][2] = {
      {1.0f, 0.0f},         {0.92388f, 0.38268f},   {0.70711f, 0.70711f},
      {0.38268f, 0.92388f}, {0.0f, 1.0f},           {-0.38268f, 0.92388f},
      {-0.70711f, 0.70711f}, {-0.92388f, 0.38268f}, {-1.0f, 0.0f},
      {-0.92388f, -0.38268f}, {-0.70711f, -0.70711f}, {-0.38268f, -0.92388f},
      {0.0f, -1.0f},        {0.38268f, -0.92388f},  {0.70711f, -0.70711f},
      {0.92388f, -0.38268f}};

  const float fx = std::floor(x);
  const float fz = std::floor(z);
  const int ix = static_cast<int>(fx);
  const int iz = static_cast<int>(fz);
  const float px = x - fx;
  const float pz = z - fz;

  const float ux = px * px * px * (px * (px * 6.0f - 15.0f) + 10.0f);
  const float uz = pz * pz * pz * (pz * (pz * 6.0f - 15.0f) + 10.0f);
  const float dux = 30.0f * px * px * (px * (px - 2.0f) + 1.0f);
  const float duz = 30.0f * pz * pz * (pz * (pz - 2.0f) + 1.0f);

  auto dir = [&](int i, int j) { return kDirs[(hashBits(ix + i, iz + j) >> 8) & 15]; };
  const float *ga = dir(0, 0);
  const float *gb = dir(1, 0);
  const float *gc = dir(0, 1);
  const float *gd = dir(1, 1);

  const float va = ga[0] * px + ga[1] * pz;
  const float vb = gb[0] * (px - 1.0f) + gb[1] * pz;
  const float vc = gc[0] * px + gc[1] * (pz - 1.0f);
  const float vd = gd[0] * (px - 1.0f) + gd[1] * (pz - 1.0f);
  const float k = va - vb - vc + vd;

  Sample s;
  s.v = va + ux * (vb - va) + uz * (vc - va) + ux * uz * k;
  s.dx = ga[0] + ux * (gb[0] - ga[0]) + uz * (gc[0] - ga[0]) +
         ux * uz * (ga[0] - gb[0] - gc[0] + gd[0]) + dux * (uz * k + vb - va);
  s.dz = ga[1] + ux * (gb[1] - ga[1]) + uz * (gc[1] - ga[1]) +
         uz * ux * (ga[1] - gb[1] - gc[1] + gd[1]) + duz * (ux * k + vc - va);
  return s;
}

// Fractal sum of gradient noise remapped to roughly [0, 1]. Each octave is
// rotated to hide the lattice.
inline float fbm01(float x, float z, int octaves) {
  float sum = 0.0f;
  float amp = 0.5f;
  float norm = 0.0f;
  for (int i = 0; i < octaves; ++i) {
    sum += amp * gradientNoise(x, z).v;
    norm += amp;
    const float nx = 1.6f * x - 1.2f * z;
    z = 1.2f * x + 1.6f * z;
    x = nx;
    amp *= 0.5f;
  }
  return std::clamp(0.5f + 0.9f * sum / norm, 0.0f, 1.0f);
}

// Erosion-flavoured fBm: octaves are damped by the accumulated slope, so
// steep faces stay smooth while flats and ridge tops keep fine detail.
// Returns a signed value centred near 0.
inline float erosion(float x, float z, int octaves) {
  float sum = 0.0f;
  float amp = 1.0f;
  float norm = 0.0f;
  float dx = 0.0f;
  float dz = 0.0f;
  for (int i = 0; i < octaves; ++i) {
    const Sample n = gradientNoise(x, z);
    dx += n.dx;
    dz += n.dz;
    sum += amp * n.v / (1.0f + dx * dx + dz * dz);
    norm += amp;
    const float nx = 1.6f * x - 1.2f * z;
    z = 1.2f * x + 1.6f * z;
    x = nx;
    amp *= 0.5f;
  }
  return sum / norm;
}

} // namespace engine::noise
