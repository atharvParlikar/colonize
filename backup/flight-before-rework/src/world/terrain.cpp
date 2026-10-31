#include "world/terrain.hpp"

#include "raymath.h"
#include "world/noise.hpp"

#include <algorithm>
#include <cmath>

float terrainHeight(const float x, const float z) {
  using namespace noise;

  const float edge = std::max(std::fabs(x), std::fabs(z));
  const float radius = std::sqrt(x * x + z * z);

  // Domain warp keeps features from looking grid-aligned.
  const float wx = x + 300.0f * gradientNoise(x * 0.0006f + 3.1f, z * 0.0006f + 7.7f).v;
  const float wz = z + 300.0f * gradientNoise(x * 0.0006f + 9.2f, z * 0.0006f + 1.4f).v;

  // Continent-scale mask: where mountain ranges and highlands live.
  const float continent = fbm01(wx * 0.00035f, wz * 0.00035f, 4);

  // Meadow hills, flattened into a clearing under the platform.
  const float hills = (fbm01(wx * 0.004f + 11.0f, wz * 0.004f + 7.0f, 4) - 0.5f) * 22.0f;
  const float detail = (fbm01(x * 0.03f, z * 0.03f, 3) - 0.5f) * 2.5f;
  const float meadow = smooth(20.0f, 50.0f, edge);
  const float base = -0.1f + (hills + detail + 0.1f) * meadow;

  // Rolling highlands around the mountains.
  const float foothills =
      smooth(0.38f, 0.55f, continent) * smooth(150.0f, 600.0f, radius) *
      fbm01(wx * 0.0022f + 40.0f, wz * 0.0022f + 2.0f, 5) * 140.0f;

  // Eroded mountain ranges.
  const float range = smooth(0.46f, 0.66f, continent) * smooth(200.0f, 900.0f, radius);
  const float eroded = erosion(wx * 0.0012f + 5.0f, wz * 0.0012f + 8.0f, 8);
  const float peaks = std::clamp(0.5f + 1.5f * eroded, 0.0f, 1.0f);
  const float mountains = range * (200.0f + 900.0f * std::pow(peaks, 1.6f));

  return base + foothills + mountains;
}

Vector3 terrainNormal(const float x, const float z) {
  constexpr float e = 1.0f;
  return Vector3Normalize({terrainHeight(x - e, z) - terrainHeight(x + e, z),
                           2.0f * e,
                           terrainHeight(x, z - e) - terrainHeight(x, z + e)});
}

float surfaceFriction(const float x, const float z) {
  using namespace noise;
  const float height = terrainHeight(x, z);
  const float slope = 1.0f - terrainNormal(x, z).y;

  // Mirrors the rock and snow bands used to colour the terrain.
  const float rock = std::max(smooth(0.12f, 0.3f, slope), smooth(450.0f, 750.0f, height) * 0.85f);
  const float snow = smooth(650.0f, 900.0f, height) * (1.0f - 0.6f * smooth(0.3f, 0.6f, slope));

  float friction = 0.6f + (0.85f - 0.6f) * rock;
  friction += (0.25f - friction) * snow;
  return friction;
}

float forestDensity(const float x, const float z) {
  using namespace noise;
  const float large = fbm01(x * 0.004f + 5.0f, z * 0.004f + 9.0f, 3);
  const float small = fbm01(x * 0.02f + 70.0f, z * 0.02f + 3.0f, 2);
  return smooth(0.46f, 0.6f, large + (small - 0.5f) * 0.12f);
}

void collectTrees(const float x0, const float z0, const float x1,
                  const float z1, std::vector<TreePlacement> &out) {
  using namespace noise;
  constexpr float cell = 7.0f;

  const int cx0 = static_cast<int>(std::floor(x0 / cell)) - 1;
  const int cx1 = static_cast<int>(std::floor(x1 / cell)) + 1;
  const int cz0 = static_cast<int>(std::floor(z0 / cell)) - 1;
  const int cz1 = static_cast<int>(std::floor(z1 / cell)) + 1;

  for (int cx = cx0; cx <= cx1; ++cx) {
    for (int cz = cz0; cz <= cz1; ++cz) {
      const float x = (cx + hash2(cx, cz, 1)) * cell;
      const float z = (cz + hash2(cx, cz, 2)) * cell;
      if (x < x0 || x >= x1 || z < z0 || z >= z1) {
        continue;
      }
      if (std::max(std::fabs(x), std::fabs(z)) < 26.0f) {
        continue;
      }

      const float chance = forestDensity(x, z) * 0.85f + 0.02f;
      if (hash2(cx, cz, 3) > chance) {
        continue;
      }

      const float height = terrainHeight(x, z);
      if (height > 260.0f || terrainNormal(x, z).y < 0.8f) {
        continue;
      }

      const float altitude =
          smooth(70.0f, 150.0f, height + (hash2(cx, cz, 4) - 0.5f) * 40.0f);
      TreePlacement tree{};
      tree.position = {x, height, z};
      tree.scale = 0.85f + hash2(cx, cz, 5) * 0.7f;
      tree.yaw = hash2(cx, cz, 6) * 360.0f;
      tree.variation = hash2(cx, cz, 7);
      tree.kind = altitude > 0.5f ? TreeKind::Conifer : TreeKind::Round;
      tree.seed = hashBits(cx, cz, 8);
      out.push_back(tree);
    }
  }
}
