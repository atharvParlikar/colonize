#include "render/scenery.hpp"

#include "core/constants.hpp"
#include "raymath.h"
#include "render/shaders.hpp"
#include "rlgl.h"
#include "world/noise.hpp"
#include "world/terrain.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr Vector3 kSunDir = {0.6f, 0.45f, 0.55f};
constexpr Vector3 kFogColor = {0.74f, 0.83f, 0.93f};

// Terrain is a quadtree of fixed-resolution tiles: level 0 tiles are 64 m
// wide, every level doubles that. Tiles near the camera split into finer ones.
constexpr int kCells = 32;
constexpr float kBaseSize = 64.0f;
constexpr int kRootLevel = 6;
constexpr float kSplitFactor = 2.2f;
constexpr float kViewDistance = 7000.0f;

// Trees live on a separate 128 m grid with two detail tiers.
constexpr float kTreeChunk = 128.0f;
constexpr float kTreeDetailDistance = 320.0f;
constexpr float kTreeFarDistance = 1200.0f;

// Resident meshes are only dropped when this vertex budget is exceeded, and
// then least-recently-used first, so ground you just left stays cached.
constexpr std::size_t kVertexBudget = 14'000'000;
// A tile keeps its finer/coarser state a bit past the split distance so the
// boundary does not flicker while you move along it.
constexpr float kHysteresis = 1.3f;
constexpr int kMaxUploadsPerFrame = 8;

struct MeshData {
  std::vector<float> positions;
  std::vector<float> normals;
  std::vector<unsigned char> colors;
};

struct Chunk {
  Mesh mesh{};
  Vector3 center{};
  float radius = 0.0f;
};

struct Key {
  std::int8_t kind; // 0 terrain, 1 trees
  std::int8_t level;
  std::int32_t ix;
  std::int32_t iz;
  bool operator==(const Key &) const = default;
};

struct KeyHash {
  std::size_t operator()(const Key &k) const {
    std::uint64_t h = static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.ix)) * 0x9E3779B97F4A7C15ull;
    h ^= (static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.iz)) + 0x7F4A7C15ull) * 0xC2B2AE3D27D4EB4Full;
    h ^= static_cast<std::uint64_t>(k.kind * 31 + k.level + 7) * 0x165667B1ull;
    return static_cast<std::size_t>(h ^ (h >> 29));
  }
};

struct Node {
  Chunk chunk;
  std::uint64_t lastUsed = 0;
};

struct Job {
  Key key;
  float priority;
};

struct Result {
  Key key;
  MeshData data;
  Vector3 center;
  float radius;
};

struct Scenery {
  Shader world{};
  Shader sky{};
  Shader post{};
  Shader blur{};
  RenderTexture2D blurSrc{}; // post-filtered scene, input to the blur pass
  float blurAmount = 0.0f;
  int blurAmountLoc = -1;
  Material worldMaterial{};
  Material skyMaterial{};
  Mesh skyMesh{};
  RenderTexture2D target{};
  float time = 0.0f;
  int worldCamLoc = -1;
  int skyTimeLoc = -1;

  // Main-thread state.
  std::unordered_map<Key, Node, KeyHash> nodes;
  std::unordered_set<Key, KeyHash> requested;
  std::unordered_map<Key, float, KeyHash> wanted;
  std::vector<const Chunk *> drawList;
  std::unordered_set<Key, KeyHash> splitNow, splitPrev;
  std::unordered_set<Key, KeyHash> detailNow, detailPrev;
  std::size_t residentVerts = 0;
  std::uint64_t frame = 0;
  Vector3 cam{};
  Vector3 forward{};
  float aboveGround = 0.0f;

  // Shared with workers.
  std::mutex mutex;
  std::vector<Job> queue;
  std::vector<Result> done;
  std::vector<std::jthread> workers;
};

Scenery g;

Vector3 normalized(Vector3 v) { return Vector3Normalize(v); }

Color scaled(Color c, float s) {
  auto ch = [s](unsigned char v) {
    return static_cast<unsigned char>(std::clamp(v * s, 0.0f, 255.0f));
  };
  return {ch(c.r), ch(c.g), ch(c.b), c.a};
}

Color lerpColor(Color a, Color b, float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  auto mix = [t](unsigned char x, unsigned char y) {
    return static_cast<unsigned char>(x + (y - x) * t);
  };
  return {mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b), 255};
}

void pushOne(MeshData &m, Vector3 p, Vector3 n, Color c) {
  m.positions.insert(m.positions.end(), {p.x, p.y, p.z});
  m.normals.insert(m.normals.end(), {n.x, n.y, n.z});
  m.colors.insert(m.colors.end(), {c.r, c.g, c.b, 255});
}

// Emits a triangle, flipping winding so the face points along its vertex
// normals (raylib culls clockwise faces). doubleSided emits both windings.
void pushTri(MeshData &m, const Vector3 p[3], const Vector3 n[3],
             const Color c[3], bool doubleSided = false) {
  const Vector3 face = Vector3CrossProduct(Vector3Subtract(p[1], p[0]),
                                           Vector3Subtract(p[2], p[0]));
  const Vector3 avg = Vector3Add(Vector3Add(n[0], n[1]), n[2]);
  const bool flip = Vector3DotProduct(face, avg) < 0.0f;
  const int order[3] = {0, flip ? 2 : 1, flip ? 1 : 2};
  for (const int i : order) {
    pushOne(m, p[i], n[i], c[i]);
  }
  if (doubleSided) {
    for (int k = 2; k >= 0; --k) {
      pushOne(m, p[order[k]], n[order[k]], c[order[k]]);
    }
  }
}

Chunk uploadMesh(const MeshData &m, Vector3 center, float radius) {
  Mesh mesh{};
  const int vertexCount = static_cast<int>(m.positions.size() / 3);
  mesh.vertexCount = vertexCount;
  mesh.triangleCount = vertexCount / 3;
  mesh.vertices = static_cast<float *>(MemAlloc(m.positions.size() * sizeof(float)));
  mesh.normals = static_cast<float *>(MemAlloc(m.normals.size() * sizeof(float)));
  mesh.colors = static_cast<unsigned char *>(MemAlloc(m.colors.size()));
  std::memcpy(mesh.vertices, m.positions.data(), m.positions.size() * sizeof(float));
  std::memcpy(mesh.normals, m.normals.data(), m.normals.size() * sizeof(float));
  std::memcpy(mesh.colors, m.colors.data(), m.colors.size());
  UploadMesh(&mesh, false);
  // The GPU owns the data now; drop the CPU copies.
  MemFree(mesh.vertices);
  MemFree(mesh.normals);
  MemFree(mesh.colors);
  mesh.vertices = nullptr;
  mesh.normals = nullptr;
  mesh.colors = nullptr;
  return {mesh, center, radius};
}

void measure(const MeshData &m, Vector3 &center, float &radius) {
  Vector3 lo{1e30f, 1e30f, 1e30f};
  Vector3 hi{-1e30f, -1e30f, -1e30f};
  for (std::size_t i = 0; i + 2 < m.positions.size(); i += 3) {
    lo = {std::min(lo.x, m.positions[i]), std::min(lo.y, m.positions[i + 1]),
          std::min(lo.z, m.positions[i + 2])};
    hi = {std::max(hi.x, m.positions[i]), std::max(hi.y, m.positions[i + 1]),
          std::max(hi.z, m.positions[i + 2])};
  }
  center = Vector3Scale(Vector3Add(lo, hi), 0.5f);
  radius = Vector3Distance(lo, hi) * 0.5f;
}

// ---- terrain tiles ---------------------------------------------------------

Color terrainColor(float x, float y, float z, Vector3 n) {
  using namespace noise;
  const float slope = 1.0f - n.y;
  const float patch = fbm01(x * 0.02f, z * 0.02f, 3);

  Color grass = lerpColor({122, 170, 62, 255}, {76, 136, 58, 255}, patch);
  const float dry = smooth(0.58f, 0.75f, fbm01(x * 0.004f + 50.0f, z * 0.004f, 3));
  grass = lerpColor(grass, {178, 170, 84, 255}, dry * 0.5f);

  // Forest floors read darker from far away, where individual trees are gone.
  const float forest = forestDensity(x, z) * (1.0f - smooth(150.0f, 260.0f, y));
  grass = lerpColor(grass, {48, 98, 46, 255}, forest * 0.6f);

  const float alpine = smooth(150.0f, 350.0f, y + (patch - 0.5f) * 80.0f);
  Color col = lerpColor(grass, {120, 142, 98, 255}, alpine);

  const float rockAmount =
      std::max(smooth(0.12f, 0.3f, slope + (patch - 0.5f) * 0.1f),
               smooth(450.0f, 750.0f, y + (patch - 0.5f) * 120.0f) * 0.85f);
  col = lerpColor(col, {122, 108, 116, 255}, rockAmount);

  const float snow = smooth(650.0f, 900.0f, y + (patch - 0.5f) * 160.0f) *
                     (1.0f - 0.6f * smooth(0.3f, 0.6f, slope));
  return lerpColor(col, {240, 244, 250, 255}, snow);
}

float tileSize(int level) { return kBaseSize * static_cast<float>(1 << level); }

MeshData generateTerrain(const Key &key) {
  const float size = tileSize(key.level);
  const float cell = size / kCells;
  const float x0 = key.ix * size;
  const float z0 = key.iz * size;

  // Heights include a one-cell border so normals come from the same grid.
  constexpr int stride = kCells + 3;
  std::vector<float> h(stride * stride);
  for (int j = 0; j < stride; ++j) {
    for (int i = 0; i < stride; ++i) {
      h[j * stride + i] = terrainHeight(x0 + (i - 1) * cell, z0 + (j - 1) * cell);
    }
  }

  struct Vertex {
    Vector3 pos;
    Vector3 normal;
    Color color;
  };
  constexpr int side = kCells + 1;
  std::vector<Vertex> grid(side * side);
  for (int j = 0; j < side; ++j) {
    for (int i = 0; i < side; ++i) {
      const int gi = i + 1;
      const int gj = j + 1;
      const float x = x0 + i * cell;
      const float z = z0 + j * cell;
      const float y = h[gj * stride + gi];
      const Vector3 n = Vector3Normalize(
          {h[gj * stride + gi - 1] - h[gj * stride + gi + 1], 2.0f * cell,
           h[(gj - 1) * stride + gi] - h[(gj + 1) * stride + gi]});
      grid[j * side + i] = {{x, y, z}, n, terrainColor(x, y, z, n)};
    }
  }

  MeshData data;
  for (int j = 0; j < kCells; ++j) {
    for (int i = 0; i < kCells; ++i) {
      const Vertex &a = grid[j * side + i];
      const Vertex &b = grid[j * side + i + 1];
      const Vertex &c = grid[(j + 1) * side + i];
      const Vertex &d = grid[(j + 1) * side + i + 1];
      const Vector3 p1[3] = {a.pos, c.pos, b.pos};
      const Vector3 n1[3] = {a.normal, c.normal, b.normal};
      const Color c1[3] = {a.color, c.color, b.color};
      pushTri(data, p1, n1, c1);
      const Vector3 p2[3] = {b.pos, c.pos, d.pos};
      const Vector3 n2[3] = {b.normal, c.normal, d.normal};
      const Color c2[3] = {b.color, c.color, d.color};
      pushTri(data, p2, n2, c2);
    }
  }

  // Skirts hide cracks where neighbouring tiles use different resolutions.
  const float drop = cell * 0.8f + 1.0f;
  auto skirt = [&](const Vertex &a, const Vertex &b) {
    const Vector3 a2 = {a.pos.x, a.pos.y - drop, a.pos.z};
    const Vector3 b2 = {b.pos.x, b.pos.y - drop, b.pos.z};
    const Vector3 p1[3] = {a.pos, b.pos, a2};
    const Vector3 n1[3] = {a.normal, b.normal, a.normal};
    const Color c1[3] = {a.color, b.color, a.color};
    pushTri(data, p1, n1, c1, true);
    const Vector3 p2[3] = {b.pos, b2, a2};
    const Vector3 n2[3] = {b.normal, b.normal, a.normal};
    const Color c2[3] = {b.color, b.color, a.color};
    pushTri(data, p2, n2, c2, true);
  };
  for (int i = 0; i < kCells; ++i) {
    skirt(grid[i], grid[i + 1]);
    skirt(grid[kCells * side + i], grid[kCells * side + i + 1]);
    skirt(grid[i * side], grid[(i + 1) * side]);
    skirt(grid[i * side + kCells], grid[(i + 1) * side + kCells]);
  }
  return data;
}

// ---- trees -----------------------------------------------------------------

struct TreeBuilder {
  MeshData &mesh;
  Vector3 origin;
  float scale;
  float yawRad;
  std::uint32_t seed;
  bool simple;

  Vector3 place(Vector3 local) const {
    const float c = std::cos(yawRad);
    const float s = std::sin(yawRad);
    const Vector3 r{local.x * c - local.z * s, local.y, local.x * s + local.z * c};
    return Vector3Add(origin, Vector3Scale(r, scale));
  }

  void trunk(float height, float bottomRadius, float topRadius, Color color) {
    const int sides = simple ? 4 : 6;
    for (int i = 0; i < sides; ++i) {
      const float a0 = 2.0f * PI * i / sides;
      const float a1 = 2.0f * PI * (i + 1) / sides;
      const Vector3 d0{std::cos(a0), 0.0f, std::sin(a0)};
      const Vector3 d1{std::cos(a1), 0.0f, std::sin(a1)};
      const Vector3 b0 = place({d0.x * bottomRadius, -0.4f, d0.z * bottomRadius});
      const Vector3 b1 = place({d1.x * bottomRadius, -0.4f, d1.z * bottomRadius});
      const Vector3 t0 = place({d0.x * topRadius, height, d0.z * topRadius});
      const Vector3 t1 = place({d1.x * topRadius, height, d1.z * topRadius});
      const Vector3 n0 = place({d0.x, 0.0f, d0.z});
      const Vector3 n1 = place({d1.x, 0.0f, d1.z});
      const Vector3 o = place({0, 0, 0});
      const Vector3 nn0 = normalized(Vector3Subtract(n0, o));
      const Vector3 nn1 = normalized(Vector3Subtract(n1, o));
      const Color dark = scaled(color, 0.75f);
      const Color cb[3] = {dark, dark, color};
      const Vector3 pa[3] = {b0, b1, t0};
      const Vector3 na[3] = {nn0, nn1, nn0};
      pushTri(mesh, pa, na, cb);
      const Vector3 pb[3] = {b1, t1, t0};
      const Vector3 nb[3] = {nn1, nn1, nn0};
      const Color cc[3] = {dark, color, color};
      pushTri(mesh, pb, nb, cc);
    }
  }

  // Lumpy sphere whose normals radiate from the whole canopy, which gives the
  // puffy "one big shape" shading typical of stylised trees.
  void blob(Vector3 center, float radius, Vector3 canopyCenter, Color base,
            std::uint32_t blobSeed) {
    const int slices = simple ? 6 : 7;
    const int stacks = simple ? 3 : 5;
    struct V {
      Vector3 p;
      Vector3 n;
      Color c;
    };
    V verts[6][8];

    for (int j = 0; j <= stacks; ++j) {
      for (int i = 0; i <= slices; ++i) {
        const int wrapped = i % slices;
        const float theta = 2.0f * PI * wrapped / slices + 0.4f;
        const float phi = PI * j / stacks;
        const Vector3 dir{std::sin(phi) * std::cos(theta), std::cos(phi),
                          std::sin(phi) * std::sin(theta)};
        const bool pole = j == 0 || j == stacks;
        const float lump =
            pole ? 1.0f
                 : 0.86f + 0.28f * noise::hash2(wrapped, j, seed + blobSeed);
        const Vector3 local = Vector3Add(
            center, {dir.x * radius * lump, dir.y * radius * lump * 0.85f,
                     dir.z * radius * lump});
        const Vector3 p = place(local);
        const Vector3 outward =
            normalized(Vector3Subtract(p, place(canopyCenter)));
        const Vector3 n = outward;
        const float top = dir.y * 0.5f + 0.5f;
        const float jitter = 0.93f + 0.14f * noise::hash2(wrapped, j, seed + blobSeed + 9);
        verts[j][i] = {p, n, scaled(base, (0.68f + 0.4f * top) * jitter)};
      }
    }

    for (int j = 0; j < stacks; ++j) {
      for (int i = 0; i < slices; ++i) {
        const V &a = verts[j][i];
        const V &b = verts[j][i + 1];
        const V &c = verts[j + 1][i];
        const V &d = verts[j + 1][i + 1];
        if (j != 0) {
          const Vector3 p[3] = {a.p, b.p, c.p};
          const Vector3 n[3] = {a.n, b.n, c.n};
          const Color col[3] = {a.c, b.c, c.c};
          pushTri(mesh, p, n, col);
        }
        if (j != stacks - 1) {
          const Vector3 p[3] = {b.p, d.p, c.p};
          const Vector3 n[3] = {b.n, d.n, c.n};
          const Color col[3] = {b.c, d.c, c.c};
          pushTri(mesh, p, n, col);
        }
      }
    }
  }

  void cone(float baseY, float radius, float height, Color color) {
    const int sides = simple ? 6 : 8;
    const Vector3 tip = place({0.0f, baseY + height, 0.0f});
    for (int i = 0; i < sides; ++i) {
      const float a0 = 2.0f * PI * i / sides;
      const float a1 = 2.0f * PI * (i + 1) / sides;
      const float mid = (a0 + a1) * 0.5f;
      const Vector3 b0 = place({std::cos(a0) * radius, baseY, std::sin(a0) * radius});
      const Vector3 b1 = place({std::cos(a1) * radius, baseY, std::sin(a1) * radius});
      const Vector3 o = place({0, baseY, 0});
      const Vector3 up = place({0, baseY + 1.0f, 0});
      const Vector3 upDir = Vector3Subtract(up, o);
      auto outward = [&](float a) {
        const Vector3 d = Vector3Subtract(place({std::cos(a), baseY, std::sin(a)}), o);
        return normalized(Vector3Add(d, Vector3Scale(upDir, 0.55f)));
      };
      const Vector3 p[3] = {b0, b1, tip};
      const Vector3 n[3] = {outward(a0), outward(a1), outward(mid)};
      const Color c[3] = {scaled(color, 0.7f), scaled(color, 0.7f), scaled(color, 1.1f)};
      pushTri(mesh, p, n, c);
    }
  }
};


Color foliageColor(const TreePlacement &t) {
  if (t.kind == TreeKind::Conifer) {
    return lerpColor({38, 98, 72, 255}, {58, 120, 80, 255}, t.variation);
  }
  if (t.variation > 0.92f) {
    return {196, 142, 52, 255};
  }
  return lerpColor({84, 154, 52, 255}, {120, 168, 62, 255}, t.variation);
}

void addTree(MeshData &mesh, const TreePlacement &t, bool simple) {
  TreeBuilder b{mesh, t.position, t.scale, t.yaw * DEG2RAD, t.seed, simple};
  const Color foliage = foliageColor(t);
  const Color bark{104, 76, 58, 255};

  if (t.kind == TreeKind::Round) {
    b.trunk(3.4f, 0.32f, 0.2f, bark);
    const Vector3 canopy{0.0f, 4.4f, 0.0f};
    if (simple) {
      b.blob(canopy, 2.7f, canopy, foliage, 1);
      return;
    }
    b.blob(canopy, 2.2f, canopy, foliage, 1);
    b.blob({1.3f, 3.5f, 0.6f}, 1.55f, canopy, foliage, 2);
    b.blob({-1.2f, 3.7f, -0.7f}, 1.6f, canopy, foliage, 3);
    b.blob({0.2f, 5.7f, 0.2f}, 1.4f, canopy, foliage, 4);
  } else if (simple) {
    b.cone(0.5f, 2.6f, 4.5f, foliage);
    b.cone(3.6f, 1.8f, 4.0f, foliage);
  } else {
    b.trunk(2.2f, 0.3f, 0.2f, bark);
    for (int i = 0; i < 4; ++i) {
      b.cone(1.0f + i * 1.7f, 2.3f - i * 0.5f, 3.0f, foliage);
    }
  }
}

MeshData generateTrees(const Key &key) {
  const float x0 = key.ix * kTreeChunk;
  const float z0 = key.iz * kTreeChunk;
  std::vector<TreePlacement> trees;
  collectTrees(x0, z0, x0 + kTreeChunk, z0 + kTreeChunk, trees);

  MeshData data;
  for (const TreePlacement &tree : trees) {
    addTree(data, tree, key.level == 1);
  }
  return data;
}

// ---- streaming -------------------------------------------------------------

Result generate(const Key &key) {
  Result r{key, key.kind == 0 ? generateTerrain(key) : generateTrees(key), {}, 0.0f};
  if (!r.data.positions.empty()) {
    measure(r.data, r.center, r.radius);
  }
  return r;
}

void workerLoop(const std::stop_token &stop) {
  while (!stop.stop_requested()) {
    Job job{};
    bool have = false;
    {
      std::lock_guard lock(g.mutex);
      if (!g.queue.empty()) {
        job = g.queue.front();
        g.queue.erase(g.queue.begin());
        have = true;
      }
    }
    if (!have) {
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
      continue;
    }
    Result r = generate(job.key);
    std::lock_guard lock(g.mutex);
    g.done.push_back(std::move(r));
  }
}

void want(const Key &key, float priority) {
  auto [it, inserted] = g.wanted.try_emplace(key, priority);
  if (!inserted) {
    it->second = std::min(it->second, priority);
  }
}

float horizontalGap(float lo, float hi, float v) {
  return std::max({lo - v, 0.0f, v - hi});
}

float tileDistance(int level, int ix, int iz) {
  const float size = tileSize(level);
  const float dx = horizontalGap(ix * size, (ix + 1) * size, g.cam.x);
  const float dz = horizontalGap(iz * size, (iz + 1) * size, g.cam.z);
  return std::sqrt(dx * dx + dz * dz + g.aboveGround * g.aboveGround);
}

void touchAndDraw(Node &node) {
  node.lastUsed = g.frame;
  g.drawList.push_back(&node.chunk);
}

// Draws whatever is already resident beneath a tile whose own mesh is missing,
// so ground never disappears while its replacement is being built.
void drawResident(int level, int ix, int iz) {
  const auto it = g.nodes.find({0, static_cast<std::int8_t>(level), ix, iz});
  if (it != g.nodes.end()) {
    touchAndDraw(it->second);
    return;
  }
  if (level == 0) {
    return;
  }
  for (int c = 0; c < 4; ++c) {
    drawResident(level - 1, ix * 2 + (c & 1), iz * 2 + (c >> 1));
  }
}

void selectTerrain(int level, int ix, int iz) {
  const float size = tileSize(level);
  const float dist = tileDistance(level, ix, iz);
  if (dist > kViewDistance) {
    return;
  }

  const Key key{0, static_cast<std::int8_t>(level), ix, iz};
  const auto self = g.nodes.find(key);

  const float splitLimit =
      size * kSplitFactor * (g.splitPrev.contains(key) ? kHysteresis : 1.0f);
  if (level > 0 && dist < splitLimit) {
    Key kids[4];
    bool allReady = true;
    for (int c = 0; c < 4; ++c) {
      kids[c] = {0, static_cast<std::int8_t>(level - 1), ix * 2 + (c & 1), iz * 2 + (c >> 1)};
      // Children past the view distance are never drawn, so never require them
      // (they would never count as used and would be rebuilt forever).
      if (tileDistance(kids[c].level, kids[c].ix, kids[c].iz) > kViewDistance) {
        continue;
      }
      if (!g.nodes.contains(kids[c])) {
        allReady = false;
        want(kids[c], dist);
      }
    }
    if (allReady) {
      g.splitNow.insert(key);
      if (self != g.nodes.end()) {
        self->second.lastUsed = g.frame; // keep the coarse tile for merging back
      }
      for (const Key &k : kids) {
        selectTerrain(k.level, k.ix, k.iz);
      }
      return;
    }
  }

  if (self != g.nodes.end()) {
    touchAndDraw(self->second);
    return;
  }

  // Root tiles have no coarser fallback, so they jump the queue.
  want(key, level == kRootLevel ? -1.0f : dist);
  if (level > 0) {
    for (int c = 0; c < 4; ++c) {
      drawResident(level - 1, ix * 2 + (c & 1), iz * 2 + (c >> 1));
    }
  }
}

void selectTrees() {
  const int x0 = static_cast<int>(std::floor((g.cam.x - kTreeFarDistance) / kTreeChunk));
  const int x1 = static_cast<int>(std::floor((g.cam.x + kTreeFarDistance) / kTreeChunk));
  const int z0 = static_cast<int>(std::floor((g.cam.z - kTreeFarDistance) / kTreeChunk));
  const int z1 = static_cast<int>(std::floor((g.cam.z + kTreeFarDistance) / kTreeChunk));

  for (int ix = x0; ix <= x1; ++ix) {
    for (int iz = z0; iz <= z1; ++iz) {
      const float cx = (ix + 0.5f) * kTreeChunk - g.cam.x;
      const float cz = (iz + 0.5f) * kTreeChunk - g.cam.z;
      const float dist = std::sqrt(cx * cx + cz * cz);
      if (dist > kTreeFarDistance + 0.71f * kTreeChunk) {
        continue;
      }
      const Key detailKey{1, 0, ix, iz};
      const float detailLimit =
          kTreeDetailDistance * (g.detailPrev.contains(detailKey) ? kHysteresis : 1.0f);
      const int tier = dist < detailLimit ? 0 : 1;
      if (tier == 0) {
        g.detailNow.insert(detailKey);
      }
      const Key preferred{1, static_cast<std::int8_t>(tier), ix, iz};
      const Key other{1, static_cast<std::int8_t>(1 - tier), ix, iz};

      auto it = g.nodes.find(preferred);
      if (it == g.nodes.end()) {
        want(preferred, dist + 50.0f);
        it = g.nodes.find(other);
      }
      if (it != g.nodes.end()) {
        it->second.lastUsed = g.frame;
        if (it->second.chunk.mesh.vertexCount > 0) {
          g.drawList.push_back(&it->second.chunk);
        }
      }
    }
  }
}

void pumpResults() {
  std::vector<Result> results;
  {
    std::lock_guard lock(g.mutex);
    const std::size_t take = std::min<std::size_t>(g.done.size(), kMaxUploadsPerFrame);
    for (std::size_t i = 0; i < take; ++i) {
      results.push_back(std::move(g.done[i]));
    }
    g.done.erase(g.done.begin(), g.done.begin() + take);
  }
  for (Result &r : results) {
    g.requested.erase(r.key);
    Node node;
    node.lastUsed = g.frame;
    if (!r.data.positions.empty()) {
      node.chunk = uploadMesh(r.data, r.center, r.radius);
      g.residentVerts += node.chunk.mesh.vertexCount;
    }
    g.nodes[r.key] = node;
  }
}

void submitRequests() {
  std::lock_guard lock(g.mutex);

  // Refresh priorities of queued work and drop jobs nobody needs any more.
  std::vector<Job> kept;
  for (Job &job : g.queue) {
    const auto it = g.wanted.find(job.key);
    if (it == g.wanted.end()) {
      g.requested.erase(job.key);
      continue;
    }
    job.priority = it->second;
    g.wanted.erase(it);
    kept.push_back(job);
  }
  g.queue = std::move(kept);

  for (const auto &[key, priority] : g.wanted) {
    if (g.requested.insert(key).second) {
      g.queue.push_back({key, priority});
    }
  }
  std::sort(g.queue.begin(), g.queue.end(),
            [](const Job &a, const Job &b) { return a.priority < b.priority; });
}

void evict() {
  if (g.residentVerts <= kVertexBudget) {
    return;
  }

  std::vector<std::pair<std::uint64_t, Key>> candidates;
  for (const auto &[key, node] : g.nodes) {
    if (node.lastUsed < g.frame) {
      candidates.emplace_back(node.lastUsed, key);
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  for (const auto &[used, key] : candidates) {
    if (g.residentVerts <= kVertexBudget) {
      break;
    }
    const auto it = g.nodes.find(key);
    if (it->second.chunk.mesh.vertexCount > 0) {
      g.residentVerts -= it->second.chunk.mesh.vertexCount;
      UnloadMesh(it->second.chunk.mesh);
    }
    g.nodes.erase(it);
  }
}

// Returns true once everything the camera currently needs is resident.
bool streamScenery(const Camera3D &camera) {
  ++g.frame;
  g.cam = camera.position;
  g.forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
  g.aboveGround =
      std::max(0.0f, g.cam.y - terrainHeight(g.cam.x, g.cam.z));
  g.drawList.clear();
  g.wanted.clear();
  g.splitPrev = std::move(g.splitNow);
  g.splitNow.clear();
  g.detailPrev = std::move(g.detailNow);
  g.detailNow.clear();

  pumpResults();

  const float rootSize = tileSize(kRootLevel);
  const int rx0 = static_cast<int>(std::floor((g.cam.x - kViewDistance) / rootSize));
  const int rx1 = static_cast<int>(std::floor((g.cam.x + kViewDistance) / rootSize));
  const int rz0 = static_cast<int>(std::floor((g.cam.z - kViewDistance) / rootSize));
  const int rz1 = static_cast<int>(std::floor((g.cam.z + kViewDistance) / rootSize));
  for (int ix = rx0; ix <= rx1; ++ix) {
    for (int iz = rz0; iz <= rz1; ++iz) {
      selectTerrain(kRootLevel, ix, iz);
    }
  }
  selectTrees();

  const bool complete = g.wanted.empty();
  submitRequests();
  evict();
  return complete;
}

bool chunkVisible(const Chunk &chunk) {
  const Vector3 toChunk = Vector3Subtract(chunk.center, g.cam);
  const float dist = Vector3Length(toChunk);
  if (dist <= chunk.radius) {
    return true;
  }
  const float cosAngle = Vector3DotProduct(toChunk, g.forward) / dist;
  const float angle = std::acos(std::clamp(cosAngle, -1.0f, 1.0f));
  const float angular = std::asin(std::min(chunk.radius / dist, 1.0f));
  return angle < 64.0f * DEG2RAD + angular;
}

std::string compose(const char *header, const char *body) {
  return std::string(header) + shaders::kNoiseLib + body;
}

void setVec3(Shader &shader, const char *name, Vector3 v) {
  SetShaderValue(shader, GetShaderLocation(shader, name), &v,
                 SHADER_UNIFORM_VEC3);
}

} // namespace

Shader &worldShader() { return g.world; }

void setMotionBlur(float amount) { g.blurAmount = amount; }

Color terrainSurfaceColor(float x, float y, float z, Vector3 normal) {
  return terrainColor(x, y, z, normal);
}

void initScenery() {
  rlSetClipPlanes(0.2, 12000.0);

  const std::string worldFrag = compose(shaders::kWorldFrag, shaders::kWorldFragMain);
  const std::string skyFrag = compose(shaders::kSkyFrag, shaders::kSkyFragMain);
  g.world = LoadShaderFromMemory(shaders::kWorldVert, worldFrag.c_str());
  g.sky = LoadShaderFromMemory(shaders::kSkyVert, skyFrag.c_str());
  g.post = LoadShaderFromMemory(shaders::kPostVert, shaders::kPostFrag);
  g.blur = LoadShaderFromMemory(shaders::kPostVert, shaders::kBlurFrag);
  g.blurAmountLoc = GetShaderLocation(g.blur, "amount");
  g.blurSrc = LoadRenderTexture(kScreenWidth, kScreenHeight);

  const Vector3 sun = normalized(kSunDir);
  setVec3(g.world, "sunDir", sun);
  setVec3(g.world, "fogColor", kFogColor);
  setVec3(g.sky, "sunDir", sun);
  setVec3(g.sky, "fogColor", kFogColor);
  g.worldCamLoc = GetShaderLocation(g.world, "camPos");
  g.skyTimeLoc = GetShaderLocation(g.sky, "time");

  g.worldMaterial = LoadMaterialDefault();
  g.worldMaterial.shader = g.world;
  g.skyMaterial = LoadMaterialDefault();
  g.skyMaterial.shader = g.sky;
  g.skyMesh = GenMeshSphere(1.0f, 24, 24);

  g.target = LoadRenderTexture(kScreenWidth, kScreenHeight);
  const Vector2 texel{1.0f / kScreenWidth, 1.0f / kScreenHeight};
  SetShaderValue(g.post, GetShaderLocation(g.post, "texel"), &texel,
                 SHADER_UNIFORM_VEC2);

  const unsigned cores = std::thread::hardware_concurrency();
  const unsigned count = std::clamp(cores > 2 ? cores - 2 : 1u, 1u, 6u);
  for (unsigned i = 0; i < count; ++i) {
    g.workers.emplace_back(workerLoop);
  }
}

void preloadScenery(const Camera3D &camera) {
  while (!WindowShouldClose()) {
    const bool ready = streamScenery(camera);
    BeginDrawing();
    ClearBackground({18, 22, 30, 255});
    DrawText("Generating world...", 40, kScreenHeight - 70, 24, RAYWHITE);
    EndDrawing();
    if (ready) {
      break;
    }
  }
}

void shutdownScenery() {
  g.workers.clear(); // jthread joins on destruction
  for (auto &[key, node] : g.nodes) {
    if (node.chunk.mesh.vertexCount > 0) {
      UnloadMesh(node.chunk.mesh);
    }
  }
  g.nodes.clear();
  UnloadMesh(g.skyMesh);
  UnloadRenderTexture(g.target);
  UnloadShader(g.world);
  UnloadShader(g.sky);
  UnloadShader(g.post);
  UnloadShader(g.blur);
  UnloadRenderTexture(g.blurSrc);
}

void beginScene(const Camera3D &camera) {
  g.time += GetFrameTime();
  streamScenery(camera);

  BeginTextureMode(g.target);
  ClearBackground({static_cast<unsigned char>(kFogColor.x * 255),
                   static_cast<unsigned char>(kFogColor.y * 255),
                   static_cast<unsigned char>(kFogColor.z * 255), 255});
  BeginMode3D(camera);

  SetShaderValue(g.sky, g.skyTimeLoc, &g.time, SHADER_UNIFORM_FLOAT);
  SetShaderValue(g.world, g.worldCamLoc, &camera.position, SHADER_UNIFORM_VEC3);

  rlDisableDepthMask();
  rlDisableBackfaceCulling();
  const Matrix skyTransform = MatrixMultiply(
      MatrixScale(8000.0f, 8000.0f, 8000.0f),
      MatrixTranslate(camera.position.x, camera.position.y, camera.position.z));
  DrawMesh(g.skyMesh, g.skyMaterial, skyTransform);
  rlEnableBackfaceCulling();
  rlEnableDepthMask();
}

void drawLandscape() {
  for (const Chunk *chunk : g.drawList) {
    if (chunkVisible(*chunk)) {
      DrawMesh(chunk->mesh, g.worldMaterial, MatrixIdentity());
    }
  }
}

void endScene() {
  EndMode3D();
  EndTextureMode();

  const Rectangle flipped = {0.0f, 0.0f, static_cast<float>(g.target.texture.width),
                             -static_cast<float>(g.target.texture.height)};
  if (g.blurAmount <= 0.001f) {
    BeginShaderMode(g.post);
    DrawTextureRec(g.target.texture, flipped, {0.0f, 0.0f}, WHITE);
    EndShaderMode();
    return;
  }

  // Fast: brush-filter into an offscreen target. Drawing the (GL-flipped)
  // scene unflipped into it flips it once, so blurSrc is upright and the final
  // draw must not flip again.
  BeginTextureMode(g.blurSrc);
  BeginShaderMode(g.post);
  DrawTextureRec(g.target.texture,
                 {0.0f, 0.0f, flipped.width, static_cast<float>(g.target.texture.height)},
                 {0.0f, 0.0f}, WHITE);
  EndShaderMode();
  EndTextureMode();

  BeginShaderMode(g.blur);
  SetShaderValue(g.blur, g.blurAmountLoc, &g.blurAmount, SHADER_UNIFORM_FLOAT);
  DrawTextureRec(g.blurSrc.texture,
                 {0.0f, 0.0f, flipped.width, static_cast<float>(g.blurSrc.texture.height)},
                 {0.0f, 0.0f}, WHITE);
  EndShaderMode();
}
