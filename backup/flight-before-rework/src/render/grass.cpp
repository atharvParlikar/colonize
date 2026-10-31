#define GL_SILENCE_DEPRECATION
#include "render/grass.hpp"

#include "render/grass_shaders.hpp"
#include "render/motion.hpp"
#include "render/scenery.hpp"
#include "rlgl.h"
#include "raymath.h"
#include "world/noise.hpp"
#include "world/terrain.hpp"

#include <OpenGL/gl3.h>

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr float kTileSize = 8.0f;
constexpr int kTexels = 17; // 16 cells + 1 so neighbouring tiles share edges

// Tier 1: blades.
constexpr float kBladeDist = 70.0f;
constexpr int kFullBlades = 4096; // per tile at full density (64 per m^2)
// Tier 2: tufts.
constexpr float kTuftMin = 10.0f; // tiles nearer than this are all blades
constexpr float kTuftFar = 160.0f;
constexpr int kFullTufts = 144;

// At full motion blur only this fraction of the range is drawn / streamed.
constexpr float kFastRangeScale = 0.3f;

// Resident tiles live in a toroidal grid: tile (x, z) always occupies slot
// (z mod W, x mod W), which is also its layer in the texture arrays. Every tile
// within reach of the camera maps to a distinct slot, so lookup is an array
// index (no hashing), a tile entering range simply overwrites the one that
// just left it (no eviction pass), and a slot's key says whether it is current.
constexpr int kReach = static_cast<int>(kTuftFar / kTileSize); // 20
constexpr int kGridW = 44;                                       // >= 2 * kReach + 1
static_assert(kGridW >= 2 * kReach + 1);
constexpr int kSlots = kGridW * kGridW;

// Terrain generation is ~0.3 ms per tile, so the limits below are about GL
// and queue pressure, not CPU: a full ring refills in a few frames.
constexpr int kMaxPending = 192;
constexpr int kMaxUploadsPerFrame = 96;
constexpr int kMaxTilesPerDraw = 256; // matches `tiles[256]` in the shaders
// Below this height above ground tiles are never out of range; above it a tile
// is only wanted when it is within true 3D range.
constexpr float kGroundSlack = 30.0f;

// Wind bitmap: a world-space window around the camera.
constexpr int kWindRes = 256;
constexpr float kWindSize = 512.0f;

struct LodRing {
  float from;   // nearest tile distance this ring covers
  int segments; // blade strip = 2 * segments + 1 vertices; tuft = 18 * segments
};
constexpr std::array<LodRing, 3> kBladeRings = {{{0.0f, 3}, {22.0f, 2}, {45.0f, 1}}};
constexpr std::array<LodRing, 3> kTuftRings = {{{kTuftMin, 3}, {60.0f, 2}, {110.0f, 1}}};

struct TileKey {
  int x = INT_MIN, z = INT_MIN;
  bool operator==(const TileKey &) const = default;
};

struct TileData {
  TileKey key{};
  std::vector<float> height;        // kTexels^2
  std::vector<unsigned char> color; // kTexels^2 * 4, alpha = grass coverage
  float minY = 0, maxY = 0, maxCover = 0;
};

enum class SlotState : std::uint8_t { Empty, Pending, Ready };

struct Slot {
  TileKey key{};
  SlotState state = SlotState::Empty;
  float minY = 0, maxY = 0, maxCover = 0;
};

struct Uniforms {
  GLint viewProj, camPos, tiles, perTile, tileSize, gNear, maxDist, segs, time, push, heightTex,
      colorTex, texN, windTex, windOrigin, windSize, sunDir, fogColor, tuft;
};

struct Program {
  GLuint id = 0;
  Uniforms u{};
};

struct TileRef {
  float x, z, layer, pad;
};

struct Grass {
  Program blade, tuft;
  GLuint windProgram = 0;
  GLint windOrigin = -1, windSize = -1, windTime = -1;
  GLuint windTex = 0, windFbo = 0;
  Vector2 windCorner{};
  GLuint heightArray = 0, colorArray = 0;
  GLuint vao = 0;

  std::array<Slot, kSlots> slots{};
  int pending = 0; // requested, result not yet consumed
  std::uint64_t frame = 0;

  std::mutex mutex;
  std::condition_variable_any wake;
  std::vector<TileKey> queue; // worker input
  std::vector<TileData> done; // worker output
  std::vector<std::jthread> workers;

  // Per-frame scratch, kept to avoid reallocating.
  std::array<std::vector<TileRef>, 3> bladeLists, tuftLists;
};

Grass g;

// Terrain sun / fog must match the world shader.
constexpr Vector3 kSunDir = {0.6f, 0.45f, 0.55f};
constexpr Vector3 kFogColor = {0.74f, 0.83f, 0.93f};

int wrapIndex(const int v) {
  const int m = v % kGridW;
  return m < 0 ? m + kGridW : m;
}
int slotOf(const TileKey k) { return wrapIndex(k.z) * kGridW + wrapIndex(k.x); }

// Worker thread: samples the CPU terrain into per-tile textures.
TileData buildTile(const TileKey key) {
  using namespace noise;
  constexpr int pad = kTexels + 2; // one-texel border for central differences
  constexpr float step = kTileSize / (kTexels - 1);
  const float x0 = key.x * kTileSize, z0 = key.z * kTileSize;

  std::array<float, pad * pad> h{};
  for (int j = 0; j < pad; ++j) {
    for (int i = 0; i < pad; ++i) {
      h[j * pad + i] = terrainHeight(x0 + (i - 1) * step, z0 + (j - 1) * step);
    }
  }

  TileData t;
  t.key = key;
  t.height.resize(kTexels * kTexels);
  t.color.resize(kTexels * kTexels * 4);
  t.minY = 1e9f;
  t.maxY = -1e9f;
  for (int j = 0; j < kTexels; ++j) {
    for (int i = 0; i < kTexels; ++i) {
      const float y = h[(j + 1) * pad + i + 1];
      const float dx = h[(j + 1) * pad + i + 2] - h[(j + 1) * pad + i];
      const float dz = h[(j + 2) * pad + i + 1] - h[j * pad + i + 1];
      const Vector3 n = Vector3Normalize({-dx, 2.0f * step, -dz});
      const float x = x0 + i * step, z = z0 + j * step;

      const Color c = terrainSurfaceColor(x, y, z, n);
      const float slope = 1.0f - n.y;
      // No grass through the spawn platform (32 m square, flat clearing).
      const float platform = smooth(16.0f, 18.0f, std::max(std::fabs(x), std::fabs(z)));
      const float cover = (1.0f - smooth(0.10f, 0.22f, slope)) *
                          (1.0f - smooth(300.0f, 420.0f, y)) * platform;

      const int idx = j * kTexels + i;
      t.height[idx] = y;
      t.color[idx * 4 + 0] = c.r;
      t.color[idx * 4 + 1] = c.g;
      t.color[idx * 4 + 2] = c.b;
      t.color[idx * 4 + 3] = static_cast<unsigned char>(std::clamp(cover, 0.0f, 1.0f) * 255.0f);
      t.minY = std::min(t.minY, y);
      t.maxY = std::max(t.maxY, y);
      t.maxCover = std::max(t.maxCover, cover);
    }
  }
  return t;
}

void workerLoop(const std::stop_token &stop) {
  while (!stop.stop_requested()) {
    TileKey key{};
    {
      std::unique_lock lock(g.mutex);
      g.wake.wait(lock, stop, [] { return !g.queue.empty(); });
      if (stop.stop_requested()) {
        return;
      }
      key = g.queue.front();
      g.queue.erase(g.queue.begin());
    }
    TileData data = buildTile(key);
    std::lock_guard lock(g.mutex);
    g.done.push_back(std::move(data));
  }
}

GLuint makeArray(const GLenum internal) {
  GLuint id = 0;
  glGenTextures(1, &id);
  glBindTexture(GL_TEXTURE_2D_ARRAY, id);
  glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, static_cast<GLint>(internal), kTexels, kTexels, kSlots, 0,
               internal == GL_R32F ? GL_RED : GL_RGBA,
               internal == GL_R32F ? GL_FLOAT : GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  return id;
}

GLuint compile(const GLenum type, const std::string &src) {
  const GLuint s = glCreateShader(type);
  const char *text = src.c_str();
  glShaderSource(s, 1, &text, nullptr);
  glCompileShader(s);
  GLint ok = 0;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[2048];
    glGetShaderInfoLog(s, sizeof log, nullptr, log);
    TraceLog(LOG_ERROR, "GRASS: shader compile failed: %s", log);
  }
  return s;
}

GLuint link(const std::string &vs, const std::string &fs) {
  const GLuint v = compile(GL_VERTEX_SHADER, vs);
  const GLuint f = compile(GL_FRAGMENT_SHADER, fs);
  const GLuint p = glCreateProgram();
  glAttachShader(p, v);
  glAttachShader(p, f);
  glLinkProgram(p);
  GLint ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (!ok) {
    char log[2048];
    glGetProgramInfoLog(p, sizeof log, nullptr, log);
    TraceLog(LOG_ERROR, "GRASS: program link failed: %s", log);
  }
  glDeleteShader(v);
  glDeleteShader(f);
  return p;
}

Program makeProgram(const std::string &vertexMain) {
  Program p;
  p.id = link(std::string(shaders::kGrassCommon) + vertexMain, shaders::kGrassFrag);
  auto loc = [&](const char *n) { return glGetUniformLocation(p.id, n); };
  p.u = {loc("viewProj"), loc("camPos"),  loc("tiles"),      loc("perTile"),  loc("tileSize"),
         loc("gNear"),    loc("maxDist"), loc("segs"),       loc("time"),     loc("push"),
         loc("heightTex"), loc("colorTex"), loc("texN"),     loc("windTex"),  loc("windOrigin"),
         loc("windSize"), loc("sunDir"),  loc("fogColor"),   loc("tuft")};
  return p;
}

void createWindMap() {
  glGenTextures(1, &g.windTex);
  glBindTexture(GL_TEXTURE_2D, g.windTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kWindRes, kWindRes, 0, GL_RGBA, GL_UNSIGNED_BYTE,
               nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glGenFramebuffers(1, &g.windFbo);
  glBindFramebuffer(GL_FRAMEBUFFER, g.windFbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g.windTex, 0);
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    TraceLog(LOG_ERROR, "GRASS: wind framebuffer incomplete");
  }
}

float density(const float d) { // blades; must match the vertex shader
  const float f = std::max(1.0f - d / kBladeDist, 0.0f);
  return f * f;
}

float smoothstepf(const float a, const float b, const float x) {
  const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

float tuftDensity(const float d) { // must match the vertex shader
  return 1.0f + (0.25f - 1.0f) * smoothstepf(40.0f, 150.0f, d);
}

template <std::size_t N>
int ringFor(const std::array<LodRing, N> &rings, const float dist) {
  int ring = 0;
  for (int r = 0; r < static_cast<int>(N); ++r) {
    if (dist >= rings[r].from) {
      ring = r;
    }
  }
  return ring;
}

// Height above the terrain under `p`, minus slack: how far the viewer is from
// any ground it could be looking at.
float heightAboveGround(const Vector3 &p) {
  return std::max(p.y - terrainHeight(p.x, p.z) - kGroundSlack, 0.0f);
}

void setCommonUniforms(const Program &p, const float *viewProj, const Vector3 &cam,
                       const Vector3 &skiff, const float time) {
  glUseProgram(p.id);
  glUniformMatrix4fv(p.u.viewProj, 1, GL_FALSE, viewProj);
  glUniform3f(p.u.camPos, cam.x, cam.y, cam.z);
  const Vector3 sun = Vector3Normalize(kSunDir);
  glUniform3f(p.u.sunDir, sun.x, sun.y, sun.z);
  glUniform3f(p.u.fogColor, kFogColor.x, kFogColor.y, kFogColor.z);
  glUniform1f(p.u.tileSize, kTileSize);
  glUniform1f(p.u.maxDist, kBladeDist);
  glUniform1f(p.u.time, time);
  glUniform4f(p.u.push, skiff.x, skiff.y, skiff.z, 7.0f);
  glUniform1f(p.u.texN, static_cast<float>(kTexels));
  glUniform1i(p.u.heightTex, 4);
  glUniform1i(p.u.colorTex, 5);
  glUniform1i(p.u.windTex, 6);
  glUniform2f(p.u.windOrigin, g.windCorner.x, g.windCorner.y);
  glUniform1f(p.u.windSize, kWindSize);
}

// One batch per LOD ring: every tile of the ring in a handful of draw calls,
// the shader finds its tile from gl_InstanceID.
void drawBatch(const Program &p, const std::vector<TileRef> &list, const float gNear,
               const int segs, const int perTile, const GLenum mode, const int vertices) {
  if (list.empty()) {
    return;
  }
  glUniform1f(p.u.gNear, gNear);
  glUniform1i(p.u.segs, segs);
  glUniform1i(p.u.perTile, perTile);
  for (std::size_t i = 0; i < list.size(); i += kMaxTilesPerDraw) {
    const int n = static_cast<int>(std::min<std::size_t>(kMaxTilesPerDraw, list.size() - i));
    glUniform4fv(p.u.tiles, n, &list[i].x);
    glDrawArraysInstanced(mode, 0, vertices, n * perTile);
  }
}

void uploadFinishedTiles() {
  std::vector<TileData> ready;
  {
    std::lock_guard lock(g.mutex);
    const std::size_t n = std::min<std::size_t>(g.done.size(), kMaxUploadsPerFrame);
    ready.assign(std::make_move_iterator(g.done.begin()),
                 std::make_move_iterator(g.done.begin() + static_cast<std::ptrdiff_t>(n)));
    g.done.erase(g.done.begin(), g.done.begin() + static_cast<std::ptrdiff_t>(n));
  }
  if (ready.empty()) {
    return;
  }
  for (TileData &d : ready) {
    --g.pending;
    const int slot = slotOf(d.key);
    Slot &s = g.slots[slot];
    if (!(s.key == d.key) || s.state != SlotState::Pending) {
      continue; // the camera moved on and the slot was reassigned
    }
    glBindTexture(GL_TEXTURE_2D_ARRAY, g.heightArray);
    glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, slot, kTexels, kTexels, 1, GL_RED, GL_FLOAT,
                    d.height.data());
    glBindTexture(GL_TEXTURE_2D_ARRAY, g.colorArray);
    glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, slot, kTexels, kTexels, 1, GL_RGBA,
                    GL_UNSIGNED_BYTE, d.color.data());
    s.state = SlotState::Ready;
    s.minY = d.minY;
    s.maxY = d.maxY;
    s.maxCover = d.maxCover;
  }
  glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
}

} // namespace

void initGrass() {
  GLint maxLayers = 0;
  glGetIntegerv(GL_MAX_ARRAY_TEXTURE_LAYERS, &maxLayers);
  if (maxLayers < kSlots) {
    TraceLog(LOG_ERROR, "GRASS: GL supports %d array layers, need %d", maxLayers, kSlots);
  }

  g.blade = makeProgram(shaders::kBladeMain);
  g.tuft = makeProgram(shaders::kTuftMain);
  g.windProgram = link(shaders::kWindVert, shaders::kWindFrag);
  g.windOrigin = glGetUniformLocation(g.windProgram, "origin");
  g.windSize = glGetUniformLocation(g.windProgram, "size");
  g.windTime = glGetUniformLocation(g.windProgram, "time");
  createWindMap();
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  g.heightArray = makeArray(GL_R32F);
  g.colorArray = makeArray(GL_RGBA8);
  glBindTexture(GL_TEXTURE_2D_ARRAY, 0);

  // Core profile needs a bound VAO even though the draws read no attributes.
  glGenVertexArrays(1, &g.vao);

  const unsigned cores = std::thread::hardware_concurrency();
  const unsigned count = std::clamp(cores > 4 ? cores / 3 : 1u, 1u, 3u);
  for (unsigned i = 0; i < count; ++i) {
    g.workers.emplace_back(workerLoop);
  }
}

void shutdownGrass() {
  for (auto &w : g.workers) {
    w.request_stop();
  }
  g.workers.clear();
  glDeleteTextures(1, &g.heightArray);
  glDeleteTextures(1, &g.colorArray);
  glDeleteFramebuffers(1, &g.windFbo);
  glDeleteTextures(1, &g.windTex);
  glDeleteVertexArrays(1, &g.vao);
  glDeleteProgram(g.blade.id);
  glDeleteProgram(g.tuft.id);
  glDeleteProgram(g.windProgram);
}

void updateGrassWind(const Camera3D &camera, const float time) {
  // Nothing will sample the bitmap when the ground is out of grass range.
  if (heightAboveGround(camera.position) >= kTuftFar) {
    return;
  }
  rlDrawRenderBatchActive(); // flush raylib before touching GL state

  GLint prevFbo = 0;
  GLint prevViewport[4];
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prevFbo);
  glGetIntegerv(GL_VIEWPORT, prevViewport);
  const GLboolean depth = glIsEnabled(GL_DEPTH_TEST);
  const GLboolean blend = glIsEnabled(GL_BLEND);
  const GLboolean cull = glIsEnabled(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);

  // Centred on the camera and snapped to the texel grid, so the field is
  // stable in world space.
  const float texel = kWindSize / kWindRes;
  g.windCorner = {std::floor((camera.position.x - kWindSize * 0.5f) / texel) * texel,
                  std::floor((camera.position.z - kWindSize * 0.5f) / texel) * texel};

  glUseProgram(g.windProgram);
  glBindVertexArray(g.vao);
  glBindFramebuffer(GL_FRAMEBUFFER, g.windFbo);
  glViewport(0, 0, kWindRes, kWindRes);
  glUniform2f(g.windOrigin, g.windCorner.x, g.windCorner.y);
  glUniform1f(g.windSize, kWindSize);
  glUniform1f(g.windTime, time);
  glDrawArrays(GL_TRIANGLES, 0, 3);

  glBindVertexArray(0);
  glUseProgram(0);
  glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(prevFbo));
  glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
  if (depth) glEnable(GL_DEPTH_TEST);
  if (blend) glEnable(GL_BLEND);
  if (cull) glEnable(GL_CULL_FACE);
}

void drawGrass(const Camera3D &camera, const Vector3 skiff, const Vector3 velocity,
               const float time) {
  ++g.frame;
  uploadFinishedTiles();

  const Vector3 cam = camera.position;
  const Vector3 fwd = Vector3Normalize(Vector3Subtract(camera.target, camera.position));

  // Speed gating: under motion blur only grass near the craft is worth it.
  const float speed = Vector3Length(velocity);
  const float fast = motionBlurAmount(speed);
  const float rangeScale = 1.0f + (kFastRangeScale - 1.0f) * fast;
  const float bladeRange = kBladeDist * rangeScale;
  const float tuftRange = kTuftFar * rangeScale;
  const Vector3 heading = speed > 1.0f ? Vector3Scale(velocity, 1.0f / speed) : fwd;

  // Height above ground now and half a second ahead: when diving, tiles are
  // wanted before the camera is low enough to see them.
  const Vector3 ahead = Vector3Add(cam, Vector3Scale(velocity, 0.5f));
  const float agl = std::min(heightAboveGround(cam), heightAboveGround(ahead));
  if (agl >= tuftRange) {
    return; // too high for any grass: no scan, no requests, no draws
  }

  // Camera frustum as a cone wide enough for the screen diagonal (and roll).
  constexpr float aspect = 16.0f / 9.0f;
  const float halfV = std::tan(camera.fovy * DEG2RAD * 0.5f);
  const float halfAngle = std::atan(halfV * std::sqrt(1.0f + aspect * aspect));

  const int cx = static_cast<int>(std::floor(cam.x / kTileSize));
  const int cz = static_cast<int>(std::floor(cam.z / kTileSize));
  const int reach = std::min(kReach, static_cast<int>(std::ceil(tuftRange / kTileSize)));

  struct Visible {
    int slot;
    float dist;
  };
  struct Wanted {
    float dist;
    TileKey key;
  };
  std::vector<Visible> visible;
  std::vector<Wanted> wanted;
  visible.reserve(512);

  for (int dz = -reach; dz <= reach; ++dz) {
    for (int dx = -reach; dx <= reach; ++dx) {
      const TileKey key{cx + dx, cz + dz};
      const float x0 = key.x * kTileSize, z0 = key.z * kTileSize;
      const float nx = std::clamp(cam.x, x0, x0 + kTileSize);
      const float nz = std::clamp(cam.z, z0, z0 + kTileSize);
      const float flat = std::hypot(cam.x - nx, cam.z - nz);
      const float reachDist = std::sqrt(flat * flat + agl * agl);
      if (reachDist >= tuftRange) {
        continue;
      }

      // Tiles the craft has already passed are neither streamed nor drawn at
      // speed (the frustum test would drop most of them anyway).
      const Vector3 center = {x0 + kTileSize * 0.5f, cam.y, z0 + kTileSize * 0.5f};
      if (fast > 0.0f) {
        const Vector3 v = Vector3Subtract(center, cam);
        if (Vector3DotProduct(v, heading) < -kTileSize * 1.5f && flat > 12.0f) {
          continue;
        }
      }

      const int slotIndex = slotOf(key);
      Slot &slot = g.slots[slotIndex];
      if (!(slot.key == key)) {
        wanted.push_back({reachDist, key});
        continue;
      }
      if (slot.state != SlotState::Ready || slot.maxCover < 0.02f) {
        continue;
      }

      const float ny = std::clamp(cam.y, slot.minY, slot.maxY + 1.0f);
      const float dist = Vector3Distance(cam, {nx, ny, nz});
      if (dist >= tuftRange) {
        continue;
      }
      const Vector3 tileCenter = {center.x, (slot.minY + slot.maxY) * 0.5f + 0.5f, center.z};
      const float radius = std::hypot(kTileSize * 0.7072f, (slot.maxY - slot.minY) * 0.5f + 0.6f);
      const Vector3 v = Vector3Subtract(tileCenter, cam);
      const float len = Vector3Length(v);
      if (len > radius) {
        const float along = Vector3DotProduct(v, fwd);
        const float angle = std::acos(std::clamp(along / len, -1.0f, 1.0f));
        if (angle > halfAngle + std::asin(std::min(radius / len, 1.0f))) {
          continue;
        }
      }
      visible.push_back({slotIndex, dist});
    }
  }

  // Request the nearest missing tiles, as many as the pending budget allows.
  // Claiming the slot now also hides the stale tile that used to live there.
  const int budget = kMaxPending - g.pending;
  if (budget > 0 && !wanted.empty()) {
    const auto n = std::min<std::size_t>(wanted.size(), static_cast<std::size_t>(budget));
    std::partial_sort(wanted.begin(), wanted.begin() + static_cast<std::ptrdiff_t>(n),
                      wanted.end(), [](const Wanted &a, const Wanted &b) { return a.dist < b.dist; });
    {
      std::lock_guard lock(g.mutex);
      for (std::size_t i = 0; i < n; ++i) {
        Slot &slot = g.slots[slotOf(wanted[i].key)];
        slot.key = wanted[i].key;
        slot.state = SlotState::Pending;
        g.queue.push_back(wanted[i].key);
        ++g.pending;
      }
    }
    g.wake.notify_all();
  }

  if (visible.empty()) {
    return;
  }

  // Bucket the visible tiles by LOD ring.
  for (auto &l : g.bladeLists) l.clear();
  for (auto &l : g.tuftLists) l.clear();
  for (const Visible &v : visible) {
    const Slot &slot = g.slots[v.slot];
    const TileRef ref = {slot.key.x * kTileSize, slot.key.z * kTileSize, static_cast<float>(v.slot),
                         0.0f};
    if (v.dist < bladeRange) {
      g.bladeLists[ringFor(kBladeRings, v.dist)].push_back(ref);
    }
    if (v.dist >= kTuftMin) {
      g.tuftLists[ringFor(kTuftRings, v.dist)].push_back(ref);
    }
  }

  // ---- draw ---------------------------------------------------------------
  rlDrawRenderBatchActive(); // flush raylib's pending geometry first

  const Matrix viewProj = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
  const float vp[16] = {viewProj.m0,  viewProj.m1,  viewProj.m2,  viewProj.m3,
                        viewProj.m4,  viewProj.m5,  viewProj.m6,  viewProj.m7,
                        viewProj.m8,  viewProj.m9,  viewProj.m10, viewProj.m11,
                        viewProj.m12, viewProj.m13, viewProj.m14, viewProj.m15};

  glBindVertexArray(g.vao);
  glDisable(GL_CULL_FACE);
  glActiveTexture(GL_TEXTURE4);
  glBindTexture(GL_TEXTURE_2D_ARRAY, g.heightArray);
  glActiveTexture(GL_TEXTURE5);
  glBindTexture(GL_TEXTURE_2D_ARRAY, g.colorArray);
  glActiveTexture(GL_TEXTURE6);
  glBindTexture(GL_TEXTURE_2D, g.windTex);

  // Tier 1: blades.
  setCommonUniforms(g.blade, vp, cam, skiff, time);
  glUniform1i(g.blade.u.tuft, 0);
  for (std::size_t r = 0; r < kBladeRings.size(); ++r) {
    const float gNear = density(kBladeRings[r].from);
    const int perTile = std::max(1, static_cast<int>(std::ceil(kFullBlades * gNear)));
    const int segs = kBladeRings[r].segments;
    drawBatch(g.blade, g.bladeLists[r], gNear, segs, perTile, GL_TRIANGLE_STRIP, 2 * segs + 1);
  }

  // Tier 2: tufts.
  setCommonUniforms(g.tuft, vp, cam, skiff, time);
  glUniform1i(g.tuft.u.tuft, 1);
  for (std::size_t r = 0; r < kTuftRings.size(); ++r) {
    const float gNear = tuftDensity(kTuftRings[r].from);
    const int perTile = std::max(1, static_cast<int>(std::ceil(kFullTufts * gNear)));
    const int segs = kTuftRings[r].segments;
    drawBatch(g.tuft, g.tuftLists[r], gNear, segs, perTile, GL_TRIANGLES, 18 * segs);
  }

  // Leave GL the way raylib expects it.
  glActiveTexture(GL_TEXTURE6);
  glBindTexture(GL_TEXTURE_2D, 0);
  for (const GLenum unit : {GL_TEXTURE5, GL_TEXTURE4}) {
    glActiveTexture(unit);
    glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
  }
  glActiveTexture(GL_TEXTURE0);
  glBindVertexArray(0);
  glUseProgram(0);
  glEnable(GL_CULL_FACE);
}
