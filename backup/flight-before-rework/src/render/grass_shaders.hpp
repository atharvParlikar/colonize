#pragma once

// Grass is generated entirely in vertex shaders from gl_InstanceID and
// gl_VertexID (no vertex buffers, no per-instance CPU data), after the Ghost of
// Tsushima GDC talk. Two tiers share one wind bitmap (see kWindFrag):
//   1. blades   - real per-blade geometry, close to the camera
//   2. tufts    - 3-card inked spiky clumps, thinned and enlarged with distance
// The terrain shader (shaders.hpp) adds cheap tuft-shaped marks past that.
// Density functions here must match density() / tuftDensity() in grass.cpp.
namespace shaders {

constexpr const char *kGrassCommon = R"glsl(
#version 330
uniform mat4 viewProj;
uniform vec3 camPos;
uniform vec4 tiles[256];   // per tile: xy = world min corner, z = texture array layer
uniform int perTile;       // instances per tile; tile = gl_InstanceID / perTile
uniform float tileSize;
uniform float gNear;       // density factor at the tile's nearest point
uniform float maxDist;     // blade range
uniform int segs;
uniform float time;
uniform vec4 push;         // xyz = skiff position, w = radius
uniform sampler2DArray heightTex;
uniform sampler2DArray colorTex;
uniform float texN;        // texels per side
uniform sampler2D windTex; // r gust, g crest band, ba wind direction
uniform vec2 windOrigin;
uniform float windSize;

out vec3 vNrm;
out vec3 vCol;
out vec3 vPos;
out float vT;
out float vGust;
out vec2 vUV;

uint pcg(uint v) {
  uint s = v * 747796405u + 2891336453u;
  uint w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
  return (w >> 22u) ^ w;
}
float rnd(inout uint s) {
  s = pcg(s);
  return float(s >> 8) * (1.0 / 16777216.0);
}
float hash(vec2 p) {
  p = fract(p * vec2(123.34, 456.21));
  p += dot(p, p + 45.32);
  return fract(p.x * p.y);
}
float vnoise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x),
             mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}
vec3 bezier(vec3 a, vec3 b, vec3 c, vec3 d, float t) {
  float u = 1.0 - t;
  return u * u * u * a + 3.0 * u * u * t * b + 3.0 * u * t * t * c + t * t * t * d;
}
vec3 bezierTan(vec3 a, vec3 b, vec3 c, vec3 d, float t) {
  float u = 1.0 - t;
  return 3.0 * u * u * (b - a) + 6.0 * u * t * (c - b) + 3.0 * t * t * (d - c);
}

// Blade layout is deterministic per tile, whatever batch it is drawn in.
uint tileHash(vec2 corner) {
  ivec2 i = ivec2(floor(corner / tileSize + 0.5)) + 100000;
  return uint(i.x) * 73856093u ^ uint(i.y) * 19349663u;
}

vec4 sampleWind(vec2 xz) {
  return texture(windTex, (xz - windOrigin) / windSize);
}

// Blade / tuft colour from terrain albedo, shared so the tiers match.
vec3 grassColor(vec3 terrain, vec2 xz, float t, float tone) {
  // Meadow-scale colour patches (lush / dark / dry) read from far above.
  float patch = vnoise(xz * 0.05 + 13.0);
  vec3 tint = mix(vec3(0.78, 0.95, 0.85), vec3(1.15, 1.05, 0.65), smoothstep(0.45, 0.75, patch));
  tint *= mix(0.8, 1.0, smoothstep(0.1, 0.4, patch));
  vec3 base = terrain * vec3(0.5, 0.7, 0.5);
  vec3 tip = terrain * vec3(1.12, 1.12, 0.8) * tint + vec3(0.03, 0.05, 0.0);
  return mix(base, tip, pow(t, 0.7)) * mix(0.8, 1.2, tone);
}

void cullInstance() {
  gl_Position = vec4(2.0, 2.0, 2.0, 1.0); // outside the clip volume: no raster work
  vNrm = vec3(0, 1, 0);
  vCol = vec3(0);
  vPos = vec3(0);
  vT = 0.0;
  vGust = 0.0;
  vUV = vec2(0);
}

bool outsideFrustum(vec4 a, vec4 b) {
  float m = 0.4;
  return (a.x > a.w * (1.0 + m) && b.x > b.w * (1.0 + m)) ||
         (a.x < -a.w * (1.0 + m) && b.x < -b.w * (1.0 + m)) ||
         (a.y > a.w * (1.0 + m) && b.y > b.w * (1.0 + m)) ||
         (a.y < -a.w * (1.0 + m) && b.y < -b.w * (1.0 + m)) ||
         (a.w < 0.0 && b.w < 0.0);
}
)glsl";

// Tier 1: one blade per instance (perTile per tile), triangle strip of
// 2 * segs + 1 vertices.
constexpr const char *kBladeMain = R"glsl(
void main() {
  int ti = gl_InstanceID / perTile;
  vec4 tile = tiles[ti];
  uint s = pcg(tileHash(tile.xy) ^ pcg(uint(gl_InstanceID - ti * perTile) + 1u));
  float rx = rnd(s), rz = rnd(s), rKeep = rnd(s), rYaw = rnd(s);
  float rHeight = rnd(s), rLean = rnd(s), rTone = rnd(s), rCover = rnd(s);

  vec2 local = vec2(rx, rz) * tileSize;
  vec2 xz = tile.xy + local;

  // Terrain sample: texel centres sit at i * (tileSize / (texN - 1)).
  vec2 uv = (local / tileSize * (texN - 1.0) + 0.5) / texN;
  float ground = texture(heightTex, vec3(uv, tile.z)).r;
  vec4 surf = texture(colorTex, vec3(uv, tile.z));
  vec3 root = vec3(xz.x, ground - 0.1, xz.y);

  // Continuous density falloff; blades past a ring boundary widen to compensate.
  float d = distance(root, camPos);
  float g = pow(max(1.0 - d / maxDist, 0.0), 2.0);
  if (rKeep > g / gNear || rCover > surf.a) { cullInstance(); return; }

  // Clumped height variation + wind from the shared bitmap.
  float clump = vnoise(xz * 0.35);
  float height = mix(0.35, 0.95, rHeight) * mix(0.55, 1.15, clump) * (0.5 + 0.5 * surf.a);
  vec4 w = sampleWind(xz);
  vec2 windDir = normalize(w.ba * 2.0 - 1.0 + vec2(1e-4));
  float flutter = vnoise(xz * 0.9 - windDir * time * 4.5);
  float wind = (w.r * 0.85 + w.g * 0.5 + flutter * 0.25) * height * 0.55;

  // Skiff downwash flattens and pushes grass away.
  vec2 away = xz - push.xz;
  float pd = length(away);
  float pushAmt = (1.0 - smoothstep(0.0, push.w, pd)) * step(push.y - ground, 14.0);
  vec2 pushDir = pd > 0.001 ? away / pd : vec2(1, 0);

  float yaw = rYaw * 6.2831853;
  vec2 facing = vec2(cos(yaw), sin(yaw));
  vec2 side2 = vec2(-facing.y, facing.x);

  // Widen edge-on blades toward the camera so thin blades never vanish.
  vec2 toCam = normalize(camPos.xz - xz + vec2(1e-4));
  float edge = abs(dot(side2, toCam));
  vec2 camSide = vec2(-toCam.y, toCam.x);
  camSide *= sign(dot(camSide, side2) + 1e-4);
  side2 = normalize(mix(side2, camSide, smoothstep(0.55, 1.0, edge) * 0.8));

  float lean = height * mix(0.2, 0.7, rLean);
  vec2 bend = facing * lean + windDir * wind + pushDir * pushAmt * height * 0.8;
  float h = height * (1.0 - 0.6 * pushAmt);

  vec3 p0 = root;
  vec3 p1 = root + vec3(0, h * 0.4, 0);
  vec3 p2 = root + vec3(bend.x * 0.45, h * 0.8, bend.y * 0.45);
  vec3 p3 = root + vec3(bend.x, h * 0.92, bend.y);

  if (outsideFrustum(viewProj * vec4(p0, 1.0), viewProj * vec4(p3, 1.0))) {
    cullInstance();
    return;
  }

  int level = gl_VertexID / 2;
  float sideSign = (gl_VertexID & 1) == 0 ? -1.0 : 1.0;
  float t = float(level) / float(segs);

  float widthScale = clamp(pow(g, -0.6), 1.0, 4.0);
  // Seen from above a vertical ribbon is a hairline: fatten it with elevation.
  float top = smoothstep(0.45, 0.95, normalize(camPos - root).y);
  float halfW = 0.034 * widthScale * (1.0 + 1.3 * top) * mix(0.8, 1.2, rTone) *
                (1.0 - pow(t, 1.4));

  vec3 center = bezier(p0, p1, p2, p3, t);
  vec3 tang = normalize(bezierTan(p0, p1, p2, p3, t));
  vec3 side = vec3(side2.x, 0, side2.y);
  vec3 pos = center + side * sideSign * halfW;

  vec3 n = normalize(cross(side, tang));
  if (dot(n, camPos - pos) < 0.0) n = -n;
  // Round the blade, then soften toward the ground normal like the GDC talk.
  n = normalize(n + side * sideSign * 0.5 * (1.0 - t));
  n = normalize(mix(n, vec3(0, 1, 0), 0.2));

  vNrm = n;
  vCol = grassColor(surf.rgb, xz, t, rTone);
  vPos = pos;
  vT = t;
  vGust = w.g * 0.7 + w.r * 0.3;
  vUV = vec2(0.5, t);
  gl_Position = viewProj * vec4(pos, 1.0);
}
)glsl";

// Tier 2: one tuft per instance = 3 spiky cards fanned out from the root
// (a fountain shape, so it still reads from straight above). GL_TRIANGLES,
// 18 * segs vertices. The fragment shader cuts the spike silhouette.
constexpr const char *kTuftMain = R"glsl(
const int IDX[6] = int[6](0, 1, 2, 2, 1, 3);

// Tuft density with distance; must match tuftDensity() in grass.cpp.
float tuftDensity(float d) { return mix(1.0, 0.25, smoothstep(40.0, 150.0, d)); }

void main() {
  int quad = gl_VertexID / 6;
  int c = IDX[gl_VertexID % 6];
  float ux = float(c & 1);
  int yi = c >> 1;
  int card = quad / segs;
  int seg = quad % segs;
  float t = float(seg + yi) / float(segs);

  int ti = gl_InstanceID / perTile;
  vec4 tile = tiles[ti];
  uint s = pcg(tileHash(tile.xy) ^ pcg(uint(gl_InstanceID - ti * perTile) + 1u));
  float rx = rnd(s), rz = rnd(s), rKeep = rnd(s), rYaw = rnd(s);
  float rHeight = rnd(s), rLean = rnd(s), rTone = rnd(s), rCover = rnd(s);

  vec2 local = vec2(rx, rz) * tileSize;
  vec2 xz = tile.xy + local;
  vec2 uv = (local / tileSize * (texN - 1.0) + 0.5) / texN;
  float ground = texture(heightTex, vec3(uv, tile.z)).r;
  vec4 surf = texture(colorTex, vec3(uv, tile.z));
  vec3 root = vec3(xz.x, ground - 0.05, xz.y);

  // Hand over to blades up close, and fade out at the far edge.
  float d = distance(root, camPos);
  float tf = tuftDensity(d);
  float fade = smoothstep(20.0, 42.0, d) * (1.0 - smoothstep(135.0, 160.0, d));
  if (rKeep > tf / gNear * fade || rCover > surf.a) { cullInstance(); return; }

  // Fewer tufts far away are made bigger so the field keeps its coverage.
  float sizeScale = clamp(pow(tf, -0.5), 1.0, 2.2);
  float clump = vnoise(xz * 0.2);
  float height = mix(0.6, 1.1, rHeight) * mix(0.7, 1.2, clump) * (0.5 + 0.5 * surf.a) * sizeScale;
  float halfW = height * mix(0.28, 0.4, rLean);

  vec4 w = sampleWind(xz);
  vec2 windDir = normalize(w.ba * 2.0 - 1.0 + vec2(1e-4));
  float wind = (w.r * 0.8 + w.g * 0.6) * height * 0.45;

  vec2 away = xz - push.xz;
  float pd = length(away);
  float pushAmt = (1.0 - smoothstep(0.0, push.w, pd)) * step(push.y - ground, 14.0);
  vec2 pushDir = pd > 0.001 ? away / pd : vec2(1, 0);

  float az = (rYaw + float(card) / 3.0) * 6.2831853;
  vec2 o = vec2(cos(az), sin(az));
  vec2 sd = vec2(-o.y, o.x);
  vec3 side = vec3(sd.x, 0, sd.y);

  float lean = height * mix(0.4, 0.7, rLean);
  vec2 sway = o * lean + windDir * wind + pushDir * pushAmt * height * 0.7;
  float h = height * (1.0 - 0.5 * pushAmt);

  vec3 tip = root + vec3(sway.x, h, sway.y);
  if (outsideFrustum(viewProj * vec4(root, 1.0), viewProj * vec4(tip, 1.0))) {
    cullInstance();
    return;
  }

  vec3 center = root + vec3(sway.x * t * t, h * t, sway.y * t * t);
  vec3 pos = center + side * (ux * 2.0 - 1.0) * halfW * (1.0 - 0.15 * t);
  vec3 tang = normalize(vec3(sway.x * 2.0 * t, h, sway.y * 2.0 * t));

  vec3 n = normalize(cross(side, tang));
  if (dot(n, camPos - pos) < 0.0) n = -n;
  n = normalize(mix(n, vec3(0, 1, 0), 0.45)); // soft, painterly shading

  vNrm = n;
  vCol = grassColor(surf.rgb, xz, t, rTone);
  vPos = pos;
  vT = t;
  vGust = w.g * 0.7 + w.r * 0.3;
  vUV = vec2(ux, t);
  gl_Position = viewProj * vec4(pos, 1.0);
}
)glsl";

constexpr const char *kGrassFrag = R"glsl(
#version 330
in vec3 vNrm;
in vec3 vCol;
in vec3 vPos;
in float vT;
in float vGust;
in vec2 vUV;
uniform vec3 camPos;
uniform vec3 sunDir;
uniform vec3 fogColor;
uniform int tuft;
out vec4 finalColor;

void main() {
  float tuftInk = 1.0, vein = 0.0;
  if (tuft == 1) {
    // Three spikes across the card, lower toward its edges.
    float e = abs(2.0 * vUV.x - 1.0);
    float s = abs(fract(vUV.x * 3.0) - 0.5) * 2.0;
    float top = 0.3 + 0.7 * (1.0 - s) * (1.0 - 0.6 * e * e);
    if (vUV.y > top) discard;
    tuftInk = smoothstep(0.0, 0.07, min(top - vUV.y, min(vUV.x, 1.0 - vUV.x) * 1.2));
    vein = 1.0 - smoothstep(0.0, 0.1, s);
  }

  vec3 n = normalize(vNrm);
  vec3 v = normalize(camPos - vPos);
  float d = length(camPos - vPos);

  // Same banded, cool-shadow / warm-light ramp as the terrain shader.
  float lit = dot(n, sunDir) * 0.5 + 0.5;
  float shade = smoothstep(0.34, 0.42, lit) * 0.45 + smoothstep(0.56, 0.64, lit) * 0.55;
  vec3 shadowTint = vec3(0.40, 0.44, 0.72);
  vec3 lightTint = vec3(1.08, 1.0, 0.84);
  vec3 col = vCol * mix(shadowTint, lightTint, shade);

  // Fake AO toward the roots, anime-style hard-edged wind crest highlight,
  // and light glowing through the blade when the sun is behind it.
  col *= mix(0.55, 1.0, smoothstep(0.0, 0.7, vT));
  col *= mix(0.78, 1.22, smoothstep(0.3, 0.45, vGust));
  float back = pow(max(dot(v, -sunDir), 0.0), 3.0) * vT;
  col += vCol * vec3(0.55, 0.6, 0.15) * back * 0.5;

  // Tufts get an ink outline and a faint centre vein, anime style.
  col *= mix(0.38, 1.0, tuftInk);
  col *= 1.0 - 0.18 * vein * vT;

  float fog = 1.0 - exp(-pow(d * 0.00035, 1.5));
  col = mix(col, fogColor, clamp(fog, 0.0, 1.0));
  finalColor = vec4(col, 1.0);
}
)glsl";

// Wind bitmap: rendered on the GPU every frame into a world-space window
// around the camera. r = gust strength, g =
// crest band (anime-style travelling waves), ba = wind direction. Everything is
// a pure function of world position and time, so snapping the window to the
// texel grid causes no swimming.
constexpr const char *kWindVert = R"glsl(
#version 330
out vec2 uv;
void main() {
  vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2); // fullscreen triangle
  uv = p;
  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)glsl";

constexpr const char *kWindFrag = R"glsl(
#version 330
in vec2 uv;
uniform vec2 origin;
uniform float size;
uniform float time;
out vec4 o;

float hash(vec2 p) {
  p = fract(p * vec2(123.34, 456.21));
  p += dot(p, p + 45.32);
  return fract(p.x * p.y);
}
float vnoise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x),
             mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}
float fbm(vec2 p) {
  float a = 0.5, s = 0.0;
  for (int i = 0; i < 4; i++) {
    s += a * vnoise(p);
    p = p * 2.03 + 17.0;
    a *= 0.5;
  }
  return s;
}

void main() {
  vec2 p = origin + uv * size;
  // Prevailing wind drifts slowly and varies across the landscape.
  float a = 0.45 + (vnoise(p * 0.0012 + 3.0) - 0.5) * 1.4 + 0.12 * sin(time * 0.05);
  vec2 dir = vec2(cos(a), sin(a));

  // Gust field: noise advected along the wind with a domain warp.
  vec2 q = p - dir * time * 5.0;
  vec2 warp = vec2(vnoise(q * 0.008), vnoise(q * 0.008 + 7.0)) * 2.0 - 1.0;
  float gust = smoothstep(0.25, 0.8, fbm(q * 0.01 + warp * 0.8));

  // Long wave fronts rolling downwind, bent by the gusts, then cut to a band.
  float x = dot(p, dir) * 0.05 - time * 1.1 + (gust - 0.5) * 2.5 +
            (vnoise(p * 0.004) - 0.5) * 3.0;
  float crest = smoothstep(0.55, 0.95, 0.5 + 0.5 * sin(x)) * (0.35 + 0.65 * gust);
  o = vec4(gust, crest, dir * 0.5 + 0.5);
}
)glsl";

} // namespace shaders
