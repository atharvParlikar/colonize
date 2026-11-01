#pragma once

// GLSL sources embedded so the binary does not depend on the working directory.
namespace shaders {

constexpr const char *kNoiseLib = R"glsl(
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
float fbm3(vec2 p) {
  float s = 0.5 * vnoise(p);
  p = p * 2.03 + 17.0;
  s += 0.25 * vnoise(p);
  p = p * 2.03 + 17.0;
  return s + 0.125 * vnoise(p);
}
float fbm2(vec2 p) {
  float s = 0.5 * vnoise(p);
  return s + 0.25 * vnoise(p * 2.03 + 17.0);
}
mat2 rot(float a) { float c = cos(a), s = sin(a); return mat2(c, -s, s, c); }
)glsl";

constexpr const char *kWorldVert = R"glsl(
#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec4 vertexColor;
uniform mat4 mvp;
out vec3 wPos;
out vec3 wNrm;
out vec4 vCol;
void main() {
  wPos = vertexPosition;
  wNrm = vertexNormal;
  vCol = vertexColor;
  gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)glsl";

// Painterly lighting: noisy brush strokes perturb the albedo and the light
// bands, shadows go cool/purple, lit areas warm, distance fades to haze.
constexpr const char *kWorldFrag = R"glsl(
#version 330
in vec3 wPos;
in vec3 wNrm;
in vec4 vCol;
uniform vec3 camPos;
uniform vec3 sunDir;
uniform vec3 fogColor;
out vec4 finalColor;
)glsl";

constexpr const char *kWorldFragMain = R"glsl(
vec2 surfUV(vec3 p, vec3 n) {
  vec3 a = abs(n);
  if (a.y >= a.x && a.y >= a.z) return p.xz;
  if (a.x >= a.z) return p.zy;
  return p.xy;
}

void main() {
  vec3 n = wNrm;
  n = length(n) < 0.01 ? vec3(0, 1, 0) : normalize(n);
  vec2 uv = surfUV(wPos, n);
  float d = length(wPos - camPos);
  float near = 1.0 - smoothstep(60.0, 500.0, d);

  // Fine brush strokes only matter up close; skip them (and drop octaves on
  // the broad ones) to keep the fragment cost down.
  float strokeA = 0.5, strokeB = 0.0;
  if (near > 0.001) {
    strokeA = fbm3(rot(0.6) * uv * vec2(0.45, 2.0));
    strokeB = fbm3(rot(-0.9) * uv * vec2(1.7, 0.55) + 31.0);
  }
  float strokeC = fbm3(rot(0.2) * uv * 0.09 + 7.0);
  float strokeFar = fbm2(rot(-0.4) * uv * vec2(0.006, 0.02) + 3.0);
  float brush = near > 0.001 ? mix(strokeA, strokeB, 0.5) : 0.5;

  vec3 albedo = vCol.rgb;
  albedo *= 1.0 + (brush - 0.5) * 0.6 * near;
  albedo *= 1.0 + (strokeC - 0.5) * 0.35;
  albedo *= 1.0 + (strokeFar - 0.5) * 0.4;
  albedo = mix(albedo, albedo * vec3(1.14, 1.06, 0.78),
               smoothstep(0.55, 0.8, strokeB) * 0.55 * near);

  // Mid-range grass hint: where tufts thin out, scatter small inked tuft
  // shapes on the ground (a cheap decal, no geometry). Invisible up close
  // (real grass) and far away (not worth it).
  float cover = (1.0 - smoothstep(0.10, 0.22, 1.0 - n.y)) *
                (1.0 - smoothstep(300.0, 420.0, wPos.y));
  float marks = smoothstep(45.0, 100.0, d) * (1.0 - smoothstep(500.0, 800.0, d)) * cover;
  if (marks > 0.001) {
    vec2 cell = wPos.xz / 5.0;
    vec2 id = floor(cell);
    vec2 q = fract(cell) - 0.5 - (vec2(hash(id + 3.7), hash(id + 9.1)) - 0.5) * 0.5;
    float u = q.x / 0.7 + 0.5;
    float v = (q.y + 0.3) / 0.7;
    float s = abs(fract(u * 3.0) - 0.5) * 2.0;
    float top = 0.3 + 0.7 * (1.0 - s) * (1.0 - 0.6 * pow(abs(2.0 * u - 1.0), 2.0));
    float inside = step(0.0, u) * step(u, 1.0) * step(0.0, v) * step(v, top);
    float ink = smoothstep(0.0, 0.14, min(top - v, min(u, 1.0 - u) * 1.4));
    float on = inside * step(hash(id), 0.7);
    vec3 fill = albedo * vec3(1.2, 1.2, 0.8);
    albedo = mix(albedo, mix(albedo * 0.38, fill, ink), on * marks);
  }

  float lit = dot(n, sunDir) * 0.5 + 0.5;
  lit += (brush - 0.5) * 0.3 * near;
  float shade = smoothstep(0.34, 0.42, lit) * 0.45 + smoothstep(0.56, 0.64, lit) * 0.55;

  vec3 shadowTint = vec3(0.40, 0.44, 0.72);
  vec3 lightTint = vec3(1.08, 1.0, 0.84);
  vec3 col = albedo * mix(shadowTint, lightTint, shade);

  float fog = 1.0 - exp(-pow(d * 0.00035, 1.5));
  col = mix(col, fogColor, clamp(fog, 0.0, 1.0));
  finalColor = vec4(col, vCol.a);
}
)glsl";

constexpr const char *kSkyVert = R"glsl(
#version 330
in vec3 vertexPosition;
uniform mat4 mvp;
out vec3 dir;
void main() {
  dir = vertexPosition;
  gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)glsl";

constexpr const char *kSkyFrag = R"glsl(
#version 330
in vec3 dir;
uniform vec3 sunDir;
uniform vec3 fogColor;
uniform float time;
out vec4 finalColor;
)glsl";

constexpr const char *kSkyFragMain = R"glsl(
void main() {
  vec3 v = normalize(dir);
  float y = v.y;

  vec3 zenith = vec3(0.20, 0.44, 0.86);
  vec3 sky = mix(fogColor, zenith, pow(clamp(y, 0.0, 1.0), 0.5));
  sky = mix(sky, fogColor * 0.95, smoothstep(0.0, -0.3, y));

  float sd = max(dot(v, sunDir), 0.0);
  sky += vec3(1.0, 0.86, 0.55) * (pow(sd, 90.0) * 0.8 + pow(sd, 6.0) * 0.14);

  if (y > 0.0) {
    vec2 cuv = v.xz / (y + 0.2) * 0.9 + vec2(time * 0.006, 0.0);
    float c = fbm3(cuv * 1.3);
    float body = smoothstep(0.50, 0.58, c);
    if (body > 0.0) {
      float toSun = fbm3(cuv * 1.3 + normalize(sunDir.xz + 1e-4) * 0.18);
      float light = smoothstep(-0.02, 0.05, c - toSun + 0.02);
      light = floor(light * 3.0 + 0.5) / 3.0;
      vec3 cloud = mix(vec3(0.66, 0.70, 0.90), vec3(1.0, 0.98, 0.94), light);
      sky = mix(sky, cloud, body * smoothstep(0.0, 0.12, y) * 0.95);
    }
  }
  finalColor = vec4(sky, 1.0);
}
)glsl";

// Generalised Kuwahara filter: flattens detail into oil-paint daubs while
// keeping edges, then adds paper grain and a soft vignette.
constexpr const char *kPostVert = R"glsl(
#version 330
in vec3 vertexPosition;
in vec2 vertexTexCoord;
uniform mat4 mvp;
out vec2 fragTexCoord;
void main() {
  fragTexCoord = vertexTexCoord;
  gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)glsl";

constexpr const char *kPostFrag = R"glsl(
#version 330
in vec2 fragTexCoord;
uniform sampler2D texture0;
uniform vec2 texel;
out vec4 finalColor;

const int R = 2;       // 3x3 taps per quadrant, 2 texels apart (same footprint)
const float STEP = 2.0;

float hash(vec2 p) {
  p = fract(p * vec2(123.34, 456.21));
  p += dot(p, p + 45.32);
  return fract(p.x * p.y);
}

void main() {
  vec3 mean[4];
  vec3 sq[4];
  for (int q = 0; q < 4; q++) {
    mean[q] = vec3(0.0);
    sq[q] = vec3(0.0);
    float sx = (q % 2 == 0) ? -1.0 : 1.0;
    float sy = (q < 2) ? -1.0 : 1.0;
    for (int a = 0; a <= R; a++) {
      for (int b = 0; b <= R; b++) {
        vec3 c = texture(texture0, fragTexCoord + vec2(sx * a, sy * b) * STEP * texel).rgb;
        mean[q] += c;
        sq[q] += c * c;
      }
    }
  }

  float n = float((R + 1) * (R + 1));
  float best = 1e9;
  vec3 col = vec3(0.0);
  for (int q = 0; q < 4; q++) {
    vec3 m = mean[q] / n;
    vec3 v = sq[q] / n - m * m;
    float var = v.r + v.g + v.b;
    if (var < best) {
      best = var;
      col = m;
    }
  }

  float luma = dot(col, vec3(0.299, 0.587, 0.114));
  col = mix(vec3(luma), col, 1.12);
  col *= 0.965 + 0.07 * hash(gl_FragCoord.xy);
  vec2 q = fragTexCoord - 0.5;
  col *= 1.0 - dot(q, q) * 0.45;
  finalColor = vec4(col, 1.0);
}
)glsl";

} // namespace shaders
