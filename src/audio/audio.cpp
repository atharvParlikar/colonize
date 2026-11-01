#include "audio/audio.hpp"

#include "raylib.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace {

constexpr unsigned kSampleRate = 44100;
constexpr float kTwoPi = 6.2831853f;

// Written by the game thread, read by the audio thread.
std::atomic<float> gThrottle{0.0f}; // 0 off, 1 wingsuit jet, 2 launch burn
std::atomic<float> gWind{0.0f};     // airspeed in m/s
std::atomic<float> gRpm{0.0f};      // engine speed, 0..12000
std::atomic<float> gTurbo{1.0f};    // forward-jet thrust multiplier, 1..3
std::atomic<float> gHover{0.0f};    // ground speed / top speed while hovering, -1 when airborne
std::atomic<float> gMuffle{0.0f};   // 1 when on the ground (wind is quieter)
std::atomic<float> gDim{0.0f};      // 0..1 greyout / blackout / redout: the world goes muffled and quiet
std::atomic<float> gHeart{0.0f};    // 0..1 heartbeat loudness and pace, from the g dose

AudioStream gStream{};
bool gReady = false;

struct Voice {
  uint32_t rng = 0x12345678u;
  float spool = 0.0f;    // engine speed, lags the throttle
  float throttle = 0.0f; // smoothed command
  float wind = 0.0f;
  float phase[3] = {0, 0, 0};
  float phaseSub = 0, phaseWhine = 0, phasePulse = 0, phaseThump = 0;
  float svfLow = 0, svfBand = 0;     // resonant filter on the oscillator stack
  float lpAir = 0, lpWind1 = 0, lpWind2 = 0;
  float thump = 0.0f;                // ignition envelope
  float lastCommand = 0.0f;
  float turbo = 1.0f, lastTurbo = 1.0f;
  float rpm = 0.0f;
  float hover = 0.0f, phaseHover = 0, phaseHover2 = 0, lpHover = 0;
  float pssh = 0.0f, surge = 0.0f;   // blow-off hiss on engage, compressor surge on cut-out
  float phaseSiren = 0, phaseSaw2 = 0, phaseSurge = 0, phaseKick = 0, kick = 0;
  float lpPssh = 0, lpAirT1 = 0, lpAirT2 = 0, lpSurge = 0;
  float dim = 0.0f, heart = 0.0f, lpMaster1 = 0, lpMaster2 = 0, phaseBeat = 0, phaseHeart = 0;
} v;

float noise(uint32_t &s) {
  s ^= s << 13; s ^= s >> 17; s ^= s << 5;
  return static_cast<float>(s) * (2.0f / 4294967296.0f) - 1.0f;
}

float onePole(float cutoffHz) {
  return 1.0f - std::exp(-kTwoPi * cutoffHz / kSampleRate);
}

float wrap(float p) { return p >= 1.0f ? p - 1.0f : p; }

void synth(void *buffer, unsigned frames) {
  float *out = static_cast<float *>(buffer);
  const float command = gThrottle.load(std::memory_order_relaxed);
  const float airspeed = gWind.load(std::memory_order_relaxed);
  const float muffle = gMuffle.load(std::memory_order_relaxed);

  if (command > v.lastCommand + 0.5f) v.thump = 1.0f;
  v.lastCommand = command;

  const float hoverCommand = gHover.load(std::memory_order_relaxed);
  const float rpmCommand = gRpm.load(std::memory_order_relaxed);
  const float turboCommand = gTurbo.load(std::memory_order_relaxed);
  // Turbo engages: wastegate "pssh" and a hard kick. It cuts out: surge flutter.
  if (turboCommand > 1.02f && v.lastTurbo <= 1.02f) { v.pssh = 1.0f; v.kick = 1.0f; }
  if (turboCommand < 1.9f && v.lastTurbo >= 1.9f) v.surge = 1.0f;
  v.lastTurbo = turboCommand;

  const float dimCommand = gDim.load(std::memory_order_relaxed);
  const float heartCommand = gHeart.load(std::memory_order_relaxed);
  const float windTarget = std::min(airspeed / 60.0f, 1.6f);
  const float dt = 1.0f / kSampleRate;

  for (unsigned i = 0; i < frames; ++i) {
    const float rate = command > v.throttle ? 1.0f / 0.2f : 1.0f / 0.7f;
    v.throttle += std::clamp(command - v.throttle, -rate * dt, rate * dt);
    v.rpm += (rpmCommand - v.rpm) * (1.0f / (0.05f * kSampleRate));
    v.spool = std::clamp(v.rpm / 6000.0f, 0.0f, 1.0f); // turbo threshold = full "base" spool
    v.wind += (windTarget - v.wind) * (1.0f / (0.15f * kSampleRate));
    v.thump *= 1.0f - 5.0f * dt;

    v.turbo += (turboCommand - v.turbo) * (1.0f / (0.08f * kSampleRate));
    v.hover += (hoverCommand - v.hover) * (1.0f / (0.25f * kSampleRate));
    v.pssh *= 1.0f - 4.5f * dt;
    v.surge *= 1.0f - 2.2f * dt;
    v.kick *= 1.0f - 7.0f * dt;
    const float u = std::clamp(v.turbo - 1.0f, 0.0f, 2.0f); // 0..2 turbo depth

    const float n = noise(v.rng);
    // Idle is a quiet whirr; level climbs with RPM up to the turbo threshold.
    const float level = v.rpm < 50.0f ? 0.0f
                                      : 0.2f + 0.8f * std::clamp((v.rpm - 2500.0f) / 3500.0f, 0.0f, 1.0f);
    const float boost = std::clamp(v.throttle - 1.0f, 0.0f, 1.0f);

    float jet = 0.0f;
    if (level > 0.002f) {
      // Industrial core: detuned saws plus a sub, through a resonant filter
      // that opens as the drive "charges".
      const float f0 = (46.0f + 62.0f * v.spool + 26.0f * boost) * (1.0f + 0.3f * u);
      v.phase[0] = wrap(v.phase[0] + f0 * dt);
      v.phase[1] = wrap(v.phase[1] + f0 * 1.006f * dt);
      v.phase[2] = wrap(v.phase[2] + f0 * 0.497f * dt);
      v.phaseSub = wrap(v.phaseSub + f0 * 0.5f * dt);
      const float saws = (2.0f * v.phase[0] - 1.0f) + (2.0f * v.phase[1] - 1.0f) +
                         0.8f * (2.0f * v.phase[2] - 1.0f);
      const float cutoff = 140.0f + 900.0f * v.spool + 700.0f * boost + 1500.0f * u;
      const float g = 2.0f * std::sin(3.14159265f * cutoff / kSampleRate);
      const float damp = 0.45f; // resonance
      v.svfLow += g * v.svfBand;
      const float high = saws - v.svfLow - damp * v.svfBand;
      v.svfBand += g * high;
      const float sub = std::sin(kTwoPi * v.phaseSub);

      // Slow industrial throb on the core.
      v.phasePulse = wrap(v.phasePulse + (7.0f + 5.0f * v.spool + 9.0f * u) * dt);
      const float throb = 0.82f + 0.18f * std::sin(kTwoPi * v.phasePulse);

      // Clean rising whine plus a fifth above, lightly tremolo'd: the sci-fi part.
      const float whineHz = 520.0f + 1900.0f * v.spool + 500.0f * boost;
      v.phaseWhine = wrap(v.phaseWhine + whineHz * dt);
      const float trem = 0.75f + 0.25f * std::sin(kTwoPi * v.phasePulse * 3.0f);
      const float whine =
          trem * (std::sin(kTwoPi * v.phaseWhine) + 0.3f * std::sin(kTwoPi * v.phaseWhine * 1.5f));

      // Faint air: soft low-passed hiss so the thrust is not pure synth.
      v.lpAir += onePole(1100.0f) * (n - v.lpAir);

      // Turbo layers: an octave-up saw growl, a siren-like turbine sweeping up
      // with boost, and a rushing air bed.
      v.phaseSaw2 = wrap(v.phaseSaw2 + f0 * 2.0f * 1.004f * dt);
      const float growl = 2.0f * v.phaseSaw2 - 1.0f;
      v.phaseSiren = wrap(v.phaseSiren + (700.0f + 1500.0f * u) * dt);
      const float siren = std::sin(kTwoPi * v.phaseSiren) + 0.4f * std::sin(kTwoPi * v.phaseSiren * 2.0f);
      v.lpAirT1 += onePole(1800.0f + 1500.0f * u) * (n - v.lpAirT1);
      v.lpAirT2 += onePole(1800.0f + 1500.0f * u) * (v.lpAirT1 - v.lpAirT2);
      const float uu = std::min(u, 1.0f);
      const float turboLayer = uu * (0.07f * growl + 0.04f * siren) + 0.22f * u * v.lpAirT2;

      jet = level * (throb * (0.5f * v.svfLow + 0.45f * sub) * (0.5f + 0.5f * v.spool) +
                     0.028f * whine * v.spool + 0.14f * v.lpAir * v.spool) +
            boost * 0.22f * v.svfLow + level * turboLayer;
    }

    // Ignition: a descending pitch thump with a short click.
    if (v.thump > 0.003f) {
      v.phaseThump = wrap(v.phaseThump + (50.0f + 220.0f * v.thump * v.thump) * dt);
      jet += v.thump * (0.7f * std::sin(kTwoPi * v.phaseThump) +
                        (v.thump > 0.9f ? 0.15f * n : 0.0f));
    }

    // One-shots: kick thump, high wastegate hiss, and surge flutter.
    if (v.kick > 0.003f) {
      v.phaseKick = wrap(v.phaseKick + (40.0f + 130.0f * v.kick * v.kick) * dt);
      jet += v.kick * 0.9f * std::sin(kTwoPi * v.phaseKick);
    }
    v.lpPssh += onePole(3000.0f) * (n - v.lpPssh);
    if (v.pssh > 0.003f) jet += v.pssh * v.pssh * 0.35f * (n - v.lpPssh);
    if (v.surge > 0.003f) {
      v.phaseSurge = wrap(v.phaseSurge + 17.0f * dt);
      v.lpSurge += onePole(1400.0f) * (n - v.lpSurge);
      const float chop = std::sin(kTwoPi * v.phaseSurge) > 0.2f ? 1.0f : 0.0f;
      jet += v.surge * 0.3f * chop * v.lpSurge;
    }

    // Hover drive: a smooth anti-grav hum that rises in pitch and sheen with speed,
    // plus a soft ground-effect hiss.
    if (v.hover > 0.001f) {
      const float sp = std::clamp(v.hover, 0.0f, 1.0f);
      v.phaseHover = wrap(v.phaseHover + (62.0f + 90.0f * sp) * dt);
      v.phaseHover2 = wrap(v.phaseHover2 + (124.5f + 181.0f * sp) * dt);
      v.lpHover += onePole(500.0f + 1800.0f * sp) * (n - v.lpHover);
      const float hum = std::sin(kTwoPi * v.phaseHover) + 0.5f * std::sin(kTwoPi * v.phaseHover2) *
                                                              (0.6f + 0.4f * std::sin(kTwoPi * v.phaseHover * 0.05f));
      jet += v.hover * (0.22f * hum + (0.08f + 0.3f * sp) * v.lpHover);
    }

    // Wind: soft, mostly low rush that brightens with speed.
    const float windCut = 220.0f + 1500.0f * std::min(v.wind, 1.5f) / 1.5f;
    v.lpWind1 += onePole(windCut) * (n - v.lpWind1);
    v.lpWind2 += onePole(windCut) * (v.lpWind1 - v.lpWind2);
    const float windGain = v.wind * v.wind * (1.0f - 0.6f * muffle) * 1.8f;

    float mix = std::tanh(0.4f * jet + v.lpWind2 * windGain) * 0.6f;

    // Under load the world goes muffled and quiet (a two-pole low-pass that
    // closes from ~12 kHz to ~300 Hz, and a duck), and a heartbeat comes up
    // under it, faster as the dose builds.
    v.dim += (dimCommand - v.dim) * (1.0f / (0.1f * kSampleRate));
    v.heart += (heartCommand - v.heart) * (1.0f / (0.3f * kSampleRate));
    const float cut = 12000.0f * std::pow(300.0f / 12000.0f, v.dim);
    v.lpMaster1 += onePole(cut) * (mix - v.lpMaster1);
    v.lpMaster2 += onePole(cut) * (v.lpMaster1 - v.lpMaster2);
    mix = v.lpMaster2 * (1.0f - 0.65f * v.dim);
    if (v.heart > 0.002f) {
      v.phaseBeat += (1.1f + 1.2f * v.heart) * dt; // beats per second
      if (v.phaseBeat >= 1.0f) v.phaseBeat -= 1.0f;
      // Lub at 0, dub at 0.28 of the beat: short decaying low thumps.
      auto thumpAt = [&](float at, float gain) {
        const float since = v.phaseBeat - at;
        return since >= 0.0f && since < 0.25f ? gain * std::exp(-since * 28.0f) : 0.0f;
      };
      const float env = thumpAt(0.0f, 1.0f) + thumpAt(0.28f, 0.7f);
      v.phaseHeart = wrap(v.phaseHeart + 52.0f * dt);
      mix += v.heart * 0.5f * env * std::sin(kTwoPi * v.phaseHeart);
    }
    out[i] = mix;
  }
}

} // namespace

void initAudio() {
  InitAudioDevice();
  if (!IsAudioDeviceReady()) return;
  SetAudioStreamBufferSizeDefault(2048);
  gStream = LoadAudioStream(kSampleRate, 32, 1);
  SetAudioStreamCallback(gStream, synth);
  PlayAudioStream(gStream);
  gReady = true;
}

void updateAudio(const Player &player) {
  if (!gReady) return;
  const GLoad &load = player.load;
  gDim.store(std::max({0.5f * load.greyout, load.blackout, load.redout}), std::memory_order_relaxed);
  gHeart.store(std::clamp(std::max(load.dose / tuning.tunnelDose, load.negDose / tuning.redoutDose), 0.0f, 1.0f),
               std::memory_order_relaxed);
  gThrottle.store(player.engineRpm > 1.0f ? 1.0f : 0.0f, std::memory_order_relaxed);
  gRpm.store(player.engineRpm, std::memory_order_relaxed);
  gTurbo.store(1.0f, std::memory_order_relaxed);
  gWind.store(Vector3Length(player.velocity), std::memory_order_relaxed);
  gHover.store(player.engineRpm > 1.0f ? 0.7f * (1.0f - player.convert) : 0.0f, std::memory_order_relaxed);
  gMuffle.store(0.0f, std::memory_order_relaxed);
}

void shutdownAudio() {
  if (!gReady) return;
  StopAudioStream(gStream);
  UnloadAudioStream(gStream);
  CloseAudioDevice();
  gReady = false;
}
