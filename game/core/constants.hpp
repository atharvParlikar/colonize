#pragma once

#include "raylib.h"
#include "raymath.h"
#include "core/tuning.hpp" // live flight tuning: hover, conversion, jet and g numbers

// Units: 1 world unit = 1 metre, SI everywhere (kg, N, m/s).
constexpr int kScreenWidth = 1280;
constexpr int kScreenHeight = 720;
constexpr Vector3 kWorldUp = {0.0f, 1.0f, 0.0f};

// Physics runs on a fixed step so aerodynamics stays stable at any frame rate.
constexpr float kPhysicsDt = 1.0f / 120.0f;

// Body
constexpr float kPlayerRadius = 0.35f;
constexpr float kPlayerHeight = 1.8f;
constexpr float kEyeHeight = 1.65f;
constexpr float kPlayerMass = 80.0f; // pilot only; the craft's mass is kAirframeMass
constexpr float kGravity = 9.81f;
constexpr float kPlayerWeight = kPlayerMass * kGravity;

// Ground contact
constexpr float kLandingEps = 0.01f; // collision resolution leaves us exactly on the surface
constexpr float kGroundSnap = 0.35f;
constexpr float kStoneFriction = 0.8f;
constexpr float kSkidFriction = 0.6f;  // skids on the ground, times g, as far as the engine is not carrying the weight

// ---- Hover: a game, not a helicopter ----------------------------------------
// Kinematic: WASD asks for a velocity relative to where you look, Space / Shift
// climb and descend, and the velocity gets there on a tuned ramp. The body
// tilt you see is cosmetic. Tuning numbers: core/tuning.hpp (live, Tab panel).

// ---- Conversion ------------------------------------------------------------
// F arms it; it starts once the speed gate allows (numbers in core/tuning.hpp).

// ---- Jet: kinematic, with momentum -----------------------------------------
// The mouse moves a spring-loaded stick: X turns (the jet banks into it on its
// own), Y pitches; A / D roll by hand. Speed is one number: throttle against
// drag, gravity along the path, and the bleed a hard turn costs. The flight
// path follows the nose with a short lag. Below the stall speed the wing stops
// holding the nose up.
constexpr float kJetMinSpeed = 5.0f;

// ---- g -----------------------------------------------------------------------

// Engine sound / gauge: rpm follows the spooled thrust; it idles on the
// ground and shuts off after kIdleTimeout at rest.
constexpr float kIdleRpm = 2500.0f;
constexpr float kTurboStartRpm = 6000.0f;     // gauge's hot zone
constexpr float kRedlineRpm = 12000.0f;
constexpr float kIdleTimeout = 6.0f;          // s

// ---- Combat ----------------------------------------------------------------
// A nose cannon and a field of target drones around spawn (combat/combat.cpp).
constexpr float kCannonRate = 12.0f;          // rounds per second
constexpr float kMuzzleSpeed = 700.0f;        // m/s, on top of the craft's velocity
constexpr float kRoundLifetime = 2.5f;        // s
constexpr float kGunConvergence = 400.0f;     // m: rounds cross the aim line here
constexpr float kDroneRadius = 1.6f;          // m
constexpr int kDroneCount = 24;
constexpr int kDroneHealth = 3;               // rounds to break one
constexpr float kDroneRespawnTime = 8.0f;     // s
constexpr float kSparkLifetime = 1.2f;        // s

// Landing
constexpr float kCrashSpeed = 10.0f;          // m/s of velocity lost to impact
constexpr float kCrashMessageTime = 3.0f;
