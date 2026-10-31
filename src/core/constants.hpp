#pragma once

#include "raylib.h"
#include "raymath.h"

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

// Atmosphere and the fixed wing
constexpr float kAirDensitySeaLevel = 1.225f; // kg/m^3
constexpr float kAirScaleHeight = 8500.0f;    // m
constexpr float kWingArea = 10.0f;            // m^2
constexpr float kWingSpan = 10.0f;            // m
constexpr float kWingChord = kWingArea / kWingSpan;
constexpr float kAspectRatio = kWingSpan * kWingSpan / kWingArea;
constexpr float kLiftSlope = 5.0f;            // dCL/dalpha per radian, attached
constexpr float kStallAngleDeg = 20.0f;
constexpr float kStallBlendDeg = 8.0f;
constexpr float kProfileDrag = 0.03f;         // CD0 of the wing (the hull has its own drag)
constexpr float kOswaldEfficiency = 0.8f;
constexpr float kFlatPlateDrag = 1.4f;        // extra CD at 90 deg
constexpr float kFlatPlateLift = 0.9f;
constexpr float kSideForceSlope = 0.5f;
constexpr float kFallingDragArea = 0.55f;     // CdA of an unfolded body, m^2

// Airframe: one vectored-thrust engine, a fixed wing and a fly-by-wire
// attitude loop. The nozzle swings from straight down (hover) to straight aft
// (flight); the wing takes the weight over as the speed builds.
constexpr float kAirframeMass = 600.0f;       // kg, craft + pilot
constexpr float kAirframeWeight = kAirframeMass * kGravity;
constexpr float kHullDragForward = 0.4f;      // CdA of the hull along each body axis, m^2
constexpr float kHullDragSide = 2.0f;
constexpr float kHullDragUp = 3.0f;
constexpr float kEngineMaxThrust = 14000.0f;  // N, about 2.4 g; hovering takes ~42%
constexpr float kEngineSpoolTau = 0.15f;      // s, thrust lag behind its command
constexpr float kNozzleRate = 2.5f;           // rad/s the nozzle can swing
constexpr float kNozzleHoverDeg = 12.0f;      // nozzle travel either side of vertical while hovering
constexpr float kNozzleAftDeg = 90.0f;        // fully aft, in flight
constexpr float kNozzleBrakeDeg = -30.0f;     // tipped forward, to brake in flight
constexpr float kConvertStart = 6.0f;         // m/s, the nozzle's flight travel opens up from here
constexpr float kConvertEnd = 30.0f;

// Attitude loop: the pilot law asks for an attitude, this flies the body to it.
constexpr float kAttitudeGain = 5.0f;         // 1/s, attitude error -> rate
constexpr float kAttitudeTau = 0.08f;         // s, body rate lag behind its command
constexpr float kMaxRollRate = 3.5f;          // rad/s
constexpr float kMaxPitchRate = 1.8f;
constexpr float kMaxYawRate = 1.8f;
constexpr float kRollAccel = 14.0f;           // rad/s^2 the rate shaping plans to brake with
constexpr float kPitchAccel = 7.0f;
constexpr float kYawAccel = 6.0f;
constexpr float kBrakeMargin = 0.5f;          // share of that the shaping actually plans on

// Speed: W / S move a speed command along the aim. Letting go holds the
// speed the craft has, or brings it to a hover below kCruiseLatch.
constexpr float kMaxSpeed = 150.0f;           // m/s
constexpr float kReverseSpeed = 8.0f;         // S from a hover backs up this fast
constexpr float kCruiseLatch = 35.0f;         // m/s
constexpr float kThrottleRateHover = 10.0f;   // m/s^2 of speed command while W / S are held
constexpr float kThrottleRateFlight = 25.0f;
constexpr float kSpeedLead = 25.0f;           // m/s the command may run ahead of the real speed while W is held
// S is a hard brake: the command drops fast and far enough below the speed
// that the law asks for the full kFlightMaxBrake (airbrake, then nozzle).
constexpr float kBrakeRate = 250.0f;          // m/s^2 of speed command while S is held
constexpr float kBrakeLead = 60.0f;           // m/s the command may run below the real speed while S is held

// Hover (low speed): velocity command, tilt to get it.
constexpr float kStrafeSpeed = 14.0f;         // m/s, A / D
constexpr float kClimbSpeed = 8.0f;           // m/s, Space
constexpr float kDescendSpeed = 6.0f;         // m/s, Shift
constexpr float kHoverVelGain = 1.6f;         // 1/s, horizontal velocity error -> acceleration
constexpr float kHoverClimbGain = 2.0f;       // 1/s, vertical
constexpr float kHoverMaxAccel = 8.0f;        // m/s^2 horizontal
constexpr float kHoverMaxBrake = 20.0f;       // m/s^2 horizontal when slowing down (airbrake + tilt + nozzle)
constexpr float kHoverMaxTiltDeg = 30.0f;
constexpr float kAimMaxElevationDeg = 85.0f;  // the hover aim stops short of straight up / down

// Flight (wing-borne): the flight path is turned onto the aim point.
constexpr float kWingSpeedLo = 25.0f;         // m/s, flight law starts blending in
constexpr float kWingSpeedHi = 45.0f;         // m/s, fully wing-borne
constexpr float kAimSensitivity = 0.0025f;    // rad of aim point per mouse count
constexpr float kAimMaxAngleDeg = 60.0f;      // flying: how far the aim point may lead the flight path
constexpr float kAimReturnTau = 0.3f;         // s, flying: the aim point eases back onto the path (a spring-loaded stick)
constexpr float kAimHoldDeg = 1.0f;           // lead that does not spring back (holds the path on release)
constexpr float kAimHoldElevationDeg = 3.0f;  // the same for the elevation offset, wider so turns hold their altitude
// Arcade turning: no g limit, the path turns as hard as the wing can pull
// (a 180 at 450 km/h takes about two seconds), up to kMaxTurnRateDeg.
constexpr float kPathGain = 3.0f;             // 1/s, path-to-aim angle -> turn rate
constexpr float kPathAccel = 5.0f;            // rad/s^2, how fast the turn rate may build / ease
constexpr float kMaxTurnRateDeg = 100.0f;     // deg/s the flight path may turn
constexpr float kTurnAlphaDeg = 12.0f;        // angle of attack the turn plans on (headroom below kMaxAlphaDeg)
constexpr float kMaxPushG = 2.0f;             // negative-g limit when pushing
constexpr float kPushPreferDeg = 45.0f;       // rolling past knife edge to pull (loop top) only when it needs this much less roll
constexpr float kFlatTurnDeg = 15.0f;         // sideways aim error below which turns are flown wings level (skid), above it banked
constexpr float kMaxSkidDeg = 6.0f;           // how far the nose may yaw off the path in a flat turn
constexpr float kSpeedGain = 0.8f;            // 1/s, speed error -> acceleration along the path
constexpr float kFlightMaxAccel = 14.0f;      // m/s^2 along the path
constexpr float kFlightMaxBrake = 40.0f;      // m/s^2, about 4 g
constexpr float kAirbrakeArea = 20.0f;        // m^2 of extra drag area, fully deployed (arcade-big: full brake down to 45 m/s)
constexpr float kMaxAlphaDeg = 15.0f;         // angle of attack kept clear of the 20 deg stall
constexpr float kMinAlphaDeg = -8.0f;
constexpr float kManualRollRate = 3.0f;       // rad/s, A / D at speed

// Engine sound / gauge: rpm follows the spooled thrust; it idles on the
// ground and shuts off after kIdleTimeout at rest.
constexpr float kIdleRpm = 2500.0f;
constexpr float kTurboStartRpm = 6000.0f;     // gauge's hot zone
constexpr float kRedlineRpm = 12000.0f;
constexpr float kIdleTimeout = 6.0f;          // s

// Landing
constexpr float kCrashSpeed = 10.0f;          // m/s of velocity lost to impact
constexpr float kCrashMessageTime = 3.0f;
