#pragma once

#include "raylib.h"
#include "raymath.h"

// Units: 1 world unit = 1 metre, SI everywhere (kg, N, m/s).
constexpr int kScreenWidth = 1280;
constexpr int kScreenHeight = 720;
constexpr float kMouseSensitivity = 0.1f;
constexpr float kMaxPitch = 89.0f;
constexpr float kMinPitch = -89.0f;
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

// Atmosphere and rigid deployable wings
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

// Deployment
constexpr float kAutoDeploySpeed = 18.0f;
constexpr float kManualDeploySpeed = 10.0f;

// Wing envelope the flight law keeps the craft inside.
constexpr float kMinAlphaDeg = -10.0f;        // controller keeps the wing unstalled
constexpr float kMaxAlphaDeg = 17.0f;
constexpr float kControlRefSpeed = 25.0f;     // control authority ~ (V/Vref)^2
constexpr float kDeployBlendTime = 0.5f;      // s, camera easing into the wings

// Forward jet
constexpr float kMaxJetpackFuel = 1.0f;
constexpr float kJetThrust = 825.0f;          // ~1.05 x body weight
constexpr float kFuelBurnRate = 0.04f;
constexpr float kLiftThrust = 1100.0f;        // Shift: straight up, ~1.4 x body weight
constexpr float kLiftFuelBurnRate = 0.06f;
// Engine RPM and turbo: once lit the engine idles; holding W spools it up at
// a steady rate to the turbo threshold, where thrust ramps 1x -> 2x, then it
// keeps easing toward the redline where thrust reaches 3x. Thrust follows RPM,
// so releasing W spools the power back down.
constexpr float kIdleRpm = 2500.0f;
constexpr float kLowThrottleRpm = 5500.0f;       // Shift / S hold just under the turbo
constexpr float kTurboStartRpm = 6000.0f;        // 1x thrust until here
constexpr float kTurboKickRpm = 8000.0f;         // 2x here (linear from the start)
constexpr float kRedlineRpm = 12000.0f;          // 3x here
constexpr float kTurboMax = 3.0f;                // thrust multiplier at redline
constexpr float kSpoolUpRate = 3000.0f;          // rpm/s below the turbo threshold
constexpr float kSpoolEaseTime = 2.5f;           // s, time constant above it
constexpr float kSpoolDownRate = 5000.0f;        // rpm/s
constexpr float kIdleTimeout = 6.0f;             // s idling before the engine shuts off
constexpr float kTurboFuelFactor = 0.5f;         // extra burn per extra 1x of thrust
constexpr float kMaxLoadFactor = 5.0f;           // g limit of the pitch loop
constexpr float kBrakeThrust = 1800.0f;       // S: jet flipped against the flight path
constexpr float kBrakeFuelBurnRate = 0.08f;
constexpr float kLaunchThrust = 3000.0f;      // ~3.8 x body weight, wings folded
constexpr float kLaunchFuelBurnRate = 0.125f;
constexpr float kFuelRechargeRate = 0.5f;
constexpr float kFuelRechargeDelay = 3.0f;

// Airframe: a rigid body pushed around by the jets in player/airframe.hpp
constexpr float kAirframeMass = 600.0f;       // kg, craft + pilot
constexpr float kAirframeWeight = kAirframeMass * kGravity;
constexpr float kInertiaRoll = 1900.0f;       // kg m^2, about the forward axis
constexpr float kInertiaPitch = 2600.0f;      // about the right axis
constexpr float kInertiaYaw = 4300.0f;        // about the up axis
constexpr float kHullDragForward = 0.4f;     // CdA of the hull along each body axis, m^2
constexpr float kHullDragSide = 2.0f;
constexpr float kHullDragUp = 3.0f;
// Aerodynamic stability of the airframe (dimensionless coefficients, scaled by q S ref).
constexpr float kCmAlpha = 0.6f;              // pitch stiffness per rad of angle of attack
constexpr float kCmq = 40.0f;                 // pitch damping
constexpr float kCnBeta = 0.1f;               // weathervane stiffness per rad of sideslip
constexpr float kCnr = 0.6f;                  // yaw damping
constexpr float kClp = 0.25f;                 // roll damping
// Control surface authority: torque available is q (Pa) times this (m^3).
constexpr float kSurfaceRoll = 6.0f;
constexpr float kSurfacePitch = 6.0f;
constexpr float kSurfaceYaw = 4.0f;
constexpr float kAngularDamping = 0.6f;       // 1/s, aerodynamic rate damping
constexpr float kJetSpoolTau = 0.12f;         // s, jet thrust lag behind its command
constexpr float kLiftJetMaxThrust = 4000.0f;  // N each; four of them hover at ~37%
constexpr float kMainJetMaxThrust = 2800.0f;  // N each; top speed about 100 m/s
constexpr float kLiftJetArmX = 2.2f;          // m, left/right lift jets from the centre of mass
constexpr float kLiftJetArmZ = 2.5f;          // m, fore/aft lift jets
constexpr float kMainJetArmX = 2.8f;          // m, main jets out on the wings
constexpr float kYawReaction = 1.5f;          // N m of yaw per N of thrust (canted nozzles)

// Hover controller: the pilot commands a velocity, the craft tilts to get it.
constexpr float kHoverMaxSpeed = 20.0f;       // m/s horizontal
constexpr float kClimbSpeed = 6.0f;           // m/s
constexpr float kDescendSpeed = 5.0f;
constexpr float kHoverVelGain = 1.2f;         // 1/s, horizontal velocity error -> acceleration
constexpr float kHoverClimbGain = 2.0f;       // 1/s, vertical
constexpr float kHoverMaxTiltDeg = 35.0f;
constexpr float kHoverTiltGain = 4.0f;        // 1/s, tilt error -> rate command
constexpr float kHoverYawGain = 3.0f;
constexpr float kHoverMaxTiltRate = 2.0f;     // rad/s
constexpr float kHoverMaxYawRate = 1.6f;      // rad/s
constexpr float kHoverRateGain = 6.0f;        // 1/s, rate error -> torque (per unit inertia)
constexpr float kBrakeMargin = 0.5f;          // fraction of torque authority the rate shaping plans to brake with
constexpr float kMaxTiltTorque = 5500.0f;     // N m, roll/pitch demand kept inside the jets' authority
constexpr float kMaxYawTorque = 5000.0f;      // N m, so a hard turn never starves the collective

// Flight control law ("instructor"): the mouse moves an aim point and the loop
// flies the nose to it, rolling the lift toward the target first, then pulling.
constexpr float kLiftFadeStart = 18.0f;       // m/s, lift jets start handing over to the wing
constexpr float kLiftFadeEnd = 34.0f;         // m/s, wing carries everything
constexpr float kAimSensitivity = 0.0025f;    // rad of aim point per mouse count
constexpr float kAimMaxAngleDeg = 100.0f;     // how far the aim point may lead the nose
constexpr float kFlightAttGain = 4.0f;        // 1/s, attitude error -> rate command
constexpr float kFlightRollGain = 6.0f;       // 1/s, roll-to-target error -> roll rate
constexpr float kFlightMaxPitchRate = 2.2f;   // rad/s
constexpr float kFlightMaxRollRate = 4.0f;
constexpr float kFlightMaxYawRate = 1.2f;
constexpr float kFlightPitchAccel = 4.5f;     // rad/s^2 the rate shaping may plan on
constexpr float kFlightRollAccel = 8.0f;
constexpr float kFlightYawAccel = 2.0f;
constexpr float kFlightBetaGain = 2.0f;       // rad/s of yaw per rad of sideslip (coordination)
constexpr float kAlphaMargin = 4.0f;          // deg over which pitch authority fades near the alpha limits
constexpr float kTrimGain = 1.2f;             // 1/s, integrates "circle above flight path" into nose trim
constexpr float kTrimLead = 0.6f;             // proportional share of the same error
constexpr float kMaxTrimDeg = 18.0f;
constexpr float kSagGain = 1.2f;              // aim higher by this share of how far the path sags below the circle
constexpr float kBrakeReverse = 0.5f;         // S: reverse thrust, in multiples of the 1x main-jet rating
constexpr float kGroundFriction = 2.5f;       // m/s^2, skids dragging along the ground in flight mode

// Landing
constexpr float kCrashSpeed = 10.0f;          // m/s of velocity lost to impact
constexpr float kCrashMessageTime = 3.0f;
