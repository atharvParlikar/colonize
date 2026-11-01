#include "engine/collision/capsule.hpp"

#include "raymath.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

Vector3 closestPointAabb(Vector3 point, BoundingBox box) {
  return {std::clamp(point.x, box.min.x, box.max.x),
          std::clamp(point.y, box.min.y, box.max.y),
          std::clamp(point.z, box.min.z, box.max.z)};
}

void closestPointsSegmentAabb(Vector3 segmentStart, Vector3 segmentEnd,
                              BoundingBox box, Vector3 &segmentPoint,
                              Vector3 &boxPoint) {
  const Vector3 segment = Vector3Subtract(segmentEnd, segmentStart);
  float t = 0.0f;

  for (int i = 0; i < 2; ++i) {
    const Vector3 pointOnSegment =
        Vector3Add(segmentStart, Vector3Scale(segment, t));
    const Vector3 pointOnBox = closestPointAabb(pointOnSegment, box);
    const Vector3 delta = Vector3Subtract(pointOnBox, pointOnSegment);

    const float segmentLengthSq = Vector3DotProduct(segment, segment);
    if (segmentLengthSq <= 1e-8f) {
      break;
    }

    t += Vector3DotProduct(delta, segment) / segmentLengthSq;
    t = std::clamp(t, 0.0f, 1.0f);
  }

  segmentPoint = Vector3Add(segmentStart, Vector3Scale(segment, t));
  boxPoint = closestPointAabb(segmentPoint, box);
}

float distanceSegmentAabb(Vector3 segmentStart, Vector3 segmentEnd,
                          BoundingBox box, Vector3 &segmentPoint,
                          Vector3 &boxPoint) {
  closestPointsSegmentAabb(segmentStart, segmentEnd, box, segmentPoint,
                           boxPoint);
  return Vector3Distance(segmentPoint, boxPoint);
}

CollisionInfo minimumAxisPenetration(const Capsule &capsule, BoundingBox box) {
  const Vector3 capsuleMin{
      std::min(capsule.start.x, capsule.end.x) - capsule.radius,
      std::min(capsule.start.y, capsule.end.y) - capsule.radius,
      std::min(capsule.start.z, capsule.end.z) - capsule.radius};
  const Vector3 capsuleMax{
      std::max(capsule.start.x, capsule.end.x) + capsule.radius,
      std::max(capsule.start.y, capsule.end.y) + capsule.radius,
      std::max(capsule.start.z, capsule.end.z) + capsule.radius};

  const float overlaps[3] = {
      std::min(capsuleMax.x, box.max.x) - std::max(capsuleMin.x, box.min.x),
      std::min(capsuleMax.y, box.max.y) - std::max(capsuleMin.y, box.min.y),
      std::min(capsuleMax.z, box.max.z) - std::max(capsuleMin.z, box.min.z),
  };

  if (overlaps[0] <= 0.0f || overlaps[1] <= 0.0f || overlaps[2] <= 0.0f) {
    return {};
  }

  float minOverlap = overlaps[0];
  int axis = 0;
  for (int i = 1; i < 3; ++i) {
    if (overlaps[i] < minOverlap) {
      minOverlap = overlaps[i];
      axis = i;
    }
  }

  const float capsuleCenter[3] = {
      (capsuleMin.x + capsuleMax.x) * 0.5f,
      (capsuleMin.y + capsuleMax.y) * 0.5f,
      (capsuleMin.z + capsuleMax.z) * 0.5f,
  };
  const float boxCenter[3] = {
      (box.min.x + box.max.x) * 0.5f,
      (box.min.y + box.max.y) * 0.5f,
      (box.min.z + box.max.z) * 0.5f,
  };

  Vector3 normal{0.0f, 0.0f, 0.0f};
  if (axis == 0) {
    normal.x = (capsuleCenter[0] >= boxCenter[0]) ? 1.0f : -1.0f;
  } else if (axis == 1) {
    normal.y = (capsuleCenter[1] >= boxCenter[1]) ? 1.0f : -1.0f;
  } else {
    normal.z = (capsuleCenter[2] >= boxCenter[2]) ? 1.0f : -1.0f;
  }

  return {normal, minOverlap, true};
}

void applyPenetrationResolution(const CollisionInfo &info, Capsule &capsule,
                                Vector3 &anchor) {
  if (!info.hit || info.depth <= 0.0f) {
    return;
  }

  const Vector3 offset = Vector3Scale(info.normal, info.depth);
  capsule.start = Vector3Add(capsule.start, offset);
  capsule.end = Vector3Add(capsule.end, offset);
  anchor = Vector3Add(anchor, offset);
}

} // namespace

bool intersectsCapsuleAabb(const Capsule &capsule, BoundingBox box) {
  Vector3 segmentPoint{};
  Vector3 boxPoint{};
  const float distance =
      distanceSegmentAabb(capsule.start, capsule.end, box, segmentPoint,
                          boxPoint);
  return distance < capsule.radius;
}

CollisionInfo computeCapsuleAabbPenetration(const Capsule &capsule,
                                            BoundingBox box) {
  Vector3 segmentPoint{};
  Vector3 boxPoint{};
  const float distance =
      distanceSegmentAabb(capsule.start, capsule.end, box, segmentPoint,
                          boxPoint);

  if (distance >= capsule.radius) {
    return {};
  }

  if (distance > 1e-6f) {
    const Vector3 normal =
        Vector3Normalize(Vector3Subtract(segmentPoint, boxPoint));
    return {normal, capsule.radius - distance, true};
  }

  return minimumAxisPenetration(capsule, box);
}

void resolveCapsuleCollisions(Capsule &capsule, Vector3 &anchor,
                              std::span<const BoundingBox> colliders) {
  constexpr int kIterations = 3;
  const int axes[3] = {0, 1, 2};

  for (int iteration = 0; iteration < kIterations; ++iteration) {
    for (const int axis : axes) {
      for (const BoundingBox &collider : colliders) {
        const CollisionInfo info = computeCapsuleAabbPenetration(capsule,
                                                                 collider);
        if (!info.hit) {
          continue;
        }

        const float axisNormal = (axis == 0)   ? std::fabs(info.normal.x)
                                 : (axis == 1) ? std::fabs(info.normal.y)
                                               : std::fabs(info.normal.z);
        if (axisNormal < 0.5f) {
          continue;
        }

        applyPenetrationResolution(info, capsule, anchor);
      }
    }
  }

  for (const BoundingBox &collider : colliders) {
    const CollisionInfo info = computeCapsuleAabbPenetration(capsule, collider);
    applyPenetrationResolution(info, capsule, anchor);
  }
}

bool isCapsuleGrounded(const Capsule &capsule,
                       std::span<const BoundingBox> colliders, float tolerance) {
  Capsule probe = capsule;
  probe.start.y -= tolerance;
  probe.end.y -= tolerance;

  for (const BoundingBox &collider : colliders) {
    if (intersectsCapsuleAabb(probe, collider)) {
      return true;
    }
  }

  return false;
}

} // namespace engine
