#include "collision/capsule.hpp"

Vector3 playerFeetPosition(const Player &player) {
  return {player.position.x, player.position.y - kEyeHeight, player.position.z};
}

engine::Capsule makePlayerCapsuleAtEyePosition(const Vector3 eyePosition) {
  const float feetY = eyePosition.y - kEyeHeight;
  return {{eyePosition.x, feetY + kPlayerRadius, eyePosition.z},
          {eyePosition.x, feetY + kPlayerHeight - kPlayerRadius, eyePosition.z},
          kPlayerRadius};
}

engine::Capsule makePlayerCapsule(const Player &player) {
  return makePlayerCapsuleAtEyePosition(player.position);
}
