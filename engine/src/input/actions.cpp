#include "engine/input/actions.hpp"

#include "raylib.h"

namespace engine {

bool isBindingDown(InputBinding binding) {
  switch (binding.device) {
  case InputBinding::Device::Key:
    return IsKeyDown(binding.code);
  case InputBinding::Device::MouseButton:
    return IsMouseButtonDown(binding.code);
  }
  return false;
}

} // namespace engine
