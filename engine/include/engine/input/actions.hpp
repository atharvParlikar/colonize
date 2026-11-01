#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace engine {

// One physical control an action can be bound to. Codes are raylib's
// (KEY_W, MOUSE_BUTTON_LEFT, ...).
struct InputBinding {
  enum class Device { Key, MouseButton };
  Device device;
  int code;

  static constexpr InputBinding key(int code) { return {Device::Key, code}; }
  static constexpr InputBinding mouse(int button) { return {Device::MouseButton, button}; }
};

// Is this binding held right now?
bool isBindingDown(InputBinding binding);

// Named game actions ("Forward", "Fire") instead of raw keys. The game defines
// an enum ending in `Count`, binds each action to one or more controls, and
// reads the actions; which key does what is then one table, not scattered
// IsKeyDown calls.
//
// Call update() once per frame before reading. An action is down while any of
// its bindings is down; pressed / released are the frames it changes.
template <typename Action>
class ActionMap {
public:
  static constexpr std::size_t kCount = static_cast<std::size_t>(Action::Count);

  void bind(Action action, InputBinding binding) { bindings_[index(action)].push_back(binding); }
  void unbind(Action action) { bindings_[index(action)].clear(); }

  void update() {
    for (std::size_t i = 0; i < kCount; ++i) {
      bool now = false;
      for (const InputBinding &b : bindings_[i]) now = now || isBindingDown(b);
      previous_[i] = current_[i];
      current_[i] = now;
    }
  }

  bool down(Action action) const { return current_[index(action)]; }
  bool pressed(Action action) const { return current_[index(action)] && !previous_[index(action)]; }
  bool released(Action action) const { return !current_[index(action)] && previous_[index(action)]; }

private:
  static constexpr std::size_t index(Action action) { return static_cast<std::size_t>(action); }

  std::array<std::vector<InputBinding>, kCount> bindings_{};
  std::array<bool, kCount> current_{};
  std::array<bool, kCount> previous_{};
};

} // namespace engine
