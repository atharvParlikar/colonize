#include "player/input.hpp"

void samplePlayerInput(const Controls &controls, PlayerInput &input) {
  input.forward = controls.down(Action::Forward);
  input.back = controls.down(Action::Back);
  input.left = controls.down(Action::Left);
  input.right = controls.down(Action::Right);
  input.ascend = controls.down(Action::Ascend);
  input.descend = controls.down(Action::Descend);
  input.fire = controls.down(Action::Fire);
  // Held until a physics step consumes it.
  input.convert = input.convert || controls.pressed(Action::Convert);
}
