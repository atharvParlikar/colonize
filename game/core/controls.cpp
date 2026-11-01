#include "core/controls.hpp"

#include "raylib.h"

namespace {

using engine::InputBinding;

struct DefaultBinding {
  Action action;
  InputBinding binding;
};

constexpr DefaultBinding kDefaults[] = {
    {Action::Forward, InputBinding::key(KEY_W)},
    {Action::Back, InputBinding::key(KEY_S)},
    {Action::Left, InputBinding::key(KEY_A)},
    {Action::Right, InputBinding::key(KEY_D)},
    {Action::Ascend, InputBinding::key(KEY_SPACE)},
    {Action::Descend, InputBinding::key(KEY_LEFT_SHIFT)},
    {Action::Convert, InputBinding::key(KEY_F)},
    {Action::Fire, InputBinding::mouse(MOUSE_BUTTON_LEFT)},
    {Action::TuningPanel, InputBinding::key(KEY_TAB)},
};

} // namespace

void bindDefaultControls(Controls &controls) {
  for (const DefaultBinding &d : kDefaults) controls.bind(d.action, d.binding);
}
