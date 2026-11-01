#include "player/input.hpp"

void samplePlayerInput(PlayerInput &input) {
  input.forward = IsKeyDown(KEY_W);
  input.back = IsKeyDown(KEY_S);
  input.left = IsKeyDown(KEY_A);
  input.right = IsKeyDown(KEY_D);
  input.ascend = IsKeyDown(KEY_SPACE);
  input.descend = IsKeyDown(KEY_LEFT_SHIFT);
  input.convert = input.convert || IsKeyPressed(KEY_F); // held until a physics step consumes it
}
