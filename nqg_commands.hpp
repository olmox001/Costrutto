/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2026 olmox001
 *
 * nqg_commands.hpp — Abstract command / input state.
 * Zero dependency on SDL or any windowing system.
 * Adapters (SDL keyboard/mouse/touch, future gamepad, network, AI…)
 * write into a Commands instance each frame.
 */
#pragma once

#include <cstdint>

namespace nqg {
namespace cmd {

// ---------------------------------------------------------------------------
// Continuous axes & one-shot actions used by the capsule observer / game.
// ---------------------------------------------------------------------------
// Transport-agnostic control state (CLI | SDL | iOS touch | native).
// Axes are sticky until cleared; edge flags are one-shot per step batch.
struct Commands {
  // Movement axes in local capsule frame: X = right, Y = forward
  // Range typically [-1, +1]. Analog (touch stick) or digital (keys).
  float moveX = 0.0f; // +right / -left
  float moveY = 0.0f; // +forward / -back

  // Look deltas (radians-ish, scaled by adapter sensitivity)
  float lookYaw = 0.0f;   // +turn right
  float lookPitch = 0.0f; // +look up

  // Discrete / held actions
  bool jumpHeld = false;   // SPACE held (fly / thrust)
  bool jumpPressed = false; // edge: just pressed this frame
  bool jumpReleased = false;
  bool crouch = false;     // C
  bool boost = false;      // Shift

  // Gameplay actions (one-shot preferred)
  bool action1 = false; // spawn water
  bool action2 = false; // spawn sand
  bool action3 = false; // spawn solid
  bool toggleWind = false;
  bool toggleGravity = false;
  bool toggleLight = false;
  bool clearDynamics = false;

  // Meta
  bool quit = false;
  bool toggleHud = false;
  bool toggleMute = false;

  // Helper: any movement input present?
  bool hasMove() const {
    return moveX * moveX + moveY * moveY > 1e-6f;
  }
  bool hasLook() const {
    return lookYaw * lookYaw + lookPitch * lookPitch > 1e-12f;
  }

  void clearEdges() {
    // Call after the observer has consumed edge-triggered flags
    jumpPressed = false;
    jumpReleased = false;
    action1 = action2 = action3 = false;
    toggleWind = toggleGravity = toggleLight = false;
    clearDynamics = false;
    toggleHud = toggleMute = false;
  }
};

} // namespace cmd
} // namespace nqg
