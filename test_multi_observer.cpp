/* SPDX-License-Identifier: GPL-2.0-or-later
 * Multi-observer consistency: positions, independence, inside/outside.
 */
#include "nqg_engine_api.hpp"
#include <cmath>
#include <iostream>
using nqg::api::Host;
using nqg::cmd::Commands;

static int g_pass = 0, g_fail = 0;
#define CHECK(c, m) do { if (c) { ++g_pass; std::cout << "[PASS] " << m << "\n"; } \
  else { ++g_fail; std::cout << "[FAIL] " << m << "\n"; } } while (0)

int main() {
  std::cout << "=== Multi-observer ===\n";
  Host h;
  h.setParallelRender(false);
  for (std::size_t i = 0; i < h.capsuleCount(); ++i)
    h.capsule(i).renderEnabled = false;
  h.setViewResolution(64, 48, 0);
  h.addCapsule("obs1", 48, 36);
  h.addCapsule("obs2", 48, 36);
  h.capsule(1).renderEnabled = false;
  h.capsule(2).renderEnabled = false;
  CHECK(h.capsuleCount() == 3, "three observers");

  h.applyTextCommand("tp 0 -3 1.75", 0);
  h.applyTextCommand("tp 40 40 2", 1);
  h.applyTextCommand("tp -20 15 2", 2);
  h.step(1. / 60);

  CHECK(h.capsule(0).observation.inside_structure(), "cap0 inside structure");
  CHECK(!h.capsule(1).observation.inside_structure(), "cap1 far outside");
  CHECK(!h.capsule(2).observation.inside_structure(), "cap2 outside");

  const double x1a = h.capsule(1).observation.position.x;
  const double x2a = h.capsule(2).observation.position.x;
  Commands L, R;
  L.moveX = -1;
  R.moveX = 1;
  h.setCommands(L, 1);
  h.setCommands(R, 2);
  h.stepN(20);
  CHECK(h.capsule(1).observation.position.x < x1a, "cap1 moved left independent");
  CHECK(h.capsule(2).observation.position.x > x2a, "cap2 moved right independent");
  // cap0 still near spawn
  CHECK(std::abs(h.capsule(0).observation.position.x) < 5.0, "cap0 not dragged by others");

  // different resolutions
  h.setViewResolution(80, 60, 0);
  h.setViewResolution(40, 30, 1);
  CHECK(h.capsule(0).viewW == 80 && h.capsule(1).viewW == 40, "independent view sizes");

  for (std::size_t i = 0; i < h.capsuleCount(); ++i) {
    CHECK(std::isfinite(h.capsule(i).observation.position.z), "finite z all observers");
    CHECK(h.capsule(i).observation.gravity > 0, "gravity positive");
  }

  std::cout << "RESULT: " << g_pass << " PASS, " << g_fail << " FAIL\n";
  return g_fail ? 1 : 0;
}
