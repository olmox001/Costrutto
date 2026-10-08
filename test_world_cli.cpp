/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "nqg_engine_api.hpp"
#include <cmath>
#include <iostream>
using nqg::api::Host;
using nqg::cmd::Commands;
static int g_pass = 0, g_fail = 0;
#define CHECK(c, m)                                                            \
  do {                                                                         \
    if (c) {                                                                   \
      ++g_pass;                                                                \
      std::cout << "[PASS] " << m << "\n";                                     \
    } else {                                                                   \
      ++g_fail;                                                                \
      std::cout << "[FAIL] " << m << "\n";                                     \
    }                                                                          \
  } while (0)
int main() {
  std::cout << "=== World CLI scenarios ===\n";
  Host h;
  h.capsule().renderEnabled = false;
  h.setViewResolution(64, 48);
  CHECK(h.scene().sampleBed(0, -3).z < 0.5, "S0 bed floor not ceiling");
  // Exit south through door first (x≈0 path)
  h.applyTextCommand("move 0 -1");
  h.applyTextCommand("step 50");
  std::cout << "  y=" << h.capsule().observation.position.y
            << " room=" << h.capsule().observation.insideRoom << "\n";
  CHECK(h.capsule().observation.position.y < -5.0, "S2 exit door");
  CHECK(!h.capsule().observation.insideRoom, "S2 outside");
  CHECK(h.scene().sampleBed(0, -12).z < 0.5, "S2 terrain bed");
  h.applyTextCommand("stop");
  h.applyTextCommand("move 1 0");
  h.applyTextCommand("step 20");
  CHECK(h.capsule().observation.position.x > 0.3, "S1 move east outside");
  h.addCapsule("b", 48, 36);
  h.capsule(1).renderEnabled = false;
  Commands L, R;
  L.moveX = -1;
  R.moveX = 1;
  h.setCommands(L, 0);
  h.setCommands(R, 1);
  h.stepN(15);
  CHECK(h.capsuleCount() == 2, "S5 two capsules");
  CHECK(h.scene().solids.size() >= 10, "S6 solids");
  bool heavy = false;
  for (auto &s : h.scene().solids)
    if (s.isRoomSlab && s.mass > 1000)
      heavy = true;
  CHECK(heavy, "S7 house concrete mass");
  std::cout << "RESULT: " << g_pass << " PASS, " << g_fail << " FAIL\n";
  return g_fail ? 1 : 0;
}
