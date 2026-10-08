/* SPDX-License-Identifier: GPL-2.0-or-later
 * Unified HUD line builder (formal English telemetry keys).
 * Does NOT include observer headers — call with Observation fields filled.
 */
#pragma once
#include <cstdio>
#include <string>
#include <vector>

namespace nqg {
namespace hud {

struct TelemetryView {
  double x = 0, y = 0, z = 0;
  double foot_z = 0, head_z = 0;
  bool inside_structure = false;
  bool spillway = false;
  double altitude = 0;
  double revolution = 0;
  long turns = 0;
  double gravity = 9.81;
  double speed = 0;
  double fps = 0;
  double density = 1.225;
  double wind = 0;
  const char *mode = "WALK";
  int view_w = 0, view_h = 0;
  std::string capsule_name = "capsule";
};

inline void build_lines(const TelemetryView &v, std::vector<std::string> &out) {
  out.clear();
  char buf[192];
  out.push_back("=== " + v.capsule_name + " ===");
  std::snprintf(buf, sizeof(buf), "position_m: %.3f %.3f %.3f", v.x, v.y, v.z);
  out.push_back(buf);
  std::snprintf(buf, sizeof(buf), "feet_head_z_m: %.3f %.3f", v.foot_z, v.head_z);
  out.push_back(buf);
  std::snprintf(buf, sizeof(buf), "inside_structure: %s | spillway: %s",
                v.inside_structure ? "yes" : "no", v.spillway ? "yes" : "no");
  out.push_back(buf);
  std::snprintf(buf, sizeof(buf),
                "altitude_m: %.3f | revolution: %.4f | turns: %ld", v.altitude,
                v.revolution, v.turns);
  out.push_back(buf);
  std::snprintf(buf, sizeof(buf),
                "gravity_mps2: %.4f | speed_mps: %.3f | fps: %.1f", v.gravity,
                v.speed, v.fps);
  out.push_back(buf);
  std::snprintf(buf, sizeof(buf), "density_kg_m3: %.5f | wind_mps: %.3f",
                v.density, v.wind);
  out.push_back(buf);
  std::snprintf(buf, sizeof(buf), "mode: %s | view_px: %dx%d", v.mode, v.view_w,
                v.view_h);
  out.push_back(buf);
}

} // namespace hud
} // namespace nqg
