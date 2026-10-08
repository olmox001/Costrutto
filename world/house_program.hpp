/* SPDX-License-Identifier: GPL-2.0-or-later
 * House as a startup program on the planetary solid:
 * 1) pick world coordinate (lat/lon or local ENU)
 * 2) find local mean height
 * 3) flatten a pad (volume-conserving edit of terrain solid)
 * 4) spawn house concrete slabs + props + player on the pad
 */
#pragma once

#include "world/house.hpp"
#include "world/interaction_grid.hpp"
#include "world/volume_ledger.hpp"
#include "world/spherical_harmonics.hpp"
#include "physics/nasa_rules.hpp"
#include <cmath>
#include <vector>

namespace nqg {
namespace world {

struct HouseProgramResult {
  real pad_z = 0;
  real volume_cut = 0;
  std::vector<SolidDesc> slabs;
  real player_x = 0, player_y = 0, player_z = 0;
};

class HouseProgram {
public:
  HouseBlueprint blueprint;
  real pad_margin = 2.0; // metres beyond house footprint

  // height_fn(x,y) returns absolute local surface height before edit
  template <typename HeightFn>
  HouseProgramResult run(HeightFieldEdit &field, HeightFn &&height_fn,
                         real origin_x, real origin_y) {
    HouseProgramResult r;
    const real cx = 0.5 * (blueprint.x_min + blueprint.x_max) + origin_x;
    const real cy = 0.5 * (blueprint.y_min + blueprint.y_max) + origin_y;
    // Mean height on footprint
    real sum = 0;
    int n = 0;
    for (real y = blueprint.y_min; y <= blueprint.y_max; y += field.dx) {
      for (real x = blueprint.x_min; x <= blueprint.x_max; x += field.dx) {
        sum += height_fn(origin_x + x, origin_y + y);
        ++n;
      }
    }
    NQG_REQUIRE(n > 0);
    r.pad_z = sum / n;
    // Seed field from height_fn
    for (int j = 0; j < field.ny; ++j) {
      for (int i = 0; i < field.nx; ++i) {
        const real x = field.x0 + i * field.dx;
        const real y = field.y0 + j * field.dx;
        field.at(i, j) = height_fn(x, y);
      }
    }
    const real hx =
        0.5 * (blueprint.x_max - blueprint.x_min) + pad_margin;
    const real hy =
        0.5 * (blueprint.y_max - blueprint.y_min) + pad_margin;
    r.volume_cut = field.flatten_pad(cx, cy, hx, hy, r.pad_z);
    r.slabs = blueprint.build_slabs();
    // Lift slabs onto pad
    for (auto &s : r.slabs) {
      s.cx += origin_x;
      s.cy += origin_y;
      s.cz += r.pad_z;
    }
    r.player_x = origin_x;
    r.player_y = origin_y - 3.0;
    r.player_z = r.pad_z + 1.75;
    return r;
  }
};

} // namespace world
} // namespace nqg
