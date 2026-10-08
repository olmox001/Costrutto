/* SPDX-License-Identifier: GPL-2.0-or-later
 * House = set of physical solid slabs (concrete) on a planet surface.
 * Stability must come from mass, friction, and contact — not script locks.
 */
#pragma once

#include "physics/nasa_rules.hpp"
#include <vector>

namespace nqg {
namespace world {

using real = double;

struct SolidDesc {
  real cx = 0, cy = 0, cz = 0; // local ENU centre [m]
  real sx = 1, sy = 1, sz = 1; // full size [m]
  real density = 2400.0;       // concrete [kg/m^3]
  real friction = 0.70;
  real restitution = 0.0;
  real roughness = 0.80;
  bool is_floor = false;
  bool is_ceiling = false;
  bool is_static = true; // still has real mass for contacts/water

  real volume() const {
    NQG_REQUIRE(sx > 0 && sy > 0 && sz > 0);
    return sx * sy * sz;
  }
  real mass() const { return density * volume(); }
};

struct HouseBlueprint {
  real x_min = -6, x_max = 6;
  real y_min = -5, y_max = 5;
  real z_min = 0, z_max = 3.2;
  real wall_t = 0.20;
  real door_x0 = -1, door_x1 = 1;
  real door_z1 = 2.20;
  real concrete_density = 2400.0;

  std::vector<SolidDesc> build_slabs() const {
    std::vector<SolidDesc> out;
    const real cx = 0.5 * (x_min + x_max);
    const real cy = 0.5 * (y_min + y_max);
    const real cz = 0.5 * (z_min + z_max);
    const real hx = (x_max - x_min);
    const real hy = (y_max - y_min);
    const real hz = (z_max - z_min);
    const real T = wall_t;

    auto add = [&](real x, real y, real z, real sx, real sy, real sz, bool floor,
                   bool ceil) {
      SolidDesc d;
      d.cx = x;
      d.cy = y;
      d.cz = z;
      d.sx = sx;
      d.sy = sy;
      d.sz = sz;
      d.density = concrete_density;
      d.is_floor = floor;
      d.is_ceiling = ceil;
      out.push_back(d);
    };

    // Floor / ceiling slabs
    add(cx, cy, z_min - T * 0.5, hx + 2 * T, hy + 2 * T, T, true, false);
    add(cx, cy, z_max + T * 0.5, hx + 2 * T, hy + 2 * T, T, false, true);
    // N/E/W walls
    add(cx, y_max + T * 0.5, cz, hx + T, T, hz, false, false);
    add(x_max + T * 0.5, cy, cz, T, hy + T, hz, false, false);
    add(x_min - T * 0.5, cy, cz, T, hy + T, hz, false, false);
    // South wall with door gap
    add(0.5 * (x_min + door_x0), y_min - T * 0.5, cz, (door_x0 - x_min), T, hz,
        false, false);
    add(0.5 * (door_x1 + x_max), y_min - T * 0.5, cz, (x_max - door_x1), T, hz,
        false, false);
    add(0.5 * (door_x0 + door_x1), y_min - T * 0.5, 0.5 * (door_z1 + z_max),
        (door_x1 - door_x0), T, (z_max - door_z1), false, false);
    return out;
  }
};

} // namespace world
} // namespace nqg
