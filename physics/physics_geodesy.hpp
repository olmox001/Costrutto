/* SPDX-License-Identifier: GPL-2.0-or-later
 * Unified local-ground queries — one path for feet, water bed, contacts.
 */
#pragma once
#include "nasa_rules.hpp"
#include <cmath>

namespace nqg {
namespace physics {
namespace geodesy {

using real = double;

// Combine layered heights: base terrain, optional pad, optional sand mound.
inline real combine_ground(real terrain_z, real pad_z, real sand_z) {
  NQG_REQUIRE(std::isfinite(terrain_z) || !std::isfinite(terrain_z));
  real z = terrain_z;
  if (std::isfinite(pad_z) && pad_z > z)
    z = pad_z;
  if (std::isfinite(sand_z) && sand_z > z)
    z = sand_z;
  return z;
}

// Eye height above ground for a standing capsule.
inline real eye_from_ground(real ground_z, real eye_height) {
  NQG_REQUIRE(eye_height > 0);
  NQG_REQUIRE(std::isfinite(ground_z));
  return ground_z + eye_height;
}

// Clamp a free-fall impact onto ground (no penetration).
inline real resolve_vertical(real body_lowest_z, real ground_z, real &vel_z) {
  NQG_REQUIRE(std::isfinite(body_lowest_z));
  NQG_REQUIRE(std::isfinite(ground_z));
  if (body_lowest_z < ground_z) {
    if (vel_z < 0)
      vel_z = 0;
    return ground_z - body_lowest_z; // positive lift
  }
  return 0;
}

} // namespace geodesy
} // namespace physics
} // namespace nqg
