/* SPDX-License-Identifier: GPL-2.0-or-later
 * Quadratic aerodynamic drag (modular, English API).
 */
#pragma once

#include "nasa_rules.hpp"
#include "nqg_engine3d.hpp"
#include <cmath>

namespace nqg {
namespace physics {
namespace drag {

using engine::Vec3;
using real = double;

inline real quadratic_coefficient(real density, real cd, real area, real speed) {
  NQG_REQUIRE(density >= 0);
  NQG_REQUIRE(cd >= 0);
  NQG_REQUIRE(area >= 0);
  NQG_REQUIRE(speed >= 0);
  return 0.5 * density * cd * area * speed;
}

// Force opposing relative velocity (not acceleration).
inline Vec3 quadratic_force(real density, real cd, real area, const Vec3 &v_rel) {
  NQG_REQUIRE(density >= 0);
  NQG_REQUIRE(cd >= 0);
  NQG_REQUIRE(area >= 0);
  const real speed = v_rel.norm();
  const real k = quadratic_coefficient(density, cd, area, speed);
  return Vec3(-v_rel.x * k, -v_rel.y * k, -v_rel.z * k);
}

} // namespace drag
} // namespace physics
} // namespace nqg
