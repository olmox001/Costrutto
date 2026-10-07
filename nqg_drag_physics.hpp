// SPDX-License-Identifier: GPL-2.0-or-later
#ifndef NQG_DRAG_PHYSICS_HPP
#define NQG_DRAG_PHYSICS_HPP

#include "nqg_engine3d.hpp"
#include "nqg_physics_core.hpp"

namespace nqg {
namespace cleanroom {

using engine::Vec3;

inline real quadraticDragCoeff(real rho, real Cd, real area, real speed) {
  return 0.5 * rho * Cd * area * speed;
}

inline Vec3 quadraticDrag(real rho, real Cd, real area, const Vec3 &vRel) {
  const real k = quadraticDragCoeff(rho, Cd, area, vRel.norm());
  return Vec3(-vRel.x * k, -vRel.y * k, -vRel.z * k);
}

} // namespace cleanroom
} // namespace nqg

#endif // NQG_DRAG_PHYSICS_HPP
