// SPDX-License-Identifier: GPL-2.0-or-later
#ifndef NQG_DRAG_PHYSICS_HPP
#define NQG_DRAG_PHYSICS_HPP

#include "physics/physics_drag.hpp"

namespace nqg {
namespace cleanroom {

using engine::Vec3;
using real = double;

// Legacy aliases → modular English API
inline real quadraticDragCoeff(real rho, real Cd, real area, real speed) {
  return physics::drag::quadratic_coefficient(rho, Cd, area, speed);
}

inline Vec3 quadraticDrag(real rho, real Cd, real area, const Vec3 &vRel) {
  return physics::drag::quadratic_force(rho, Cd, area, vRel);
}

} // namespace cleanroom
} // namespace nqg

#endif
