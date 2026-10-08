/* SPDX-License-Identifier: GPL-2.0-or-later
 * Information-theoretic / BH entropy helpers.
 */
#pragma once

#include "nasa_rules.hpp"
#include "physics_constants.hpp"
#include <cmath>

namespace nqg {
namespace physics {
namespace info {

using real = double;
using constants::C;
using constants::G;
using constants::HBAR;
using constants::PI;

inline real schwarzschild_radius_from_energy(real energy) {
  NQG_REQUIRE(energy >= 0);
  return 2.0 * G * energy / std::pow(C, 4);
}

inline real black_hole_entropy(real energy) {
  NQG_REQUIRE(energy > 0);
  const real rs = schwarzschild_radius_from_energy(energy);
  NQG_REQUIRE(rs > 0);
  return PI * rs * rs * C * C * C / (HBAR * G);
}

inline real bekenstein_entropy(real radius, real energy) {
  NQG_REQUIRE(radius > 0);
  NQG_REQUIRE(energy >= 0);
  return 2.0 * PI * radius * energy / (HBAR * C);
}

inline real compton_radius(real mass) {
  NQG_REQUIRE(mass > 0);
  return HBAR / (mass * C);
}

inline real schwarzschild_radius_from_mass(real mass) {
  NQG_REQUIRE(mass >= 0);
  return 2.0 * G * mass / (C * C);
}

inline real kretschmann_scalar(real mass, real radius) {
  NQG_REQUIRE(mass >= 0);
  NQG_REQUIRE(radius > 0);
  const real r3 = radius * radius * radius;
  return 48.0 * mass * mass / (r3 * r3);
}

} // namespace info
} // namespace physics
} // namespace nqg
