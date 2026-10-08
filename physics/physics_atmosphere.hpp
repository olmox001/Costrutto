/* SPDX-License-Identifier: GPL-2.0-or-later
 * Atmospheric density/pressure → drag and terminal-speed limits (no hard caps).
 */
#pragma once

#include "nasa_rules.hpp"
#include "physics_constants.hpp"
#include <cmath>

namespace nqg {
namespace physics {
namespace atmosphere {

using real = double;

// ISA-lite pressure [Pa] vs altitude [m]
inline real pressure(real altitude_m) {
  NQG_REQUIRE(std::isfinite(altitude_m));
  constexpr real P0 = 101325.0;
  constexpr real H = 8500.0; // scale height
  if (altitude_m < -500.0)
    altitude_m = -500.0;
  return P0 * std::exp(-altitude_m / H);
}

inline real density(real altitude_m, real temperature_k = 288.15) {
  NQG_REQUIRE(temperature_k > 0);
  constexpr real R_spec = 287.05; // J/(kg·K) dry air
  return pressure(altitude_m) / (R_spec * temperature_k);
}

// Terminal speed from mg = 0.5 rho Cd A v^2
inline real terminal_speed(real mass, real rho, real cd, real area) {
  NQG_REQUIRE(mass > 0);
  NQG_REQUIRE(rho >= 0);
  NQG_REQUIRE(cd > 0);
  NQG_REQUIRE(area > 0);
  if (rho < 1e-12)
    return 1e6; // vacuum: no aerodynamic limit
  constexpr real g = 9.81;
  return std::sqrt((2.0 * mass * g) / (rho * cd * area));
}

// Soft speed limit factor in [0,1]: scales control authority by dynamic pressure.
inline real control_authority(real speed, real v_term) {
  NQG_REQUIRE(speed >= 0);
  NQG_REQUIRE(v_term > 0);
  const real x = speed / v_term;
  if (x <= 0.5)
    return 1.0;
  if (x >= 1.0)
    return 0.05;
  // smoothstep from 1 → 0.05 between 0.5 and 1.0
  const real t = (x - 0.5) / 0.5;
  return 1.0 - 0.95 * (t * t * (3.0 - 2.0 * t));
}

} // namespace atmosphere
} // namespace physics
} // namespace nqg
