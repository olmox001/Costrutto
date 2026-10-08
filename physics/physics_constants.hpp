/* SPDX-License-Identifier: GPL-2.0-or-later
 * Physical constants (SI) — pure functions, NASA-style bounds.
 */
#pragma once

#include "nasa_rules.hpp"
#include <cmath>

namespace nqg {
namespace physics {
namespace constants {

using real = double;

constexpr real PI = 3.14159265358979323846264338327950288;
constexpr real G = 6.67430e-11;           // gravitational constant [m^3 kg^-1 s^-2]
constexpr real C = 299792458.0;           // speed of light [m/s]
constexpr real HBAR = 1.054571817e-34;    // reduced Planck constant [J s]
constexpr real MPC = 3.0857e22;           // megaparsec [m]
constexpr real H0_DEFAULT = 67.4e3 / MPC; // Hubble parameter [s^-1]
constexpr real GAMMA_BARBERO_IMMIRZI = 0.2375;

inline real planck_length() {
  NQG_REQUIRE(G > 0);
  NQG_REQUIRE(HBAR > 0);
  return std::sqrt(G * HBAR / (C * C * C));
}

inline real planck_mass() {
  NQG_REQUIRE(G > 0);
  NQG_REQUIRE(HBAR > 0);
  return std::sqrt(HBAR * C / G);
}

} // namespace constants
} // namespace physics
} // namespace nqg
