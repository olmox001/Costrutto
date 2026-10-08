/* SPDX-License-Identifier: GPL-2.0-or-later
 * Schwarzschild geometry helpers (geometric units G=c=1 where noted).
 */
#pragma once

#include "nasa_rules.hpp"
#include "physics_constants.hpp"
#include <algorithm>
#include <cmath>

namespace nqg {
namespace physics {
namespace schwarzschild {

using real = double;
using constants::PI;

// Lapse alpha = sqrt(1 - 2M/r) in geometric units.
inline real lapse(real mass, real radius) {
  NQG_REQUIRE(mass >= 0);
  NQG_REQUIRE(radius > 0);
  const real x = 1.0 - 2.0 * mass / radius;
  return x > 0 ? std::sqrt(x) : 0.0;
}

inline real observed_frequency(real rest_freq, real alpha) {
  NQG_REQUIRE(rest_freq >= 0);
  NQG_REQUIRE(alpha > 0);
  return rest_freq / alpha;
}

inline real redshift_factor(real mass, real r_emit, real r_obs) {
  NQG_REQUIRE(mass >= 0);
  NQG_REQUIRE(r_emit > 2.0 * mass);
  NQG_REQUIRE(r_obs > 2.0 * mass);
  return lapse(mass, r_obs) / lapse(mass, r_emit);
}

inline real nyquist_minimum(real frequency) {
  NQG_REQUIRE(frequency >= 0);
  return 2.0 * frequency;
}

// Proper time from horizon to singularity for radial free-fall: pi*M
inline real proper_time_horizon_to_singularity(real mass) {
  NQG_REQUIRE(mass >= 0);
  return PI * mass;
}

inline real circular_angular_momentum(real mass, real radius) {
  NQG_REQUIRE(mass > 0);
  NQG_REQUIRE(radius > 3.0 * mass); // outside ISCO for Schwarzschild
  return std::sqrt(mass * radius) / std::sqrt(1.0 - 3.0 * mass / radius);
}

} // namespace schwarzschild
} // namespace physics
} // namespace nqg
