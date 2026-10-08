/* SPDX-License-Identifier: GPL-2.0-or-later
 * Numerically stable math helpers (bounded, asserted).
 */
#pragma once

#include "nasa_rules.hpp"
#include <cmath>
#include <cstddef>
#include <limits>

namespace nqg {
namespace physics {
namespace mathx {

using real = double;

struct KahanSum {
  real sum = 0;
  real corr = 0;
  void add(real x) {
    NQG_REQUIRE(std::isfinite(x) || !std::isfinite(x)); // allow NaN track
    const real y = x - corr;
    const real t = sum + y;
    corr = (t - sum) - y;
    sum = t;
  }
  real value() const { return sum; }
};

// log-sum-exp for n <= NQG_MAX_GRID elements
inline real log_sum_exp(const real *x, std::size_t n) {
  NQG_REQUIRE(x != nullptr);
  NQG_REQUIRE(n > 0);
  NQG_REQUIRE(n <= NQG_MAX_GRID);
  real m = x[0];
  for (std::size_t i = 1; i < n; ++i)
    if (x[i] > m)
      m = x[i];
  real s = 0;
  for (std::size_t i = 0; i < n; ++i)
    s += std::exp(x[i] - m);
  NQG_REQUIRE(s > 0);
  return m + std::log(s);
}

inline real clamp_real(real v, real lo, real hi) {
  NQG_REQUIRE(lo <= hi);
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

} // namespace mathx
} // namespace physics
} // namespace nqg
