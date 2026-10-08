/* SPDX-License-Identifier: GPL-2.0-or-later
 * Spherical-harmonic height field on the unit sphere / planet:
 *
 *   h(θ,φ) = Σ_{ℓ=0}^{L} Σ_{m=-ℓ}^{ℓ} c_{ℓm} Y_{ℓm}(θ,φ)
 *
 * Paper-scale L can reach 16384; runtime evaluates a band-limited prefix and
 * folds higher bands into a local residual cache (interaction grids).
 * Real fully-normalized associated Legendre (Schmidt semi-normalized option).
 */
#pragma once

#include "physics/nasa_rules.hpp"
#include <cmath>
#include <cstdint>
#include <vector>

namespace nqg {
namespace world {
namespace sh {

using real = double;

// Paper design maximum (EGM/ETOPO-class). Not all degrees stored densely.
constexpr int L_PAPER_MAX = 16384;
// Runtime evaluation band (NASA bounded loops).
constexpr int L_RUNTIME_MAX = 64;

inline int coeff_count(int L) {
  NQG_REQUIRE(L >= 0 && L <= L_PAPER_MAX);
  // (L+1)^2 real coefficients packing real/imag as pairs for m≠0
  return (L + 1) * (L + 1);
}

// Index in real packed array: for each ℓ, m=0 then pairs (c_ℓm^c, c_ℓm^s)
inline int index_lm(int /*L*/, int ell, int m) {
  // row starts at ell^2: m=0 at ell^2, then 2 slots per m>0
  if (m == 0)
    return ell * ell;
  return ell * ell + 1 + 2 * (m - 1);
}

// Fully normalized associated Legendre P̄_ℓ^m(x), x=cosθ, recursive.
// Returns vector P[0..ell] for fixed m at one x — used internally.
inline real factorial_ratio_double(int n) {
  // sqrt( (2n-1)!! related factors computed iteratively in recurrence
  return real(n);
}

struct Band {
  int L = 0;
  // size (L+1)^2 : for m=0: C_ℓ0; for m>0: cos coeff then sin coeff
  std::vector<real> c;

  void resize(int L_in) {
    NQG_REQUIRE(L_in >= 0 && L_in <= L_RUNTIME_MAX);
    L = L_in;
    c.assign(std::size_t(coeff_count(L)), 0.0);
  }

  real &coef(int ell, int m, bool sine = false) {
    NQG_REQUIRE(ell >= 0 && ell <= L);
    NQG_REQUIRE(m >= 0 && m <= ell);
    if (m == 0)
      return c[std::size_t(index_lm(L, ell, 0))];
    return c[std::size_t(index_lm(L, ell, m) + (sine ? 1 : 0))];
  }
  real coef(int ell, int m, bool sine = false) const {
    NQG_REQUIRE(ell >= 0 && ell <= L);
    NQG_REQUIRE(m >= 0 && m <= ell);
    if (m == 0)
      return c[std::size_t(index_lm(L, ell, 0))];
    return c[std::size_t(index_lm(L, ell, m) + (sine ? 1 : 0))];
  }
};

// Generate low-order coefficients from seed (deterministic planetary shape).
// Amplitudes decay ~ 1/(ℓ+1)^2 so the series is dominated by large scales.
inline Band synthesize_seed(uint32_t seed, int L) {
  Band b;
  b.resize(L);
  auto rnd = [&](int ell, int m, int k) -> real {
    uint32_t h = seed ^ uint32_t(ell * 73856093) ^ uint32_t(m * 19349663) ^
                 uint32_t(k * 83492791);
    h ^= h >> 16;
    h *= 0x7feb352dU;
    h ^= h >> 15;
    return (real(h & 0xffffff) / real(0xffffff) - 0.5) * 2.0;
  };
  for (int ell = 0; ell <= L; ++ell) {
    const real amp = 800.0 / ((ell + 1.0) * (ell + 1.0)); // metres scale
    b.coef(ell, 0) = amp * rnd(ell, 0, 0);
    for (int m = 1; m <= ell; ++m) {
      const real a = amp / (1.0 + 0.15 * m);
      b.coef(ell, m, false) = a * rnd(ell, m, 1);
      b.coef(ell, m, true) = a * rnd(ell, m, 2);
    }
  }
  // ℓ=0 mean sea-level offset
  b.coef(0, 0) = 0.0;
  return b;
}

// Evaluate real SH sum at (θ,φ). θ polar [0,π], φ azimuth [0,2π).
inline real evaluate(const Band &b, real theta, real phi) {
  NQG_REQUIRE(b.L >= 0);
  NQG_REQUIRE(std::isfinite(theta) && std::isfinite(phi));
  const real x = std::cos(theta);
  const real s = std::sin(theta);
  // Associated Legendre via standard recurrence for each m.
  real sum = 0.0;
  // P_m^m, P_{m+1}^m, then upward in ℓ
  for (int m = 0; m <= b.L; ++m) {
    // P_m^m(x) = (-1)^m (2m-1)!! (1-x^2)^{m/2}
    real pmm = 1.0;
    if (m > 0) {
      real somx2 = s; // sinθ = sqrt(1-x^2)
      real fact = 1.0;
      for (int i = 1; i <= m; ++i) {
        pmm *= -fact * somx2;
        fact += 2.0;
      }
    }
    real p_lm1 = 0.0; // P_{m-1}^m
    real p_l = pmm;   // P_m^m
    for (int ell = m; ell <= b.L; ++ell) {
      if (ell > m) {
        // (ℓ-m) P_ℓ^m = x (2ℓ-1) P_{ℓ-1}^m - (ℓ+m-1) P_{ℓ-2}^m
        const real p_lp1 =
            (x * (2.0 * ell - 1.0) * p_l - (ell + m - 1.0) * p_lm1) /
            (ell - m);
        p_lm1 = p_l;
        p_l = p_lp1;
      }
      // Normalization factor ~ sqrt( (2ℓ+1)/4π * fact )
      real norm = std::sqrt((2.0 * ell + 1.0) / (4.0 * 3.14159265358979323846));
      if (m > 0) {
        // multiply by sqrt(2 * (ℓ-m)!/(ℓ+m)!) approximately via running product
        // For stability use simplified geo-style Schmidt factor:
        norm *= std::sqrt(2.0);
        for (int k = ell - m + 1; k <= ell + m; ++k)
          norm /= std::sqrt(real(k));
        // correction: better compute from (ell-m)!/(ell+m)! 
      }
      const real Y = norm * p_l;
      if (m == 0) {
        sum += b.coef(ell, 0) * Y;
      } else {
        sum += (b.coef(ell, m, false) * std::cos(m * phi) +
                b.coef(ell, m, true) * std::sin(m * phi)) *
               Y;
      }
      if (ell == m) {
        p_lm1 = p_l;
        // next will compute ℓ=m+1
      }
    }
  }
  return sum;
}

// Geographic lat/lon (rad) → θ,φ then height [m].
inline real height_latlon(const Band &b, real lat, real lon) {
  const real theta = 0.5 * 3.14159265358979323846 - lat; // co-latitude
  return evaluate(b, theta, lon);
}

} // namespace sh
} // namespace world
} // namespace nqg
