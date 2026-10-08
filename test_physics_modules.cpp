/* SPDX-License-Identifier: GPL-2.0-or-later
 * Dual tests per public function of modular physics (NASA rule 5 density).
 */
#include "physics/physics.hpp"
#include <cmath>
#include <iostream>

using namespace nqg::physics;
using real = double;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    if (cond) {                                                                \
      ++g_pass;                                                                \
      std::cout << "[PASS] " << msg << "\n";                                   \
    } else {                                                                   \
      ++g_fail;                                                                \
      std::cout << "[FAIL] " << msg << "\n";                                   \
    }                                                                          \
  } while (0)

int main() {
  std::cout << "=== Modular physics (NASA-style) dual tests ===\n";

  // constants::planck_length — two checks
  {
    const real lp = constants::planck_length();
    CHECK(lp > 0 && lp < 1e-30, "planck_length positive tiny");
    CHECK(std::isfinite(lp), "planck_length finite");
  }
  // constants::planck_mass
  {
    const real mp = constants::planck_mass();
    CHECK(mp > 1e-10 && mp < 1e-5, "planck_mass order of magnitude");
    CHECK(std::isfinite(mp), "planck_mass finite");
  }

  // schwarzschild::lapse
  {
    CHECK(std::abs(schwarzschild::lapse(1.0, 4.0) - std::sqrt(0.5)) < 1e-12,
          "lapse at r=4M");
    CHECK(schwarzschild::lapse(1.0, 2.0) == 0.0, "lapse at horizon is 0");
  }
  // observed_frequency
  {
    CHECK(std::abs(schwarzschild::observed_frequency(100.0, 0.5) - 200.0) < 1e-12,
          "observed_frequency redshift");
    CHECK(schwarzschild::observed_frequency(0.0, 1.0) == 0.0,
          "observed_frequency zero");
  }
  // redshift_factor
  {
    const real z = schwarzschild::redshift_factor(1.0, 4.0, 100.0);
    CHECK(z > 1.0, "redshift_factor emit closer than obs");
    CHECK(std::isfinite(z), "redshift_factor finite");
  }
  // nyquist_minimum
  {
    CHECK(schwarzschild::nyquist_minimum(50.0) == 100.0, "nyquist_minimum *2");
    CHECK(schwarzschild::nyquist_minimum(0.0) == 0.0, "nyquist_minimum zero");
  }
  // proper_time_horizon_to_singularity
  {
    CHECK(std::abs(schwarzschild::proper_time_horizon_to_singularity(2.0) -
                   2.0 * constants::PI) < 1e-12,
          "proper_time_horizon pi*M");
    CHECK(schwarzschild::proper_time_horizon_to_singularity(0.0) == 0.0,
          "proper_time_horizon M=0");
  }
  // circular_angular_momentum
  {
    const real L = schwarzschild::circular_angular_momentum(1.0, 6.0);
    CHECK(L > 0, "circular_L positive outside ISCO");
    CHECK(std::isfinite(L), "circular_L finite");
  }

  // info::schwarzschild_radius_from_energy
  {
    const real rs = info::schwarzschild_radius_from_energy(1e20);
    CHECK(rs > 0, "rs_from_energy positive");
    CHECK(std::isfinite(rs), "rs_from_energy finite");
  }
  // black_hole_entropy
  {
    const real S = info::black_hole_entropy(1e30);
    CHECK(S > 0, "BH entropy positive");
    CHECK(std::isfinite(S), "BH entropy finite");
  }
  // bekenstein_entropy
  {
    const real S = info::bekenstein_entropy(1.0, 1.0);
    CHECK(S > 0, "Bekenstein entropy positive");
    CHECK(std::isfinite(S), "Bekenstein entropy finite");
  }
  // compton_radius
  {
    const real rc = info::compton_radius(1e-30);
    CHECK(rc > 0, "compton_radius positive");
    CHECK(std::isfinite(rc), "compton_radius finite");
  }
  // schwarzschild_radius_from_mass
  {
    const real rs = info::schwarzschild_radius_from_mass(1.988e30); // ~Sun
    CHECK(rs > 1e3 && rs < 1e4, "solar Rs ~3km order");
    CHECK(std::isfinite(rs), "Rs mass finite");
  }
  // kretschmann_scalar
  {
    const real K = info::kretschmann_scalar(1.0, 10.0);
    CHECK(K > 0, "Kretschmann positive");
    CHECK(std::isfinite(K), "Kretschmann finite");
  }

  // drag::quadratic_coefficient
  {
    const real k = drag::quadratic_coefficient(1.2, 0.5, 0.1, 10.0);
    CHECK(std::abs(k - 0.5 * 1.2 * 0.5 * 0.1 * 10.0) < 1e-12, "drag coeff formula");
    CHECK(drag::quadratic_coefficient(0, 1, 1, 1) == 0, "drag coeff zero density");
  }
  // drag::quadratic_force
  {
    nqg::engine::Vec3 v(1, 0, 0);
    auto f = drag::quadratic_force(1.0, 1.0, 1.0, v);
    CHECK(f.x < 0, "drag force opposes +x velocity");
    CHECK(std::abs(f.y) < 1e-15 && std::abs(f.z) < 1e-15, "drag force no lateral");
  }

  
  // mathx::log_sum_exp
  {
    real a[3] = {0.0, 0.0, 0.0};
    CHECK(std::abs(mathx::log_sum_exp(a, 3) - std::log(3.0)) < 1e-12, "log_sum_exp equal");
    real b[2] = {1000.0, 1000.0};
    CHECK(std::isfinite(mathx::log_sum_exp(b, 2)), "log_sum_exp stable large");
  }
  // mathx::clamp_real
  {
    CHECK(mathx::clamp_real(5.0, 0.0, 1.0) == 1.0, "clamp high");
    CHECK(mathx::clamp_real(-1.0, 0.0, 1.0) == 0.0, "clamp low");
  }
  // mathx::KahanSum
  {
    mathx::KahanSum k;
    for (int i = 0; i < 1000; ++i) k.add(0.1);
    CHECK(std::abs(k.value() - 100.0) < 1e-9, "KahanSum 1000*0.1");
    CHECK(std::isfinite(k.value()), "KahanSum finite");
  }

  std::cout << "RESULT: " << g_pass << " PASS, " << g_fail << " FAIL\n";
  return g_fail == 0 ? 0 : 1;
}
