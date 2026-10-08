/* SPDX-License-Identifier: GPL-2.0-or-later
 * Unified physics + world module suite (NASA-style assertions at runtime).
 */
#include "physics/physics.hpp"
#include "world/world.hpp"
#include <cmath>
#include <iostream>
#include <vector>

static int g_pass = 0, g_fail = 0;
#define CHECK(c, m)                                                            \
  do {                                                                         \
    if (c) {                                                                   \
      ++g_pass;                                                                \
      std::cout << "[PASS] " << m << "\n";                                     \
    } else {                                                                   \
      ++g_fail;                                                                \
      std::cout << "[FAIL] " << m << "\n";                                     \
    }                                                                          \
  } while (0)

int main() {
  using namespace nqg::physics;
  using namespace nqg::world;
  std::cout << "=== Unified physics/world suite (NASA) ===\n";
  CHECK(MODULE_INDEX_VERSION >= 4, "physics module index");
  CHECK(WORLD_MODULE_VERSION >= 3, "world module index");
  CHECK(NQG_MAX_FUNCTION_LINES == 60, "style target 60 lines");
  CHECK(NQG_MAX_SH_DEGREE == 64, "SH degree bound");

  // constants
  CHECK(constants::C > 0 && constants::G > 0, "SI constants");
  CHECK(constants::planck_length() > 0, "planck length");

  // atmosphere
  const real rho0 = atmosphere::density(0);
  const real rho5 = atmosphere::density(5000);
  CHECK(rho0 > rho5 && rho5 > 0, "density decreases with altitude");
  const real vt = atmosphere::terminal_speed(80, rho0, 1.0, 0.5);
  CHECK(vt > 1 && vt < 200, "terminal speed range");
  CHECK(atmosphere::control_authority(0, vt) > 0.9, "authority at rest");
  CHECK(atmosphere::control_authority(vt, vt) < 0.2, "authority at terminal");

  // drag
  CHECK(drag::quadratic_coefficient(1.2, 0.5, 0.5, 10) > 0, "quadratic drag");

  // schwarzschild
  CHECK(schwarzschild::proper_time_horizon_to_singularity(1.9885e30) > 1e3, "sun Rs");

  // math
  mathx::KahanSum ks;
  for (int i = 0; i < 1000; ++i)
    ks.add(0.1);
  CHECK(std::abs(ks.value() - 100.0) < 1e-9, "KahanSum");

  // fluid surface
  auto d = fluid_surface::drainage_direction(nqg::engine::Vec3(1, 0, 0),
                                             nqg::engine::Vec3(0, 0, -9.81));
  CHECK(d.z < 0, "wall drains down");
  CHECK(fluid_surface::retention_fraction(0, 0.5) > 0.5, "floor retains");
  CHECK(fluid_surface::retention_fraction(1, 0.1) < 0.5, "vertical runoff");

  // geodesy
  CHECK(geodesy::combine_ground(1, 2, 0.5) == 2.0, "combine pad wins");
  CHECK(geodesy::combine_ground(1, 0.5, 3) == 3.0, "combine sand wins");
  CHECK(geodesy::eye_from_ground(0, 1.7) == 1.7, "eye height");
  real vz = -5;
  real lift = geodesy::resolve_vertical(-0.1, 0.0, vz);
  CHECK(lift > 0 && vz == 0, "resolve vertical impact");

  // SH
  CHECK(sh::L_PAPER_MAX == 16384, "paper L");
  auto band = sh::synthesize_seed(1, 4);
  CHECK(std::isfinite(sh::height_latlon(band, 0.5, 0.2)), "SH height");

  // volume ledger
  VolumeLedger led;
  led.add_solid("a", 10);
  led.transfer_solid_to_air_dust("a", 3);
  CHECK(led.check_conservation(10), "ledger conservation");

  // material mix
  MaterialMix mix;
  mix.add(nqg::materials::Id::Concrete, 0.7);
  mix.add(nqg::materials::Id::Steel, 0.3);
  mix.normalize();
  CHECK(mix.density() > 2000, "mix density");
  CHECK(std::abs(mix.component_volume(10, 0) - 7) < 1e-9, "mix volume");

  // planet
  PlanetBody earth;
  CHECK(earth.flattening() > 0, "flattening");

  // fragment
  std::vector<real> frags;
  real dust = 0;
  fragment_volume(2.0, 0.01, 0.3, 2.5, frags, dust);
  real sum = dust;
  for (real v : frags)
    sum += v;
  CHECK(std::abs(sum - 2.0) < 1e-9, "fragment conserve");

  std::cout << "RESULT: " << g_pass << " PASS, " << g_fail << " FAIL\n";
  return g_fail ? 1 : 0;
}
