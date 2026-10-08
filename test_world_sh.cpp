/* SPDX-License-Identifier: GPL-2.0-or-later
 * Spherical harmonics world, volume conservation, house program.
 */
#include "world/world.hpp"
#include <cmath>
#include <iostream>

static int g_pass = 0, g_fail = 0;
#define CHECK(c, m) do { if (c) { ++g_pass; std::cout << "[PASS] " << m << "\n"; } else { ++g_fail; std::cout << "[FAIL] " << m << "\n"; } } while(0)

int main() {
  using namespace nqg::world;
  std::cout << "=== SH + volume + house program ===\n";

  // Paper constant present
  CHECK(sh::L_PAPER_MAX == 16384, "paper L_max 16384");
  CHECK(sh::coeff_count(2) == 9, "(L+1)^2 coeffs");

  auto band = sh::synthesize_seed(42, 8);
  CHECK(band.L == 8, "band L=8");
  const real h0 = sh::height_latlon(band, 0.5, 0.3);
  const real h1 = sh::height_latlon(band, 0.5, 0.3);
  CHECK(std::isfinite(h0) && h0 == h1, "SH eval finite deterministic");
  const real h2 = sh::height_latlon(band, 0.51, 0.3);
  CHECK(std::abs(h2 - h0) < 500.0, "SH smooth in lat");

  // Volume ledger conservation
  VolumeLedger led;
  led.add_solid("concrete", 100.0);
  led.add_solid("regolith", 50.0);
  const real expected = led.total_matter();
  led.transfer_solid_to_air_dust("concrete", 10.0);
  CHECK(led.solid_volume("concrete") == 90.0, "solid after transfer");
  CHECK(led.air().dust_volume == 10.0, "dust volume");
  CHECK(led.check_conservation(expected), "conservation after transfer");
  led.deposit_dust_on_surface(4.0);
  CHECK(led.air().dust_volume == 6.0, "dust after deposit");
  CHECK(led.check_conservation(expected), "conservation after deposit");

  // Fragmentation conserves volume
  std::vector<real> frags;
  real dust = 0;
  fragment_volume(1.0, 0.01, 0.2, 2.5, frags, dust);
  real sum = dust;
  for (real v : frags) sum += v;
  CHECK(std::abs(sum - 1.0) < 1e-9, "fragment volume conserved");
  CHECK(!frags.empty() || dust > 0, "produced fragments or dust");

  // Humidity deposition probability
  const real p = led.deposition_probability(0.8, 0.5);
  CHECK(p > 0 && p <= 1.0, "deposition probability in (0,1]");

  // Interaction grid bake
  InteractionGrid grid;
  grid.configure(16, 0.5, 0, 0, 0);
  grid.bake_from_height([](real x, real y) { return 0.1 * x + 0.05 * y; });
  const real d0 = grid.query(0, 0, 1.0);
  CHECK(d0 > 0, "above surface SDF positive");
  const real d1 = grid.query(0, 0, -1.0);
  CHECK(d1 < 0, "below surface SDF negative");

  // House program flattens pad with volume account
  VolumeLedger soil;
  HeightFieldEdit field;
  field.configure(64, 64, 0.5, 0, 0);
  field.ledger = &soil;
  HouseProgram prog;
  auto result = prog.run(field, [](real x, real y) {
    return 2.0 + 0.01 * x + 0.02 * y; // gentle slope
  }, 0, 0);
  CHECK(result.slabs.size() >= 6, "house slabs spawned");
  CHECK(std::isfinite(result.pad_z), "pad_z finite");
  CHECK(std::isfinite(result.volume_cut), "volume_cut finite");
  CHECK(result.player_z > result.pad_z, "player above pad");

  // Terrain cache with SH
  LocalFrame frame;
  frame.latitude = 0.7;
  frame.longitude = 0.1;
  TerrainCache tc;
  tc.configure(64, 64, 1.0, 99, frame);
  tc.rebuild();
  CHECK(std::isfinite(tc.sample_height(0, 0)), "SH terrain sample");

  std::cout << "RESULT: " << g_pass << " PASS, " << g_fail << " FAIL\n";
  return g_fail ? 1 : 0;
}
