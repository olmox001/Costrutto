/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "world/world.hpp"
#include "physics/physics.hpp"
#include "nqg_earth_environment.hpp"
#include <cmath>
#include <iostream>

static int g_pass = 0, g_fail = 0;
#define CHECK(c, m) do { if (c) { ++g_pass; std::cout << "[PASS] " << m << "\n"; } else { ++g_fail; std::cout << "[FAIL] " << m << "\n"; } } while(0)

int main() {
  using namespace nqg::world;
  using namespace nqg::physics;
  std::cout << "=== World + terrain (real harmonics) ===\n";

  PlanetBody body;
  CHECK(body.flattening() > 0 && body.flattening() < 0.01, "Earth flattening");
  CHECK(body.radius_at_latitude(0) > body.radius_at_latitude(1.0), "eq > polar radius");

  LocalFrame frame;
  frame.latitude = 0.8;
  frame.longitude = 0.2;
  frame.body = body;

  // Match nqg::earth::TerrainGenerator coefficients at origin
  const real phase = 0.0;
  const real h_direct = TerrainCache::base_height(frame.latitude, frame.longitude, phase);
  const real h_legacy = nqg::earth::TerrainGenerator{}.baseHeight(frame.latitude, frame.longitude);
  CHECK(std::abs(h_direct - h_legacy) < 1e-9, "base_height matches TerrainGenerator");

  TerrainCache tc;
  tc.configure(TerrainCache::DEFAULT_RES, TerrainCache::DEFAULT_RES,
               TerrainCache::DEFAULT_CELL_M, 1337, frame);
  CHECK(tc.nx() == 512 && tc.ny() == 512, "default resolution 512x512");
  CHECK(tc.cell_size() == 0.25, "default cell 0.25 m");
  tc.rebuild();
  CHECK(tc.raw_cells().size() == 512u * 512u, "raw cell count");

  // Origin sample equals field at origin lat/lon (phase from seed may differ)
  const real h0 = tc.sample_height(0, 0);
  CHECK(std::isfinite(h0), "origin height finite");

  // Continuity: neighbouring samples differ smoothly (no hash noise spikes)
  const real h1 = tc.sample_height(0.25, 0);
  const real h2 = tc.sample_height(0.5, 0);
  CHECK(std::abs(h1 - h0) < 50.0, "local smoothness 0.25m");
  CHECK(std::abs(h2 - h0) < 100.0, "local smoothness 0.5m");

  // Slope finite
  real de = 0, dn = 0;
  tc.sample_slope(0, 0, de, dn);
  CHECK(std::isfinite(de) && std::isfinite(dn), "slope finite");

  // Surface classification present
  auto cell = tc.sample_cell(0, 0);
  CHECK(cell.roughness > 0, "roughness set");

  HouseBlueprint hb;
  auto slabs = hb.build_slabs();
  real total_mass = 0;
  for (auto &s : slabs)
    total_mass += s.mass();
  CHECK(total_mass > 1e5, "house concrete mass");

  real rho = atmosphere::density(0);
  real vt = atmosphere::terminal_speed(80.0, rho, 1.0, 0.5);
  CHECK(vt > 1 && vt < 200, "terminal speed");
  CHECK(atmosphere::control_authority(vt, vt) < 0.1, "authority at terminal");

  auto d = fluid_surface::drainage_direction(nqg::engine::Vec3(1, 0, 0),
                                             nqg::engine::Vec3(0, 0, -9.81));
  CHECK(d.z < 0, "wall drains down");

  // Cross-check: evaluate_surface vs rebuild cell at same lat/lon without seed phase
  TerrainCache tc0;
  tc0.configure(64, 64, 1.0, 0, frame); // seed 0 → phase 0
  tc0.rebuild();
  const real h_cache = tc0.sample_height(0, 0);
  const real h_eval = TerrainCache::evaluate_surface(frame.latitude, frame.longitude, 0, 0).height;
  CHECK(std::abs(h_cache - h_eval) < 1e-6, "cache origin matches evaluate_surface");

  std::cout << "RESULT: " << g_pass << " PASS, " << g_fail << " FAIL\n";
  return g_fail ? 1 : 0;
}
