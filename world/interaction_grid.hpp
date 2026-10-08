/* SPDX-License-Identifier: GPL-2.0-or-later
 * Per-object local interaction grid: SDF samples of the world solid in range.
 * Enables treating the planet as one solid with common contacts while keeping
 * edits (flatten, dig, house pad) in a bounded cache.
 */
#pragma once

#include "physics/nasa_rules.hpp"
#include "nqg_sdf.hpp"
#include "world/volume_ledger.hpp"
#include <cmath>
#include <cstdint>
#include <vector>

namespace nqg {
namespace world {

using real = double;

struct InteractionGrid {
  static constexpr int MAX_N = 128;

  int n = 0;          // n×n×n samples
  real cell = 0.25;   // metres
  real origin_x = 0, origin_y = 0, origin_z = 0;
  std::vector<real> sdf; // signed distance, + outside

  void configure(int n_in, real cell_m, real ox, real oy, real oz) {
    NQG_REQUIRE(n_in > 1 && n_in <= MAX_N);
    NQG_REQUIRE(cell_m > 0);
    n = n_in;
    cell = cell_m;
    origin_x = ox;
    origin_y = oy;
    origin_z = oz;
    sdf.assign(std::size_t(n) * n * n, 0.0);
  }

  std::size_t index(int i, int j, int k) const {
    return std::size_t(k * n * n + j * n + i);
  }

  // World position of sample (i,j,k)
  void sample_pos(int i, int j, int k, real &x, real &y, real &z) const {
    x = origin_x + (i - 0.5 * (n - 1)) * cell;
    y = origin_y + (j - 0.5 * (n - 1)) * cell;
    z = origin_z + (k - 0.5 * (n - 1)) * cell;
  }

  // Trilinear SDF query
  real query(real x, real y, real z) const {
    NQG_REQUIRE(n > 1);
    const real fx = (x - origin_x) / cell + 0.5 * (n - 1);
    const real fy = (y - origin_y) / cell + 0.5 * (n - 1);
    const real fz = (z - origin_z) / cell + 0.5 * (n - 1);
    int i0 = (int)std::floor(fx);
    int j0 = (int)std::floor(fy);
    int k0 = (int)std::floor(fz);
    auto clip = [&](int v) {
      if (v < 0) return 0;
      if (v > n - 2) return n - 2;
      return v;
    };
    i0 = clip(i0);
    j0 = clip(j0);
    k0 = clip(k0);
    const real tx = std::clamp(fx - i0, 0.0, 1.0);
    const real ty = std::clamp(fy - j0, 0.0, 1.0);
    const real tz = std::clamp(fz - k0, 0.0, 1.0);
    auto V = [&](int i, int j, int k) { return sdf[index(i, j, k)]; };
    const real c00 = V(i0, j0, k0) * (1 - tx) + V(i0 + 1, j0, k0) * tx;
    const real c01 = V(i0, j0, k0 + 1) * (1 - tx) + V(i0 + 1, j0, k0 + 1) * tx;
    const real c10 = V(i0, j0 + 1, k0) * (1 - tx) + V(i0 + 1, j0 + 1, k0) * tx;
    const real c11 = V(i0, j0 + 1, k0 + 1) * (1 - tx) + V(i0 + 1, j0 + 1, k0 + 1) * tx;
    const real c0 = c00 * (1 - ty) + c10 * ty;
    const real c1 = c01 * (1 - ty) + c11 * ty;
    return c0 * (1 - tz) + c1 * tz;
  }

  // Fill from a height field: SDF ≈ z - h(x,y) (ground solid below surface).
  template <typename HeightFn>
  void bake_from_height(HeightFn &&h) {
    NQG_REQUIRE(n > 1);
    for (int k = 0; k < n; ++k) {
      for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
          real x, y, z;
          sample_pos(i, j, k, x, y, z);
          const real surface = h(x, y);
          sdf[index(i, j, k)] = z - surface; // <0 below ground
        }
      }
    }
  }

};

// Height field editor with volume ledger of displaced soil.
struct HeightFieldEdit {
  real dx = 0.25;
  int nx = 0, ny = 0;
  real x0 = 0, y0 = 0;
  std::vector<real> h;
  VolumeLedger *ledger = nullptr;
  std::string soil_mat = "regolith";

  void configure(int nx_in, int ny_in, real cell, real ox, real oy) {
    NQG_REQUIRE(nx_in > 1 && ny_in > 1);
    nx = nx_in;
    ny = ny_in;
    dx = cell;
    x0 = ox - 0.5 * (nx - 1) * dx;
    y0 = oy - 0.5 * (ny - 1) * dx;
    h.assign(std::size_t(nx * ny), 0.0);
  }

  real &at(int i, int j) { return h[std::size_t(j * nx + i)]; }
  real at(int i, int j) const { return h[std::size_t(j * nx + i)]; }

  real sample(real x, real y) const {
    const real fx = (x - x0) / dx;
    const real fy = (y - y0) / dx;
    int i0 = (int)std::floor(fx);
    int j0 = (int)std::floor(fy);
    if (i0 < 0) i0 = 0;
    if (j0 < 0) j0 = 0;
    if (i0 > nx - 2) i0 = nx - 2;
    if (j0 > ny - 2) j0 = ny - 2;
    const real tx = fx - i0, ty = fy - j0;
    const real h00 = at(i0, j0), h10 = at(i0 + 1, j0);
    const real h01 = at(i0, j0 + 1), h11 = at(i0 + 1, j0 + 1);
    return (h00 * (1 - tx) + h10 * tx) * (1 - ty) +
           (h01 * (1 - tx) + h11 * tx) * ty;
  }

  // Create a level pad; returns net volume removed (positive = cut).
  real flatten_pad(real cx, real cy, real half_x, real half_y, real target_z) {
    real net = 0;
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {
        const real x = x0 + i * dx;
        const real y = y0 + j * dx;
        if (std::abs(x - cx) > half_x || std::abs(y - cy) > half_y)
          continue;
        const real old = at(i, j);
        const real dV = (old - target_z) * dx * dx; // cut positive
        at(i, j) = target_z;
        net += dV;
      }
    }
    if (ledger && net > 0)
      ledger->add_solid(soil_mat, net); // spoil pile accounted
    else if (ledger && net < 0)
      ledger->transfer_solid_to_air_dust(soil_mat, -net); // fill from stock
    return net;
  }
};

} // namespace world
} // namespace nqg
