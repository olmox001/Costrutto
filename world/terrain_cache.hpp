/* SPDX-License-Identifier: GPL-2.0-or-later
 * Cached terrain grid driven by the same multi-harmonic field as
 * earth::TerrainGenerator (no toy hash noise). Stores raw cell data, seed,
 * dimensions; samples with bilinear height + analytic slope; lat/lon from
 * local ENU on the planetary ellipsoid.
 */
#pragma once

#include "physics/nasa_rules.hpp"
#include "world/planet_frame.hpp"
#include "world/spherical_harmonics.hpp"
#include "nqg_engine3d.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

namespace nqg {
namespace world {

using real = double;
using engine::Rgb;

enum class TerrainSurface : uint8_t {
  Water = 0,
  Sand,
  Grass,
  Rock,
  Snow,
  Ice
};

struct TerrainCell {
  real height = 0;       // orthometric-style height [m] (same units as baseHeight)
  real roughness = 0.2;
  real tempC = 15.0;
  Rgb albedo{0.3f, 0.5f, 0.25f};
  TerrainSurface type = TerrainSurface::Grass;
};

class TerrainCache {
public:
  // High-resolution local patch (metres). Cap keeps NASA bounded loops.
  static constexpr int MAX_RES = 2048;
  static constexpr int DEFAULT_RES = 512;
  static constexpr real DEFAULT_CELL_M = 0.25; // 512 * 0.25 m ≈ 128 m patch

  void configure(int nx, int ny, real cell_m, uint32_t seed,
                 const LocalFrame &frame) {
    NQG_REQUIRE(nx > 1 && nx <= MAX_RES);
    NQG_REQUIRE(ny > 1 && ny <= MAX_RES);
    NQG_REQUIRE(cell_m > 0);
    nx_ = nx;
    ny_ = ny;
    cell_ = cell_m;
    seed_ = seed;
    frame_ = frame;
    cells_.assign(std::size_t(nx_) * std::size_t(ny_), TerrainCell{});
    built_ = false;
    if (use_sh_)
      sh_band_ = sh::synthesize_seed(seed, std::min(32, sh::L_RUNTIME_MAX));
  }


  uint32_t seed() const { return seed_; }
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  real cell_size() const { return cell_; }
  const std::vector<TerrainCell> &raw_cells() const { return cells_; }
  // Back-compat for callers that only need heights
  std::vector<real> raw_heights() const {
    std::vector<real> h(cells_.size());
    for (std::size_t i = 0; i < cells_.size(); ++i)
      h[i] = cells_[i].height;
    return h;
  }

  // Fill cache from the real planetary harmonic field (same coefficients as
  // earth::TerrainGenerator::baseHeight) evaluated at each cell's lat/lon.
  void rebuild() {
    NQG_REQUIRE(nx_ > 1 && ny_ > 1);
    const real origin_lat = frame_.latitude;
    const real origin_lon = frame_.longitude;
    const real R = frame_.body.radius_at_latitude(origin_lat);
    NQG_REQUIRE(R > 0);
    const real cos_lat = std::cos(origin_lat);
    const real inv_R = 1.0 / R;
    const real inv_Rcos = 1.0 / std::max(R * std::max(std::abs(cos_lat), 1e-6), 1.0);

    // Seed perturbs phase of higher harmonics only (deterministic, continuous).
    const real phase = seed_phase(seed_);

    for (int j = 0; j < ny_; ++j) {
      for (int i = 0; i < nx_; ++i) {
        const real east = (real(i) - 0.5 * nx_) * cell_;
        const real north = (real(j) - 0.5 * ny_) * cell_;
        // ENU → geographic offsets (local tangent plane on ellipsoid)
        const real lat = origin_lat + north * inv_R;
        const real lon = origin_lon + east * inv_Rcos;
        if (use_sh_) {
          TerrainCell cell;
          cell.height = sh::height_latlon(sh_band_, lat, lon);
          // classify using same rules as evaluate_surface
          TerrainCell cls = evaluate_surface(lat, lon, 0.0, 0.0);
          cell.type = cls.type;
          cell.roughness = cls.roughness;
          cell.albedo = cls.albedo;
          cell.tempC = cls.tempC;
          // replace height with SH (classification thresholds still from legacy abs height scale)
          cells_[idx(i, j)] = cell;
        } else {
          cells_[idx(i, j)] = evaluate_surface(lat, lon, 0.0, phase);
        }
      }
    }
    built_ = true;
  }

  // Continuous height at local ENU (east, north) [m]. Bilinear on cache.
  real sample_height(real east, real north) const {
    NQG_REQUIRE(built_);
    real h00, h10, h01, h11, tx, ty;
    gather(east, north, h00, h10, h01, h11, tx, ty);
    const real hx0 = h00 * (1.0 - tx) + h10 * tx;
    const real hx1 = h01 * (1.0 - tx) + h11 * tx;
    return hx0 * (1.0 - ty) + hx1 * ty;
  }

  // Alias used by scene
  real sample(real east, real north) const { return sample_height(east, north); }

  // Analytic-style slope from bilinear gradients [dh/deast, dh/dnorth]
  void sample_slope(real east, real north, real &dh_de, real &dh_dn) const {
    NQG_REQUIRE(built_);
    real h00, h10, h01, h11, tx, ty;
    gather(east, north, h00, h10, h01, h11, tx, ty);
    const real inv_c = 1.0 / cell_;
    // ∂h/∂tx, ∂h/∂ty then scale by cell size
    const real dh_dtx = (1.0 - ty) * (h10 - h00) + ty * (h11 - h01);
    const real dh_dty = (1.0 - tx) * (h01 - h00) + tx * (h11 - h10);
    dh_de = dh_dtx * inv_c;
    dh_dn = dh_dty * inv_c;
  }

  TerrainCell sample_cell(real east, real north) const {
    NQG_REQUIRE(built_);
    const real ox = east / cell_ + 0.5 * nx_;
    const real oy = north / cell_ + 0.5 * ny_;
    int i = (int)std::floor(ox + 0.5);
    int j = (int)std::floor(oy + 0.5);
    i = clampi(i, 0, nx_ - 1);
    j = clampi(j, 0, ny_ - 1);
    TerrainCell c = cells_[idx(i, j)];
    c.height = sample_height(east, north);
    return c;
  }

  // Direct evaluation (no cache) — same math as rebuild, for verification.
  static TerrainCell evaluate_surface(real lat, real lon, real alt_above_field,
                                      real phase = 0.0) {
    TerrainCell s;
    s.height = base_height(lat, lon, phase);
    const real a = std::abs(lat);
    constexpr real PI_E = 3.14159265358979323846;
    if (s.height < 0.0) {
      s.type = TerrainSurface::Water;
      s.roughness = 0.02;
      s.albedo = {0.05f, 0.15f, 0.42f};
    } else if (a > 1.18 || s.height > 3800.0) {
      s.type = TerrainSurface::Snow;
      s.roughness = 0.12;
      s.albedo = {0.92f, 0.94f, 0.98f};
    } else if (a > 1.05) {
      s.type = TerrainSurface::Ice;
      s.roughness = 0.05;
      s.albedo = {0.78f, 0.86f, 0.92f};
    } else if (s.height < 4.0) {
      s.type = TerrainSurface::Sand;
      s.roughness = 0.08;
      s.albedo = {0.82f, 0.72f, 0.48f};
    } else if (s.height < 450.0) {
      s.type = TerrainSurface::Grass;
      s.roughness = 0.25;
      s.albedo = {0.26f, 0.46f, 0.20f};
    } else {
      s.type = TerrainSurface::Rock;
      s.roughness = 0.55;
      s.albedo = {0.44f, 0.39f, 0.34f};
    }
    s.tempC = 30.0 - 6.5 * (s.height / 1000.0) - 50.0 * (a / PI_E) -
              0.0065 * alt_above_field;
    return s;
  }

  // Identical harmonic expansion to earth::TerrainGenerator::baseHeight
  // with optional phase shift from seed (does not change coefficient structure).
  static real base_height(real lat, real lon, real phase = 0.0) {
    if (!std::isfinite(lat) || !std::isfinite(lon))
      return 0.0;
    const real p = phase;
    const real sl1 = std::sin(lat * 1.7 + 0.3 + p);
    const real cl1 = std::cos(lon * 1.1 + p * 0.7);
    const real sl2 = std::sin(lat * 3.1 + 1.2 + p * 1.1);
    const real cl2 = std::cos(lon * 2.3 + 0.5 + p * 0.5);
    const real sl3 = std::sin(lat * 5.7 + 2.1 + p * 0.3);
    const real cl3 = std::cos(lon * 4.7 + 1.7 + p * 0.9);
    const real sl4 = std::sin(lat * 11.3 + 0.7 + p * 1.3);
    const real cl4 = std::cos(lon * 9.1 + 2.5 + p * 0.2);
    const real sl5 = std::sin(lat * 23.1 + 3.3 + p * 0.6);
    const real cl5 = std::cos(lon * 17.9 + 0.9 + p * 1.0);
    real h = 900.0 * sl1 * cl1 + 450.0 * sl2 * cl2 + 180.0 * sl3 * cl3 +
             70.0 * sl4 * cl4 + 25.0 * sl5 * cl5 - 250.0;
    const real aa = std::abs(lat);
    if (aa > 1.20)
      h += (aa - 1.20) * 1200.0;
    return h;
  }

private:
  static real seed_phase(uint32_t seed) {
    // Map seed → [0, 2π) without changing harmonic amplitudes.
    constexpr real TWO_PI = 6.28318530717958647692;
    return (real(seed % 1000003u) / 1000003.0) * TWO_PI;
  }

  static int clampi(int v, int lo, int hi) {
    if (v < lo)
      return lo;
    if (v > hi)
      return hi;
    return v;
  }

  std::size_t idx(int i, int j) const {
    return std::size_t(j) * std::size_t(nx_) + std::size_t(i);
  }

  void gather(real east, real north, real &h00, real &h10, real &h01, real &h11,
              real &tx, real &ty) const {
    const real ox = east / cell_ + 0.5 * nx_;
    const real oy = north / cell_ + 0.5 * ny_;
    int i0 = (int)std::floor(ox);
    int j0 = (int)std::floor(oy);
    i0 = clampi(i0, 0, nx_ - 2);
    j0 = clampi(j0, 0, ny_ - 2);
    tx = ox - i0;
    ty = oy - j0;
    if (tx < 0)
      tx = 0;
    if (ty < 0)
      ty = 0;
    if (tx > 1)
      tx = 1;
    if (ty > 1)
      ty = 1;
    h00 = cells_[idx(i0, j0)].height;
    h10 = cells_[idx(i0 + 1, j0)].height;
    h01 = cells_[idx(i0, j0 + 1)].height;
    h11 = cells_[idx(i0 + 1, j0 + 1)].height;
  }

  int nx_ = 0, ny_ = 0;
  real cell_ = DEFAULT_CELL_M;
  uint32_t seed_ = 1337;
  LocalFrame frame_{};
  sh::Band sh_band_{};
  // Keep the cache on the canonical TerrainGenerator-compatible field until
  // the seeded SH band has an equivalent public evaluation contract.
  bool use_sh_ = false;
  std::vector<TerrainCell> cells_;
  bool built_ = false;
};

} // namespace world
} // namespace nqg
