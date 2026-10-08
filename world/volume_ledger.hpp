/* SPDX-License-Identifier: GPL-2.0-or-later
 * Global volume conservation: matter is never destroyed.
 * Fine debris → atmospheric mass fraction; humidity can redeposit as film.
 */
#pragma once

#include "physics/nasa_rules.hpp"
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace nqg {
namespace world {

using real = double;

struct MaterialFraction {
  std::string id; // e.g. "concrete", "quartz"
  real volume = 0; // m^3 of solid-equivalent
};

// Probabilistic / percentage state for performance (no per-grain entities).
struct AirLoad {
  real vapor_volume = 0;   // equivalent liquid volume of H2O [m^3]
  real dust_volume = 0;    // suspended fines [m^3]
  real humidity = 0.4;     // [0,1] relative — drives deposition probability
};

class VolumeLedger {
public:
  void add_solid(const std::string &mat, real vol) {
    NQG_REQUIRE(vol >= 0);
    solid_[mat] += vol;
    total_matter_ += vol;
  }

  void transfer_solid_to_air_dust(const std::string &mat, real vol) {
    NQG_REQUIRE(vol >= 0);
    real &s = solid_[mat];
    const real take = std::min(s, vol);
    s -= take;
    air_.dust_volume += take;
  }

  void deposit_dust_on_surface(real vol) {
    NQG_REQUIRE(vol >= 0);
    const real take = std::min(air_.dust_volume, vol);
    air_.dust_volume -= take;
    solid_["surface_film"] += take;
  }

  // Humidity: probability of depositing vapor as film on material this step.
  real deposition_probability(real humidity, real surface_roughness) const {
    NQG_REQUIRE(humidity >= 0 && humidity <= 1.5);
    NQG_REQUIRE(surface_roughness >= 0);
    return std::clamp(humidity * (0.2 + 0.5 * surface_roughness), 0.0, 1.0);
  }

  real total_matter() const {
    real s = air_.dust_volume + air_.vapor_volume;
    for (const auto &kv : solid_)
      s += kv.second;
    return s;
  }
  real solid_volume(const std::string &mat) const {
    auto it = solid_.find(mat);
    return it == solid_.end() ? 0.0 : it->second;
  }
  AirLoad &air() { return air_; }
  const AirLoad &air() const { return air_; }

  // Verify conservation within epsilon (call after operations).
  bool check_conservation(real expected, real eps = 1e-6) const {
    return std::abs(total_matter() - expected) <= eps * std::max(1.0, expected);
  }

private:
  std::unordered_map<std::string, real> solid_;
  AirLoad air_;
  real total_matter_ = 0;
};

// Fragment a solid of volume V into N pieces with sizes from a power-law
// (classical brittle fragmentation ~ r^{-α}); residual below r_min → dust.
inline void fragment_volume(real volume, real r_min, real r_max, real alpha,
                            std::vector<real> &out_volumes, real &dust_out) {
  NQG_REQUIRE(volume > 0);
  NQG_REQUIRE(r_min > 0 && r_max > r_min);
  NQG_REQUIRE(alpha > 1.0);
  out_volumes.clear();
  dust_out = 0;
  real remaining = volume;
  // Bounded number of fragments
  constexpr int MAX_FRAG = 256;
  for (int n = 0; n < MAX_FRAG && remaining > 1e-12; ++n) {
    const real t = (n + 0.5) / MAX_FRAG;
    const real r = r_min * std::pow(r_max / r_min, t);
    const real v = (4.0 / 3.0) * 3.14159265358979323846 * r * r * r;
    if (r < r_min * 1.01) {
      dust_out += remaining;
      remaining = 0;
      break;
    }
    const real take = std::min(remaining, v * std::pow(r / r_max, -alpha + 1));
    if (take < 1e-9)
      continue;
    out_volumes.push_back(take);
    remaining -= take;
  }
  if (remaining > 0)
    dust_out += remaining;
}

} // namespace world
} // namespace nqg
