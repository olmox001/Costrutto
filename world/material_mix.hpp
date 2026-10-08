/* SPDX-License-Identifier: GPL-2.0-or-later
 * Material mixture: aesthetic fractions map to real internal volumes.
 */
#pragma once
#include "physics/nasa_rules.hpp"
#include "nqg_materials.hpp"
#include <array>

namespace nqg {
namespace world {
using real = double;

struct MixComponent {
  materials::Id id = materials::Id::Concrete;
  real fraction = 0;
};

struct MaterialMix {
  static constexpr int MAX_COMP = 8;
  std::array<MixComponent, MAX_COMP> comps{};
  int count = 0;
  void clear() { count = 0; }
  void add(materials::Id id, real fraction) {
    NQG_REQUIRE(fraction >= 0);
    NQG_REQUIRE(count < MAX_COMP);
    comps[count++] = {id, fraction};
  }
  void normalize() {
    real s = 0;
    for (int i = 0; i < count; ++i) s += comps[i].fraction;
    if (s <= 1e-12) return;
    for (int i = 0; i < count; ++i) comps[i].fraction /= s;
  }
  real density() const {
    real d = 0;
    for (int i = 0; i < count; ++i)
      d += comps[i].fraction * materials::get(comps[i].id).density;
    return d;
  }
  real component_volume(real V, int i) const {
    NQG_REQUIRE(V >= 0 && i >= 0 && i < count);
    return V * comps[i].fraction;
  }
  real component_mass(real V, int i) const {
    return component_volume(V, i) * materials::get(comps[i].id).density;
  }
  engine::Rgb albedo() const {
    engine::Rgb a{0, 0, 0};
    for (int i = 0; i < count; ++i) {
      const auto &m = materials::get(comps[i].id);
      a.r += float(comps[i].fraction) * m.albedo.r;
      a.g += float(comps[i].fraction) * m.albedo.g;
      a.b += float(comps[i].fraction) * m.albedo.b;
    }
    return a;
  }
};
} // namespace world
} // namespace nqg
