/* SPDX-License-Identifier: GPL-2.0-or-later
 * Multi-face fluid films: distribute volume across oriented solid faces.
 * Works for floors, walls, ceilings, and moving rigid bodies.
 */
#pragma once

#include "nasa_rules.hpp"
#include "nqg_engine3d.hpp"
#include <cmath>
#include <algorithm>

namespace nqg {
namespace physics {
namespace fluid_surface {

using engine::Vec3;
using real = double;

struct FaceFilm {
  Vec3 origin;   // face centre (world)
  Vec3 normal;   // outward unit normal
  Vec3 tangent;  // in-plane basis u
  Vec3 bitangent;
  real area = 0;     // [m^2]
  real thickness = 0; // [m] mean film height
  real volume = 0;    // [m^3] = area * thickness
};

// Project gravity onto face → in-plane drainage direction (unit or zero).
inline Vec3 drainage_direction(const Vec3 &face_normal, const Vec3 &gravity) {
  NQG_REQUIRE(std::isfinite(face_normal.x));
  const real gn = gravity.x * face_normal.x + gravity.y * face_normal.y +
                  gravity.z * face_normal.z;
  Vec3 parallel = gravity - face_normal * gn;
  const real m = parallel.norm();
  if (m < 1e-12)
    return Vec3(0, 0, 0);
  return parallel * (1.0 / m);
}

// Stable film thickness limit ~ capillary length scale (order of mm–cm).
inline real capillary_limit(real surface_tension, real density, real g_mag) {
  NQG_REQUIRE(surface_tension > 0);
  NQG_REQUIRE(density > 0);
  NQG_REQUIRE(g_mag > 0);
  return std::sqrt(surface_tension / (density * g_mag));
}

// Split excess volume that cannot stick on a steep face (runs off).
inline void distribute_volume(FaceFilm &film, real added_volume, real max_thick) {
  NQG_REQUIRE(added_volume >= 0);
  NQG_REQUIRE(max_thick > 0);
  NQG_REQUIRE(film.area > 0);
  film.volume += added_volume;
  film.thickness = film.volume / film.area;
  if (film.thickness > max_thick) {
    film.thickness = max_thick;
    film.volume = film.area * max_thick;
  }
}

// Fraction of volume retained vs runoff given face tilt (0=floor, 1=vertical).
inline real retention_fraction(real tilt_from_upright, real roughness) {
  NQG_REQUIRE(tilt_from_upright >= 0 && tilt_from_upright <= 1.0);
  NQG_REQUIRE(roughness >= 0);
  const real hold = std::clamp(0.15 + 0.5 * roughness, 0.0, 0.9);
  return std::clamp(1.0 - tilt_from_upright * (1.0 - hold), 0.05, 1.0);
}

} // namespace fluid_surface
} // namespace physics
} // namespace nqg
