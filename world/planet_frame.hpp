/* SPDX-License-Identifier: GPL-2.0-or-later
 * Planet reference frame: equatorial sinusoids, polar flattening, COM, ground.
 * Local ENU coordinates attach to a geographic origin on the body.
 */
#pragma once

#include "physics/nasa_rules.hpp"
#include <cmath>

namespace nqg {
namespace world {

using real = double;

// WGS-84-like ellipsoid parameters (Earth defaults; other bodies override).
struct PlanetBody {
  real equatorial_radius = 6378137.0; // a [m]
  real polar_radius = 6356752.314245; // b [m]
  real mass = 5.9722e24;              // [kg]
  real rotation_rate = 7.292115e-5;    // omega [rad/s]
  real mean_sea_level = 0.0;          // reference geoid offset [m]

  real flattening() const {
    NQG_REQUIRE(equatorial_radius > 0);
    return (equatorial_radius - polar_radius) / equatorial_radius;
  }

  real eccentricity_sq() const {
    const real a = equatorial_radius;
    const real b = polar_radius;
    NQG_REQUIRE(a > 0);
    return (a * a - b * b) / (a * a);
  }

  // Geocentric radius at geodetic latitude (sinusoidal polar reduction).
  real radius_at_latitude(real lat_rad) const {
    NQG_REQUIRE(std::isfinite(lat_rad));
    const real s = std::sin(lat_rad);
    const real c = std::cos(lat_rad);
    const real a = equatorial_radius;
    const real b = polar_radius;
    const real num = (a * a * c) * (a * a * c) + (b * b * s) * (b * b * s);
    const real den = (a * c) * (a * c) + (b * s) * (b * s);
    NQG_REQUIRE(den > 0);
    return std::sqrt(num / den);
  }

  // Surface gravity magnitude (approx. Somigliana-lite).
  real surface_gravity(real lat_rad) const {
    NQG_REQUIRE(std::isfinite(lat_rad));
    constexpr real G = 6.67430e-11;
    const real r = radius_at_latitude(lat_rad);
    NQG_REQUIRE(r > 0);
    const real g0 = G * mass / (r * r);
    const real s = std::sin(lat_rad);
    return g0 - rotation_rate * rotation_rate * r * (c_cos2(s));
  }

private:
  static real c_cos2(real s) {
    const real c2 = 1.0 - s * s;
    return c2; // centrifugal term ~ omega^2 * r * cos^2(lat)
  }
};

// Geographic origin + local ENU frame on the planet.
struct LocalFrame {
  PlanetBody body;
  real latitude = 0;  // rad
  real longitude = 0; // rad
  real ground_altitude = 0; // height above mean radius [m]

  real ground_radius() const {
    return body.radius_at_latitude(latitude) + ground_altitude;
  }

  // Closed-surface centre of mass for a uniform spherical shell approx.
  // For a solid sphere COM is at centre; shell COM at centre too.
  void center_of_mass(real &x, real &y, real &z) const {
    x = 0;
    y = 0;
    z = 0; // planet COM at origin of body frame
  }

  // ECEF-like from local ENU (east, north, up) deltas.
  void enu_to_ecef(real east, real north, real up, real &X, real &Y,
                   real &Z) const {
    const real r = ground_radius();
    const real clat = std::cos(latitude);
    const real slat = std::sin(latitude);
    const real clon = std::cos(longitude);
    const real slon = std::sin(longitude);
    // ENU basis
    const real ex = -slon, ey = clon, ez = 0;
    const real nx = -slat * clon, ny = -slat * slon, nz = clat;
    const real ux = clat * clon, uy = clat * slon, uz = slat;
    X = (r * ux) + east * ex + north * nx + up * ux;
    Y = (r * uy) + east * ey + north * ny + up * uy;
    Z = (r * uz) + east * ez + north * nz + up * uz;
  }
};

} // namespace world
} // namespace nqg
