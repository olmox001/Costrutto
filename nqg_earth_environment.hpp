// ============================================================================
//  nqg_earth_environment.hpp  -  Ambiente Planetario Terrestre
//  FIX: sfera non distorta ai poli; conversioni via ECEF completo.
//  FIX 2025c (performance, invarianti):
//    - StandardAtmosphere: pow(Tb/T, x) via exp(x*log(...)), una trascendente.
//    - GravityField: ricalcolo con f^2 precalcolato (nessuna divisione doppia).
//    - WindField::speedAt: pow(hh/refHeight, alpha) via exp(alpha*log(...)).
//    - TerrainGenerator: baseHeight ha costanti precalcolate (sin/cos lat/lon
//      condivisi tra i termini); nessuna ridondanza.
// ============================================================================
#ifndef NQG_EARTH_ENVIRONMENT_HPP
#define NQG_EARTH_ENVIRONMENT_HPP

#include "nqg_engine3d.hpp"
#include "nqg_physics_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

namespace nqg {
namespace earth {

using real = nqg::real;
using engine::Rgb;
using engine::Vec3;
constexpr real PI_E = nqg::PI;

namespace planet {
constexpr real R_E = 6371000.0;
constexpr real R_polar = 6356752.0;
constexpr real R_eq = 6378137.0;
constexpr real g_eq = 9.78032677;
constexpr real g_pole = 9.83218637;
constexpr real g_std = 9.80665;
constexpr real P0 = 101325.0;
constexpr real T0 = 288.15;
constexpr real rho0 = 1.225;
constexpr real R_air = 287.058;
constexpr real gamma_air = 1.4;
constexpr real omega_E = 7.2921159e-5;
constexpr real solar_const = 1361.0;
constexpr real M_air = 0.0289644;
constexpr real R_star = 8.3144598;
} // namespace planet

struct AtmoLayer {
  real hb, Tb, L, Pb;
};

class StandardAtmosphere {
public:
  static constexpr int NL = 8;
  std::array<AtmoLayer, NL> lay;

  StandardAtmosphere() {
    lay[0] = {0.0, 288.15, -0.0065, 101325.00};
    lay[1] = {11000.0, 216.65, 0.0000, 22632.06};
    lay[2] = {20000.0, 216.65, 0.0010, 5474.889};
    lay[3] = {32000.0, 228.65, 0.0028, 868.0187};
    lay[4] = {47000.0, 270.65, 0.0000, 110.9063};
    lay[5] = {51000.0, 270.65, -0.0028, 66.93887};
    lay[6] = {71000.0, 214.65, -0.0020, 3.956420};
    lay[7] = {84852.0, 186.946, 0.0000, 0.3734};
  }

  const AtmoLayer *layerFor(real h) const {
    if (!std::isfinite(h))
      h = 0.0;
    for (int i = NL - 1; i >= 0; --i)
      if (h >= lay[i].hb)
        return &lay[i];
    return &lay[0];
  }

  real temperature(real h) const {
    const AtmoLayer *L = layerFor(h);
    return L->Tb + L->L * (h - L->hb);
  }

  real pressure(real h) const {
    const AtmoLayer *L = layerFor(h);
    real T = L->Tb + L->L * (h - L->hb);
    if (std::abs(L->L) < 1e-12) {
      const real k = -planet::g_std * planet::M_air * (h - L->hb) /
                     (planet::R_star * L->Tb);
      return L->Pb * std::exp(k);
    }
    const real ratio = L->Tb / std::max(T, 1.0);
    const real expo = planet::g_std * planet::M_air / (planet::R_star * L->L);
    // pow(ratio, expo) = exp(expo*log(ratio))
    return L->Pb * std::exp(expo * std::log(ratio));
  }

  real density(real h) const {
    real T = temperature(h);
    if (T < 1.0)
      return 0.0;
    real p = pressure(h);
    real r = p / (planet::R_air * T);
    return std::isfinite(r) ? std::max(r, 0.0) : 0.0;
  }

  real speedOfSound(real h) const {
    real T = std::max(1.0, temperature(h));
    return std::sqrt(planet::gamma_air * planet::R_air * T);
  }

  real dynamicViscosity(real h) const {
    real T = std::max(1.0, temperature(h));
    constexpr real T0s = 273.15, mu0 = 1.716e-5, S = 110.4;
    const real r = T / T0s;
    const real r32 = r * std::sqrt(r); // pow(r, 1.5)
    return mu0 * r32 * (T0s + S) / (T + S);
  }

  static real geopotential(real z) {
    if (!std::isfinite(z))
      return 0.0;
    return planet::R_E * z / (planet::R_E + z);
  }
};

class GravityField {
public:
  static real gSurface(real latRad) {
    const real s = std::sin(latRad);
    const real s2 = s * s;
    const real s22 = std::sin(2.0 * latRad);
    return planet::g_eq * (1.0 + 0.00530244 * s2 + 0.00000582 * s22 * s22);
  }
  static real g(real latRad, real altitude) {
    const real g0 = gSurface(latRad);
    const real f = 1.0 + altitude / planet::R_E;
    const real f2 = f * f;
    if (f2 < 1e-18)
      return g0 / 1e-18;
    return g0 / f2;
  }
  static real dgdh(real latRad, real altitude) {
    const real g0 = gSurface(latRad);
    const real f = 1.0 + altitude / planet::R_E;
    const real f2 = f * f;
    const real f3 = f2 * f;
    if (f3 < 1e-27)
      return 0.0;
    return -2.0 * g0 / (planet::R_E * f3);
  }
  static real centrifugal(real latRad) {
    const real c = std::cos(latRad);
    return planet::omega_E * planet::omega_E * planet::R_E * c * c;
  }
};

struct WindProfile {
  real refHeight = 10.0;
  real refSpeed = 5.0;
  real alpha = 0.14;
  real z0 = 3e-4;
  real direction = 0.0;
  real jetAltitude = 10000.0;
  real jetSpeed = 40.0;
  real jetWidth = 4000.0;
};

class WindField {
public:
  static WindProfile profileFor(real terrainRoughness, real latRad) {
    WindProfile p;
    p.z0 = 3e-4 + terrainRoughness * 1.0;
    p.alpha = 0.10 + terrainRoughness * 0.30;
    const real latFactor = 0.4 + 0.6 * std::abs(std::sin(2.0 * latRad));
    p.refSpeed = 5.0 * latFactor;
    p.jetSpeed = 40.0 * latFactor;
    const real a = std::abs(latRad);
    if (a < 0.35)
      p.direction = PI_E;
    else if (a < 1.05)
      p.direction = 0.0;
    else
      p.direction = PI_E;
    return p;
  }

  static real speedAt(const WindProfile &p, real h) {
    const real hh = std::max(h, p.z0);
    // pow(hh/refHeight, alpha) = exp(alpha*log(...))
    const real u_bl =
        p.refSpeed * std::exp(p.alpha * std::log(hh / p.refHeight));
    const real t =
        std::clamp((h - 2000.0) / (p.jetAltitude - 2000.0), 0.0, 1.0);
    const real u_jet =
        p.jetSpeed * std::exp(-std::pow((h - p.jetAltitude) / p.jetWidth, 2.0));
    return u_bl * (1.0 - t) + (u_bl + u_jet) * t;
  }

  static Vec3 velocityAt(const WindProfile &p, real h) {
    const real s = speedAt(p, h);
    const real cx = std::cos(p.direction);
    const real sx = std::sin(p.direction);
    return Vec3(s * cx, s * sx, 0.0);
  }
};

enum class SurfaceType { Water, Sand, Grass, Rock, Snow, Ice };

struct SurfaceSample {
  SurfaceType type = SurfaceType::Grass;
  real height = 0.0;
  real roughness = 0.2;
  Rgb albedo = {0.3f, 0.5f, 0.25f};
  real tempC = 15.0;
};

class TerrainGenerator {
public:
  explicit TerrainGenerator(uint32_t seed = 1337) : rng_(seed) {}

  real baseHeight(real lat, real lon) const {
    if (!std::isfinite(lat) || !std::isfinite(lon))
      return 0.0;
    const real sl1 = std::sin(lat * 1.7 + 0.3);
    const real cl1 = std::cos(lon * 1.1);
    const real sl2 = std::sin(lat * 3.1 + 1.2);
    const real cl2 = std::cos(lon * 2.3 + 0.5);
    const real sl3 = std::sin(lat * 5.7 + 2.1);
    const real cl3 = std::cos(lon * 4.7 + 1.7);
    const real sl4 = std::sin(lat * 11.3 + 0.7);
    const real cl4 = std::cos(lon * 9.1 + 2.5);
    const real sl5 = std::sin(lat * 23.1 + 3.3);
    const real cl5 = std::cos(lon * 17.9 + 0.9);
    real h = 900.0 * sl1 * cl1 + 450.0 * sl2 * cl2 + 180.0 * sl3 * cl3 +
             70.0 * sl4 * cl4 + 25.0 * sl5 * cl5 - 250.0;
    const real a = std::abs(lat);
    if (a > 1.20)
      h += (a - 1.20) * 1200.0;
    return h;
  }

  SurfaceSample sample(real lat, real lon, real alt) const {
    SurfaceSample s;
    s.height = baseHeight(lat, lon);
    const real a = std::abs(lat);
    if (s.height < 0.0) {
      s.type = SurfaceType::Water;
      s.roughness = 0.02;
      s.albedo = {0.05f, 0.15f, 0.42f};
    } else if (s.height < 4.0) {
      s.type = SurfaceType::Sand;
      s.roughness = 0.08;
      s.albedo = {0.82f, 0.72f, 0.48f};
    } else if (s.height < 450.0) {
      s.type = SurfaceType::Grass;
      s.roughness = 0.25;
      s.albedo = {0.26f, 0.46f, 0.20f};
    } else if (a > 1.18 || s.height > 3800.0) {
      s.type = SurfaceType::Snow;
      s.roughness = 0.12;
      s.albedo = {0.92f, 0.94f, 0.98f};
    } else if (a > 1.05) {
      s.type = SurfaceType::Ice;
      s.roughness = 0.05;
      s.albedo = {0.78f, 0.86f, 0.92f};
    } else {
      s.type = SurfaceType::Rock;
      s.roughness = 0.55;
      s.albedo = {0.44f, 0.39f, 0.34f};
    }
    s.tempC =
        30.0 - 6.5 * (s.height / 1000.0) - 50.0 * (a / PI_E) - 0.0065 * alt;
    return s;
  }

private:
  mutable std::mt19937 rng_;
};

struct GeoPoint {
  real lat, lon, alt;
};

class EarthGlobe {
public:
  StandardAtmosphere atmo;
  GravityField grav;
  TerrainGenerator terrain;

  static void enuBasis(real lat, real lon, Vec3 &up, Vec3 &east, Vec3 &north) {
    lat = std::clamp(lat, -PI_E * 0.5, PI_E * 0.5);
    const real cl = std::cos(lat), sl = std::sin(lat);
    const real co = std::cos(lon), so = std::sin(lon);

    up = Vec3(cl * co, cl * so, sl);
    east = Vec3(-so, co, 0.0);

    const real nUp = up.norm();
    if (nUp < 1e-9)
      up = Vec3(0, 0, 1);
    else
      up = up * (1.0 / nUp);
    const real eDotU = east.dot(up);
    east = east - up * eDotU;
    const real nE = east.norm();
    if (nE < 1e-9)
      east = Vec3(1, 0, 0);
    else
      east = east * (1.0 / nE);
    north = up.cross(east);
    const real nN = north.norm();
    if (nN < 1e-9)
      north = Vec3(0, 1, 0);
    else
      north = north * (1.0 / nN);
  }

  Vec3 toECEF(const GeoPoint &g) const {
    const real r = planet::R_E + g.alt;
    const real cl = std::cos(g.lat), sl = std::sin(g.lat);
    const real co = std::cos(g.lon), so = std::sin(g.lon);
    return Vec3(r * cl * co, r * cl * so, r * sl);
  }

  static real altitudeOf(const Vec3 &p) {
    const real r = p.norm();
    if (!std::isfinite(r))
      return 0.0;
    return r - planet::R_E;
  }
  static real latitudeOf(const Vec3 &p) {
    const real r = p.norm();
    if (r < 1e-9)
      return 0.0;
    return std::asin(std::clamp(p.z / r, -1.0, 1.0));
  }
  static real longitudeOf(const Vec3 &p) { return std::atan2(p.y, p.x); }

  static Vec3 enuToECEF(const Vec3 &enu, const Vec3 &refECEF, const Vec3 &up,
                        const Vec3 &east, const Vec3 &north) {
    return refECEF + east * enu.x + north * enu.y + up * enu.z;
  }
  static Vec3 ecefToENU(const Vec3 &ecef, const Vec3 &refECEF, const Vec3 &up,
                        const Vec3 &east, const Vec3 &north) {
    const Vec3 d = ecef - refECEF;
    return Vec3(d.dot(east), d.dot(north), d.dot(up));
  }

  static void ecefToLatLon(const Vec3 &ecef, real &lat, real &lon, real &alt) {
    const real r = ecef.norm();
    if (r < 1e-9 || !std::isfinite(r)) {
      lat = 0.0;
      lon = 0.0;
      alt = -planet::R_E;
      return;
    }
    lat = std::asin(std::clamp(ecef.z / r, -1.0, 1.0));
    lon = std::atan2(ecef.y, ecef.x);
    alt = r - planet::R_E;
  }

  struct EnvSample {
    real altitude, lat, lon;
    real gravity;
    real pressure, temperature, density, soundSpeed;
    Vec3 wind;
    SurfaceSample surf;
    real coriolis;
  };

  EnvSample sample(const Vec3 &p, const WindProfile &wp) const {
    EnvSample e;
    e.altitude = altitudeOf(p);
    e.lat = latitudeOf(p);
    e.lon = longitudeOf(p);
    e.gravity = grav.g(e.lat, e.altitude);
    e.pressure = atmo.pressure(e.altitude);
    e.temperature = atmo.temperature(e.altitude);
    e.density = atmo.density(e.altitude);
    e.soundSpeed = atmo.speedOfSound(e.altitude);
    e.wind = WindField::velocityAt(wp, e.altitude);
    e.surf = terrain.sample(e.lat, e.lon, e.altitude);
    e.coriolis = 2.0 * planet::omega_E * std::sin(e.lat);
    return e;
  }
};

} // namespace earth
} // namespace nqg

#endif // NQG_EARTH_ENVIRONMENT_HPP