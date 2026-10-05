// ============================================================================
//  nqg_air_physics.hpp  -  Termodinamica e Meccanica dei Fluidi dell'Aria
//  FIX 2025: 'g' parametrico in computeAerodynamicForces, commento Archimede
//  corretto, guardie numeriche su pow/divisioni.
//  FIX 2025c (performance):
//    - pow(r,1.5) -> r*sqrt(r); pow(Re,0.687) -> exp(0.687*log(Re))
//    - eliminata una divisione vettoriale (vRel/vMag) usando scalare k
//    - v2 = vRel.dot(vRel) riusato per dragMag (evita vMag*vMag)
//    - vRel testata come v2 (no sqrt inutile sul ramo d'uscita)
// ============================================================================
#ifndef NQG_AIR_PHYSICS_HPP
#define NQG_AIR_PHYSICS_HPP

#include "nqg_engine3d.hpp"
#include "nqg_physics_core.hpp"

namespace nqg {
namespace cleanroom {

using engine::Rgb;
using engine::Vec3;

struct AirProperties {
  real temperatureK = 293.15;
  real pressurePa = 101325.0;
  Vec3 windVelocity = Vec3(0, 0, 0);
  real scatteringCoeff = 0.0035;
  Rgb airHazeColor = {0.85f, 0.90f, 0.98f};

  static constexpr real R_air = 287.058;
  static constexpr real gamma_air = 1.4;

  real density() const {
    if (temperatureK <= 1.0)
      return 0.0;
    return pressurePa / (R_air * temperatureK);
  }

  real dynamicViscosity() const {
    constexpr real T0 = 273.15;
    constexpr real mu0 = 1.716e-5;
    constexpr real S = 110.4;
    const real T = temperatureK;
    if (T <= 1.0)
      return 0.0;
    // pow(T/T0, 1.5) = (T/T0) * sqrt(T/T0)
    const real r = T / T0;
    const real r32 = r * std::sqrt(r);
    return mu0 * r32 * (T0 + S) / (T + S);
  }

  real kinematicViscosity() const {
    const real rho = density();
    if (rho < 1e-9 || !std::isfinite(rho))
      return 0.0;
    return dynamicViscosity() / rho;
  }

  real speedOfSound() const {
    const real T = temperatureK > 1.0 ? temperatureK : 1.0;
    return std::sqrt(gamma_air * R_air * T);
  }

  static real sphereDragCoefficient(real Re) {
    if (!(Re > 1e-6))
      return 0.0;
    if (Re < 1.0)
      return 24.0 / Re;
    if (Re < 1000.0) {
      // pow(Re,0.687) = exp(0.687*log(Re))  (una sola trascendente)
      return (24.0 / Re) * (1.0 + 0.15 * std::exp(0.687 * std::log(Re)));
    }
    if (Re < 2.0e5)
      return 0.44;
    return 0.15;
  }

  void computeAerodynamicForces(real sphereRadius, real sphereMass,
                                const Vec3 &pos, const Vec3 &vel, Vec3 &fDrag,
                                Vec3 &fBuoyancy, real &ReOut,
                                real gravityMag = 9.80665) const {
    (void)pos;
    (void)sphereMass;
    const real rho = density();
    real mu = dynamicViscosity();
    if (mu < 1e-12)
      mu = 1e-12;
    const real r2 = sphereRadius * sphereRadius;
    const real area = PI * r2;
    const real volume = (4.0 / 3.0) * PI * r2 * sphereRadius;

    fBuoyancy = Vec3(0, 0, rho * volume * gravityMag);

    const Vec3 vRel = vel - windVelocity;
    const real v2 = vRel.dot(vRel);
    if (v2 < 1e-14 || rho < 1e-9 || !std::isfinite(v2)) {
      fDrag = Vec3(0, 0, 0);
      ReOut = 0.0;
      return;
    }
    const real vMag = std::sqrt(v2);

    const real Re = (rho * vMag * (2.0 * sphereRadius)) / mu;
    ReOut = Re;
    const real Cd = sphereDragCoefficient(Re);
    // dragMag = 1/2 rho Cd A v^2 ; direzione: vRel/vMag * (-dragMag)
    const real k = -(0.5 * rho * Cd * area * v2) / vMag;
    fDrag = Vec3(vRel.x * k, vRel.y * k, vRel.z * k);
  }
};

} // namespace cleanroom
} // namespace nqg

#endif