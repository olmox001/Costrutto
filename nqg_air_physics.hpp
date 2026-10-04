// ============================================================================
//  nqg_air_physics.hpp  -  Termodinamica e Meccanica dei Fluidi dell'Aria
//  Equazione di stato gas perfetti, legge di Sutherland per viscosita',
//  velocita' del suono, regimi di Reynolds e resistenza aerodinamica.
// ============================================================================
#ifndef NQG_AIR_PHYSICS_HPP
#define NQG_AIR_PHYSICS_HPP

#include "nqg_physics_core.hpp"
#include "nqg_engine3d.hpp"

namespace nqg {
namespace cleanroom {

using engine::Vec3;
using engine::Rgb;

struct AirProperties {
  real temperatureK = 293.15; // 20 °C in Kelvin
  real pressurePa = 101325.0; // 1 atm standard a livello del mare
  Vec3 windVelocity = Vec3(0, 0, 0); // m/s
  real scatteringCoeff = 0.0035;     // 1/m (estinzione ottica aria pulita)
  Rgb airHazeColor = {0.85f, 0.90f, 0.98f}; // Diffusione Rayleigh atmosferica

  // Costante dei gas perfetti specifica per aria secca: R = 287.058 J/(kg K)
  static constexpr real R_air = 287.058;
  static constexpr real gamma_air = 1.4; // Rapporto calori specifici Cp/Cv

  // Densita' dell'aria: rho = P / (R * T) [kg/m^3]
  real density() const {
    if (temperatureK <= 1.0) return 0.0;
    return pressurePa / (R_air * temperatureK);
  }

  // Viscosita' dinamica dell'aria secondo la legge di Sutherland:
  // mu = mu0 * (T/T0)^(3/2) * (T0 + S) / (T + S)
  real dynamicViscosity() const {
    constexpr real T0 = 273.15;
    constexpr real mu0 = 1.716e-5; // Pa*s a 0 °C
    constexpr real S = 110.4;      // Costante di Sutherland per aria
    if (temperatureK <= 1.0) return 0.0;
    return mu0 * std::pow(temperatureK / T0, 1.5) * (T0 + S) / (temperatureK + S);
  }

  // Viscosita' cinematica: nu = mu / rho [m^2/s]
  real kinematicViscosity() const {
    real rho = density();
    return rho > 1e-9 ? dynamicViscosity() / rho : 0.0;
  }

  // Velocita' del suono nell'aria: c_s = sqrt(gamma * R * T) [m/s]
  real speedOfSound() const {
    return std::sqrt(gamma_air * R_air * temperatureK);
  }

  // Coefficiente di resistenza aerodinamica per una sfera in funzione di Reynolds
  static real sphereDragCoefficient(real Re) {
    if (Re < 1e-6) return 0.0;
    if (Re < 1.0) {
      // Regime laminare puro (Legge di Stokes): Cd = 24 / Re
      return 24.0 / Re;
    } else if (Re < 1000.0) {
      // Regime di transizione (Formula di Schiller-Naumann)
      return (24.0 / Re) * (1.0 + 0.15 * std::pow(Re, 0.687));
    } else if (Re < 2.0e5) {
      // Regime turbolento subcritico (quasi costante per sfera liscia)
      return 0.44;
    } else {
      // Crisi della resistenza (boundary layer turbolento)
      return 0.15;
    }
  }

  // Calcolo delle forze aerodinamiche (Resistenza + Spinta di Archimede)
  void computeAerodynamicForces(real sphereRadius, real sphereMass, const Vec3 &pos,
                                const Vec3 &vel, Vec3 &fDrag, Vec3 &fBuoyancy, real &ReOut) const {
    (void)pos;
    (void)sphereMass;
    const real rho = density();
    const real mu = dynamicViscosity();
    const real area = PI * sphereRadius * sphereRadius;
    const real volume = (4.0 / 3.0) * PI * std::pow(sphereRadius, 3.0);

    // 1. Spinta di Archimede: F_b = - rho_aria * Volume * g
    const real g = 9.80665;
    fBuoyancy = Vec3(0, 0, rho * volume * g);

    // 2. Velocita' relativa rispetto al vento
    Vec3 vRel = vel - windVelocity;
    real vMag = vRel.norm();

    if (vMag < 1e-7 || rho < 1e-9) {
      fDrag = Vec3(0, 0, 0);
      ReOut = 0.0;
      return;
    }

    // 3. Numero di Reynolds: Re = (rho * v * D) / mu
    real diameter = 2.0 * sphereRadius;
    real Re = (rho * vMag * diameter) / (mu + 1e-12);
    ReOut = Re;

    real Cd = sphereDragCoefficient(Re);

    // 4. Forza di resistenza aerodinamica: F_d = - 0.5 * rho * Cd * A * v * vRel_unit
    real dragMag = 0.5 * rho * Cd * area * vMag * vMag;
    fDrag = vRel.normalized() * (-dragMag);
  }
};

} // namespace cleanroom
} // namespace nqg

#endif // NQG_AIR_PHYSICS_HPP
