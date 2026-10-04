// ============================================================================
//  nqg_continuum_physics.hpp  -  Acqua, Vento, Sabbia, Solidi, EM, Quanti
//  ---------------------------------------------------------------------------
//  ACQUA: superficie globale + spillway dalla porta
//    - NIENTE bacino circolare invisibile
//    - Dominio = stanza + striscia di tracimazione attraverso la porta
//    - Volume d'acqua (m^3) -> livello uniforme su tutto il dominio
//    - Onde di Gerstner, ripples da impatti, accoppiamento vento
//    - Snell + Fresnel + Beer-Lambert
//    - Archimede ovunque, + drag quadratico sommerso
//    - BACK-COMPAT: basinCenter/basinRadius per test legacy
//  VENTO: curl noise a divergenza nulla
//  SABBIA: BCRE (angolo di riposo 33) con height field continuo
//  SOLIDI: Newton-Euler DP45, gravita, archimede, vento, EM, drag acqua
//  EM: Coulomb + Lorentz + dipoli
//  QUANTI: |psi|^2 + Compton + Schwarzschild
// ============================================================================
#ifndef NQG_CONTINUUM_PHYSICS_HPP
#define NQG_CONTINUUM_PHYSICS_HPP

#include "nqg_air_physics.hpp"
#include "nqg_engine3d.hpp"
#include "nqg_physics_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

namespace nqg {
namespace continuum {

using cleanroom::AirProperties;
using engine::Rgb;
using engine::Vec3;

namespace phys {
using nqg::consts::c;
using nqg::consts::G;
using nqg::consts::hbar;
constexpr real k_coulomb = 8.98755179e9;
constexpr real e_charge = 1.602176634e-19;
constexpr real m_electron = 9.1093837015e-31;
constexpr real mu_0 = 4.0 * PI * 1e-7;
inline real epsilon_0() { return 1.0 / (mu_0 * c * c); }
constexpr real g_earth = 9.80665;
} // namespace phys

// ============================================================================
// 1. ACQUA
// ============================================================================
struct WaterWave {
  real amplitude = 0.005;
  real wavelength = 1.2;
  real speed = 1.5;
  Vec3 dir = Vec3(1, 0.5, 0).normalized();
};

class ContinuousWaterBody {
public:
  // Stanza
  real xMin = -6.0, xMax = 6.0;
  real yMin = -5.0, yMax = 5.0;
  real floorZ = 0.0;

  // Porta + spillway
  real doorX0 = -1.0, doorX1 = 1.0;
  real doorSpillLength = 3.0; // metri oltre la soglia

  // Back-compat (hint per i test)
  Vec3 basinCenter = Vec3(0.0, 0.0, 0.0);
  real basinRadius = 3.0;

  // Stato acqua
  real waterVolume = 0.0;     // m^3 aggiunti
  real intrinsicLevel = 0.02; // 2 cm baseline
  real maxLevel = 2.0;        // 2 m
  real waterDensity = 998.0;
  real refractiveIndex = 1.333;
  Vec3 absorptionCoeff = Vec3(0.55, 0.12, 0.04);

  Vec3 windVel = Vec3(0, 0, 0);

  std::vector<WaterWave> waves;

  struct Ripple {
    Vec3 center;
    real startTime = 0;
    real amplitude = 0.06;
    real frequency = 14.0;
    real speed = 1.8;
    real decay = 1.2;
  };
  std::vector<Ripple> ripples;

  // -------------------------------------------------------------------------
  ContinuousWaterBody() {
    auto addWave = [&](real amp, real lambda, Vec3 d) {
      WaterWave w;
      w.amplitude = amp;
      w.wavelength = lambda;
      w.dir = d.normalized();
      real k = 2.0 * PI / lambda;
      real omega = std::sqrt(phys::g_earth * k * std::tanh(k * 0.5));
      w.speed = omega / k;
      waves.push_back(w);
    };
    addWave(0.005, 1.20, Vec3(1.0, 0.3, 0));
    addWave(0.003, 0.65, Vec3(-0.6, 0.8, 0));
    addWave(0.002, 0.32, Vec3(0.4, -0.9, 0));
  }

  // -------------------------------------------------------------------------
  void setRoomBounds(real xmin, real xmax, real ymin, real ymax, real floorZ_) {
    xMin = xmin;
    xMax = xmax;
    yMin = ymin;
    yMax = ymax;
    floorZ = floorZ_;
    basinCenter = Vec3(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), floorZ_);
    basinRadius = std::sqrt(((xmax - xmin) * (ymax - ymin)) / PI);
  }

  void setDoor(real dx0, real dx1, real spillLen) {
    doorX0 = dx0;
    doorX1 = dx1;
    doorSpillLength = spillLen;
  }

  real mainArea() const {
    return std::max(0.01, (xMax - xMin) * (yMax - yMin));
  }
  real spillArea() const {
    return std::max(0.0, (doorX1 - doorX0) * doorSpillLength);
  }
  real floorArea() const { return std::max(0.01, mainArea() + spillArea()); }

  real currentLevel() const {
    return intrinsicLevel + waterVolume / floorArea();
  }

  // Dominio = stanza + spillway dalla porta
  bool isInsideMain(real x, real y) const {
    return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
  }
  bool isInsideSpillway(real x, real y) const {
    return x >= doorX0 && x <= doorX1 && y >= yMin - doorSpillLength &&
           y < yMin;
  }
  bool isInsideBasin(real x, real y) const {
    return isInsideMain(x, y) || isInsideSpillway(x, y);
  }

  // -------------------------------------------------------------------------
  void addWaterVolume(real m3) {
    real cap = maxLevel * floorArea();
    waterVolume = std::min(cap, waterVolume + std::max(0.0, m3));
  }

  void addImpulse(const Vec3 &pos, real time, real strength = 0.06) {
    Ripple r;
    r.center = pos;
    r.startTime = time;
    r.amplitude = strength;
    ripples.push_back(r);
    addWaterVolume(strength * 0.4 * floorArea());
  }

  void addRipple(const Vec3 &pos, real time, real strength = 0.03) {
    Ripple r;
    r.center = pos;
    r.startTime = time;
    r.amplitude = strength;
    r.frequency = 10.0 + 8.0 * strength;
    r.speed = 1.4 + 4.0 * strength;
    ripples.push_back(r);
    // Cap per performance
    if (ripples.size() > 40) {
      ripples.erase(ripples.begin(), ripples.begin() + (ripples.size() - 40));
    }
  }

  // -------------------------------------------------------------------------
  real evaluateHeight(real x, real y, real t) const {
    if (!isInsideBasin(x, y))
      return -1e9;
    real h = currentLevel();

    for (const auto &w : waves) {
      real k = 2.0 * PI / w.wavelength;
      real phase = k * (w.dir.x * x + w.dir.y * y) - k * w.speed * t;
      h += w.amplitude * std::cos(phase);
    }

    for (const auto &r : ripples) {
      real dt = t - r.startTime;
      if (dt < 0 || dt > 4.0)
        continue;
      real dx = x - r.center.x;
      real dy = y - r.center.y;
      real rDist = std::sqrt(dx * dx + dy * dy);
      real dr = rDist - r.speed * dt;
      real envelope = std::exp(-dr * dr * 12.0) * std::exp(-r.decay * dt);
      h += r.amplitude * std::cos(r.frequency * dr) * envelope;
    }
    return h;
  }

  Vec3 evaluateNormal(real x, real y, real t) const {
    const real eps = 0.008;
    real hL = evaluateHeight(x - eps, y, t);
    real hR = evaluateHeight(x + eps, y, t);
    real hD = evaluateHeight(x, y - eps, t);
    real hU = evaluateHeight(x, y + eps, t);

    real dhdx = (hR - hL) / (2.0 * eps);
    real dhdy = (hU - hD) / (2.0 * eps);
    return Vec3(-dhdx, -dhdy, 1.0).normalized();
  }

  // -------------------------------------------------------------------------
  bool intersectWater(const Vec3 &ro, const Vec3 &rd, real time, real &tOut,
                      Vec3 &nOut, real &depthOut) const {
    if (std::abs(rd.z) < 1e-6)
      return false;

    real tEst = (currentLevel() - ro.z) / rd.z;
    if (tEst < 0.01)
      return false;

    real t = tEst;
    for (int iter = 0; iter < 4; ++iter) {
      Vec3 p = ro + rd * t;
      if (!isInsideBasin(p.x, p.y))
        return false;
      real h = evaluateHeight(p.x, p.y, time);
      real err = p.z - h;
      t -= err / rd.z;
    }

    Vec3 hitP = ro + rd * t;
    if (!isInsideBasin(hitP.x, hitP.y))
      return false;

    real hFinal = evaluateHeight(hitP.x, hitP.y, time);
    if (std::abs(hitP.z - hFinal) > 0.10)
      return false;

    tOut = t;
    nOut = evaluateNormal(hitP.x, hitP.y, time);
    depthOut = std::max(0.01, hitP.z - floorZ);
    return true;
  }

  // -------------------------------------------------------------------------
  static bool refractRay(const Vec3 &I, const Vec3 &N, real eta,
                         Vec3 &refracted) {
    real nDotI = N.dot(I);
    real cosThetaI = -nDotI;
    Vec3 norm = N;
    if (cosThetaI < 0) {
      cosThetaI = -cosThetaI;
      norm = N * (-1.0);
      eta = 1.0 / eta;
    }
    real sin2ThetaT = eta * eta * (1.0 - cosThetaI * cosThetaI);
    if (sin2ThetaT > 1.0)
      return false;
    real cosThetaT = std::sqrt(1.0 - sin2ThetaT);
    refracted = (I * eta + norm * (eta * cosThetaI - cosThetaT)).normalized();
    return true;
  }

  static real fresnelDielectric(real cosTheta, real n1 = 1.0, real n2 = 1.333) {
    real r0 = (n1 - n2) / (n1 + n2);
    r0 = r0 * r0;
    return r0 +
           (1.0 - r0) * std::pow(1.0 - std::clamp(cosTheta, 0.0, 1.0), 5.0);
  }

  Vec3 beerLambertTransmission(real pathLength) const {
    return Vec3(std::exp(-absorptionCoeff.x * pathLength),
                std::exp(-absorptionCoeff.y * pathLength),
                std::exp(-absorptionCoeff.z * pathLength));
  }

  // -------------------------------------------------------------------------
  // Archimede + drag acqua applicato al corpo
  // -------------------------------------------------------------------------
  Vec3 buoyancyForce(const Vec3 &bodyPos, const Vec3 &bodySize,
                     real time) const {
    if (!isInsideBasin(bodyPos.x, bodyPos.y))
      return Vec3(0, 0, 0);
    real waterH = evaluateHeight(bodyPos.x, bodyPos.y, time);
    real bodyBot = bodyPos.z - bodySize.z * 0.5;
    real bodyTop = bodyPos.z + bodySize.z * 0.5;
    real submerged = std::clamp(waterH - bodyBot, 0.0, bodyTop - bodyBot);
    if (submerged <= 0.0)
      return Vec3(0, 0, 0);
    real subVol = bodySize.x * bodySize.y * submerged;
    return Vec3(0, 0, waterDensity * subVol * phys::g_earth);
  }

  // Frazione sommersa 0..1 (per drag)
  real submergedFraction(const Vec3 &bodyPos, const Vec3 &bodySize,
                         real time) const {
    if (!isInsideBasin(bodyPos.x, bodyPos.y))
      return 0.0;
    real waterH = evaluateHeight(bodyPos.x, bodyPos.y, time);
    real bodyBot = bodyPos.z - bodySize.z * 0.5;
    real bodyTop = bodyPos.z + bodySize.z * 0.5;
    real submerged = std::clamp(waterH - bodyBot, 0.0, bodyTop - bodyBot);
    return std::clamp(submerged / std::max(0.01, bodySize.z), 0.0, 1.0);
  }

  // -------------------------------------------------------------------------
  void updateFromWind(const Vec3 &wind, real dt) {
    windVel = wind;
    real ws = std::sqrt(wind.x * wind.x + wind.y * wind.y);

    if (ws < 0.05) {
      for (auto &w : waves)
        w.amplitude = std::max(0.001, w.amplitude - 0.02 * dt);
      return;
    }

    Vec3 dir = Vec3(wind.x, wind.y, 0).normalized();
    real targetAmp = std::min(0.03, 0.003 + ws * 0.0035);

    for (auto &w : waves) {
      w.dir = dir;
      w.amplitude += (targetAmp - w.amplitude) * std::min(1.0, 2.5 * dt);
      w.wavelength =
          std::clamp(w.wavelength * (1.0 + 0.05 * ws * dt * 0.1), 0.15, 4.0);
    }
  }

  void cleanupRipples(real simTime) {
    ripples.erase(std::remove_if(ripples.begin(), ripples.end(),
                                 [simTime](const Ripple &r) {
                                   return (simTime - r.startTime) > 5.0;
                                 }),
                  ripples.end());
  }
};

// ============================================================================
// 2. VENTO
// ============================================================================
class ContinuousWindField {
public:
  Vec3 baseDrift = Vec3(1.2, 0.4, 0.0);
  real turbulenceIntensity = 2.4;
  real spatialScale = 0.6;

  static Vec3 vectorPotential(const Vec3 &p, real t) {
    real px = p.x * 0.7 + t * 0.4;
    real py = p.y * 0.7 - t * 0.3;
    real pz = p.z * 0.7 + t * 0.2;
    real psiX = std::sin(py * 1.5 + 0.4) * std::cos(pz * 1.2);
    real psiY = std::sin(pz * 1.5 + 0.9) * std::cos(px * 1.2);
    real psiZ = std::sin(px * 1.5 + 1.2) * std::cos(py * 1.2);
    return Vec3(psiX, psiY, psiZ);
  }

  Vec3 evaluateVelocity(const Vec3 &p, real t) const {
    const real e = 0.01;
    Vec3 pXp = vectorPotential(p + Vec3(e, 0, 0), t);
    Vec3 pXm = vectorPotential(p - Vec3(e, 0, 0), t);
    Vec3 pYp = vectorPotential(p + Vec3(0, e, 0), t);
    Vec3 pYm = vectorPotential(p - Vec3(0, e, 0), t);
    Vec3 pZp = vectorPotential(p + Vec3(0, 0, e), t);
    Vec3 pZm = vectorPotential(p - Vec3(0, 0, e), t);

    real dPz_dy = (pYp.z - pYm.z) / (2.0 * e);
    real dPy_dz = (pZp.y - pZm.y) / (2.0 * e);
    real dPx_dz = (pZp.x - pZm.x) / (2.0 * e);
    real dPz_dx = (pXp.z - pXm.z) / (2.0 * e);
    real dPy_dx = (pXp.y - pXm.y) / (2.0 * e);
    real dPx_dy = (pYp.x - pYm.x) / (2.0 * e);

    Vec3 turbulent(dPz_dy - dPy_dz, dPx_dz - dPz_dx, dPy_dx - dPx_dy);
    return baseDrift + turbulent * turbulenceIntensity;
  }

  Vec3 windForceOnBody(const Vec3 &bodyPos, const Vec3 &bodyVel,
                       const Vec3 &bodySize, real airDensity,
                       real simTime) const {
    Vec3 windVel = evaluateVelocity(bodyPos, simTime);
    Vec3 vRel = windVel - bodyVel;
    real vMag = vRel.norm();
    if (vMag < 1e-5)
      return Vec3(0, 0, 0);
    real area = std::max(bodySize.y * bodySize.z, bodySize.x * bodySize.z);
    real Cd = 1.05;
    real forceMag = 0.5 * airDensity * Cd * area * vMag * vMag;
    return vRel.normalized() * forceMag;
  }
};

// ============================================================================
// 3. SABBIA
// ============================================================================
class ContinuousSandDuneField {
public:
  Vec3 sandCenter = Vec3(3.2, 2.5, 0.0);
  real sandRadius = 2.2;
  real criticalSlope = 0.65;

  static constexpr int GRID_N = 64;
  std::array<std::array<real, GRID_N>, GRID_N> height{};
  real gridSpacing = 0.08;

  ContinuousSandDuneField() {
    for (int j = 0; j < GRID_N; ++j) {
      for (int i = 0; i < GRID_N; ++i) {
        real x = (i - GRID_N / 2) * gridSpacing;
        real y = (j - GRID_N / 2) * gridSpacing;
        real r = std::sqrt(x * x + y * y);
        height[j][i] = (r < 1.4) ? std::max(0.0, (1.4 - r) * 0.58) : 0.0;
      }
    }
  }

  void pourSand(real worldX, real worldY, real amount = 0.02) {
    real localX = worldX - sandCenter.x;
    real localY = worldY - sandCenter.y;
    int i = int(localX / gridSpacing) + GRID_N / 2;
    int j = int(localY / gridSpacing) + GRID_N / 2;
    if (i >= 2 && i < GRID_N - 2 && j >= 2 && j < GRID_N - 2) {
      height[j][i] += amount;
      relaxAvalanche();
    }
  }

  void relaxAvalanche(int iterations = 4) {
    for (int iter = 0; iter < iterations; ++iter) {
      for (int j = 1; j < GRID_N - 1; ++j) {
        for (int i = 1; i < GRID_N - 1; ++i) {
          real hC = height[j][i];
          real maxDiff = 0.0;
          int bestDi = 0, bestDj = 0;
          int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
          for (auto &d : dirs) {
            real hN = height[j + d[1]][i + d[0]];
            real diff = hC - hN;
            if (diff > maxDiff) {
              maxDiff = diff;
              bestDi = d[0];
              bestDj = d[1];
            }
          }
          real slope = maxDiff / gridSpacing;
          if (slope > criticalSlope) {
            real transfer = (slope - criticalSlope) * gridSpacing * 0.25;
            height[j][i] -= transfer;
            height[j + bestDj][i + bestDi] += transfer;
          }
        }
      }
    }
  }

  real sampleHeight(real worldX, real worldY) const {
    real localX = worldX - sandCenter.x;
    real localY = worldY - sandCenter.y;
    real fi = localX / gridSpacing + GRID_N / 2.0;
    real fj = localY / gridSpacing + GRID_N / 2.0;
    int i0 = int(std::floor(fi));
    int j0 = int(std::floor(fj));
    if (i0 < 0 || i0 >= GRID_N - 1 || j0 < 0 || j0 >= GRID_N - 1)
      return 0.0;
    real u = fi - i0, v = fj - j0;
    real h00 = height[j0][i0];
    real h10 = height[j0][i0 + 1];
    real h01 = height[j0 + 1][i0];
    real h11 = height[j0 + 1][i0 + 1];
    return (h00 * (1 - u) + h10 * u) * (1 - v) + (h01 * (1 - u) + h11 * u) * v;
  }

  Vec3 evaluateNormal(real worldX, real worldY) const {
    const real eps = 0.04;
    real hL = sampleHeight(worldX - eps, worldY);
    real hR = sampleHeight(worldX + eps, worldY);
    real hD = sampleHeight(worldX, worldY - eps);
    real hU = sampleHeight(worldX, worldY + eps);
    real dhdx = (hR - hL) / (2.0 * eps);
    real dhdy = (hU - hD) / (2.0 * eps);
    return Vec3(-dhdx, -dhdy, 1.0).normalized();
  }

  bool intersectSand(const Vec3 &ro, const Vec3 &rd, real &tOut,
                     Vec3 &nOut) const {
    if (std::abs(rd.z) < 1e-6)
      return false;
    real tFloor = -ro.z / rd.z;
    if (tFloor < 0.01)
      return false;
    real tTop = (0.9 - ro.z) / rd.z;
    real tMin = std::max(0.01, std::min(tFloor, tTop));
    real tMax = std::max(0.01, std::max(tFloor, tTop));

    const int STEPS = 8;
    real dt = (tMax - tMin) / STEPS;
    real tPrev = tMin;
    Vec3 pPrev = ro + rd * tPrev;
    real hPrev = sampleHeight(pPrev.x, pPrev.y);
    real diffPrev = pPrev.z - hPrev;

    for (int s = 1; s <= STEPS; ++s) {
      real tCurr = tMin + s * dt;
      Vec3 pCurr = ro + rd * tCurr;
      real hCurr = sampleHeight(pCurr.x, pCurr.y);
      real diffCurr = pCurr.z - hCurr;
      if (diffPrev >= 0.0 && diffCurr <= 0.0 && hCurr > 0.005) {
        real frac = diffPrev / (diffPrev - diffCurr + 1e-9);
        real tExact = tPrev + frac * (tCurr - tPrev);
        Vec3 pHit = ro + rd * tExact;
        tOut = tExact;
        nOut = evaluateNormal(pHit.x, pHit.y);
        return true;
      }
      tPrev = tCurr;
      diffPrev = diffCurr;
    }
    return false;
  }
};

// ============================================================================
// 4. SOLIDI RIGIDI con dinamica completa + drag acqua
// ============================================================================
struct RigidSolidElement {
  enum class Shape { Box, Cylinder, Sphere };
  Shape shape = Shape::Box;
  Vec3 pos = Vec3(-1.2, 3.2, 0.4);
  Vec3 vel = Vec3(0, 0, 0);
  Vec3 size = Vec3(0.4, 0.4, 0.4);
  real mass = 4.5;
  real density = 2400.0;
  real restitution = 0.35;
  real charge = 0.0;
  Rgb albedo = {0.85f, 0.82f, 0.78f};
  real metallic = 0.1;
  real roughness = 0.2;
  bool isStatic = false;

  Vec3 lastGravity = Vec3(0, 0, 0);
  Vec3 lastBuoyancy = Vec3(0, 0, 0);
  Vec3 lastWindForce = Vec3(0, 0, 0);
  Vec3 lastEMForce = Vec3(0, 0, 0);

  real volume() const { return size.x * size.y * size.z; }
  Vec3 halfExtents() const { return size * 0.5; }

  static bool intersectBox(const Vec3 &ro, const Vec3 &rd, const Vec3 &center,
                           const Vec3 &halfExt, real &tOut, Vec3 &nOut) {
    Vec3 bMin = center - halfExt;
    Vec3 bMax = center + halfExt;
    auto safeDiv = [](real num, real den) {
      if (std::abs(den) < 1e-9)
        return num * (den >= 0 ? 1e9 : -1e9);
      return num / den;
    };
    real t1 = safeDiv(bMin.x - ro.x, rd.x);
    real t2 = safeDiv(bMax.x - ro.x, rd.x);
    real t3 = safeDiv(bMin.y - ro.y, rd.y);
    real t4 = safeDiv(bMax.y - ro.y, rd.y);
    real t5 = safeDiv(bMin.z - ro.z, rd.z);
    real t6 = safeDiv(bMax.z - ro.z, rd.z);
    real tN = std::max({std::min(t1, t2), std::min(t3, t4), std::min(t5, t6)});
    real tF = std::min({std::max(t1, t2), std::max(t3, t4), std::max(t5, t6)});
    if (tN > tF || tF < 0.001)
      return false;
    tOut = tN > 0.001 ? tN : tF;
    if (tOut < 0.001)
      return false;
    Vec3 hitP = ro + rd * tOut - center;
    real dx = std::abs(hitP.x) / halfExt.x;
    real dy = std::abs(hitP.y) / halfExt.y;
    real dz = std::abs(hitP.z) / halfExt.z;
    if (dx >= dy && dx >= dz)
      nOut = Vec3(hitP.x > 0 ? 1.0 : -1.0, 0, 0);
    else if (dy >= dz)
      nOut = Vec3(0, hitP.y > 0 ? 1.0 : -1.0, 0);
    else
      nOut = Vec3(0, 0, hitP.z > 0 ? 1.0 : -1.0);
    return true;
  }

  static bool aabbOverlap(const Vec3 &posA, const Vec3 &halfA, const Vec3 &posB,
                          const Vec3 &halfB, Vec3 &normal, real &overlap) {
    real dx = std::abs(posA.x - posB.x) - (halfA.x + halfB.x);
    real dy = std::abs(posA.y - posB.y) - (halfA.y + halfB.y);
    real dz = std::abs(posA.z - posB.z) - (halfA.z + halfB.z);
    if (dx > 0 || dy > 0 || dz > 0)
      return false;
    real pen = dx;
    normal = Vec3(posA.x > posB.x ? 1.0 : -1.0, 0, 0);
    if (dy > pen) {
      pen = dy;
      normal = Vec3(0, posA.y > posB.y ? 1.0 : -1.0, 0);
    }
    if (dz > pen) {
      pen = dz;
      normal = Vec3(0, 0, posA.z > posB.z ? 1.0 : -1.0);
    }
    overlap = -pen;
    return true;
  }

  void stepDynamics(real dt, const Vec3 &gravityVec,
                    const ContinuousWaterBody &water,
                    const ContinuousWindField &wind, real airDensity,
                    real simTime) {
    if (isStatic || dt <= 0)
      return;

    lastGravity = gravityVec * mass;
    lastBuoyancy = water.buoyancyForce(pos, size, simTime);
    lastWindForce = wind.windForceOnBody(pos, vel, size, airDensity, simTime);

    // Frazione sommersa per drag acqua
    real subFrac = water.submergedFraction(pos, size, simTime);
    // Coefficiente drag acqua (kg/s): quadratico + lineare
    // F_drag = -(0.5 * rho_w * Cd * A * |v|) * v  (quadratico)
    //        - (cLin) * v                          (lineare)
    real rho_w = water.waterDensity;
    real area = std::max(size.x * size.z, size.y * size.z);
    real Cd = 1.05;
    real quadK = 0.5 * rho_w * Cd * area * subFrac; // kg/m
    real linK = 50.0 * area * subFrac;              // kg/s

    Vec3 totalForce = lastGravity + lastBuoyancy + lastWindForce + lastEMForce;

    std::array<real, 6> y0 = {pos.x, pos.y, pos.z, vel.x, vel.y, vel.z};
    auto rhs = [&](real, const std::array<real, 6> &y) -> std::array<real, 6> {
      Vec3 v(y[3], y[4], y[5]);
      Vec3 accel = totalForce * (1.0 / mass);
      // Drag aria (bassa): lineare
      real beta = airDensity * 0.5 * 1.05 * area;
      accel = accel - v * (beta / mass);
      // Drag acqua: quadratico + lineare
      real vMag = v.norm();
      if (vMag > 1e-5) {
        real dragMag = (quadK * vMag + linK) * vMag;
        accel = accel - v * (dragMag / (vMag * mass));
      }
      return {v.x, v.y, v.z, accel.x, accel.y, accel.z};
    };
    auto res = nqg::integrateDP45<6>(rhs, y0, 0.0, dt, 1e-6, 1e-8);

    // Guardie finite
    Vec3 newPos(res.y[0], res.y[1], res.y[2]);
    Vec3 newVel(res.y[3], res.y[4], res.y[5]);
    if (std::isfinite(newPos.x) && std::isfinite(newPos.y) &&
        std::isfinite(newPos.z) && std::isfinite(newVel.x) &&
        std::isfinite(newVel.y) && std::isfinite(newVel.z)) {
      pos = newPos;
      vel = newVel;
    } else {
      vel = Vec3(0, 0, 0);
    }

    // Clamp di sicurezza per evitare esplosioni numeriche
    real maxSpeed = 200.0;
    if (vel.norm() > maxSpeed)
      vel = vel.normalized() * maxSpeed;

    // Pavimento
    real bottomZ = pos.z - size.z * 0.5;
    if (bottomZ < 0.0) {
      pos.z = size.z * 0.5;
      if (vel.z < 0) {
        vel.z = -vel.z * restitution;
        vel.x *= 0.92;
        vel.y *= 0.92;
        if (std::abs(vel.z) < 0.03)
          vel.z = 0;
      }
    }
    if (bottomZ < 0.01 && vel.norm() < 0.02)
      vel = Vec3(0, 0, 0);
  }
};

// ============================================================================
// 5. ELETTROMAGNETISMO
// ============================================================================
class ElectromagneticField {
public:
  struct PointCharge {
    Vec3 position;
    real charge = 0.0;
  };
  struct MagneticDipole {
    Vec3 position;
    Vec3 moment = Vec3(0, 0, 1.0);
    real strength = 1.0;
  };

  std::vector<PointCharge> charges;
  std::vector<MagneticDipole> dipoles;
  Vec3 backgroundE = Vec3(0, 0, 0);

  Vec3 electricField(const Vec3 &p) const {
    Vec3 E = backgroundE;
    for (const auto &ch : charges) {
      Vec3 rVec = p - ch.position;
      real r = rVec.norm();
      if (r < 0.01)
        continue;
      E = E + rVec * (phys::k_coulomb * ch.charge / (r * r * r));
    }
    return E;
  }

  Vec3 magneticField(const Vec3 &p) const {
    Vec3 B(0, 0, 0);
    for (const auto &dip : dipoles) {
      Vec3 rVec = p - dip.position;
      real r = rVec.norm();
      if (r < 0.01)
        continue;
      Vec3 rHat = rVec * (1.0 / r);
      Vec3 m = dip.moment * dip.strength;
      real mDotR = m.dot(rHat);
      real r3 = r * r * r;
      real coeff = phys::mu_0 / (4.0 * PI);
      B = B + (rHat * (3.0 * mDotR) - m) * (coeff / r3);
    }
    return B;
  }

  Vec3 lorentzForce(const Vec3 &p, const Vec3 &v, real q) const {
    if (std::abs(q) < 1e-15)
      return Vec3(0, 0, 0);
    Vec3 E = electricField(p);
    Vec3 B = magneticField(p);
    return (E + v.cross(B)) * q;
  }

  real electricPotential(const Vec3 &p) const {
    real phi = 0;
    for (const auto &ch : charges) {
      real r = (p - ch.position).norm();
      if (r < 0.01)
        continue;
      phi += phys::k_coulomb * ch.charge / r;
    }
    return phi;
  }

  real totalPotentialEnergy() const {
    real U = 0;
    for (std::size_t i = 0; i < charges.size(); ++i)
      for (std::size_t j = i + 1; j < charges.size(); ++j) {
        real r = (charges[i].position - charges[j].position).norm();
        if (r < 0.01)
          continue;
        U += phys::k_coulomb * charges[i].charge * charges[j].charge / r;
      }
    return U;
  }

  void applyToSolids(std::vector<RigidSolidElement> &solids) const {
    for (auto &s : solids) {
      if (s.isStatic || std::abs(s.charge) < 1e-15)
        continue;
      s.lastEMForce = lorentzForce(s.pos, s.vel, s.charge);
    }
  }

  void applyCoulombBetweenSolids(std::vector<RigidSolidElement> &solids) const {
    for (std::size_t i = 0; i < solids.size(); ++i) {
      if (solids[i].isStatic)
        continue;
      for (std::size_t j = i + 1; j < solids.size(); ++j) {
        real qi = solids[i].charge;
        real qj = solids[j].charge;
        if (std::abs(qi) < 1e-15 || std::abs(qj) < 1e-15)
          continue;
        Vec3 rVec = solids[i].pos - solids[j].pos;
        real r = rVec.norm();
        if (r < 0.05)
          r = 0.05;
        Vec3 rHat = rVec * (1.0 / r);
        real fMag = phys::k_coulomb * qi * qj / (r * r);
        Vec3 force = rHat * fMag;
        if (!solids[i].isStatic)
          solids[i].lastEMForce = solids[i].lastEMForce + force;
        if (!solids[j].isStatic)
          solids[j].lastEMForce = solids[j].lastEMForce - force;
      }
    }
  }
};

// ============================================================================
// 6. CAMPI ELEMENTARI
// ============================================================================
struct QuantumWavepacketField {
  Vec3 center = Vec3(-2.5, 2.0, 1.2);
  real sigma = 0.28;
  real particleMass = phys::m_electron;
  real energyLevel = 1.0;
  Rgb glowColor = {0.3f, 0.85f, 1.0f};

  real comptonWavelength() const {
    return phys::hbar / (particleMass * phys::c);
  }
  real schwarzschildRadius() const {
    return nqg::info::schwarzschildRadiusM(particleMass);
  }
  static real crossingMass() { return nqg::info::crossingMass(); }

  real evaluateDensity(const Vec3 &p) const {
    Vec3 diff = p - center;
    real r2 = diff.dot(diff);
    real normFactor = 1.0 / (std::pow(PI * sigma * sigma, 1.5));
    return normFactor * std::exp(-r2 / (sigma * sigma));
  }

  real heisenbergKineticEnergy() const {
    real deltap = phys::hbar / (2.0 * sigma);
    return deltap * deltap / (2.0 * particleMass);
  }
};

} // namespace continuum
} // namespace nqg

#endif // NQG_CONTINUUM_PHYSICS_HPP