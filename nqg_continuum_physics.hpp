// ============================================================================
//  nqg_continuum_physics.hpp  -  Fisica dei Continui: Acqua, Vento, Sabbia,
//  Solidi Rigidi con Gravita' e Dinamica, Elettromagnetismo e Interazioni
//  Fondamentali.
//
//  Deriva matematicamente dal core NQG e dall'engine 3D:
//    1. ACQUA: Superficie d'onda Gerstner + dispersione omega^2 = g*k*tanh(kh),
//              Rifrazione di Snell (n=1.333), Riflessione di Fresnel,
//              Assorbimento volumetrico di Beer-Lambert I = I0 * exp(-beta*d).
//    2. VENTO: Campo vettoriale tridimensionale a divergenza nulla (Curl Noise):
//              v = curl(Psi), div(v) = 0 (conservazione della massa per fluidi).
//    3. SABBIA: Altezza continua del letto granulare regolata dall'equazione BCRE
//               di valanga con angolo di riposo critico tan(theta_c) ~ 0.65 (33°).
//    4. SOLIDI: Geometrie analitiche rigide continue con DINAMICA di Newton-Euler
//               integrata con Dormand-Prince DP45 dal core NQG, gravita',
//               galleggiamento d'Archimede, collisioni solido-pavimento,
//               collisioni solido-solido, interazione con vento.
//    5. ELETTROMAGNETISMO: Campo elettrostatico di Coulomb F = k_e q1 q2 / r^2,
//               Campo magnetico da correnti con forza di Lorentz F = q (v x B),
//               Potenziale elettrico scalare phi e campo E = -grad(phi).
//               Costanti fisiche derivate da nqg::consts (hbar, c, G).
//    6. CAMPI ELEMENTARI: Nube di probabilita' quantistica |psi(x)|^2 continua
//                         con scala Compton lambda_C = hbar / (m*c) dal core NQG.
// ============================================================================
#ifndef NQG_CONTINUUM_PHYSICS_HPP
#define NQG_CONTINUUM_PHYSICS_HPP

#include "nqg_physics_core.hpp"
#include "nqg_engine3d.hpp"
#include "nqg_air_physics.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <array>
#include <iostream>

namespace nqg {
namespace continuum {

using engine::Vec3;
using engine::Rgb;
using cleanroom::AirProperties;

// Costanti fisiche fondamentali dal core NQG (nqg_physics_core.hpp)
namespace phys {
  using nqg::consts::G;
  using nqg::consts::c;
  using nqg::consts::hbar;
  // Costante di Coulomb: k_e = 1 / (4 * pi * epsilon_0) = 8.98755e9 N m^2 / C^2
  constexpr real k_coulomb = 8.98755179e9;
  // Carica elementare
  constexpr real e_charge = 1.602176634e-19;       // C
  // Massa elettrone
  constexpr real m_electron = 9.1093837015e-31;     // kg
  // Permeabilita' magnetica del vuoto mu_0 = 4*pi*1e-7
  constexpr real mu_0 = 4.0 * PI * 1e-7;            // T m / A
  // Permittivita' del vuoto epsilon_0 = 1 / (mu_0 * c^2)
  inline real epsilon_0() { return 1.0 / (mu_0 * c * c); }
  // Gravita' terrestre standard
  constexpr real g_earth = 9.80665;                  // m/s^2
}

// ----------------------------------------------------------------------------
// 1. FISICA CONTINUA DELL'ACQUA (Onde di Gerstner, Snell & Beer-Lambert)
// ----------------------------------------------------------------------------
struct WaterWave {
  real amplitude = 0.04; // m
  real wavelength = 1.2; // m
  real speed = 1.5;      // m/s
  Vec3 dir = Vec3(1, 0.5, 0).normalized();
};

class ContinuousWaterBody {
public:
  Vec3 basinCenter = Vec3(0, 3.0, 0); // Posizione vasca nel laboratorio
  real basinRadius = 2.8;             // Raggio della vasca d'acqua
  real baseWaterLevel = 0.45;         // Altezza del pelo libero dal fondo vasca
  real depth = 0.55;                  // Profondita' della vasca
  real refractiveIndex = 1.333;       // Indice di rifrazione dell'acqua a 20 °C
  real waterDensity = 998.0;          // kg/m^3 a 20 C

  // Coefficiente di assorbimento spettrale di Beer-Lambert (m^-1)
  // Il rosso viene assorbito molto prima del blu -> colore turchese/ciano
  Vec3 absorptionCoeff = Vec3(0.55, 0.12, 0.04);

  std::vector<WaterWave> waves;

  // Perturbazione impulsiva locale (es. sasso o goccia che cade)
  struct Ripple {
    Vec3 center;
    real startTime = 0;
    real amplitude = 0.08;
    real frequency = 14.0; // rad/s
    real speed = 1.8;     // m/s
    real decay = 1.2;     // 1/s
  };
  std::vector<Ripple> ripples;

  ContinuousWaterBody() {
    // 3 componenti d'onda di Gerstner con relazione di dispersione gravitazionale
    auto addWave = [&](real amp, real lambda, Vec3 d) {
      WaterWave w;
      w.amplitude = amp;
      w.wavelength = lambda;
      w.dir = d.normalized();
      real k = 2.0 * PI / lambda;
      // Relazione di dispersione: omega^2 = g * k * tanh(k * depth)
      real omega = std::sqrt(phys::g_earth * k * std::tanh(k * depth));
      w.speed = omega / k;
      waves.push_back(w);
    };

    addWave(0.015, 0.85, Vec3(1.0, 0.3, 0));
    addWave(0.008, 0.45, Vec3(-0.6, 0.8, 0));
    addWave(0.005, 0.22, Vec3(0.4, -0.9, 0));
  }

  void addImpulse(const Vec3 &pos, real time, real strength = 0.06) {
    Ripple r;
    r.center = pos;
    r.startTime = time;
    r.amplitude = strength;
    ripples.push_back(r);
  }

  // Verifica se un punto e' dentro il bacino
  bool isInsideBasin(real x, real y) const {
    real dx = x - basinCenter.x;
    real dy = y - basinCenter.y;
    return dx * dx + dy * dy <= basinRadius * basinRadius;
  }

  // Calcolo analitico esatto della quota della superficie z = h(x, y, t)
  real evaluateHeight(real x, real y, real t) const {
    if (!isInsideBasin(x, y)) {
      return -10.0; // Fuori dalla vasca d'acqua
    }

    real h = baseWaterLevel;

    // 1. Sovrapposizione di onde gravitazionali di Gerstner
    for (const auto &w : waves) {
      real k = 2.0 * PI / w.wavelength;
      real phase = k * (w.dir.x * x + w.dir.y * y) - k * w.speed * t;
      h += w.amplitude * std::cos(phase);
    }

    // 2. Onde circolari da impatti locali
    for (const auto &r : ripples) {
      real dt = t - r.startTime;
      if (dt < 0 || dt > 4.0) continue;
      real rDist = std::sqrt((x - r.center.x) * (x - r.center.x) + (y - r.center.y) * (y - r.center.y));
      real waveFront = r.speed * dt;
      real dr = rDist - waveFront;
      real envelope = std::exp(-dr * dr * 12.0) * std::exp(-r.decay * dt);
      h += r.amplitude * std::cos(r.frequency * dr) * envelope;
    }

    return h;
  }

  // Calcolo analitico del gradiente e normale unitaria della superficie:
  // N = normalize(-dh/dx, -dh/dy, 1)
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

  // Intersezione analitica esatta Raggio - Pelo Libero d'Acqua Continuo
  bool intersectWater(const Vec3 &ro, const Vec3 &rd, real time, real &tOut, Vec3 &nOut, real &depthOut) const {
    if (std::abs(rd.z) < 1e-6) return false;
    real tEst = (baseWaterLevel - ro.z) / rd.z;
    if (tEst < 0.01) return false;

    real t = tEst;
    for (int iter = 0; iter < 4; ++iter) {
      Vec3 p = ro + rd * t;
      if (!isInsideBasin(p.x, p.y)) return false;
      real h = evaluateHeight(p.x, p.y, time);
      real err = p.z - h;
      t -= err / rd.z;
    }

    Vec3 hitP = ro + rd * t;
    if (!isInsideBasin(hitP.x, hitP.y)) return false;

    real hFinal = evaluateHeight(hitP.x, hitP.y, time);
    if (std::abs(hitP.z - hFinal) > 0.08) return false;

    tOut = t;
    nOut = evaluateNormal(hitP.x, hitP.y, time);
    depthOut = std::max(0.04, hitP.z);
    return true;
  }

  // Legge di Snell per il raggio rifratto dentro l'acqua
  static bool refractRay(const Vec3 &I, const Vec3 &N, real eta, Vec3 &refracted) {
    real nDotI = N.dot(I);
    real cosThetaI = -nDotI;
    Vec3 norm = N;
    if (cosThetaI < 0) {
      // Dall'acqua verso l'aria
      cosThetaI = -cosThetaI;
      norm = N * (-1.0);
      eta = 1.0 / eta;
    }
    real sin2ThetaT = eta * eta * (1.0 - cosThetaI * cosThetaI);
    if (sin2ThetaT > 1.0) {
      return false; // Riflessione totale interna
    }
    real cosThetaT = std::sqrt(1.0 - sin2ThetaT);
    refracted = (I * eta + norm * (eta * cosThetaI - cosThetaT)).normalized();
    return true;
  }

  // Coefficiente di riflessione di Fresnel Schlick: R(theta) = R0 + (1-R0)*(1-cos theta)^5
  static real fresnelDielectric(real cosTheta, real n1 = 1.0, real n2 = 1.333) {
    real r0 = (n1 - n2) / (n1 + n2);
    r0 = r0 * r0;
    return r0 + (1.0 - r0) * std::pow(1.0 - std::clamp(cosTheta, 0.0, 1.0), 5.0);
  }

  // Attenuazione volumetrica di Beer-Lambert: T = exp(-beta * d)
  Vec3 beerLambertTransmission(real pathLength) const {
    return Vec3(
        std::exp(-absorptionCoeff.x * pathLength),
        std::exp(-absorptionCoeff.y * pathLength),
        std::exp(-absorptionCoeff.z * pathLength)
    );
  }

  // Calcolo della forza di Archimede per un corpo immerso (parzialmente o totalmente)
  // Volume immerso stimato per una box: V_imm = sx * sy * max(0, waterLevel - (pos.z - sz/2))
  Vec3 buoyancyForce(const Vec3 &bodyPos, const Vec3 &bodySize, real time) const {
    if (!isInsideBasin(bodyPos.x, bodyPos.y)) return Vec3(0, 0, 0);
    real waterH = evaluateHeight(bodyPos.x, bodyPos.y, time);
    real bodyBottom = bodyPos.z - bodySize.z * 0.5;
    real bodyTop = bodyPos.z + bodySize.z * 0.5;
    real submergedH = std::clamp(waterH - bodyBottom, 0.0, bodyTop - bodyBottom);
    if (submergedH <= 0) return Vec3(0, 0, 0);
    real submergedVolume = bodySize.x * bodySize.y * submergedH;
    // F_b = rho_water * V_imm * g (verso l'alto)
    return Vec3(0, 0, waterDensity * submergedVolume * phys::g_earth);
  }
};

// ----------------------------------------------------------------------------
// 2. FISICA CONTINUA DEL VENTO (Campo Vettoriale a Divergenza Nulla: Curl Noise)
// ----------------------------------------------------------------------------
class ContinuousWindField {
public:
  Vec3 baseDrift = Vec3(1.2, 0.4, 0.0); // m/s
  real turbulenceIntensity = 2.4;
  real spatialScale = 0.6; // 1/m

  // Potenziale vettoriale Psi(x, y, z, t) per garantire div(v) = 0
  static Vec3 vectorPotential(const Vec3 &p, real t) {
    // Componenti lisce sinusoidali e noise periodico derivate dal core 3D
    real px = p.x * 0.7 + t * 0.4;
    real py = p.y * 0.7 - t * 0.3;
    real pz = p.z * 0.7 + t * 0.2;

    real psiX = std::sin(py * 1.5 + 0.4) * std::cos(pz * 1.2);
    real psiY = std::sin(pz * 1.5 + 0.9) * std::cos(px * 1.2);
    real psiZ = std::sin(px * 1.5 + 1.2) * std::cos(py * 1.2);

    return Vec3(psiX, psiY, psiZ);
  }

  // Calcolo del campo di velocita' tramite rotore: v = curl(Psi)
  // Per identita' vettoriale div(curl(Psi)) == 0 esatto (Conservazione della massa d'aria)
  Vec3 evaluateVelocity(const Vec3 &p, real t) const {
    const real e = 0.01;
    Vec3 pXp = vectorPotential(p + Vec3(e, 0, 0), t);
    Vec3 pXm = vectorPotential(p - Vec3(e, 0, 0), t);
    Vec3 pYp = vectorPotential(p + Vec3(0, e, 0), t);
    Vec3 pYm = vectorPotential(p - Vec3(0, e, 0), t);
    Vec3 pZp = vectorPotential(p + Vec3(0, 0, e), t);
    Vec3 pZm = vectorPotential(p - Vec3(0, 0, e), t);

    // curl(Psi) = (dPsi_z/dy - dPsi_y/dz, dPsi_x/dz - dPsi_z/dx, dPsi_y/dx - dPsi_x/dy)
    real dPz_dy = (pYp.z - pYm.z) / (2.0 * e);
    real dPy_dz = (pZp.y - pZm.y) / (2.0 * e);

    real dPx_dz = (pZp.x - pZm.x) / (2.0 * e);
    real dPz_dx = (pXp.z - pXm.z) / (2.0 * e);

    real dPy_dx = (pXp.y - pXm.y) / (2.0 * e);
    real dPx_dy = (pYp.x - pYm.x) / (2.0 * e);

    Vec3 turbulent(dPz_dy - dPy_dz, dPx_dz - dPz_dx, dPy_dx - dPx_dy);

    return baseDrift + turbulent * turbulenceIntensity;
  }

  // Forza aerodinamica del vento su un corpo rigido di sezione frontale A
  // F_wind = 0.5 * rho_air * Cd * A * |v_rel|^2 * hat(v_rel)
  Vec3 windForceOnBody(const Vec3 &bodyPos, const Vec3 &bodyVel,
                       const Vec3 &bodySize, real airDensity, real simTime) const {
    Vec3 windVel = evaluateVelocity(bodyPos, simTime);
    Vec3 vRel = windVel - bodyVel;
    real vMag = vRel.norm();
    if (vMag < 1e-5) return Vec3(0, 0, 0);
    // Sezione frontale approssimata: max(sy*sz, sx*sz)
    real area = std::max(bodySize.y * bodySize.z, bodySize.x * bodySize.z);
    real Cd = 1.05; // Cd per parallelepipedo
    real forceMag = 0.5 * airDensity * Cd * area * vMag * vMag;
    return vRel.normalized() * forceMag;
  }
};

// ----------------------------------------------------------------------------
// 3. FISICA CONTINUA DELLA SABBIA (Mappa di Elevazione BCRE e Angolo di Riposo)
// ----------------------------------------------------------------------------
class ContinuousSandDuneField {
public:
  Vec3 sandCenter = Vec3(3.2, 2.5, 0.0);
  real sandRadius = 2.2;
  real criticalSlope = 0.65; // tan(33°) -> Angolo di riposo naturale sabbia asciutta

  // Griglia 2D di elevazione continua della sabbia
  static constexpr int GRID_N = 64;
  std::array<std::array<real, GRID_N>, GRID_N> height{};
  real gridSpacing = 0.08; // 64 * 0.08 = 5.12 m di estensione

  ContinuousSandDuneField() {
    // Crea cumulo naturale di sabbia all'equilibrio conica a 30°
    for (int j = 0; j < GRID_N; ++j) {
      for (int i = 0; i < GRID_N; ++i) {
        real x = (i - GRID_N / 2) * gridSpacing;
        real y = (j - GRID_N / 2) * gridSpacing;
        real r = std::sqrt(x * x + y * y);
        if (r < 1.4) {
          // Cono naturale con pendenza <= 33°
          height[j][i] = std::max(0.0, (1.4 - r) * 0.58);
        } else {
          height[j][i] = 0.0;
        }
      }
    }
  }

  // Versa altra sabbia in una posizione specifica (formazione naturale di dune)
  void pourSand(real worldX, real worldY, real amount = 0.02) {
    real localX = worldX - sandCenter.x;
    real localY = worldY - sandCenter.y;
    int i = int(localX / gridSpacing) + GRID_N / 2;
    int j = int(localY / gridSpacing) + GRID_N / 2;

    if (i >= 2 && i < GRID_N - 2 && j >= 2 && j < GRID_N - 2) {
      height[j][i] += amount;
      // Diffusione non lineare BCRE per valanga se supera l'angolo di riposo
      relaxAvalanche();
    }
  }

  // Rilassamento da valanga: quando il gradiente supera tan(33°), la sabbia scivola
  void relaxAvalanche(int iterations = 4) {
    for (int iter = 0; iter < iterations; ++iter) {
      for (int j = 1; j < GRID_N - 1; ++j) {
        for (int i = 1; i < GRID_N - 1; ++i) {
          real hC = height[j][i];
          real maxDiff = 0.0;
          int bestDi = 0, bestDj = 0;

          // Controlla i 4 vicini
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

          // Se la pendenza supera l'angolo critico di riposo, la sabbia fluisce
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

  // Campionamento continuo dell'altezza della sabbia tramite interpolazione bilineare
  real sampleHeight(real worldX, real worldY) const {
    real localX = worldX - sandCenter.x;
    real localY = worldY - sandCenter.y;

    real fi = localX / gridSpacing + GRID_N / 2.0;
    real fj = localY / gridSpacing + GRID_N / 2.0;

    int i0 = int(std::floor(fi));
    int j0 = int(std::floor(fj));

    if (i0 < 0 || i0 >= GRID_N - 1 || j0 < 0 || j0 >= GRID_N - 1) {
      return 0.0;
    }

    real u = fi - i0;
    real v = fj - j0;

    real h00 = height[j0][i0];
    real h10 = height[j0][i0 + 1];
    real h01 = height[j0 + 1][i0];
    real h11 = height[j0 + 1][i0 + 1];

    return (h00 * (1 - u) + h10 * u) * (1 - v) + (h01 * (1 - u) + h11 * u) * v;
  }

  // Normale continua della superficie di sabbia
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

  // Intersezione analitica esatta Raggio - Dune di Sabbia Continue
  bool intersectSand(const Vec3 &ro, const Vec3 &rd, real &tOut, Vec3 &nOut) const {
    if (std::abs(rd.z) < 1e-6) return false;
    real tFloor = -ro.z / rd.z;
    if (tFloor < 0.01) return false;
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

// ----------------------------------------------------------------------------
// 4. FISICA DEI SOLIDI RIGIDI CON DINAMICA COMPLETA
//    (Newton-Euler, Gravita', Collisione Pavimento, Archimede, Vento)
//    Integrazione con Dormand-Prince DP45 dal core NQG (nqg_physics_core.hpp)
// ----------------------------------------------------------------------------
struct RigidSolidElement {
  enum class Shape { Box, Cylinder, Sphere };
  Shape shape = Shape::Box;
  Vec3 pos = Vec3(-1.2, 3.2, 0.4);
  Vec3 vel = Vec3(0, 0, 0);
  Vec3 size = Vec3(0.4, 0.4, 0.4); // Larghezza, profondita', altezza
  real mass = 4.5;                 // kg
  real density = 2400.0;           // kg/m^3 (Gres porcellanato / ceramica)
  real restitution = 0.35;         // Coefficiente di rimbalzo
  real charge = 0.0;               // Carica elettrica in Coulomb (per interazione EM)
  Rgb albedo = {0.85f, 0.82f, 0.78f};
  real metallic = 0.1;
  real roughness = 0.2;
  bool isStatic = false;           // Se true, il corpo non si muove (es. tavolo fissato)

  // Telemetria forze per debug HUD
  Vec3 lastGravity = Vec3(0, 0, 0);
  Vec3 lastBuoyancy = Vec3(0, 0, 0);
  Vec3 lastWindForce = Vec3(0, 0, 0);
  Vec3 lastEMForce = Vec3(0, 0, 0);

  real volume() const {
    return size.x * size.y * size.z;
  }

  Vec3 halfExtents() const {
    return size * 0.5;
  }

  // Intersezione analitica Raggio - Scatola orientata (AABB)
  static bool intersectBox(const Vec3 &ro, const Vec3 &rd, const Vec3 &center,
                           const Vec3 &halfExt, real &tOut, Vec3 &nOut) {
    Vec3 bMin = center - halfExt;
    Vec3 bMax = center + halfExt;

    auto safeDiv = [](real num, real den) {
      if (std::abs(den) < 1e-9) {
        return num * (den >= 0 ? 1e9 : -1e9);
      }
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

    if (tN > tF || tF < 0.001) return false;
    tOut = tN > 0.001 ? tN : tF;
    if (tOut < 0.001) return false;

    Vec3 hitP = ro + rd * tOut - center;
    real dx = std::abs(hitP.x) / halfExt.x;
    real dy = std::abs(hitP.y) / halfExt.y;
    real dz = std::abs(hitP.z) / halfExt.z;

    if (dx >= dy && dx >= dz) {
      nOut = Vec3(hitP.x > 0 ? 1.0 : -1.0, 0, 0);
    } else if (dy >= dz) {
      nOut = Vec3(0, hitP.y > 0 ? 1.0 : -1.0, 0);
    } else {
      nOut = Vec3(0, 0, hitP.z > 0 ? 1.0 : -1.0);
    }
    return true;
  }

  // AABB overlap test per collisioni solido-solido
  static bool aabbOverlap(const Vec3 &posA, const Vec3 &halfA,
                           const Vec3 &posB, const Vec3 &halfB,
                           Vec3 &normal, real &overlap) {
    real dx = std::abs(posA.x - posB.x) - (halfA.x + halfB.x);
    real dy = std::abs(posA.y - posB.y) - (halfA.y + halfB.y);
    real dz = std::abs(posA.z - posB.z) - (halfA.z + halfB.z);

    if (dx > 0 || dy > 0 || dz > 0) return false;

    // Trova l'asse di minima penetrazione (SAT semplificato)
    real pen = dx; // dx e' negativo
    normal = Vec3(posA.x > posB.x ? 1.0 : -1.0, 0, 0);

    if (dy > pen) { pen = dy; normal = Vec3(0, posA.y > posB.y ? 1.0 : -1.0, 0); }
    if (dz > pen) { pen = dz; normal = Vec3(0, 0, posA.z > posB.z ? 1.0 : -1.0); }

    overlap = -pen;
    return true;
  }

  // Integrazione dinamica con Dormand-Prince DP45 dal core NQG
  // Forze: Gravita' + Archimede + Vento + EM + Collisioni
  void stepDynamics(real dt, const Vec3 &gravityVec,
                    const ContinuousWaterBody &water,
                    const ContinuousWindField &wind,
                    real airDensity, real simTime) {
    if (isStatic || dt <= 0) return;

    // Calcola tutte le forze al passo corrente
    // 1. Gravita': F = m * g
    lastGravity = gravityVec * mass;

    // 2. Archimede (acqua): se parzialmente immerso
    lastBuoyancy = water.buoyancyForce(pos, size, simTime);

    // 3. Resistenza del vento
    lastWindForce = wind.windForceOnBody(pos, vel, size, airDensity, simTime);

    // Integrazione DP45 dal core NQG: y = {x, y, z, vx, vy, vz}
    Vec3 totalForce = lastGravity + lastBuoyancy + lastWindForce + lastEMForce;

    std::array<real, 6> y0 = {pos.x, pos.y, pos.z, vel.x, vel.y, vel.z};

    auto rhs = [&](real /*t*/, const std::array<real, 6> &y) -> std::array<real, 6> {
      Vec3 v(y[3], y[4], y[5]);
      Vec3 accel = totalForce * (1.0 / mass);

      // Smorzamento fluidodinamico proporzionale alla velocita' (attrito d'aria)
      // F_drag lineare approssimato per basse velocita' = -beta * v
      real beta = airDensity * 0.5 * 1.05 * std::max(size.y * size.z, size.x * size.z);
      accel = accel - v * (beta / mass);

      return {v.x, v.y, v.z, accel.x, accel.y, accel.z};
    };

    auto res = nqg::integrateDP45<6>(rhs, y0, 0.0, dt, 1e-6, 1e-8);

    pos = Vec3(res.y[0], res.y[1], res.y[2]);
    vel = Vec3(res.y[3], res.y[4], res.y[5]);

    // Collisione con il pavimento infinito z = 0
    real bottomZ = pos.z - size.z * 0.5;
    if (bottomZ < 0.0) {
      pos.z = size.z * 0.5;
      if (vel.z < 0) {
        vel.z = -vel.z * restitution;
        // Attrito dinamico col pavimento
        vel.x *= 0.92;
        vel.y *= 0.92;
        // Soglia di quiete
        if (std::abs(vel.z) < 0.03) vel.z = 0;
      }
    }

    // Smorzamento velocita' (attrito rotolamento/scivolamento sul pavimento)
    if (bottomZ < 0.01 && vel.norm() < 0.02) {
      vel = Vec3(0, 0, 0);
    }
  }
};

// ----------------------------------------------------------------------------
// 5. CAMPO ELETTROMAGNETICO CONTINUO
//    Derivato dalle costanti fondamentali del core NQG (hbar, c, G)
//    Legge di Coulomb: F = k_e * q1 * q2 / r^2
//    Forza di Lorentz:  F = q * (E + v x B)
//    Campo magnetico di un dipolo: B(r) = (mu_0 / 4pi) * (3(m.r)r/r^5 - m/r^3)
// ----------------------------------------------------------------------------
class ElectromagneticField {
public:
  // Sorgente di campo elettrostatico puntiforme
  struct PointCharge {
    Vec3 position;
    real charge = 0.0;    // Coulomb
  };

  // Sorgente di campo magnetico (dipolo magnetico)
  struct MagneticDipole {
    Vec3 position;
    Vec3 moment = Vec3(0, 0, 1.0); // A*m^2
    real strength = 1.0;
  };

  std::vector<PointCharge> charges;
  std::vector<MagneticDipole> dipoles;

  // Campo elettrico uniforme di fondo (es. carica elettrostatica nella stanza)
  Vec3 backgroundE = Vec3(0, 0, 0); // V/m

  // Campo elettrico totale in un punto: E = sum_i k_e * q_i * (r - r_i) / |r - r_i|^3
  Vec3 electricField(const Vec3 &p) const {
    Vec3 E = backgroundE;
    for (const auto &ch : charges) {
      Vec3 rVec = p - ch.position;
      real r = rVec.norm();
      if (r < 0.01) continue; // Evita singolarita'
      E = E + rVec * (phys::k_coulomb * ch.charge / (r * r * r));
    }
    return E;
  }

  // Campo magnetico totale in un punto: B = sum di dipoli + fondo
  // B_dipolo(r) = (mu_0 / 4*pi) * [3*(m dot rhat)*rhat - m] / r^3
  Vec3 magneticField(const Vec3 &p) const {
    Vec3 B(0, 0, 0);
    for (const auto &dip : dipoles) {
      Vec3 rVec = p - dip.position;
      real r = rVec.norm();
      if (r < 0.01) continue;
      Vec3 rHat = rVec * (1.0 / r);
      Vec3 m = dip.moment * dip.strength;
      real mDotR = m.dot(rHat);
      real r3 = r * r * r;
      real coeff = phys::mu_0 / (4.0 * PI);
      B = B + (rHat * (3.0 * mDotR) - m) * (coeff / r3);
    }
    return B;
  }

  // Forza di Lorentz su una carica in moto: F = q * (E + v x B)
  Vec3 lorentzForce(const Vec3 &p, const Vec3 &v, real q) const {
    if (std::abs(q) < 1e-15) return Vec3(0, 0, 0);
    Vec3 E = electricField(p);
    Vec3 B = magneticField(p);
    // F = q * (E + v x B)
    Vec3 vCrossB = v.cross(B);
    return (E + vCrossB) * q;
  }

  // Potenziale elettrico scalare phi(r) = sum k_e * q_i / |r - r_i|
  real electricPotential(const Vec3 &p) const {
    real phi = 0;
    for (const auto &ch : charges) {
      real r = (p - ch.position).norm();
      if (r < 0.01) continue;
      phi += phys::k_coulomb * ch.charge / r;
    }
    return phi;
  }

  // Energia potenziale di un sistema di cariche: U = sum_{i<j} k_e * q_i * q_j / r_ij
  real totalPotentialEnergy() const {
    real U = 0;
    for (std::size_t i = 0; i < charges.size(); ++i) {
      for (std::size_t j = i + 1; j < charges.size(); ++j) {
        real r = (charges[i].position - charges[j].position).norm();
        if (r < 0.01) continue;
        U += phys::k_coulomb * charges[i].charge * charges[j].charge / r;
      }
    }
    return U;
  }

  // Aggiorna le forze EM sui solidi rigidi
  void applyToSolids(std::vector<RigidSolidElement> &solids) const {
    for (auto &s : solids) {
      if (s.isStatic || std::abs(s.charge) < 1e-15) continue;
      s.lastEMForce = lorentzForce(s.pos, s.vel, s.charge);
    }
  }

  // Integra forze EM tra coppie di solidi carichi (Coulomb diretto)
  void applyCoulombBetweenSolids(std::vector<RigidSolidElement> &solids) const {
    for (std::size_t i = 0; i < solids.size(); ++i) {
      if (solids[i].isStatic) continue;
      for (std::size_t j = i + 1; j < solids.size(); ++j) {
        real qi = solids[i].charge;
        real qj = solids[j].charge;
        if (std::abs(qi) < 1e-15 || std::abs(qj) < 1e-15) continue;

        Vec3 rVec = solids[i].pos - solids[j].pos;
        real r = rVec.norm();
        if (r < 0.05) r = 0.05; // Softening
        Vec3 rHat = rVec * (1.0 / r);
        real fMag = phys::k_coulomb * qi * qj / (r * r);
        Vec3 force = rHat * fMag;

        if (!solids[i].isStatic) solids[i].lastEMForce = solids[i].lastEMForce + force;
        if (!solids[j].isStatic) solids[j].lastEMForce = solids[j].lastEMForce - force;
      }
    }
  }
};

// ----------------------------------------------------------------------------
// 6. CAMPO CONTINUO ELEMENTARE (Densita' di Probabilita' Quantistica |psi(x)|^2)
//    Usa le costanti fondamentali dal core NQG: hbar, c, G
// ----------------------------------------------------------------------------
struct QuantumWavepacketField {
  Vec3 center = Vec3(-2.5, 2.0, 1.2);
  real sigma = 0.28; // Ampiezza pacchetto d'onda gaussiano
  real particleMass = phys::m_electron; // kg (massa a riposo elettrone dal core NQG)
  real energyLevel = 1.0;
  Rgb glowColor = {0.3f, 0.85f, 1.0f};

  // Lunghezza d'onda di Compton quantistica: lambda_C = hbar / (m * c)
  // Usa le costanti dal core NQG (nqg_physics_core.hpp -> nqg::consts)
  real comptonWavelength() const {
    return phys::hbar / (particleMass * phys::c);
  }

  // Raggio di Schwarzschild della particella: r_s = 2 G m / c^2
  // Dal core NQG (nqg::info::schwarzschildRadiusM)
  real schwarzschildRadius() const {
    return nqg::info::schwarzschildRadiusM(particleMass);
  }

  // Raggio di incrocio Compton-Schwarzschild dal core NQG
  static real crossingMass() {
    return nqg::info::crossingMass();
  }

  // Densita' volumetrica di probabilita' |psi(r)|^2 = (1 / (pi * sigma^2)^(3/2)) * exp(-r^2 / sigma^2)
  real evaluateDensity(const Vec3 &p) const {
    Vec3 diff = p - center;
    real r2 = diff.dot(diff);
    real normFactor = 1.0 / (std::pow(PI * sigma * sigma, 1.5));
    return normFactor * std::exp(-r2 / (sigma * sigma));
  }

  // Energia cinetica stimata dal principio di indeterminazione di Heisenberg:
  // Delta_p >= hbar / (2 * sigma),  E_k = Delta_p^2 / (2 * m)
  real heisenbergKineticEnergy() const {
    real deltap = phys::hbar / (2.0 * sigma);
    return deltap * deltap / (2.0 * particleMass);
  }
};

} // namespace continuum
} // namespace nqg

#endif // NQG_CONTINUUM_PHYSICS_HPP
