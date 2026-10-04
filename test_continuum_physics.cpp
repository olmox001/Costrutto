// ============================================================================
//  test_continuum_physics.cpp  -  Test Suite Fisica dei Mezzi Continui (NQG)
// ============================================================================
#include "nqg_continuum_physics.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace nqg;
using namespace nqg::continuum;
using namespace nqg::engine;

static int passCount = 0;
static int failCount = 0;

#define CHECK(cond, name, desc) \
  do { \
    if (cond) { \
      std::cout << "[PASS] " << name << "    " << desc << "\n"; \
      passCount++; \
    } else { \
      std::cerr << "[FAIL] " << name << "    " << desc << "\n"; \
      failCount++; \
    } \
  } while (0)

int main() {
  std::cout << "====================================================\n";
  std::cout << "  TEST FISICA DEI MEZZI CONTINUI (NQG CONTINUUM)\n";
  std::cout << "====================================================\n";

  // --------------------------------------------------------------------------
  // 1. Acqua Continua: Onde di Gerstner, Snell, Fresnel e Beer-Lambert
  // --------------------------------------------------------------------------
  ContinuousWaterBody water;
  real h0 = water.evaluateHeight(water.basinCenter.x, water.basinCenter.y, 0.0);
  CHECK(h0 > 0.40 && h0 < 0.55, "K1a", "Altezza pelo libero d'acqua ragionevole: h=" + std::to_string(h0) + " m");

  Vec3 n0 = water.evaluateNormal(water.basinCenter.x, water.basinCenter.y, 0.0);
  CHECK(n0.z > 0.85, "K1b", "Normale dell'acqua prevalentemente verticale verso l'alto");

  // Snell e Fresnel: raggio obliquo a 45 gradi
  Vec3 rd(std::sqrt(0.5), 0, -std::sqrt(0.5));
  Vec3 normal(0, 0, 1);
  real cosI = std::clamp(-normal.dot(rd), 0.0, 1.0);
  real F = ContinuousWaterBody::fresnelDielectric(cosI, 1.0, 1.333);
  CHECK(F > 0.02 && F < 0.20, "K1c", "Riflettanza di Fresnel acqua (n=1.333) corretta a 45°: F=" + std::to_string(F));

  Vec3 refrDir;
  bool refrOk = ContinuousWaterBody::refractRay(rd, normal, 1.0 / 1.333, refrDir);
  CHECK(refrOk && refrDir.z < 0.0, "K1d", "Raggio rifratto di Snell entra nel volume d'acqua");

  Vec3 trans = water.beerLambertTransmission(2.0); // 2 metri sott'acqua
  CHECK(trans.z > trans.x, "K1e", "Assorbimento Beer-Lambert selettivo: la luce blu viaggia piu' a fondo del rosso");

  real tW_hit, dW_hit;
  Vec3 nW_hit;
  Vec3 rdToWater = (Vec3(water.basinCenter.x, water.basinCenter.y, 0.45) - Vec3(0, 0, 1.75)).normalized();
  bool hitW = water.intersectWater(Vec3(0, 0, 1.75), rdToWater, 0.0, tW_hit, nW_hit, dW_hit);
  CHECK(hitW && tW_hit > 0.0, "K1f", "Intersezione analitica raggio-pelo d'acqua continuo: t=" + std::to_string(tW_hit));

  // --------------------------------------------------------------------------
  // 2. Vento Continuo: Divergenza Nulla div(u) = 0 (Curl-Noise)
  // --------------------------------------------------------------------------
  ContinuousWindField wind;
  Vec3 p(1.5, 2.0, 1.0);
  Vec3 u = wind.evaluateVelocity(p, 1.0);
  CHECK(u.norm() > 0.5, "K2a", "Campo di vento continuo non degenere: |u|=" + std::to_string(u.norm()) + " m/s");

  // Verifica numerica div(u) = du_x/dx + du_y/dy + du_z/dz == 0
  const real eps = 0.005;
  Vec3 uXp = wind.evaluateVelocity(p + Vec3(eps, 0, 0), 1.0);
  Vec3 uXm = wind.evaluateVelocity(p - Vec3(eps, 0, 0), 1.0);
  Vec3 uYp = wind.evaluateVelocity(p + Vec3(0, eps, 0), 1.0);
  Vec3 uYm = wind.evaluateVelocity(p - Vec3(0, eps, 0), 1.0);
  Vec3 uZp = wind.evaluateVelocity(p + Vec3(0, 0, eps), 1.0);
  Vec3 uZm = wind.evaluateVelocity(p - Vec3(0, 0, eps), 1.0);

  real divU = (uXp.x - uXm.x) / (2.0 * eps) +
              (uYp.y - uYm.y) / (2.0 * eps) +
              (uZp.z - uZm.z) / (2.0 * eps);
  CHECK(std::abs(divU) < 1e-3, "K2b", "Divergenza del campo di vento nulla div(u)=0: " + std::to_string(divU));

  // --------------------------------------------------------------------------
  // 3. Sabbia Continua: Modello BCRE e Angolo di Riposo Naturale (33°)
  // --------------------------------------------------------------------------
  ContinuousSandDuneField sand;
  real sH = sand.sampleHeight(sand.sandCenter.x, sand.sandCenter.y);
  CHECK(sH > 0.4, "K3a", "Cima della duna di sabbia presente all'apice: h=" + std::to_string(sH) + " m");

  // Versa un mucchio ripido instabile e verifica il rilassamento da valanga
  sand.pourSand(sand.sandCenter.x, sand.sandCenter.y, 0.6);
  real hBefore = sand.sampleHeight(sand.sandCenter.x, sand.sandCenter.y);
  sand.relaxAvalanche(6);
  real hAfter = sand.sampleHeight(sand.sandCenter.x, sand.sandCenter.y);
  CHECK(hAfter < hBefore, "K3b", "Valanga BCRE: la sabbia in eccesso e' collassata ridistribuendosi sui fianchi");

  Vec3 nSand = sand.evaluateNormal(sand.sandCenter.x, sand.sandCenter.y);
  CHECK(nSand.z > 0.9, "K3c", "Normale duna di sabbia calcolata analiticamente: nz=" + std::to_string(nSand.z));

  real tS_hit;
  Vec3 nS_hit;
  Vec3 rdToSand = (Vec3(sand.sandCenter.x, sand.sandCenter.y, 0.3) - Vec3(0, 0, 1.75)).normalized();
  bool hitS = sand.intersectSand(Vec3(0, 0, 1.75), rdToSand, tS_hit, nS_hit);
  CHECK(hitS && tS_hit > 0.0, "K3d", "Intersezione analitica raggio-dune di sabbia continuo: t=" + std::to_string(tS_hit));

  // --------------------------------------------------------------------------
  // 4. Solidi Continui Rigidi: Box Analitico e BRDF
  // --------------------------------------------------------------------------
  RigidSolidElement solid;
  solid.pos = Vec3(0, 4, 1);
  solid.size = Vec3(2, 2, 2);
  real tBox;
  Vec3 nBox;
  bool hitBox = RigidSolidElement::intersectBox(Vec3(0, 0, 1), Vec3(0, 1, 0), solid.pos, solid.size * 0.5, tBox, nBox);
  CHECK(hitBox && std::abs(tBox - 3.0) < 1e-4, "K4a", "Intersezione raggio con solido rigido esatta: t=" + std::to_string(tBox));
  CHECK(nBox.y == -1.0, "K4b", "Normale della faccia colpita corretta (faccia -Y: ny=" + std::to_string(nBox.y) + ")");

  // --------------------------------------------------------------------------
  // 5. Quanti Continui: Pacchetto d'Onde e Lunghezza di Compton
  // --------------------------------------------------------------------------
  QuantumWavepacketField qf;
  real lambdaC = qf.comptonWavelength();
  CHECK(lambdaC > 1e-15 && lambdaC < 1e-10, "K5a", "Lunghezza d'onda di Compton dell'elettrone: lambda_C=" + std::to_string(lambdaC) + " m");

  real psiSqCenter = qf.evaluateDensity(qf.center);
  real psiSqFar = qf.evaluateDensity(qf.center + Vec3(1.0, 0, 0));
  CHECK(psiSqCenter > psiSqFar * 100.0, "K5b", "Densita' di probabilita' |psi|^2 localizzata nel pacchetto d'onde");

  std::cout << "\n====================================================\n";
  std::cout << "  RISULTATO TEST FISICA CONTINUO: " << passCount << " PASS, " << failCount << " FAIL\n";
  std::cout << "====================================================\n";

  return (failCount == 0) ? 0 : 1;
}
