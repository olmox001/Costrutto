// SPDX-License-Identifier: GPL-2.0-or-later
// ============================================================================
//  test_continuum_physics.cpp  -  Test Suite Fisica dei Mezzi Continui (NQG)
// ============================================================================
#include "nqg_continuum_physics.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

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
  water.initialFill(water.basinCenter.x - 2.0, water.basinCenter.x + 2.0,
                    water.basinCenter.y - 2.0, water.basinCenter.y + 2.0, 0.45);
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
  // 1b. Onde FISICHE: velocita' sqrt(g h), riva continua, infiltrazione
  // --------------------------------------------------------------------------
  {
    ContinuousWaterBody pool;
    pool.setBedProvider([](real x, real y) {
      fluid::BedSample b;
      if (std::abs(x) > 3.05 || std::abs(y) > 3.05) {
        b.solid = true;
        b.z = fluid::ShallowFlow::SOLID_Z;
      }
      return b;
    });
    pool.initialFill(-3.0, 3.0, -3.0, 3.0, 0.5);
    const real volumeBeforeImpulse = pool.totalVolume();
    pool.addImpulse(Vec3(0, 0, 0.5), 0.0, 0.05);
    const real volumeAfterImpulse = pool.totalVolume();
    const real etaAfterImpulse = pool.surfaceHeight(0.0, 0.0);
    CHECK(std::abs(volumeAfterImpulse - volumeBeforeImpulse) < 1e-12 &&
          etaAfterImpulse > 0.5,
        "K1m", "Impulso d'onda modifica la superficie senza cambiare volume: dV=" +
             std::to_string(volumeAfterImpulse - volumeBeforeImpulse));
    const real eta0 = pool.surfaceHeight(1.5, 0.0);
    real tArrive = -1.0;
    for (int i = 0; i < 300 && tArrive < 0; ++i) {
      pool.step(0.01);
      if (std::abs(pool.surfaceHeight(1.5, 0.0) - eta0) > 2e-4)
        tArrive = 0.01 * (i + 1);
    }
    const real cTh = std::sqrt(9.80665 * 0.5);
    CHECK(tArrive > 0.35 * 1.5 / cTh && tArrive < 1.6 * 1.5 / cTh, "K1g",
          "Onda fisica: arrivo a 1.5 m dopo t=" + std::to_string(tArrive) +
              " s (c teorica " + std::to_string(cTh) + " m/s)");
    const real v0 = pool.totalVolume();
    for (int i = 0; i < 100; ++i)
      pool.step(0.01);
    CHECK(std::abs(pool.totalVolume() - v0) < 1e-6 * v0 + 1e-9, "K1h",
          "Volume conservato dopo l'impulso e la propagazione");

    ContinuousWaterBody centroidPool;
    centroidPool.initialFill(-2.0, 2.0, -2.0, 2.0, 0.6);
    RigidSolidElement fullBody;
    fullBody.size = Vec3(0.8, 0.8, 1.0);
    fullBody.pos = Vec3(0.0, 0.0, 0.5);
    fullBody.syncMass();
    RigidSolidElement halfBody = fullBody;
    halfBody.fillFraction = 0.5;
    halfBody.syncMass();
    const HydroResult fullHydro =
      bodyHydro(centroidPool, fullBody, Vec3(0, 0, 0), 9.80665);
    const HydroResult halfHydro =
      bodyHydro(centroidPool, halfBody, Vec3(0, 0, 0), 9.80665);
    CHECK(fullHydro.wet && halfHydro.wet &&
          std::abs(halfHydro.submergedVolume /
                 fullHydro.submergedVolume -
               0.5) < 1e-10 &&
          (halfHydro.centroid - fullHydro.centroid).norm() < 1e-10,
        "K1l", "Frazione piena 0.5 dimezza la spinta senza spostare il centro: z=" +
             std::to_string(fullHydro.centroid.z) + "/" +
             std::to_string(halfHydro.centroid.z));

    fluid::ShallowFlow visualField(0.1, 8, 8);
    visualField.fillRect(-3.0, 3.0, -3.0, 3.0, 0.5);
    const real visualVolume = visualField.totalVolume();
    const auto fullCoverage = fluid::estimateObserverSurface(
      visualField, 0.0, 0.0, 0.1, 0.1, 64, 0x1234);
    const auto fullCoverageRepeat = fluid::estimateObserverSurface(
      visualField, 0.0, 0.0, 0.1, 0.1, 64, 0x1234);
    fluid::ShallowFlow visualShore(0.1, 8, 8);
    visualShore.fillRect(-3.0, 0.0, -3.0, 3.0, 0.5);
    const auto partialCoverage = fluid::estimateObserverSurface(
      visualShore, 0.0, 0.0, 0.4, 0.4, 256, 0x5678);
    CHECK(fullCoverage.wetProbability == 1.0 &&
          std::abs(fullCoverage.meanEta - 0.5) < 1e-10 &&
          fullCoverage.etaVariance < 1e-20 &&
          fullCoverage.wetProbability == fullCoverageRepeat.wetProbability &&
          fullCoverage.meanEta == fullCoverageRepeat.meanEta &&
          partialCoverage.wetProbability > 0.0 &&
          partialCoverage.wetProbability < 1.0 &&
          visualField.totalVolume() == visualVolume,
          "K1k", "Stima Monte Carlo observer-only: copertura=" +
               std::to_string(partialCoverage.wetProbability) +
               ", eta=" + std::to_string(fullCoverage.meanEta) +
               ", var=" + std::to_string(fullCoverage.etaVariance));

    // riva: profondita' continua (nessun salto sul bordo)
    ContinuousWaterBody beach;
    beach.initialFill(-1.0, 1.0, -1.0, 1.0, 0.02);
    real maxJump = 0.0, prev = beach.depthAt(-1.5, 0.0);
    real prevS = 0.0;
    for (real x = -1.4; x < -0.6; x += 0.01) {
      auto s = beach.flow.sample(x, 0.0);
      const real d = s.wet ? s.depth : 0.0;
      maxJump = std::max(maxJump, std::abs(d - prevS));
      prevS = d;
    }
    (void)prev;
    CHECK(maxJump < 0.004, "K1i",
          "Riva continua: salto massimo di profondita' per cm = " +
              std::to_string(maxJump) + " m");

    // infiltrazione: il terreno permeabile assorbe l'acqua e il fronte si ferma
    ContinuousWaterBody soil;
    soil.setBedProvider([](real x, real y) {
      (void)x;
      (void)y;
      fluid::BedSample b;
      b.infil = 1e-3;
      b.retention = 1e-3;
      return b;
    });
    soil.addVolume(0, 0, 0.3, 0.01);
    const real vs0 = soil.totalVolume();
    for (int i = 0; i < 400; ++i)
      soil.step(0.05);
    CHECK(soil.totalVolume() < 0.2 * vs0, "K1j",
          "Infiltrazione: volume " + std::to_string(vs0) + " -> " +
              std::to_string(soil.totalVolume()) + " m^3 (fronte fermo)");
  }

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
  CHECK(lambdaC > 1e-15 && lambdaC < 1e-10, "K5a", "Lunghezza d'onda di Compton dell'elettrone: lambda_C=" + [&]{ char b[32]; std::snprintf(b, sizeof b, "%.3e", lambdaC); return std::string(b); }() + " m");

  real psiSqCenter = qf.evaluateDensity(qf.center);
  real psiSqFar = qf.evaluateDensity(qf.center + Vec3(1.0, 0, 0));
  CHECK(psiSqCenter > psiSqFar * 100.0, "K5b", "Densita' di probabilita' |psi|^2 localizzata nel pacchetto d'onde");

  std::cout << "\n====================================================\n";
  std::cout << "  RISULTATO TEST FISICA CONTINUO: " << passCount << " PASS, " << failCount << " FAIL\n";
  std::cout << "====================================================\n";

  return (failCount == 0) ? 0 : 1;
}
