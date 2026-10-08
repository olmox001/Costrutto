// SPDX-License-Identifier: GPL-2.0-or-later
// ============================================================================
//  test_water_solver.cpp - Core matematico autonomo del solver acqua
// ============================================================================
#include "nqg_water_solver.hpp"

#include <cmath>
#include <cstdio>

using namespace nqg;
using namespace nqg::fluid;

static int passed = 0;
static int failed = 0;

#define CHECK(name, condition, description)                                     \
  do {                                                                          \
    const bool ok = (condition);                                                \
    (ok ? passed : failed)++;                                                   \
    std::printf("[%s] %-5s %s\n", ok ? "PASS" : "FAIL", name, description);  \
  } while (0)

int main() {
  ShallowFlow water(0.1, 8, 8);
  water.fillRect(-3.0, 3.0, -3.0, 3.0, 0.5);
  const real initialVolume = water.totalVolume();

  const ObserverSurfaceEstimate uniform =
      estimateObserverSurface(water, 0.0, 0.0, 0.1, 0.1, 128, 0x1234);
  const ObserverSurfaceEstimate uniformAgain =
      estimateObserverSurface(water, 0.0, 0.0, 0.1, 0.1, 128, 0x1234);
  CHECK("W1", uniform.wetProbability == 1.0 &&
                  std::abs(uniform.meanEta - 0.5) < 1e-10 &&
                  uniform.etaVariance < 1e-20,
        "uniforme: P(bagnato)=1, eta media=0.5, varianza nulla");
  CHECK("W2", uniform.wetProbability == uniformAgain.wetProbability &&
                  uniform.meanEta == uniformAgain.meanEta &&
                  uniform.etaVariance == uniformAgain.etaVariance,
        "campionamento riproducibile a seed invariato");
  CHECK("W3", water.totalVolume() == initialVolume,
        "stima observer-only non muta il volume simulato");

  ShallowFlow shore(0.1, 8, 8);
  shore.fillRect(-3.0, 0.0, -3.0, 3.0, 0.5);
  const ObserverSurfaceEstimate coverage = estimateObserverSurface(
      shore, 0.0, 0.0, 0.4, 0.4, 4096, 0x5678);
  CHECK("W4", coverage.wetProbability > 0.0 &&
                  coverage.wetProbability < 1.0 && coverage.meanDepth > 0.0,
        "footprint sul bordo produce copertura parziale e profondita' condizionata");
  const ObserverSurfaceEstimate noSamples =
      estimateObserverSurface(shore, 0.0, 0.0, 0.4, 0.4, 0, 0x5678);
  CHECK("W5", noSamples.sampleCount == 0 && noSamples.wetProbability == 0.0,
        "zero campioni restituisce stima vuota");

    ShallowFlow longStep(0.1, 8, 8);
    longStep.fillRect(-2.0, 2.0, -2.0, 2.0, 0.5);
    longStep.step(1.0);
    CHECK("W6", longStep.lastSubsteps > 24 &&
        std::abs(longStep.lastIntegratedDt - 1.0) < 1e-12 &&
        std::abs(longStep.simulatedTime - 1.0) < 1e-12,
        "CFL integra tutto dt=1s oltre 24 sottopassi");
    ShallowFlow strictCfl(0.1, 8, 8);
    strictCfl.cfl = 5e-4;
    strictCfl.fillRect(-2.0, 2.0, -2.0, 2.0, 0.5);
    strictCfl.step(1e-3);
    CHECK("W8", strictCfl.lastSubsteps > 10 &&
                    std::abs(strictCfl.lastIntegratedDt - 1e-3) < 1e-12,
          "Il passo interno rispetta CFL anche sotto 1e-4 s");
    ShallowFlow microStep(0.1, 8, 8);
    microStep.fillRect(-2.0, 2.0, -2.0, 2.0, 0.5);
    microStep.step(1e-13);
    CHECK("W9", microStep.lastSubsteps == 1 &&
            microStep.lastIntegratedDt == 1e-13 &&
            microStep.simulatedTime == 1e-13,
        "Il residuo sotto 1e-12 s non viene scartato");
    ShallowFlow invalidCfl(0.1, 8, 8);
    invalidCfl.cfl = 0.0;
    invalidCfl.fillRect(-2.0, 2.0, -2.0, 2.0, 0.5);
    const real invalidCflVolume = invalidCfl.totalVolume();
    invalidCfl.step(1e-3);
    CHECK("W10", invalidCfl.lastSubsteps == 0 &&
             invalidCfl.lastIntegratedDt == 0.0 &&
             invalidCfl.simulatedTime == 0.0 &&
             invalidCfl.totalVolume() == invalidCflVolume,
        "CFL nullo non avanza il solver ne' altera il volume");
    ShallowFlow fastFlow(0.1, 8, 8);
    fastFlow.fillRect(-2.0, 2.0, -2.0, 2.0, 0.5);
    for (int j = fastFlow.cellIndexY(-2.0); j <= fastFlow.cellIndexY(2.0); ++j)
      for (int i = fastFlow.cellIndexX(-2.0); i <= fastFlow.cellIndexX(2.0); ++i)
        if (auto *cell = fastFlow.at(i, j); cell && !cell->solid)
          cell->u = 80.0;
    fastFlow.step(1e-3);
    real maxWaterSpeed = 0.0;
    for (int j = fastFlow.cellIndexY(-2.0); j <= fastFlow.cellIndexY(2.0); ++j)
      for (int i = fastFlow.cellIndexX(-2.0); i <= fastFlow.cellIndexX(2.0); ++i)
        if (const auto *cell = fastFlow.at(i, j); cell && !cell->solid)
          maxWaterSpeed = std::max(maxWaterSpeed, std::abs(cell->u));
    std::printf("[INFO] high-speed water: max=%.3f m/s, substeps=%zu\n",
                maxWaterSpeed, fastFlow.lastSubsteps);
    CHECK("W11", maxWaterSpeed > 30.0 && fastFlow.lastSubsteps > 1 &&
                     std::abs(fastFlow.lastIntegratedDt - 1e-3) < 1e-12,
          "Velocita' acqua non tagliata e CFL adattato");
        ShallowFlow explicitlyLimited(0.1, 8, 8);
        explicitlyLimited.speedLimit = 12.0;
        explicitlyLimited.fillRect(-2.0, 2.0, -2.0, 2.0, 0.5);
        for (int j = explicitlyLimited.cellIndexY(-2.0);
             j <= explicitlyLimited.cellIndexY(2.0); ++j) {
      for (int i = explicitlyLimited.cellIndexX(-2.0);
               i <= explicitlyLimited.cellIndexX(2.0); ++i) {
        if (auto *cell = explicitlyLimited.at(i, j); cell && !cell->solid)
          cell->u = 80.0;
          }
        }
        explicitlyLimited.step(1e-4);
        real explicitlyLimitedSpeed = 0.0;
        for (int j = explicitlyLimited.cellIndexY(-2.0);
             j <= explicitlyLimited.cellIndexY(2.0); ++j) {
      for (int i = explicitlyLimited.cellIndexX(-2.0);
               i <= explicitlyLimited.cellIndexX(2.0); ++i) {
        if (const auto *cell = explicitlyLimited.at(i, j); cell && !cell->solid)
          explicitlyLimitedSpeed =
          std::max(explicitlyLimitedSpeed, std::abs(cell->u));
          }
        }
        CHECK("W12", explicitlyLimitedSpeed <= 12.0,
          "speedLimit positivo resta un limite opzionale del modello");
        ShallowFlow invalidDensity(0.1, 8, 8);
        invalidDensity.liquid.rho = 0.0;
        invalidDensity.fillRect(-2.0, 2.0, -2.0, 2.0, 0.5);
        const real invalidDensityVolume = invalidDensity.totalVolume();
        invalidDensity.step(1e-3);
        CHECK("W13", invalidDensity.lastSubsteps == 0 &&
             invalidDensity.simulatedTime == 0.0 &&
             invalidDensity.totalVolume() == invalidDensityVolume,
          "Densita' non fisica non avvia un CFL indefinito");
      longStep.clear();
      CHECK("W7", longStep.simulatedTime == 0.0 &&
          longStep.lastIntegratedDt == 0.0 &&
          longStep.lastSubsteps == 0 && longStep.totalVolume() == 0.0,
        "clear azzera stato fluido e diagnostica temporale");

  // ---- Conservazione del volume spostato (regressione 2.38 m^3 su 6.0) ----
  {
    // W14: il fondo sale sotto l'acqua ed e' marcato `body` OVUNQUE (nessuna
    // cella bagnata libera vicina): il volume non deve sparire.
    ShallowFlow w(0.1, 8, 8);
    w.bed = [](real, real) { return BedSample(); };
    w.fillRect(-1.0, 1.0, -1.0, 1.0, 0.05);
    const real v0 = w.totalVolume();
    w.bed = [](real, real) {
      BedSample b;
      b.z = 0.2;
      b.body = true;
      return b;
    };
    w.refreshBed();
    CHECK("W14", std::abs(w.totalVolume() - v0) < 1e-9 * v0 &&
                     w.absorbedBy.noNeighbour == 0.0 &&
                     w.absorbedVolume == 0.0,
          "fondo che sale (body ovunque): volume spostato conservato");

    // W15: tutto solido -> nessun posto dove mettere l'acqua: resta in coda
    // (conteggiata nel totale), non e' assorbita; riaperto il fondo, torna.
    ShallowFlow q(0.1, 8, 8);
    q.bed = [](real, real) { return BedSample(); };
    q.fillRect(-0.2, 0.2, -0.2, 0.2, 0.05);
    const real q0 = q.totalVolume();
    q.bed = [](real, real) {
      BedSample b;
      b.solid = true;
      b.body = true;
      b.z = ShallowFlow::SOLID_Z;
      return b;
    };
    q.refreshBed();
    const bool pendingKept = q.pendingVolume() > 0.0;
    CHECK("W15", pendingKept && std::abs(q.totalVolume() - q0) < 1e-9 * q0 &&
                     q.absorbedVolume == 0.0,
          "nessun vicino disponibile: volume in coda, non cancellato");
    q.bed = [](real, real) { return BedSample(); };
    q.refreshBed();
    CHECK("W16", q.pendingVolume() == 0.0 &&
                     std::abs(q.totalVolume() - q0) < 1e-9 * q0,
          "riaperto il fondo la coda viene ricollocata per intero");

    // W17: i contatori per canale sommano a absorbedVolume
    const auto &a = w.absorbedBy;
    const real sum = a.infiltration + a.retention + a.compaction + a.bedRise +
                     a.noNeighbour;
    CHECK("W17", std::abs(sum - w.absorbedVolume) < 1e-12,
          "absorbedBy: somma dei canali == absorbedVolume");
  }

  std::printf("\nRISULTATO WATER SOLVER: %d PASS, %d FAIL\n", passed, failed);
  return failed == 0 ? 0 : 1;
}
