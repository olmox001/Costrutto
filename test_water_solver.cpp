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
      longStep.clear();
      CHECK("W7", longStep.simulatedTime == 0.0 &&
          longStep.lastIntegratedDt == 0.0 &&
          longStep.lastSubsteps == 0 && longStep.totalVolume() == 0.0,
        "clear azzera stato fluido e diagnostica temporale");

  std::printf("\nRISULTATO WATER SOLVER: %d PASS, %d FAIL\n", passed, failed);
  return failed == 0 ? 0 : 1;
}
