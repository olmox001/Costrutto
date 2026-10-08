// SPDX-License-Identifier: GPL-2.0-or-later
// test_water_budget — bilancio di massa dell'acqua a livello di scena.
// Regressione: il riempimento iniziale avveniva PRIMA della costruzione del
// terreno; al primo refreshBed() il fondo saliva sotto l'acqua e 2.38 m^3 su
// 6.0 venivano cancellati (canale `noNeighbour`).
#include "nqg_engine_api.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

using nqg::api::Host;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    const bool ok_ = (cond);                                                   \
    std::printf("[%s] %s\n", ok_ ? "PASS" : "FAIL", msg);                      \
    (ok_ ? g_pass : g_fail)++;                                                 \
  } while (0)

int main() {
  Host h;
  auto &sc = h.scene();
  auto &F = sc.water.flow;

  const double v0 = F.totalVolume();
  const std::size_t wet0 = F.wetCells();
  CHECK(std::abs(v0 - 6.0) < 1e-6, "volume iniziale = 6.0 m^3");

  // Invariante di inizializzazione: il fondo memorizzato in ogni cella
  // bagnata deve coincidere col fondo che la scena restituisce ORA.
  {
    double maxErr = 0.0;
    int checked = 0;
    for (double y = -4.9; y <= 4.9; y += 0.5)
      for (double x = -5.9; x <= 5.9; x += 0.5) {
        const int i = F.cellIndexX(x), j = F.cellIndexY(y);
        const auto *c = F.at(i, j);
        if (!c || c->solid || !(c->h > 1e-4))
          continue;
        const double z = sc.sampleBed(F.centerX(i), F.centerY(j)).z;
        maxErr = std::max(maxErr, std::abs(c->b - z));
        ++checked;
      }
    std::printf("  bed consistency: %d celle, max|b - bed| = %.3e m\n", checked,
                maxErr);
    CHECK(checked > 100 && maxErr < 1e-9,
          "fondo memorizzato == fondo corrente (acqua creata dopo il terreno)");
  }

  // Il fondo memorizzato nelle celle deve coincidere col fondo corrente:
  // un refresh forzato subito dopo la costruzione non sposta volume.
  F.refreshBed();
  CHECK(std::abs(F.totalVolume() - v0) < 1e-6 && F.absorbedVolume < 1e-9,
        "refreshBed iniziale non perde volume (fondo coerente)");

  h.stepN(60, 1.0 / 60.0); // 1 s, attraversa 4 refresh del fondo (ogni 0.25 s)

  const double total = F.totalVolume() + sc.spray.volume() + F.absorbedVolume;
  CHECK(std::abs(total - v0) < 1e-3, "bilancio: flow + spray + assorbito = iniziale");
  CHECK(F.absorbedBy.noNeighbour == 0.0, "nessun volume spostato cancellato");
  CHECK(F.absorbedBy.compaction < 1e-3, "compattazione tile: perdita trascurabile");
  CHECK(F.totalVolume() > 0.98 * v0, "dopo 1 s resta >98% del volume nel flusso");
  CHECK(F.wetCells() > wet0, "il fronte si espande: piu' celle bagnate di prima");

  std::printf("RESULT: %d PASS, %d FAIL\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
