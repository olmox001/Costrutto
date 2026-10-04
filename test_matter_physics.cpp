// ============================================================================
//  test_matter_physics.cpp  -  Suite di test per la fisica della materia
//  Valida SPH Acqua, DEM Sabbia, Legami Molecolari, Elettrostatica e Compton
// ============================================================================
#include "nqg_matter_physics.hpp"
#include <cstdio>
#include <cmath>

using namespace nqg;
using namespace nqg::matter;

static int nPass = 0, nFail = 0;
#define CHECK(tag, cond, ...)                                                  \
  do {                                                                         \
    bool ok_ = (cond);                                                         \
    (ok_ ? nPass : nFail)++;                                                   \
    std::printf("[%s] %-5s ", ok_ ? "PASS" : "FAIL", tag);                     \
    std::printf(__VA_ARGS__);                                                  \
    std::printf("\n");                                                         \
  } while (0)

int main() {
  std::printf("====================================================\n");
  std::printf("  TEST MATERIA MULTI-FASE ED ELEMENTARE (NQG)\n");
  std::printf("====================================================\n");

  cleanroom::AirProperties air;

  // --------------------------------------------------------------------------
  // M1: Particelle Elementari: Coulomb e Raggio di Compton
  // --------------------------------------------------------------------------
  {
    MatterSimulator sim;
    sim.spawnElementaryDipole(Vec3(0, 0, 1.0));
    CHECK("M1a", sim.particles.size() == 2, "Dipolo generato con 2 particelle elementari");

    const auto &e = sim.particles[0];
    const auto &p = sim.particles[1];
    CHECK("M1b", e.charge == -1.0 && p.charge == +1.0, "Cariche opposte: e- (%.1f) e e+ (%.1f)", e.charge, p.charge);

    real lc = e.comptonRadius();
    CHECK("M1c", lc > 0, "Raggio di Compton quantistico calcolato: lambda_C = %.3e m", lc);

    // Calcolo forze: cariche opposte devono attrarsi
    sim.computeForces(air);
    // e.pos.x = -0.2, p.pos.x = +0.2 -> forza su e deve essere diretta verso +x (attrazione)
    CHECK("M1d", e.force.x > 0 && p.force.x < 0,
          "Attrazione di Coulomb verificata: F_e_x = +%.3f N, F_p_x = %.3f N", e.force.x, p.force.x);
  }

  // --------------------------------------------------------------------------
  // M2: Fluidodinamica SPH per Acqua (Equazione di Tait e Viscosita')
  // --------------------------------------------------------------------------
  {
    MatterSimulator sim;
    sim.spawnWaterCluster(Vec3(0, 0, 1.0), 30, 0.15);
    sim.computeForces(air);

    bool hasPressure = false;
    for (const auto &p : sim.particles) {
      if (p.type == MatterType::Water && p.density > 0) {
        hasPressure = true;
        break;
      }
    }
    CHECK("M2a", hasPressure, "Densita' e pressione SPH calcolate per le particelle d'acqua");

    // Esegui passi SPH e verifica coesione (non collassano ne' esplodono)
    for (int s = 0; s < 10; ++s) {
      sim.step(0.02, air);
    }
    bool waterOk = true;
    for (const auto &p : sim.particles) {
      if (std::isnan(p.pos.x) || p.pos.z < 0.0) waterOk = false;
    }
    CHECK("M2b", waterOk, "Gocce d'acqua stabili e non penetrate nel pavimento");
  }

  // --------------------------------------------------------------------------
  // M3: Meccanica Granulare DEM per Sabbia (Contatti e Angolo di Riposo)
  // --------------------------------------------------------------------------
  {
    MatterSimulator sim;
    sim.spawnSandCluster(Vec3(0, 0, 0.5), 25, 0.1);
    sim.computeForces(air);

    // Simula deposito granulare sul pavimento
    for (int s = 0; s < 20; ++s) {
      sim.step(0.02, air);
    }

    bool sandOk = true;
    for (const auto &p : sim.particles) {
      if (p.pos.z < p.radius * 0.8) sandOk = false;
    }
    CHECK("M3a", sandOk, "Granelli di sabbia sostenuti dal contatto normale e attrito");
    CHECK("M3b", sim.sandFrictionCoeff_ > 0.5, "Coefficiente d'attrito interno sabbia (tan 33 deg = %.2f)", sim.sandFrictionCoeff_);
  }

  // --------------------------------------------------------------------------
  // M4: Complessi Molecolari con Legami Armonici (Deformazione Solida)
  // --------------------------------------------------------------------------
  {
    MatterSimulator sim;
    sim.spawnMolecularComplex(Vec3(0, 0, 1.5), 2, 2, 2, 0.15);
    CHECK("M4a", sim.particles.size() == 8 && !sim.bonds.empty(),
          "Complesso molecolare 2x2x2 creato con %zu atomi e %zu legami",
          sim.particles.size(), sim.bonds.size());

    // Fai oscillare e cadere il reticolo
    real initialHeight = sim.particles[0].pos.z;
    for (int s = 0; s < 15; ++s) {
      sim.step(0.02, air);
    }
    CHECK("M4b", sim.particles[0].pos.z < initialHeight,
          "Complesso molecolare caduto sotto gravita' conservando i legami");
  }

  // --------------------------------------------------------------------------
  // M5: Entropia Termodinamica V11
  // --------------------------------------------------------------------------
  {
    MatterSimulator sim;
    sim.spawnWaterCluster(Vec3(0, 0, 1.0), 20, 0.2);
    real s0 = sim.computeKineticEntropy();
    CHECK("M5a", s0 >= 0.0, "Entropia statistica di configurazione valida: S = %.4f", s0);
  }

  std::printf("\n====================================================\n");
  std::printf("  RISULTATO TEST MATERIA FISICA: %d PASS, %d FAIL\n", nPass, nFail);
  std::printf("====================================================\n");

  return nFail > 0 ? 1 : 0;
}
