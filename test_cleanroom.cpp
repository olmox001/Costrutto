// ============================================================================
//  test_cleanroom.cpp  -  Test di validazione fisica della Infinite Clean Room
//  Verifica termodinamica dell'aria, fluidodinamica, illuminazione e DP45
// ============================================================================
#include "nqg_cleanroom_engine.hpp"
#include <cstdio>
#include <cmath>
#include <cassert>

using namespace nqg;
using namespace nqg::cleanroom;

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
  std::printf("  TEST NQG CLEAN ROOM: ARIA, LUCI E DINAMICA CORPI\n");
  std::printf("====================================================\n");

  // --------------------------------------------------------------------------
  // C1: Termodinamica dell'Aria (Gas perfetto & Sutherland)
  // --------------------------------------------------------------------------
  {
    AirProperties air;
    real rho = air.density();
    real mu = air.dynamicViscosity();
    real cs = air.speedOfSound();

    CHECK("C1a", std::abs(rho - 1.204) < 0.02,
          "Densita' aria a 20 C, 1 atm: rho = %.4f kg/m^3 (atteso ~1.204)", rho);
    CHECK("C1b", std::abs(mu - 1.82e-5) < 0.1e-5,
          "Viscosita' dinamica Sutherland: mu = %.3e Pa*s", mu);
    CHECK("C1c", std::abs(cs - 343.2) < 2.0,
          "Velocita' del suono nell'aria: cs = %.1f m/s", cs);
  }

  // --------------------------------------------------------------------------
  // C2: Aerodinamica, Reynolds e Spinta di Archimede
  // --------------------------------------------------------------------------
  {
    AirProperties air;
    real r = 0.2; // Raggio 20 cm
    real m = 1.5; // Massa 1.5 kg
    Vec3 pos(0, 0, 5);
    Vec3 vel(0, 0, -10); // Caduta a 10 m/s

    Vec3 fDrag, fBuoyancy;
    real Re = 0;
    air.computeAerodynamicForces(r, m, pos, vel, fDrag, fBuoyancy, Re);

    CHECK("C2a", Re > 1e5, "Numero di Reynolds per sfera a 10 m/s: Re = %.2e", Re);
    CHECK("C2b", fDrag.z > 0, "Resistenza aerodinamica verso l'alto (frena caduta): F_drag_z = +%.3f N", fDrag.z);
    CHECK("C2c", fBuoyancy.z > 0 && fBuoyancy.z < m * 9.81,
          "Spinta di Archimede: F_b = %.4f N", fBuoyancy.z);
  }

  // --------------------------------------------------------------------------
  // C3: Sistema di Illuminazione e Spettro Corpo Nero
  // --------------------------------------------------------------------------
  {
    LightingSystem sys = LightingSystem::createCleanRoomPreset();
    CHECK("C3a", sys.lights.size() >= 3, "Luci attive nel preset cleanroom: %zu", sys.lights.size());

    Rgb d65 = blackbody(6500.0);
    Rgb warm = blackbody(3000.0);

    CHECK("C3b", d65.b > warm.b && warm.r > warm.b,
          "Spettro corpo nero: 6500K piu' freddo (B=%.2f) di 3000K (R=%.2f, B=%.2f)",
          d65.b, warm.r, warm.b);
  }

  // --------------------------------------------------------------------------
  // C4: Intersezione Raggi: Pavimento e Sfere della Clean Room
  // --------------------------------------------------------------------------
  {
    CleanRoomScene scene;
    Vec3 ro(0, 0, 2.0);
    Vec3 rd(0, 0, -1.0); // Raggio puntato dritto sul pavimento
    real t;
    Vec3 n;
    bool hitFloor = scene.intersectFloor(ro, rd, t, n);
    CHECK("C4a", hitFloor && std::abs(t - 2.0) < 1e-4 && n.z == 1.0,
          "Intersezione pavimento a z=0: t = %.3f m", t);

    // Raggio puntato sulla prima sfera
    Vec3 sphCenter = scene.spheres[0].pos;
    Vec3 rdSph = (sphCenter - ro).normalized();
    real tSph;
    Vec3 nSph;
    bool hitSph = scene.intersectSphere(ro, rdSph, sphCenter, scene.spheres[0].radius, tSph, nSph);
    CHECK("C4b", hitSph && tSph > 0, "Intersezione sfera di prova: t = %.3f m", tSph);
  }

  // --------------------------------------------------------------------------
  // C5: Integrazione Dormand-Prince DP45 e Rimbalzo Pavimento
  // --------------------------------------------------------------------------
  {
    AirProperties air;
    PhysicalSphere sphere;
    sphere.pos = Vec3(0, 0, 2.0);
    sphere.vel = Vec3(0, 0, 0); // Caduta da fermo

    // Simula 10 passi da 0.05 s
    for (int i = 0; i < 20; ++i) {
      sphere.step(0.05, air);
    }

    CHECK("C5a", sphere.pos.z >= sphere.radius,
          "Pavimento non penetrato (collisione elastica rispettata): z = %.4f m >= %.4f",
          sphere.pos.z, sphere.radius);
    CHECK("C5b", sphere.lastReynolds > 0, "Reynolds registrato durante la caduta: Re = %.1f", sphere.lastReynolds);
  }

  std::printf("\n====================================================\n");
  std::printf("  RISULTATO TEST CLEAN ROOM: %d PASS, %d FAIL\n", nPass, nFail);
  std::printf("====================================================\n");

  return nFail > 0 ? 1 : 0;
}
