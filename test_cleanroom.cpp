// ============================================================================
//  test_cleanroom.cpp  -  Test di validazione fisica della Infinite Clean Room
//  Verifica termodinamica dell'aria, fluidodinamica, illuminazione e DP45
// ============================================================================
#include "nqg_cleanroom_engine.hpp"
#include <cstdio>
#include <cmath>
#include <cassert>
#include <memory>

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
    const continuum::RigidSolidElement *ball = nullptr;
    for (const auto &so : scene.solids)
      if (so.shape == continuum::RigidSolidElement::Shape::Sphere && !so.isStatic)
        ball = &so;
    Vec3 sphCenter = ball ? ball->pos : Vec3(0, 3, 2);
    Vec3 rdSph = (sphCenter - ro).normalized();
    real tSph = 0;
    Vec3 nSph;
    bool hitSph = scene.intersectSphere(ro, rdSph, sphCenter, ball ? ball->sphereRadius() : 0.35, tSph, nSph);
    CHECK("C4b", hitSph && tSph > 0, "Intersezione sfera di prova: t = %.3f m", tSph);
  }

  // --------------------------------------------------------------------------
  // C5: Integrazione Dormand-Prince DP45 e Rimbalzo Pavimento
  // --------------------------------------------------------------------------
  {
    // La sfera e' un RigidSolidElement come ogni altro corpo: massa derivata
    // da densita' x volume, stessa idrodinamica/aerodinamica e stesso contatto.
    continuum::ContinuousWaterBody dryWater;
    continuum::ContinuousWindField calmWind;
    calmWind.baseDrift = Vec3(0, 0, 0);
    calmWind.turbulenceIntensity = 0.0;
    continuum::RigidSolidElement sphere;
    sphere.shape = continuum::RigidSolidElement::Shape::Sphere;
    sphere.size = Vec3(0.7, 0.7, 0.7);
    sphere.density = 13.92;
    sphere.pos = Vec3(0, 0, 2.0);
    sphere.vel = Vec3(0, 0, 0); // Caduta da fermo
    sphere.syncMass();
    CHECK("C5m", std::abs(sphere.mass - 2.5) < 0.02,
          "Massa derivata rho*V = %.3f kg (atteso ~2.5)", sphere.mass);

    for (int i = 0; i < 20; ++i)
      sphere.stepDynamics(0.05, Vec3(0, 0, -9.80665), dryWater, calmWind, 1.204, 0.0);

    CHECK("C5a", sphere.pos.z >= sphere.sphereRadius() - 1e-6,
          "Pavimento non penetrato (collisione elastica rispettata): z = %.4f m >= %.4f",
          sphere.pos.z, sphere.sphereRadius());
    CHECK("C5b", sphere.lastReynolds > 0, "Reynolds registrato durante la caduta: Re = %.1f", sphere.lastReynolds);
  }

  // --------------------------------------------------------------------------
  // C6: Regole UNICHE per ogni forma: galleggiamento da densita' (Archimede)
  // --------------------------------------------------------------------------
  {
    using continuum::RigidSolidElement;
    auto makePool = []() {
      auto p = std::make_unique<continuum::ContinuousWaterBody>();
      p->setBedProvider([](real x, real y) {
        fluid::BedSample b;
        if (std::abs(x) > 2.05 || std::abs(y) > 2.05) {
          b.solid = true;
          b.z = fluid::ShallowFlow::SOLID_Z;
        }
        return b;
      });
      p->initialFill(-2.0, 2.0, -2.0, 2.0, 1.0);
      return p;
    };
    continuum::ContinuousWindField calm;
    calm.baseDrift = Vec3(0, 0, 0);
    calm.turbulenceIntensity = 0.0;
    const Vec3 g(0, 0, -9.80665);
    real frac[3] = {0, 0, 0};
    const RigidSolidElement::Shape shapes[3] = {RigidSolidElement::Shape::Box, RigidSolidElement::Shape::Sphere, RigidSolidElement::Shape::Cylinder};
    for (int k = 0; k < 3; ++k) {
      auto pool = makePool();
      RigidSolidElement b;
      b.shape = shapes[k];
      b.size = Vec3(0.4, 0.4, 0.4);
      b.density = 500.0; // meta' dell'acqua
      b.pos = Vec3(0, 0, 1.0);
      b.syncMass();
      // il corpo oscilla (vasca chiusa, seiche): media temporale sull'ultimo tratto
      real acc = 0;
      int na = 0;
      for (int i = 0; i < 1600; ++i) {
        pool->step(0.01);
        b.stepDynamics(0.01, g, *pool, calm, 1.204, 0.0);
        if (i >= 600) {
          acc += continuum::bodyHydro(*pool, b, b.vel, 9.80665).submergedFraction;
          ++na;
        }
      }
      frac[k] = acc / na;
    }
    CHECK("C6a", std::abs(frac[0] - 0.5) < 0.12 && std::abs(frac[1] - 0.5) < 0.12 && std::abs(frac[2] - 0.5) < 0.12,
          "Galleggiamento con densita' 500: frazione immersa box=%.2f sfera=%.2f cilindro=%.2f (atteso 0.50)",
          frac[0], frac[1], frac[2]);
  }

  // --------------------------------------------------------------------------
  // C7: Giocatore = corpo rigido: cubi lo bloccano, spinge in base alla massa
  // --------------------------------------------------------------------------
  {
    CleanRoomScene sc;
    sc.water.clear();
    apartment::CapsuleCollider cap;
    const int pi = sc.playerIndex();
    CHECK("C7a", pi >= 0 && std::abs(sc.solids[std::size_t(pi)].mass - 74.0) < 3.0,
          "Massa giocatore derivata da densita' x volume = %.1f kg", pi >= 0 ? sc.solids[std::size_t(pi)].mass : -1.0);

    // blocco statico: la capsula non lo attraversa
    continuum::RigidSolidElement wallBox;
    wallBox.pos = Vec3(-3.0, -2.0, 0.5);
    wallBox.size = Vec3(1, 1, 1);
    wallBox.isStatic = true;
    wallBox.mass = 1e9;
    sc.solids.push_back(wallBox);
    Vec3 cam(-3.0, -4.0, cap.eyeHeight + 0.002), cv(0, 0, 0);
    for (int i = 0; i < 200; ++i) {
      cam.y += 0.03;
      sc.playerMoveVel = Vec3(0, 1.8, 0);
      sc.resolvePlayerCollision(cam, cv, cap);
    }
    CHECK("C7b", cam.y <= -2.5 - cap.radius + 1e-2, "Cubo statico blocca il giocatore: y=%.3f (max %.3f)", cam.y, -2.5 - cap.radius);

    // cassa dinamica: viene spinta (massa 8 kg << 74 kg)
    continuum::RigidSolidElement crate;
    crate.pos = Vec3(2.0, -2.0, 0.4);
    crate.size = Vec3(0.8, 0.8, 0.8);
    crate.density = 15.6; // 8 kg
    const int ci = sc.addSolid(crate);
    const real y0 = sc.solids[std::size_t(ci)].pos.y;
    Vec3 cam2(2.0, -4.0, cap.eyeHeight + 0.002), cv2(0, 0, 0);
    for (int i = 0; i < 150; ++i) {
      cam2.y += 0.016 * 1.5;
      sc.playerMoveVel = Vec3(0, 1.5, 0);
      sc.resolvePlayerCollision(cam2, cv2, cap);
    }
    CHECK("C7c", sc.solids[std::size_t(ci)].pos.y > y0 + 0.2,
          "Il giocatore sposta la cassa leggera: y %.2f -> %.2f", y0, sc.solids[std::size_t(ci)].pos.y);
  }

  // --------------------------------------------------------------------------
  // C8: Statici e mobili alla pari: un solido appoggiato e' fondo per l'acqua
  // --------------------------------------------------------------------------
  {
    CleanRoomScene sc;
    int nStaticGrounded = 0;
    for (const auto &so : sc.solids)
      if (so.isStatic && !so.isRoomSlab && so.waterGrounded)
        ++nStaticGrounded;
    CHECK("C8a", nStaticGrounded > 0, "Solidi statici appoggiati = fondo per l'acqua: %d", nStaticGrounded);

    continuum::RigidSolidElement c;
    c.setMaterial(materials::Id::Concrete);
    c.size = Vec3(1.0, 1.0, 0.3);
    c.pos = Vec3(-3.5, -3.0, 0.15);
    const int ci = sc.addSolid(c);
    for (int i = 0; i < 200; ++i)
      sc.stepPhysics(0.01);
    const auto &b = sc.solids[std::size_t(ci)];
    CHECK("C8b", b.waterGrounded && !b.isStatic, "Solido MOBILE appoggiato trattato come fondo: z=%.3f", b.pos.z);

    const real v0 = sc.water.totalVolume();
    sc.water.addVolume(b.pos.x, b.pos.y, 0.2, 0.05);
    for (int i = 0; i < 300; ++i)
      sc.stepPhysics(0.01);
    const real top = b.pos.z + 0.5 * b.size.z;
    auto sm = sc.water.flow.sample(b.pos.x, b.pos.y);
    CHECK("C8c", sm.wet && sm.bed > top - 2e-3 && sm.eta >= top - 2e-3,
          "Acqua versata sopra il solido mobile resta sopra: eta=%.3f fondo=%.3f sommita'=%.3f", sm.eta, sm.bed, top);
    CHECK("C8d", std::abs((sc.water.totalVolume() - v0) - 0.05) < 0.015,
          "Volume conservato dopo il versamento: dV=%.4f (atteso 0.05)", sc.water.totalVolume() - v0);
  }

  // --------------------------------------------------------------------------
  // C9: Massa sempre derivata da materiale x geometria (nessun valore a mano)
  // --------------------------------------------------------------------------
  {
    CleanRoomScene sc;
    bool derived = true;
    int nDyn = 0;
    real ballMass = 0;
    for (const auto &so : sc.solids) {
      if (so.isStatic)
        continue;
      ++nDyn;
      const real m = so.density * so.fillFraction * so.volume();
      derived &= std::abs(so.mass - m) < 1e-9 * std::max(1.0, m);
      if (so.shape == continuum::RigidSolidElement::Shape::Sphere)
        ballMass = so.mass;
    }
    CHECK("C9a", derived && nDyn >= 4, "Massa = rho x fill x V per tutti i %d solidi mobili", nDyn);
    CHECK("C9b", std::abs(ballMass - 2.5) < 0.2, "Palla di gomma cava (guscio 1.5 mm): m=%.2f kg", ballMass);
  }

  std::printf("\n====================================================\n");
  std::printf("  RISULTATO TEST CLEAN ROOM: %d PASS, %d FAIL\n", nPass, nFail);
  std::printf("====================================================\n");

  return nFail > 0 ? 1 : 0;
}
