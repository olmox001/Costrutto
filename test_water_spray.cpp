// SPDX-License-Identifier: GPL-2.0-or-later
// ============================================================================
//  test_water_spray.cpp - gocce, getti, cascate, pioggia, film sottili
// ============================================================================
#include "nqg_cleanroom_engine.hpp"
#include <cstdio>
using namespace nqg;
using namespace nqg::cleanroom;
using namespace nqg::spray;
static int nP = 0, nF = 0;
#define CHECK(tag, cond, ...)                                                  \
  do {                                                                         \
    bool ok_ = (cond);                                                         \
    (ok_ ? nP : nF)++;                                                         \
    std::printf("[%s] %-5s ", ok_ ? "PASS" : "FAIL", tag);                     \
    std::printf(__VA_ARGS__);                                                  \
    std::printf("\n");                                                         \
  } while (0)

static Environment envOf(CleanRoomScene &sc) {
  Environment e;
  e.distance = [&sc](const Vec3 &p) { return sc.sceneDistance(p, 1.5); };
  e.wind = [](const Vec3 &, real) { return Vec3(0, 0, 0); };
  return e;
}
// volume totale del sistema: flusso + assorbito + modulo spray
static real system(CleanRoomScene &sc) {
  return sc.water.totalVolume() + sc.water.flow.absorbedVolume + sc.spray.volume() +
         sc.spray.stats.lost;
}

int main() {
  std::printf("====================================================\n");
  std::printf("  TEST NQG WATER SPRAY: GOCCE, GETTI, CASCATE, FILM\n");
  std::printf("====================================================\n");
  const fluid::Liquid L = fluid::waterAtKelvin(293.15);
  const real g = 9.80665;

  { // formule chiuse
    const real E = 0.01;
    const real q = law::overfallDischarge(g, E);
    CHECK("Q1a", std::abs(q - 0.5443 * std::sqrt(g) * std::pow(E, 1.5)) < 1e-5,
          "sfioro: q = (2/3)^1.5 sqrt(g) E^1.5 = %.6f m^2/s", q);
    CHECK("Q1b", std::abs(law::brinkSpeed(g, E) * law::brinkDepth(E) - q) < 1e-12,
          "continuita' al bordo: u_b h_b = q (u_b = %.3f m/s)", law::brinkSpeed(g, E));
    const real lc = fluid::capillaryLength(L, g);
    const real V = law::tateVolume(L, g, law::edgeRidgeWavelength(L, g));
    CHECK("Q1c", std::abs(V - L.sigma * 2 * PI * std::sqrt(2.0) * lc / (L.rho * g)) < 1e-15 &&
                     law::sphereRadius(V) > 1e-3 && law::sphereRadius(V) < 8e-3,
          "Tate/Rayleigh-Plateau: goccia di bordo r = %.2f mm", law::sphereRadius(V) * 1e3);
    const real a = law::capBaseFromVolume(2e-8, L.contactAngle);
    CHECK("Q1d", std::abs(law::capVolumeFromBase(a, L.contactAngle) - 2e-8) < 1e-16,
          "calotta sferica: V -> a -> V (a = %.2f mm)", a * 1e3);
    CHECK("Q1e", law::splashK(L, 0.006, 4.0) > law::SPLASH_K &&
                     law::splashK(L, 0.0005, 0.3) < law::SPLASH_K,
          "Mundo-Sommerfeld: K(6mm,4m/s)=%.0f splash, K(0.5mm,0.3m/s)=%.1f aderisce",
          law::splashK(L, 0.006, 4.0), law::splashK(L, 0.0005, 0.3));
    CHECK("Q1f", law::detaches(L, g, 0.01, 0.0) && !law::detaches(L, g, 0.0005, 0.0),
          "scorre o cade: h=10 mm cade, h=0.5 mm resta ancorato");
    {
      const real hp = law::pinnedDepth(L, g, 0.0), hq = law::pinnedDepth(L, g, 0.5);
      CHECK("Q1g", std::abs(law::detachDrive(L, g, hp, 0.0) - fluid::contactLineThreshold(L)) <
                           1e-12 &&
                       std::abs(law::detachDrive(L, g, hq, 0.5) - fluid::contactLineThreshold(L)) <
                           1e-12 && hq < hp,
            "soglia continua: h_pin = %.3f mm (u=0), %.3f mm (u=0.5 m/s)", hp * 1e3, hq * 1e3);
    }
  }

  { // cascata dal tavolo: il flusso non teletrasporta, esce come gocce
    CleanRoomScene sc;
    sc.water.clear();
    Environment env = envOf(sc);
    sc.water.addVolume(-3.0, 2.5, 0.25, 0.012);
    const real v0 = system(sc);
    real tFirst = -1, maxDrops = 0;
    bool onFloor = false;
    for (int i = 1; i <= 400; ++i) {
      sc.stepPhysics(0.01);
      if (tFirst < 0 && sc.spray.stats.fromFlow > 0)
        tFirst = i * 0.01;
      maxDrops = std::max<real>(maxDrops, real(sc.spray.drops.size()));
    }
    real off = 0;
    const real dx = sc.water.flow.dx;
    for (real y = 1.5; y < 3.5; y += dx)
      for (real x = -4.5; x < -1.5; x += dx)
        if (!(x > -3.9 && x < -2.1 && y > 2.05 && y < 2.95))
          off += sc.water.depthAt(x, y) * dx * dx;
    onFloor = off + sc.spray.sessileVolume() > 1e-3;
    CHECK("Q2a", tFirst > 0 && tFirst < 1.0, "il bordo emette acqua dopo t=%.2f s", tFirst);
    CHECK("Q2b", maxDrops > 20, "particelle d'acqua in volo/appoggiate: max %.0f", maxDrops);
    CHECK("Q2c", onFloor, "l'acqua e' arrivata a terra (%.4f m^3 fuori dal tavolo)",
          off + sc.spray.sessileVolume());
    CHECK("Q2d", std::abs(system(sc) - v0) < 1e-7 * v0 + 1e-9,
          "volume conservato: %.8f -> %.8f m^3", v0, system(sc));
  }

  { // film sottile sul bordo: resta (ancoraggio), nessuna goccia
    CleanRoomScene sc;
    sc.water.clear();
    Environment env = envOf(sc);
    sc.water.addVolume(-3.0, 2.5, 0.2, 2.0e-6); // ~ 0.04 mm di film
    for (int i = 0; i < 200; ++i)
      sc.stepPhysics(0.01);
    CHECK("Q3a", sc.spray.stats.fromFlow < 5e-7 + 1e-6,
          "film troppo sottile per staccarsi: estratto %.2e m^3", sc.spray.stats.fromFlow);
  }

  { // rottura del film: sotto lo spessore minimo -> perline, volume conservato
    CleanRoomScene sc;
    sc.water.clear();
    sc.water.flow.fillRect(-3.4, -2.6, 2.2, 2.8, 4.0e-4); // 0.4 mm su tavolo
    Environment env = envOf(sc);
    const real v0 = system(sc);
    for (int i = 0; i < 100; ++i)
      sc.stepPhysics(0.01);
    CHECK("Q4a", sc.spray.stats.ruptures > 0 && sc.spray.sessileVolume() > 0,
          "film sotto filmMin si separa in perline: %zu rotture, %.2e m^3",
          sc.spray.stats.ruptures, sc.spray.sessileVolume());
    CHECK("Q4b", std::abs(system(sc) - v0) < 1e-7 * v0 + 1e-10,
          "volume conservato nella rottura: %.6e -> %.6e", v0, system(sc));
  }

  { // pioggia: volume emesso = R * A * t, Marshall-Palmer
    CleanRoomScene sc;
    sc.water.clear();
    sc.setRain(50.0, -1.0, 1.0, -1.0, 1.0, 3.0);
    for (int i = 0; i < 200; ++i)
      sc.stepPhysics(0.01);
    const real expected = 50.0e-3 / 3600.0 * 4.0 * 2.0;
    CHECK("Q5a", std::abs(sc.spray.stats.emittedExternal - expected) < 0.35 * expected,
          "pioggia 50 mm/h su 4 m^2 per 2 s: %.3e m^3 (atteso %.3e)",
          sc.spray.stats.emittedExternal, expected);
    CHECK("Q5b", std::abs(system(sc) - sc.spray.stats.emittedExternal) <
                     1e-7 + 1e-6 * expected,
          "tutta la pioggia e' contabilizzata (flusso+gocce+assorbito)");
  }

  { // impatto: splash conserva il volume, goccia lenta aderisce
    CleanRoomScene sc;
    sc.water.clear();
    Environment env = envOf(sc);
    Drop big;
    big.volume = law::sphereVolume(0.003);
    big.pos = Vec3(0, 0, 0.5);
    big.vel = Vec3(0, 0, -5.0);
    sc.spray.drops.push_back(big);
    const real v0 = system(sc);
    for (int i = 0; i < 100; ++i)
      sc.stepPhysics(0.005);
    CHECK("Q6a", sc.spray.stats.splashes > 0 && std::abs(system(sc) - v0) < 1e-12,
          "splash (K>57.7): %zu eventi, volume conservato", sc.spray.stats.splashes);
    Drop slow;
    slow.volume = law::sphereVolume(0.0005);
    slow.pos = Vec3(1, 1, 0.01 + 0.0006);
    slow.vel = Vec3(0, 0, -0.3);
    CleanRoomScene s2;
    s2.water.clear();
    s2.spray.drops.push_back(slow);
    for (int i = 0; i < 40; ++i)
      s2.stepPhysics(0.005);
    bool stuck = false;
    for (auto &d : s2.spray.drops)
      stuck |= d.sessile;
    CHECK("Q6b", stuck && s2.spray.stats.splashes == 0, "goccia lenta aderisce come calotta");
  }

  { // ottimizzazione multiparticellare: budget, volume e moto conservati
    WaterSpray sp;
    sp.maxDrops = 500;
    Vec3 P0(0, 0, 0);
    real V0 = 0;
    for (int i = 0; i < 4000; ++i) {
      Drop d;
      d.volume = 1e-8;
      d.pos = Vec3(0.001 * (i % 50), 0.001 * ((i / 50) % 40), 1.0);
      d.vel = Vec3(0.01 * (i % 7), 0, -2.0);
      V0 += d.volume;
      P0 = P0 + d.vel * d.volume;
      sp.drops.push_back(d);
    }
    sp.limitParticles();
    real V1 = 0;
    Vec3 P1(0, 0, 0);
    for (auto &d : sp.drops) {
      V1 += d.volume;
      P1 = P1 + d.vel * d.volume;
    }
    CHECK("Q7a", sp.drops.size() <= 500 && std::abs(V1 - V0) < 1e-15 &&
                     (P1 - P0).norm() < 1e-12,
          "4000 -> %zu particelle, volume e quantita' di moto conservati",
          sp.drops.size());
  }

  { // rendering: la goccia in volo compare nel buffer
    CleanRoomScene sc;
    sc.water.clear();
    Drop d;
    d.volume = law::sphereVolume(0.05);
    d.pos = Vec3(0, 1.0, 1.7);
    sc.spray.drops.push_back(d);
    sc.spray.prepare();
    WaterSpray::Hit h;
    const bool hit = sc.spray.raycast(Vec3(0, -2, 1.7), Vec3(0, 1, 0), 1e9, h);
    CHECK("Q8a", hit && std::abs(h.t - (3.0 - 0.05)) < 1e-6 && h.n.y < -0.99,
          "raggio su goccia: t = %.4f, normale uscente", h.t);

    Observer simulationView;
    simulationView.valid = true;
    simulationView.pos = Vec3(0, -2, 1.7);
    simulationView.fwd = Vec3(0, 1, 0);
    simulationView.halfDiag = 0.2;
    sc.spray.simulationObserver = simulationView;
    Observer renderView;
    renderView.valid = true;
    renderView.pos = Vec3(0, -2, 1.7);
    renderView.fwd = Vec3(0, -1, 0);
    renderView.halfDiag = 0.2;
    sc.spray.prepare(renderView);
    WaterSpray::Hit hidden;
    const bool hiddenFromRender =
      !sc.spray.raycast(Vec3(0, -2, 1.7), Vec3(0, 1, 0), 1e9, hidden);
    const bool simulationViewPreserved =
      sc.spray.simulationObserver.valid &&
      (sc.spray.simulationObserver.pos - simulationView.pos).norm() < 1e-12 &&
      (sc.spray.simulationObserver.fwd - simulationView.fwd).norm() < 1e-12;
    CHECK("Q8b", hiddenFromRender && simulationViewPreserved,
        "vista render filtra la goccia senza alterare la vista fisica");
  }

  std::printf("\n====================================================\n");
  std::printf("  RISULTATO TEST WATER SPRAY: %d PASS, %d FAIL\n", nP, nF);
  std::printf("====================================================\n");
  return nF ? 1 : 0;
}
