// SPDX-License-Identifier: GPL-2.0-or-later
// ============================================================================
//  test_sdf.cpp  -  Validazione del core geometrico a Signed Distance Field
//  Primitive, CSG, normali analitiche, sphere tracing, contatti e collisioni.
// ============================================================================
#include "nqg_cleanroom_engine.hpp"
#include <cmath>
#include <cstdio>

using namespace nqg;
using engine::Vec3;

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
  std::printf("  TEST NQG SDF: GEOMETRIA A CAMPI DI DISTANZA\n");
  std::printf("====================================================\n");

  // S1: primitive e segno
  {
    sdf::Field s = sdf::Field::makeSphere(1.0);
    sdf::Field b = sdf::Field::makeBox(Vec3(1, 2, 3));
    sdf::Field c = sdf::Field::makeCylinderZ(1.0, 2.0);
    CHECK("S1a", std::abs(s.eval(Vec3(3, 0, 0)) - 2.0) < 1e-12 &&
                     s.eval(Vec3(0, 0, 0)) < 0,
          "sfera: d(3,0,0)=%.3f (atteso 2), dentro negativo", s.eval(Vec3(3, 0, 0)));
    CHECK("S1b", std::abs(b.eval(Vec3(4, 0, 0)) - 3.0) < 1e-12 &&
                     std::abs(b.eval(Vec3(0, 0, 0)) + 1.0) < 1e-12,
          "box: d fuori=3 (atteso 3), d centro=-1 (atteso -1)");
    CHECK("S1c", std::abs(c.eval(Vec3(3, 0, 0)) - 2.0) < 1e-12 &&
                     std::abs(c.eval(Vec3(0, 0, 5)) - 3.0) < 1e-12,
          "cilindro: radiale=2, assiale=3");
    Vec3 g;
    b.evalGrad(Vec3(0, 5, 0), g);
    CHECK("S1d", g.y == 1.0 && g.x == 0.0 && g.z == 0.0,
          "normale analitica esatta sulla faccia +Y: (%.1f,%.1f,%.1f)", g.x, g.y, g.z);
  }

  // S2: gradiente unitario (|grad d| = 1) fuori dalla superficie
  {
    sdf::Field c = sdf::Field::makeBox(Vec3(1, 1, 1), 0.2);
    double worst = 0;
    for (int i = 0; i < 200; ++i) {
      Vec3 p(2.5 * std::sin(i * 1.7), 2.5 * std::cos(i * 2.3), 2.5 * std::sin(i * 0.9));
      Vec3 g;
      c.evalGrad(p, g);
      worst = std::max(worst, std::abs(g.norm() - 1.0));
    }
    CHECK("S2", worst < 1e-9, "|grad d| = 1 su 200 punti (scarto max %.2e)", worst);
  }

  // S3: CSG (unione, sottrazione, intersezione, smooth)
  {
    sdf::Field f;
    int a = f.box(Vec3(0, 0, 0), Vec3(2, 2, 2));
    int h = f.sphere(Vec3(0, 0, 0), 1.0);
    f.setRoot(f.subtract(a, h));
    CHECK("S3a", f.eval(Vec3(0, 0, 0)) > 0 && f.eval(Vec3(1.5, 0, 0)) < 0,
          "subtract: cavita' sferica nel cubo (centro fuori, parete dentro)");
    sdf::Field u;
    u.setRoot(u.unite(u.sphere(Vec3(-1, 0, 0), 1), u.sphere(Vec3(1, 0, 0), 1)));
    CHECK("S3b", u.eval(Vec3(0, 0, 0)) <= 1e-12 && u.eval(Vec3(2.5, 0, 0)) > 0,
          "union: due sfere tangenti");
    sdf::Field m;
    m.setRoot(m.smoothUnite(m.sphere(Vec3(-1, 0, 0), 1), m.sphere(Vec3(1, 0, 0), 1), 0.5));
    CHECK("S3c", m.eval(Vec3(0, 0, 0)) < u.eval(Vec3(0, 0, 0)) - 1e-6,
          "smooth union: raccordo riempie la giunzione (d=%.4f)", m.eval(Vec3(0, 0, 0)));
    sdf::Field x;
    x.setRoot(x.intersect(x.box(Vec3(0, 0, 0), Vec3(1, 1, 1)), x.sphere(Vec3(0, 0, 0), 1.2)));
    CHECK("S3d", x.eval(Vec3(1.1, 0, 0)) > 0 && x.eval(Vec3(0.9, 0, 0)) < 0,
          "intersect: cubo troncato da sfera");
  }

  // S4: sphere tracing vs soluzione analitica
  {
    sdf::Field s = sdf::Field::makeSphere(1.0);
    real t;
    Vec3 n;
    bool hit = s.raycast(Vec3(0, 0, 5), Vec3(0, 0, -1), 0.001, 1e9, t, n);
    CHECK("S4a", hit && std::abs(t - 4.0) < 1e-5 && std::abs(n.z - 1.0) < 1e-6,
          "raggio su sfera: t=%.6f (atteso 4), n.z=%.6f", t, n.z);
    bool miss = s.raycast(Vec3(3, 0, 5), Vec3(0, 0, -1), 0.001, 1e9, t, n);
    CHECK("S4b", !miss, "raggio che manca la sfera");
    bool out = s.raycast(Vec3(0, 0, 0), Vec3(1, 0, 0), 0.001, 1e9, t, n);
    CHECK("S4c", out && std::abs(t - 1.0) < 1e-5 && n.x > 0.99,
          "partenza interna: uscita a t=%.6f con normale uscente", t);
    sdf::Field b = sdf::Field::makeBox(Vec3(1, 1, 1));
    bool hb = b.raycast(Vec3(-5, 0.3, 0.2), Vec3(1, 0, 0), 0.001, 1e9, t, n);
    CHECK("S4d", hb && std::abs(t - 4.0) < 1e-5 && n.x == -1.0,
          "raggio su box: t=%.6f, n.x=%.1f", t, n.x);
  }

  // S5: punti di superficie campionati dal campo
  {
    continuum::RigidSolidElement r;
    r.pos = Vec3(0, 0, 0);
    r.size = Vec3(2, 1, 0.5);
    auto pts = r.supportPointsWorld();
    double worst = 0;
    for (auto &p : pts)
      worst = std::max(worst, std::abs(r.distanceWorld(p)));
    CHECK("S5a", worst < 1e-9, "26 punti d'appoggio sulla superficie (|d| max %.2e)", worst);
    bool hasCorner = false;
    for (auto &p : pts)
      if (std::abs(p.x - 1.0) < 1e-6 && std::abs(p.y - 0.5) < 1e-6 &&
          std::abs(p.z - 0.25) < 1e-6)
        hasCorner = true;
    CHECK("S5b", hasCorner, "spigolo (1, .5, .25) recuperato per raycast");
  }

  // S6: contatti solido-solido via SDF
  {
    continuum::RigidSolidElement A, B;
    A.size = Vec3(1, 1, 1);
    B.size = Vec3(1, 1, 1);
    A.pos = Vec3(0, 0, 0);
    B.pos = Vec3(0, 0, 0.9); // B sopra A, penetrazione 0.1
    Vec3 n;
    real ov;
    bool c = continuum::RigidSolidElement::sdfOverlap(B, A, n, ov);
    CHECK("S6a", c && std::abs(ov - 0.1) < 1e-6 && n.z > 0.99,
          "overlap=%.4f (atteso 0.1), normale da A verso B = (%.2f,%.2f,%.2f)", ov, n.x, n.y, n.z);
    auto m = continuum::RigidSolidElement::buildManifold(B, A);
    CHECK("S6b", m.count >= 3, "manifold con %d punti di contatto", m.count);
    B.pos = Vec3(0, 0, 1.2);
    CHECK("S6c", !continuum::RigidSolidElement::sdfOverlap(B, A, n, ov),
          "nessuna sovrapposizione a distanza");
    // ruotato di 45 gradi attorno a z
    B.pos = Vec3(0, 0, 0.95);
    B.ex = Vec3(std::sqrt(0.5), std::sqrt(0.5), 0);
    B.ey = Vec3(-std::sqrt(0.5), std::sqrt(0.5), 0);
    B.ez = Vec3(0, 0, 1);
    CHECK("S6d", continuum::RigidSolidElement::sdfOverlap(B, A, n, ov) && n.z > 0.9,
          "solido orientato: overlap=%.4f", ov);
  }

  // S7: sfera-solido e forme non-box
  {
    continuum::RigidSolidElement b;
    b.pos = Vec3(0, 0, 0);
    b.size = Vec3(2, 2, 2);
    Vec3 n;
    real pen = 0;
    CHECK("S7a", b.sphereContact(Vec3(0, 0, 1.2), 0.3, n, pen) && std::abs(pen - 0.1) < 1e-9 && n.z == 1.0,
          "sfera r=0.3 a quota 1.2 sul cubo: pen=%.3f", pen);
    continuum::RigidSolidElement cyl;
    cyl.pos = Vec3(0, 0, 0);
    cyl.shape = continuum::RigidSolidElement::Shape::Cylinder;
    cyl.size = Vec3(1, 1, 2);
    real t = 0;
    CHECK("S7b", cyl.raycast(Vec3(3, 0, 0), Vec3(-1, 0, 0), t, n) && std::abs(t - 2.5) < 1e-5,
          "cilindro: raggio radiale t=%.5f (atteso 2.5)", t);
    continuum::RigidSolidElement sp;
    sp.pos = Vec3(0, 0, 0);
    sp.shape = continuum::RigidSolidElement::Shape::Sphere;
    sp.size = Vec3(2, 2, 2);
    CHECK("S7c", std::abs(sp.distanceWorld(Vec3(0, 0, 3)) - 2.0) < 1e-9 &&
                     std::abs(sp.volume() - 4.0 / 3.0 * PI) < 1e-9,
          "sfera SDF: distanza e volume");
  }

  // S8: stanza (shell CSG) e giocatore
  {
    apartment::RoomGeometry room;
    CHECK("S8a", room.distance(Vec3(0, 0, 1.5)) > 1.0 && room.distance(Vec3(6.1, 0, 1.5)) < 0,
          "interno stanza libero (d=%.2f), parete est solida", room.distance(Vec3(0, 0, 1.5)));
    CHECK("S8b", room.distance(Vec3(0, -5.1, 1.0)) > 0 && room.distance(Vec3(-3, -5.1, 1.0)) < 0,
          "varco della porta aperto, muro sud chiuso");
    apartment::CapsuleCollider cap;
    Vec3 pos(5.9, 0, 1.7), vel(3, 0, 0);
    bool moved = apartment::resolveCapsuleRoom(pos, vel, cap, room);
    CHECK("S8c", moved && pos.x <= room.xMax - cap.radius + 1e-3 && vel.x <= 1e-9,
          "capsula respinta dal muro: x=%.3f (max %.3f), vx=%.2f", pos.x,
          room.xMax - cap.radius, vel.x);
    Vec3 p2(0, 0, 1.65), v2(0, 0, -2);
    apartment::resolveCapsuleRoom(p2, v2, cap, room);
    CHECK("S8d", p2.z - cap.eyeHeight >= -1e-6 && v2.z >= 0,
          "capsula sorretta dal pavimento: piedi a z=%.4f", p2.z - cap.eyeHeight);
  }

  // S9: scena integrata
  {
    cleanroom::CleanRoomScene scene;
    CHECK("S9a", scene.sceneDistance(Vec3(0, 0, 1.5)) > 0.5,
          "sceneDistance: centro stanza libero (d=%.2f)", scene.sceneDistance(Vec3(0, 0, 1.5)));
    Vec3 on = Vec3(-3.0, 2.5, 0.8);
    const real ao = scene.ambientOcclusion(on + Vec3(0, 0, 0.001), Vec3(0, 0, 1));
    const real aoCorner = scene.ambientOcclusion(Vec3(5.95, 4.95, 0.01), Vec3(0, 0, 1));
    CHECK("S9b", ao <= 1.0 && aoCorner < 1.0, "AO: piano %.2f, angolo parete %.2f", ao, aoCorner);
    auto img = scene.render(32, 18, Vec3(0, -3, 1.7), 0.0, 0.0);
    real m = 0;
    for (float v : img.px)
      m += v;
    m /= img.px.size();
    CHECK("S9c", m > 0.02 && m < 0.99, "render completo via sphere tracing: media %.3f", m);
    scene.stepPhysics(0.02);
    bool fin = true;
    for (auto &s : scene.solids)
      fin &= std::isfinite(s.pos.x) && std::isfinite(s.pos.y) && std::isfinite(s.pos.z);
    CHECK("S9d", fin, "passo fisico con contatti SDF stabile");
  }


  // S10: nessuna compenetrazione, neanche con spawn nello stesso punto
  {
    continuum::RigidSolidElement A, B;
    A.size = Vec3(1, 1, 1);
    B.size = Vec3(1, 1, 1);
    A.pos = Vec3(0, 0, 0);
    B.pos = Vec3(0, 0, 0); // solidi COINCIDENTI
    Vec3 n;
    real ov = 0;
    CHECK("S10a", continuum::RigidSolidElement::sdfOverlap(A, B, n, ov) && ov > 0.99,
          "solidi coincidenti rilevati: overlap=%.3f (atteso 1.0)", ov);
    continuum::RigidSolidElement S1, S2;
    S1.shape = S2.shape = continuum::RigidSolidElement::Shape::Sphere;
    S1.size = S2.size = Vec3(1, 1, 1);
    CHECK("S10b", continuum::RigidSolidElement::sdfOverlap(S1, S2, n, ov) && ov > 0.99,
          "sfere coincidenti rilevate: overlap=%.3f", ov);

    cleanroom::CleanRoomScene sc;
    const std::size_t base = sc.solids.size();
    for (int k = 0; k < 6; ++k) {
      continuum::RigidSolidElement b;
      b.pos = Vec3(0, 0, 1.5); // sempre lo stesso punto
      b.size = Vec3(0.4, 0.4, 0.4);
      sc.addSolid(b);
    }
    auto maxPen = [&]() {
      real worst = 0;
      for (std::size_t i = 0; i < sc.solids.size(); ++i)
        for (std::size_t j = i + 1; j < sc.solids.size(); ++j) {
          if (sc.solids[i].isStatic && sc.solids[j].isStatic)
            continue;
          Vec3 nn;
          real o;
          if (continuum::RigidSolidElement::sdfOverlap(sc.solids[i], sc.solids[j], nn, o))
            worst = std::max(worst, o);
        }
      return worst;
    };
    CHECK("S10c", maxPen() == 0.0, "6 solidi spawnati nello stesso punto: penetrazione %.2e", maxPen());
    real worst = 0;
    for (int i = 0; i < 400; ++i) {
      sc.stepPhysics(0.01);
      worst = std::max(worst, maxPen());
    }
    CHECK("S10d", worst < 1e-6, "400 passi di fisica: penetrazione massima %.2e", worst);
    // spawn forzato dentro un altro (bypass di addSolid): la fisica lo separa
    continuum::RigidSolidElement f = sc.solids[base];
    f.pos = sc.solids[base].pos;
    sc.solids.push_back(f);
    sc.stepPhysics(0.01);
    CHECK("S10e", maxPen() < 1e-6, "inserimento forzato sovrapposto: separato, penetrazione %.2e", maxPen());
  }

  std::printf("\n====================================================\n");
  std::printf("  RISULTATO TEST SDF: %d PASS, %d FAIL\n", nPass, nFail);
  std::printf("====================================================\n");
  return nFail ? 1 : 0;
}
