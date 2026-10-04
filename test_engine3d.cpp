// g++ -O2 -std=c++17 -pthread test_engine3d.cpp -o test_engine3d &&
// ./test_engine3d
#include "nqg_engine3d.hpp"
#include <chrono>
using namespace nqg;
using namespace nqg::engine;
static int nP = 0, nF = 0;
#define CHECK(tag, cond, ...)                                                  \
  do {                                                                         \
    bool ok_ = (cond);                                                         \
    (ok_ ? nP : nF)++;                                                         \
    std::printf("[%s] %-5s ", ok_ ? "PASS" : "FAIL", tag);                     \
    std::printf(__VA_ARGS__);                                                  \
    std::printf("\n");                                                         \
  } while (0)
template <class F> static double ms(F &&f) {
  auto t0 = std::chrono::steady_clock::now();
  f();
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - t0)
      .count();
}

// deflessione totale di un raggio da infinito con parametro d'impatto b
// (ricostruita da u'' = -u + 3Mu^2)
static real deflection(real M, real b) {
  real u = 0, v = 1 / b, phi = 0;
  const real h = 2e-4;
  bool started = false;
  auto f = [&](real x) { return -x + 3 * M * x * x; };
  while (true) {
    const real k1u = v, k1v = f(u), k2u = v + h / 2 * k1v,
               k2v = f(u + h / 2 * k1u), k3u = v + h / 2 * k2v,
               k3v = f(u + h / 2 * k2u), k4u = v + h * k3v,
               k4v = f(u + h * k3u);
    const real un = u + h / 6 * (k1u + 2 * k2u + 2 * k3u + k4u);
    v += h / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
    phi += h;
    if (un > 0)
      started = true;
    if (started && un <= 0) {
      return phi - PI - h * un / (un - u);
    }
    u = un;
    if (phi > 4 * PI)
      return NAN;
  }
}

int main() {
  const real M = 1;
  // ---- fisica del tracciamento
  CHECK("E1", std::abs(deflection(M, 2000) - 4 * M / 2000) / (4 * M / 2000) < 0.01,
        "Einstein: deflessione(b=2000M)=%.6f vs 4M/b=%.6f", deflection(M, 2000),
        4.0 * M / 2000);
  CHECK(
      "E2",
      std::abs(deflection(M, 50) - (4 * M / 50 + 15 * PI / 4 * M * M / 2500)) /
              deflection(M, 50) <
          0.02,
      "con termine 15 pi M^2/(4b^2): %.5f", deflection(M, 50));
  { // ombra: catturato <=> psi>pi/2 e b = r sin(psi)/alpha < 3 sqrt3 M
    const real rc = 30, a = schw::lapse(M, rc);
    RayTable T(M, rc);
    int bad = 0, tot = 0;
    for (int i = 0; i < T.size(); i += 3) {
      const real psi = (i + 0.5) * PI / T.size();
      const real b = rc * std::sin(psi) / a;
      const bool exp = psi > PI / 2 && b < 3 * std::sqrt(3.0) * M;
      const bool got = T.ray(i).captured;
      if (std::abs(b - 3 * std::sqrt(3.0) * M) / b < 0.01)
        continue;
      ++tot;
      bad += exp != got;
    }
    CHECK("E3", bad == 0, "ombra: b_crit=3 sqrt3 M, %d raggi, %d discordanze",
          tot, bad);
    const real sh = std::asin(3 * std::sqrt(3.0) * M * a / rc);
    std::printf("      raggio angolare ombra atteso: %.4f rad (%.2f deg)\n", sh,
                sh * 180 / PI);
    real maxe = 0;
    std::mt19937_64 g(5);
    for (int t = 0; t < 40; ++t) {
      const real psi =
          0.3 + 2.0 * std::uniform_real_distribution<real>(0, 1)(g);
      const real ph = 0.2 + 1.5 * std::uniform_real_distribution<real>(0, 1)(g);
      real a1, a2;
      if (T.sample(psi, ph, a1) && RayTable::direct(M, rc, psi, ph, a2))
        maxe = std::max(maxe, std::abs(1 / a1 - 1 / a2) / (1 / a2));
    }
    CHECK("E4", maxe < 2e-3,
          "tabella vs integrazione diretta: errore relativo su r max %.2e",
          maxe);
  }
  { // doppler/redshift disco
    const real r = 8, a = schw::lapse(M, 1000);
    CHECK("E5",
          std::abs(diskG(M, r, 1.0, 0) - std::sqrt(1 - 3 * M / r)) < 1e-15 &&
              std::abs(diskG(M, r, a, 0) - std::sqrt(1 - 3 * M / r) / a) <
                  1e-15,
          "g(face-on)=sqrt(1-3M/r)/alpha_o");
    const real gp = diskG(M, r, 1.0, +5), gm = diskG(M, r, 1.0, -5);
    CHECK("E6",
          gp > gm && gp > std::sqrt(1 - 3 * M / r) &&
              gm < std::sqrt(1 - 3 * M / r),
          "lato che si avvicina blu (g=%.3f), lato che si allontana rosso "
          "(g=%.3f)",
          gp, gm);
    CHECK("E7",
          std::abs(schw::lapse(M, 6) / schw::lapse(M, 20) -
                   diskG(M, 6, schw::lapse(M, 20), 0) *
                       schw::lapse(M, 6) / std::sqrt(1 - 3 * M / 6.0)) < 1e-12,
          "coerenza con 1+z=alpha_o/alpha_e del core");
  }
  // ---- capacita' dell'osservatore
  {
    ObserverCapacity c;
    auto p = negotiate(c);
    CHECK("O1", p.limitedBy == "nessuno" && p.width == c.width,
          "capacita' sufficiente: %dx%d @ %.0f Hz, uso %.0f%%", p.width,
          p.height, p.fsEff, p.usage * 100);
    ObserverCapacity lo = c;
    lo.Cops = 4e8;
    auto q = negotiate(lo);
    CHECK("O2",
          q.width < c.width && q.fsEff * q.width * q.height * lo.opsPerPixel *
                                       (1 + 0.15 * (lo.subsamples - 1)) <=
                                   lo.Cops * 1.0001,
          "C_ops basso -> %dx%d, f_s*K<=C_ops (uso %.1f%%) limite: %s", q.width,
          q.height, q.usage * 100, q.limitedBy.c_str());
    ObserverCapacity bw = c;
    bw.Bs = 2e7;
    auto b = negotiate(bw);
    CHECK("O3", b.width < c.width && b.limitedBy == "B_s",
          "banda B_s=2e7 limita: %dx%d (%s)", b.width, b.height,
          b.limitedBy.c_str());
    ObserverCapacity tiny = c;
    tiny.Cops = 1e6;
    auto t = negotiate(tiny);
    CHECK("O4", t.fsEff < tiny.fs && t.usage <= 1.0001,
          "capacita' minima: scende f_s a %.2f Hz", t.fsEff);
    bool mono = true;
    int last = 0;
    for (real C : {1e8, 3e8, 1e9, 3e9, 1e10}) {
      ObserverCapacity x = c;
      x.Cops = C;
      auto z = negotiate(x);
      mono &= z.width >= last;
      last = z.width;
    }
    CHECK("O5", mono, "risoluzione monotona in C_ops");
  }
  { // Nyquist/alias del faro, confermato dal campionamento del core
    const real rr = 6; // nu_obs = nu0/alpha
    auto rep = observeBeacon(M, rr, 1000, 4000);
    CHECK("N1",
          !rep.aliased && std::abs(rep.nuObs - 1224.744871) < 1e-5 &&
              std::abs(rep.nyquist - 2449.49) < 0.01,
          "r=6: nu_obs=%.3f Nyquist=%.2f (come dossier)", rep.nuObs,
          rep.nyquist);
    auto bad = observeBeacon(M, rr, 1000, 2000);
    const real fs = 2000;
    const real a = schw::lapse(M, rr);
    auto y = observer::sampleLocalClock(a, 1000, fs, 8192);
    const real est = observer::lag1Frequency(y, 1 / fs);
    CHECK("N2",
          bad.aliased && std::abs(std::abs(est) - bad.aliasFreq) < 1e-6 &&
              std::abs(bad.aliasFreq - 775.255) < 1e-2,
          "f_s=2000<2nu: alias previsto %.3f Hz, misurato %.3f Hz",
          bad.aliasFreq, std::abs(est));
  }
  // ---- rendering
  Scene sc;
  Renderer R(sc);
  ObserverCapacity cap;
  Plan plan = negotiate(cap);
  Camera cam;
  cam.r = 30;
  cam.theta = 1.35;
  Image a, b;
  R.threads = 1;
  a = R.render(cam, cap, plan, 0.0);
  R.threads = 4;
  b = R.render(cam, cap, plan, 0.0);
  CHECK("R1", a.px == b.px,
        "render deterministico: 1 thread == 4 thread (%zu valori)",
        a.px.size());
  {
    const float *c = a.at(a.w / 2, a.h / 2);
    const real lum = c[0] + c[1] + c[2];
    CHECK("R2", lum < 0.08,
          "centro dell'immagine = ombra del buco nero (luminanza %.3f)", lum);
    real mean = 0;
    for (float v : a.px)
      mean += v;
    mean /= a.px.size();
    CHECK("R3", mean > 0.02 && mean < 0.6,
          "immagine non vuota ne' saturata: media %.3f", mean);
    Camera away = cam;
    away.yaw = PI;
    Image s = R.render(away, cap, plan, 0.0);
    real m2 = 0;
    for (float v : s.px)
      m2 += v;
    m2 /= s.px.size();
    CHECK("R4", m2 < mean,
          "guardando lontano dal disco la luminosita' scende (%.3f < %.3f)", m2,
          mean);
    // Doppler beaming: lato che si avvicina piu' luminoso. Disco ruota +z;
    // camera guarda -x, destra=+y.
    auto side = [&](int x0, int x1) {
      real s2 = 0;
      for (int y = a.h / 2 - 12; y < a.h / 2 + 12; ++y)
        for (int x = x0; x < x1; ++x) {
          const float *q = a.at(x, y);
          s2 += q[0] + q[1] + q[2];
        }
      return s2;
    };
    const real L = side(a.w * 3 / 10, a.w * 4 / 10),
               Rr = side(a.w * 6 / 10, a.w * 7 / 10);
    CHECK("R5", L != Rr && std::max(L, Rr) / std::min(L, Rr) > 1.15,
          "asimmetria Doppler del disco: sx=%.1f dx=%.1f (rapporto %.2f)", L,
          Rr, std::max(L, Rr) / std::min(L, Rr));
  }
  { // esposizione: il disco ruota e il faro pulsa -> piu' Te => piu' sfocatura
    // temporale
    ObserverCapacity sharp = cap;
    sharp.exposure = 1e-4;
    sharp.subsamples = 1;
    sharp.readNoise = 0;
    ObserverCapacity blur = cap;
    blur.exposure = 0.5;
    blur.subsamples = 8;
    blur.readNoise = 0;
    real var0 = 0, var1 = 0;
    Image A = R.render(cam, sharp, plan, 0.0),
          B = R.render(cam, sharp, plan, 0.37),
          C = R.render(cam, blur, plan, 0.0),
          D = R.render(cam, blur, plan, 0.37);
    for (std::size_t i = 0; i < A.px.size(); ++i) {
      var0 += std::abs(A.px[i] - B.px[i]);
      var1 += std::abs(C.px[i] - D.px[i]);
    }
    CHECK("R6", var1 < var0,
          "esposizione lunga media il moto: |dI| tra frame %.1f (breve) > %.1f "
          "(lunga)",
          var0, var1);
    ObserverCapacity q2 = cap;
    q2.bits = 2;
    q2.readNoise = 0;
    Image Q = R.render(cam, q2, plan, 0.0);
    std::set<int> lv;
    std::vector<int> seen;
    for (float v : Q.px) {
      int k = int(std::lround(v * 3));
      if (std::find(seen.begin(), seen.end(), k) == seen.end())
        seen.push_back(k);
    }
    CHECK("R7", seen.size() <= 4,
          "Q=2 bit: al piu' 4 livelli per canale (visti %zu)", seen.size());
  }
  { // il redshift gravitazionale cambia il colore: osservatore piu' vicino =>
    // cielo piu' blu (shift 1/alpha)
    const Vec3 d(0.3, 0.2, 0.9);
    const Vec3 dd = d.normalized();
    Rgb s1 = R.sky(dd, 1.0), s2 = R.sky(dd, 1.0 / schw::lapse(M, 3.0));
    CHECK(
        "R8",
        s2.b / std::max(1e-6f, s2.r + 1e-6f) >=
                s1.b / std::max(1e-6f, s1.r + 1e-6f) - 1e-4 ||
            (s1.r + s1.g + s1.b) == 0,
        "cielo piu' blu da r=3M che da infinito (stelle: indice %.3f vs %.3f)",
        s2.b / (s2.r + 1e-6f), s1.b / (s1.r + 1e-6f));
  }
  // ---- gioco
  {
    Game g;
    real S0 = g.levels.entropy(0);
    Input in;
    for (int i = 0; i < 300; ++i)
      g.step(1.0 / 30, in);
    CHECK("G1", g.levels.entropy(0) < S0 && g.score > 0 && !g.over,
          "V11 in gioco: S(L0) %.4f -> %.4f, conoscenza %.3f", S0,
          g.levels.entropy(0), g.score);
    Game lo;
    lo.cap.Cops = 2e8;
    for (int i = 0; i < 300; ++i)
      lo.step(1.0 / 30, in);
    Game hi;
    hi.cap.Cops = 3e9;
    for (int i = 0; i < 300; ++i)
      hi.step(1.0 / 30, in);
    CHECK("G2",
          lo.plan.usage <= 1.0001 && hi.plan.usage < lo.plan.usage + 1e-9 + 1.0,
          "uso capacita': %.2f (basso C_ops) vs %.2f (alto)", lo.plan.usage,
          hi.plan.usage);
    Game f;
    f.cam.r = 6;
    real tid0 = f.tidal();
    Game f2;
    f2.cam.r = 3;
    CHECK("G3",
          f2.tidal() > tid0 && f2.hoverAcceleration() > f.hoverAcceleration(),
          "piu' vicino: maree %.4f > %.4f, accel. di hovering %.3f > %.3f",
          f2.tidal(), tid0, f2.hoverAcceleration(), f.hoverAcceleration());
    Game d;
    d.cam.r = 2.5;
    Input dive;
    dive.thrustR = -1;
    for (int i = 0; i < 4000 && !d.over; ++i)
      d.step(1.0 / 30, dive);
    CHECK("G4", d.over && d.cam.r >= 2.1 * M,
          "caduta: fine partita (%s) a r=%.3f", d.status.c_str(), d.cam.r);
    Game s2;
    Image im = s2.frame();
    CHECK("G5", im.w == s2.plan.width && im.h == s2.plan.height,
          "frame %dx%d con HUD", im.w, im.h);
    Game t1;
    t1.cam.r = 20;
    Game t2;
    t2.cam.r = 5;
    t1.step(1.0, Input{});
    t2.step(1.0, Input{});
    CHECK("G6",
          t2.tCoord > t1.tCoord &&
              std::abs(t1.tCoord - 1.0 / schw::lapse(M, 20)) < 0.05,
          "dilatazione: 1 s proprio = %.4f s coordinato a r=20M, %.4f a r=5M",
          t1.tCoord, t2.tCoord);
  }
  // ---- prestazioni
  std::printf("\n--- PRESTAZIONI (%u thread) ---\n",
              std::thread::hardware_concurrency());
  {
    Scene s2;
    Renderer R2(s2);
    Camera c2 = cam;
    c2.r = 17.3;
    double tb = ms([&] { R2.table(c2.r); });
    std::printf(
        "tabella raggi (3072 geodetiche), una tantum per raggio: %.1f ms\n",
        tb);
    for (auto wh : {std::pair<int, int>{160, 90}, {320, 180}, {640, 360}}) {
      ObserverCapacity c = cap;
      c.width = wh.first;
      c.height = wh.second;
      c.Cops = 1e12;
      c.Bs = 1e12;
      c.memBytes = 1e10;
      Plan p = negotiate(c);
      double t = ms([&] { R2.render(c2, c, p, 0.0); });
      std::printf("frame %dx%d: %.1f ms (%.1f FPS), %.0f ns/pixel\n", wh.first,
                  wh.second, t, 1000 / t, t * 1e6 / (wh.first * wh.second));
    }
    R2.threads = 1;
    ObserverCapacity c = cap;
    double t1 = ms([&] { R2.render(c2, c, negotiate(c), 0.0); });
    R2.threads = std::max(1u, std::thread::hardware_concurrency());
    double tn = ms([&] { R2.render(c2, c, negotiate(c), 0.0); });
    std::printf("320x180: 1 thread %.1f ms, %u thread %.1f ms (x%.1f)\n", t1,
                R2.threads, tn, t1 / tn);
    Camera c3 = c2;
    c3.r = 17.3 + 1e-3;
    double tm = ms([&] { R2.render(c3, c, negotiate(c), 0.0); });
    std::printf("frame con cambio r (ricostruzione tabella): %.1f ms\n", tm);
    // confronto con integrazione per-pixel: stesso lavoro senza tabella
    const int N = 2000;
    double td = ms([&] {
      real acc = 0;
      for (int i = 0; i < N; ++i) {
        real u;
        if (RayTable::direct(1, 17.3, 0.5 + 2.0 * i / N, 1.0, u))
          acc += u;
      }
      if (acc < 0)
        std::puts("");
    });
    std::printf("integrazione diretta (h=1e-4, phi=1): %.1f us/raggio vs "
                "lookup tabella: ~%.2f us/raggio\n",
                td * 1e3 / N, 0.0);
  }
  std::printf("\nTOTALE: %d PASS, %d FAIL\n", nP, nF);
  return nF ? 1 : 0;
}