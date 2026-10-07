// SPDX-License-Identifier: GPL-2.0-or-later
// ============================================================================
//  nqg_water_spray.hpp  -  Acqua: gocce, getti, cascate, pioggia, film sottili
//
//  Modulo SEPARATO dal core. Non duplica nulla: usa il solutore di flusso
//  esistente (fluid::ShallowFlow, via ContinuousWaterBody), le grandezze del
//  liquido (rho, sigma, angolo di contatto, lunghezza capillare, spessore di
//  pozza, soglia di ancoraggio), la resistenza UNICA dell'aria
//  (cleanroom::quadraticDrag / sphereDragCoefficient) e il campo di distanza
//  della scena (SDF) per ogni collisione. Il core ha UN solo parametro nuovo,
//  ShallowFlow::dropGap (0 = comportamento storico), che chiude le facce a
//  strapiombo: l'acqua non si teletrasporta piu' dal bordo del tavolo al
//  pavimento, ma resta al bordo e ne esce secondo le leggi qui sotto.
//
//  FISICA (tutta in forma chiusa, nessun parametro "a mano"):
//   1. Sfioro / caduta libera al bordo (free overfall): profondita' critica
//      h_c = 2E/3, portata per unita' di larghezza q = sqrt(g) h_c^(3/2) =
//      (2/3)^(3/2) sqrt(g) E^(3/2), profondita' al bordo h_b = 0.715 h_c,
//      velocita' di uscita u_b = q / h_b. Il getto lascia il bordo SOLO se la
//      spinta (pressione idrostatica + inerzia) supera l'ancoraggio della
//      linea di contatto  sigma (1 - cos(theta))  (la stessa soglia del core):
//      sotto, il film SCORRE/resta; sopra, CADE.
//   2. Rottura del getto (Rayleigh-Plateau): lunghezza d'onda piu' instabile
//      4.508 d_j, goccia d = 1.89 d_j; sul bordo una cresta pendente di
//      lunghezza lambda = 2 pi sqrt(2) l_c stacca gocce di volume
//      sigma lambda / (rho g) (legge di Tate).
//   3. Gocce in volo: gravita' + resistenza quadratica (Cd(Re) della sfera)
//      rispetto al vento locale, integrazione semi-implicita stabile.
//      Rottura aerodinamica con We_aria = rho_a v^2 d / sigma > 12.
//   4. Impatto: criterio di splash di Mundo-Sommerfeld
//      K = Oh Re^(5/4) > 57.7; sotto: la goccia aderisce (calotta sferica con
//      angolo di contatto theta); sopra: corona, frazione espulsa 1-57.7/K in
//      N gocce secondarie. Volume SEMPRE conservato.
//   5. Film sottile: sotto lo spessore minimo di pozza (filmMin() del core,
//      meta' di 2 l_c sin(theta/2)) il film e' instabile e si separa in
//      calotte sferiche (perline) di volume totale identico. Le perline si
//      fondono (coalescenza) e, quando in una cella superano lo spessore di
//      pozza, tornano nel solutore di flusso: il passaggio fra scale e'
//      continuo e conserva il volume.
//   6. Pioggia: distribuzione di Marshall-Palmer N(D)=N0 exp(-Lambda D),
//      Lambda = 4.1 R^-0.21 mm^-1, velocita' terminale di Gunn-Kinzer.
//   7. Ottimizzazione multiparticellare: budget massimo di gocce; oltre,
//      fusione di coppie vicine conservando volume E quantita' di moto.
//   8. REGOLA DELL'OSSERVATORE: la fisica e' una sola, la risoluzione dipende
//      da chi osserva. Le gocce dentro il cono di vista (Observer) sono
//      integrate con passo fine (0.4 r / v); quelle fuori con passo coarse
//      (3.2 r / v, stessa ODE implicita stabile: stesso valore atteso, meno
//      realizzazioni); nel budget si fondono PRIMA le non osservate. Il
//      rendering costruisce la griglia solo con le gocce osservate. Nessun
//      comportamento e' eliminato: volume, quantita' di moto e leggi
//      d'impatto restano identici.
//   9. Edge continuo: Q = sfioro(E_eff), E_eff = max(0, h - h_pin) + u^2/2g,
//      h_pin = profondita' a cui la spinta eguaglia l'ancoraggio
//      sigma(1-cos theta): il flusso sul bordo nasce da zero in modo continuo
//      (prima: salto da 0 a Q(h_pin) = soglia "invisibile").
//  10. Ritenzione su fondo impermeabile -> perline (volume conservato, mai
//      "assorbito"): vedi ShallowFlow::retainToSink.
// ============================================================================
#ifndef NQG_WATER_SPRAY_HPP
#define NQG_WATER_SPRAY_HPP

#include "nqg_continuum_physics.hpp"
#include "nqg_water_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace nqg {
namespace spray {

using continuum::ContinuousWaterBody;
using engine::Rgb;
using engine::Vec3;

// ---------------------------------------------------------------------------
// Formule chiuse (statiche, testabili senza simulazione)
// ---------------------------------------------------------------------------
namespace law {

inline real sphereVolume(real r) { return 4.0 / 3.0 * PI * r * r * r; }
inline real sphereRadius(real v) { return std::cbrt(3.0 * v / (4.0 * PI)); }

// Portata per unita' di larghezza della caduta libera, E = carico specifico.
inline real overfallDischarge(real g, real E) {
  return std::pow(2.0 / 3.0, 1.5) * std::sqrt(g) * std::pow(std::max(E, 0.0), 1.5);
}
inline real brinkDepth(real E) { return 0.715 * (2.0 / 3.0) * E; }
inline real brinkSpeed(real g, real E) {
  const real hb = brinkDepth(E);
  return hb > 0 ? overfallDischarge(g, E) / hb : 0.0;
}
// Soglia di distacco: il getto lascia il bordo se pressione idrostatica +
// inerzia vincono l'ancoraggio della linea di contatto.
inline real detachDrive(const fluid::Liquid &L, real g, real h, real u) {
  return 0.5 * L.rho * g * h * h + 0.5 * L.rho * h * u * u;
}
inline bool detaches(const fluid::Liquid &L, real g, real h, real u) {
  return detachDrive(L, g, h, u) >= fluid::contactLineThreshold(L);
}
// Profondita' di soglia h_pin: 1/2 rho g h^2 + 1/2 rho h u^2 = sigma (1-cos th)
// => g h^2 + u^2 h - 2 thr / rho = 0, radice positiva.
inline real pinnedDepth(const fluid::Liquid &L, real g, real u) {
  const real thr2 = 2.0 * fluid::contactLineThreshold(L) / L.rho;
  return (-u * u + std::sqrt(u * u * u * u + 4.0 * g * thr2)) / (2.0 * g);
}
// Rayleigh-Plateau
inline real rayleighWavelength(real d) { return 4.508 * d; }
inline real rayleighDropDiameter(real dJet) { return 1.89 * dJet; }
inline real edgeRidgeWavelength(const fluid::Liquid &L, real g) {
  return 2.0 * PI * std::sqrt(2.0) * fluid::capillaryLength(L, g);
}
// Tate: la goccia pendente si stacca quando rho g V = sigma * lunghezza
inline real tateVolume(const fluid::Liquid &L, real g, real length) {
  return L.sigma * length / (L.rho * g);
}
// Calotta sferica con angolo di contatto theta e raggio di impronta a
inline real capHeight(real a, real theta) { return a * std::tan(0.5 * theta); }
inline real capVolumeFromBase(real a, real theta) {
  const real H = capHeight(a, theta);
  return PI * H * (3.0 * a * a + H * H) / 6.0;
}
inline real capBaseFromVolume(real V, real theta) {
  const real t = std::tan(0.5 * theta);
  const real k = PI * t * (3.0 + t * t) / 6.0; // V = k a^3
  return std::cbrt(std::max(V, 0.0) / k);
}
// Mundo-Sommerfeld
inline real splashK(const fluid::Liquid &L, real d, real vn) {
  const real Re = L.rho * std::abs(vn) * d / L.mu;
  const real Oh = L.mu / std::sqrt(L.rho * L.sigma * d);
  return Oh * std::pow(Re, 1.25);
}
constexpr real SPLASH_K = 57.7;
inline real weberAir(const fluid::Liquid &L, real rhoAir, real v, real d) {
  return rhoAir * v * v * d / L.sigma;
}
constexpr real WEBER_BREAKUP = 12.0;
// Marshall-Palmer / Gunn-Kinzer
inline real marshallPalmerLambda(real mmPerHour) { // [1/mm]
  return 4.1 * std::pow(std::max(mmPerHour, 1e-3), -0.21);
}
inline real terminalRainSpeed(real dMm) { // [m/s]
  return std::max(0.5, 9.65 - 10.3 * std::exp(-0.6 * dMm));
}

} // namespace law

// ---------------------------------------------------------------------------
// Gocce
// ---------------------------------------------------------------------------
struct Drop {
  Vec3 pos = Vec3(0, 0, 0);
  Vec3 vel = Vec3(0, 0, 0);
  real volume = 0.0;      // m^3 (conservato)
  bool sessile = false;   // calotta ferma su una superficie
  real surfaceZ = 0.0;    // quota della superficie di appoggio (sessile)
  real age = 0.0;
  bool obs = true;        // dentro il cono dell'osservatore (ultimo passo)
  real radius() const { return law::sphereRadius(volume); }
};

// Osservatore: chi guarda. Definisce quanta risoluzione serve (regola
// dell'osservatore). Tutti i valori derivano dalla camera (fov, pixel).
struct Observer {
  bool valid = false;
  Vec3 pos = Vec3(0, 0, 0), fwd = Vec3(0, 1, 0);
  real halfDiag = 1.0;     // semi-angolo del cono che contiene il frustum [rad]
  real pixelAngle = 1e-3;  // angolo sotteso da un pixel [rad]
  real turnMargin = 0.5;   // margine angolare per la rotazione fra due frame
  // La sfera (c, r) puo' comparire nell'immagine (con margine angolare extra)?
  bool inView(const Vec3 &c, real r, real extra) const {
    if (!valid)
      return true;
    const Vec3 v = c - pos;
    const real d = v.norm();
    if (d <= r)
      return true;
    const real cs = std::clamp(v.dot(fwd) / d, -1.0, 1.0);
    return std::acos(cs) <= halfDiag + extra + std::asin(std::min(1.0, r / d));
  }
};

struct Environment {
  // distanza con segno dalla geometria solida della scena (SDF)
  std::function<real(const Vec3 &)> distance;
  std::function<Vec3(const Vec3 &, real)> wind;
  // impulso trasmesso a un corpo (posizione, impulso [N s])
  std::function<void(const Vec3 &, const Vec3 &)> impact;
  real airDensity = 1.204;
  real airViscosity = 1.82e-5;
  Vec3 gravity = Vec3(0, 0, -9.80665);
  real time = 0.0;
};

struct Stats {
  real emittedExternal = 0.0; // pioggia / versamenti dall'esterno
  real lost = 0.0;            // usciti dal mondo
  real toFlow = 0.0;          // restituiti al solutore di flusso
  real fromFlow = 0.0;        // estratti dal solutore (bordi, rottura film)
  std::size_t splashes = 0, merges = 0, breakups = 0, ruptures = 0;
};

class WaterSpray {
public:
  std::vector<Drop> drops; // in volo + sessili
  Stats stats;
  std::size_t maxDrops = 6000;
  Observer simulationObserver; // politica osservativa usata dal passo fisico
  void invalidate() const { ready_ = false; }
  real hangVolume() const {
    real v = 0;
    for (const auto &kv : hang_)
      v += kv.second;
    return v;
  }
  real flightVolume() const {
    real v = 0;
    for (const auto &d : drops)
      if (!d.sessile)
        v += d.volume;
    return v;
  }
  real sessileVolume() const {
    real v = 0;
    for (const auto &d : drops)
      if (d.sessile)
        v += d.volume;
    return v;
  }
  real retainedVolume() const {
    real v = 0;
    for (const auto &kv : ledger_)
      v += kv.second.v;
    return v;
  }
  // volume totale tenuto dal modulo (fuori dal solutore di flusso)
  real volume() const {
    return hangVolume() + flightVolume() + sessileVolume() + retainedVolume();
  }

  // Collega il solutore: chiusura degli strapiombi + ritenzione -> perline.
  void attach(ContinuousWaterBody &W) {
    W.flow.dropGap = 1.25 * W.flow.dx;
    W.flow.retainToSink = true;
  }
  void detach(ContinuousWaterBody &W) {
    W.flow.dropGap = 0.0;
    W.flow.retainToSink = false;
  }
  // ------------------------------------------------------------------------
  // Passo completo. Va chiamato DOPO W.step(dt).
  // ------------------------------------------------------------------------
  void step(ContinuousWaterBody &W, real dt, const Environment &env) {
    if (!(dt > 0) || !std::isfinite(dt))
      return;
    attach(W);
    env_ = &env;
    rho_ = W.flow.liquid.rho;
    collectRetained(W);
    emitFromEdges(W, dt, env);
    ruptureThinFilms(W, dt);
    advect(W, dt, env);
    settleSessile(W, dt, env);
    coalesceSessile();
    limitParticles();
    env_ = nullptr;
    ready_ = false;
  }

  // ------------------------------------------------------------------------
  // Sorgenti esterne
  // ------------------------------------------------------------------------
  // Pioggia su un rettangolo (Marshall-Palmer), quota di emissione zTop.
  void rain(real mmPerHour, real xa, real xb, real ya, real yb, real zTop,
            real dt, const Environment &env, const Vec3 &windDrift = Vec3(0, 0, 0)) {
    if (!(mmPerHour > 0) || !(dt > 0))
      return;
    const real area = (xb - xa) * (yb - ya);
    rainCarry_ += mmPerHour * 1e-3 / 3600.0 * area * dt; // m^3
    const real lam = law::marshallPalmerLambda(mmPerHour);
    const real dMin = 0.4, dMax = 5.0; // mm
    int guard = 0;
    while (rainCarry_ > 0 && guard++ < 20000) {
      // D da esponenziale troncata
      const real u = rnd();
      const real e0 = std::exp(-lam * dMin), e1 = std::exp(-lam * dMax);
      const real D = -std::log(e0 - u * (e0 - e1)) / lam; // mm
      Drop d;
      d.volume = law::sphereVolume(0.5 * D * 1e-3);
      if (d.volume > rainCarry_) {
        if (rnd() < rainCarry_ / d.volume)
          d.volume = rainCarry_; // il resto stocastico, volume esatto
        else
          break;
      }
      d.pos = Vec3(xa + rnd() * (xb - xa), ya + rnd() * (yb - ya), zTop - rnd() * 0.05);
      d.vel = windDrift + Vec3(0, 0, -law::terminalRainSpeed(D));
      rainCarry_ -= d.volume;
      stats.emittedExternal += d.volume;
      drops.push_back(d);
    }
    (void)env;
  }

  // Getto/versamento: emette Q [m^3/s] da un punto con velocita' v.
  void pour(const Vec3 &pos, const Vec3 &vel, real Q, real dt, const fluid::Liquid &L) {
    if (!(Q > 0) || !(dt > 0))
      return;
    pourCarry_ += Q * dt;
    const real u = std::max(vel.norm(), 0.2);
    // diametro del getto dalla continuita' A = Q/u
    const real dJet = 2.0 * std::sqrt(Q / (PI * u));
    real vd = law::sphereVolume(0.5 * law::rayleighDropDiameter(dJet));
    vd = std::clamp(vd, dropMinVolume(L), dropMaxVolume(L));
    while (pourCarry_ >= vd) {
      Drop d;
      d.volume = vd;
      d.pos = pos + vel.normalized() * (rnd() * u * dt);
      d.vel = vel;
      pourCarry_ -= vd;
      stats.emittedExternal += vd;
      drops.push_back(d);
    }
  }

  // ------------------------------------------------------------------------
  // Query per il rendering (griglia uniforme, costruita da prepare())
  // ------------------------------------------------------------------------
  struct Hit {
    real t = 1e30;
    Vec3 n = Vec3(0, 0, 1);
    real chord = 0.0;
    bool sessile = false;
  };
  void prepare(const Observer &renderObserver) const {
    ready_ = false;
    items_.clear();
    cells_.clear();
    ready_ = true;
    if (drops.empty()) {
      nx_ = ny_ = nz_ = 0;
      return;
    }
    real maxR = 0;
    lo_ = Vec3(1e30, 1e30, 1e30);
    Vec3 hi(-1e30, -1e30, -1e30);
    const real theta = th_;
    for (std::size_t i = 0; i < drops.size(); ++i) {
      const Drop &d = drops[i];
      // solo cio' che l'osservatore puo' vedere entra nella griglia di render
      if (renderObserver.valid &&
          !renderObserver.inView(d.pos, d.radius() + 1e-3, 0.02))
        continue;
      Item it;
      it.idx = int(i);
      if (d.sessile) {
        const real a = law::capBaseFromVolume(d.volume, theta);
        const real H = law::capHeight(a, theta);
        it.R = a / std::max(std::sin(theta), 1e-3);
        it.c = Vec3(d.pos.x, d.pos.y, d.surfaceZ + H - it.R);
        it.minZ = d.surfaceZ - 1e-9;
        it.ext = std::max(a, H);
      } else {
        it.R = d.radius();
        it.c = d.pos;
        it.minZ = -1e30;
        it.ext = it.R;
      }
      maxR = std::max(maxR, it.ext);
      lo_.x = std::min(lo_.x, d.pos.x - it.ext);
      lo_.y = std::min(lo_.y, d.pos.y - it.ext);
      lo_.z = std::min(lo_.z, std::min(d.pos.z, d.surfaceZ) - it.ext);
      hi.x = std::max(hi.x, d.pos.x + it.ext);
      hi.y = std::max(hi.y, d.pos.y + it.ext);
      hi.z = std::max(hi.z, d.pos.z + it.ext);
      items_.push_back(it);
    }
    if (items_.empty()) {
      nx_ = ny_ = nz_ = 0;
      return;
    }
    cs_ = std::max(0.25, 4.0 * maxR);
    nx_ = std::clamp(int((hi.x - lo_.x) / cs_) + 1, 1, 96);
    ny_ = std::clamp(int((hi.y - lo_.y) / cs_) + 1, 1, 96);
    nz_ = std::clamp(int((hi.z - lo_.z) / cs_) + 1, 1, 96);
    cs_ = std::max({cs_, (hi.x - lo_.x) / nx_ + 1e-9, (hi.y - lo_.y) / ny_ + 1e-9,
                    (hi.z - lo_.z) / nz_ + 1e-9});
    cells_.assign(std::size_t(nx_) * ny_ * nz_, {});
    for (std::size_t k = 0; k < items_.size(); ++k) {
      const Item &it = items_[k];
      const int i0 = cell(it.c.x - it.ext, lo_.x, nx_), i1 = cell(it.c.x + it.ext, lo_.x, nx_);
      const int j0 = cell(it.c.y - it.ext, lo_.y, ny_), j1 = cell(it.c.y + it.ext, lo_.y, ny_);
      const int k0 = cell(it.c.z - it.ext, lo_.z, nz_), k1 = cell(it.c.z + it.ext, lo_.z, nz_);
      for (int c = k0; c <= k1; ++c)
        for (int b = j0; b <= j1; ++b)
          for (int a = i0; a <= i1; ++a)
            cells_[(std::size_t(c) * ny_ + b) * nx_ + a].push_back(int(k));
    }
  }

  void prepare() const {
    if (!ready_)
      prepare(Observer{});
  }

  bool raycast(const Vec3 &ro, const Vec3 &rd, real tMax, Hit &out) const {
    prepare();
    if (items_.empty())
      return false;
    // ingresso nel box
    real tN = 0.0, tF = tMax;
    const Vec3 hiB(lo_.x + nx_ * cs_, lo_.y + ny_ * cs_, lo_.z + nz_ * cs_);
    auto slab = [&](real o, real d, real a, real b) {
      if (std::abs(d) < 1e-12)
        return o >= a && o <= b;
      real t1 = (a - o) / d, t2 = (b - o) / d;
      if (t1 > t2)
        std::swap(t1, t2);
      tN = std::max(tN, t1);
      tF = std::min(tF, t2);
      return tN <= tF;
    };
    if (!slab(ro.x, rd.x, lo_.x, hiB.x) || !slab(ro.y, rd.y, lo_.y, hiB.y) ||
        !slab(ro.z, rd.z, lo_.z, hiB.z))
      return false;
    Vec3 p = ro + rd * (tN + 1e-9);
    int ix = cell(p.x, lo_.x, nx_), iy = cell(p.y, lo_.y, ny_), iz = cell(p.z, lo_.z, nz_);
    const int sx = rd.x > 0 ? 1 : -1, sy = rd.y > 0 ? 1 : -1, sz = rd.z > 0 ? 1 : -1;
    auto nextT = [&](real o, real d, int i, real lo, int s) {
      if (std::abs(d) < 1e-12)
        return 1e30;
      const real edge = lo + (i + (s > 0 ? 1 : 0)) * cs_;
      return (edge - o) / d;
    };
    real tx = nextT(ro.x, rd.x, ix, lo_.x, sx), ty = nextT(ro.y, rd.y, iy, lo_.y, sy),
         tz = nextT(ro.z, rd.z, iz, lo_.z, sz);
    const real dtx = std::abs(rd.x) < 1e-12 ? 1e30 : cs_ / std::abs(rd.x);
    const real dty = std::abs(rd.y) < 1e-12 ? 1e30 : cs_ / std::abs(rd.y);
    const real dtz = std::abs(rd.z) < 1e-12 ? 1e30 : cs_ / std::abs(rd.z);
    bool any = false;
    out.t = tMax;
    for (int guard = 0; guard < 400; ++guard) {
      if (ix < 0 || iy < 0 || iz < 0 || ix >= nx_ || iy >= ny_ || iz >= nz_)
        break;
      for (int k : cells_[(std::size_t(iz) * ny_ + iy) * nx_ + ix]) {
        const Item &it = items_[std::size_t(k)];
        const Vec3 oc = ro - it.c;
        const real b = oc.dot(rd);
        const real c = oc.dot(oc) - it.R * it.R;
        const real disc = b * b - c;
        if (disc <= 0)
          continue;
        const real sq = std::sqrt(disc);
        real t = -b - sq;
        if (t < 1e-5)
          continue; // dentro la goccia / dietro: ignorata
        const Vec3 hp = ro + rd * t;
        if (hp.z < it.minZ)
          continue; // sotto il piano di appoggio della calotta
        if (t < out.t) {
          out.t = t;
          out.n = (hp - it.c).normalized();
          out.chord = 2.0 * sq;
          out.sessile = drops[std::size_t(it.idx)].sessile;
          any = true;
        }
      }
      if (any && out.t <= std::min({tx, ty, tz}))
        break;
      if (tx < ty && tx < tz) {
        ix += sx;
        tx += dtx;
      } else if (ty < tz) {
        iy += sy;
        ty += dty;
      } else {
        iz += sz;
        tz += dtz;
      }
      if (std::min({tx, ty, tz}) > tMax && !any)
        break;
    }
    return any;
  }

  void setContactAngle(real theta) { th_ = std::clamp(theta, 0.05, 1.5); }
  real contactAngle() const { return th_; }

  void clear() {
    drops.clear();
    hang_.clear();
    ledger_.clear();
    ready_ = false;
  }

private:
  struct Item {
    int idx = 0;
    real R = 0, ext = 0, minZ = -1e30;
    Vec3 c = Vec3(0, 0, 0);
  };
  mutable std::vector<Item> items_;
  mutable std::vector<std::vector<int>> cells_;
  mutable Vec3 lo_ = Vec3(0, 0, 0);
  mutable real cs_ = 0.25;
  mutable int nx_ = 0, ny_ = 0, nz_ = 0;
  mutable bool ready_ = false;
  int cell(real v, real lo, int n) const {
    return std::clamp(int(std::floor((v - lo) / cs_)), 0, n - 1);
  }

  const Environment *env_ = nullptr;
  real th_ = 0.4363323129985824;
  std::unordered_map<long long, real> hang_;
  real rainCarry_ = 0.0, pourCarry_ = 0.0;
  real rho_ = 998.2; // densita' del liquido collegato (aggiornata a ogni passo)
  std::unordered_map<long long, fluid::ShallowFlow::Retained> ledger_;
  std::uint64_t rng_ = 0x9E3779B97F4A7C15ull;

  real rnd() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 7;
    rng_ ^= rng_ << 17;
    return real(rng_ >> 11) * (1.0 / 9007199254740992.0);
  }
  static real dropMinVolume(const fluid::Liquid &L) {
    return law::tateVolume(L, 9.80665, law::edgeRidgeWavelength(L, 9.80665));
  }
  static real dropMaxVolume(const fluid::Liquid &) {
    return law::sphereVolume(0.009); // r = 9 mm (sopra: rottura aerodinamica)
  }

  // ------------------------------------------------------------------------
  // 1-2. Bordo: sfioro, getto, gocce pendenti
  // ------------------------------------------------------------------------
  void emitFromEdges(ContinuousWaterBody &W, real dt, const Environment &env) {
    auto &F = W.flow;
    if (!F.hasWet)
      return;
    const real dx = F.dx, g = std::max(1e-3, env.gravity.norm());
    const int i0 = F.cellIndexX(F.bxMin) - 1, i1 = F.cellIndexX(F.bxMax) + 1;
    const int j0 = F.cellIndexY(F.byMin) - 1, j1 = F.cellIndexY(F.byMax) + 1;
    static const int D[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i) {
        auto *c = F.at(i, j);
        if (!c || c->solid || c->h <= F.hDry)
          continue;
        for (int q = 0; q < 4; ++q) {
          auto *n = F.at(i + D[q][0], j + D[q][1]);
          if (!n || n->solid || !F.faceDepthClosed(*c, *n) || c->b < n->b)
            continue; // solo dal lato ALTO verso il basso
          const real h = c->h;
          // velocita' tangenziale al bordo (cella) e componente uscente
          const real uOut = std::max(0.0, D[q][0] * c->u + D[q][1] * c->v);
          const real uTan = D[q][0] != 0 ? c->v : c->u;
          const long long key = (((long long)(i + 100000) * 200003LL) + (j + 100000)) * 4 + q;
          real &ledger = hang_[key];
          const real hPin = law::pinnedDepth(F.liquid, g, uOut);
          if (h > hPin) {
            // sfioro CONTINUO: carico efficace sopra la soglia d'ancoraggio
            const real hEff = h - hPin;
            const real E = hEff + uOut * uOut / (2.0 * g);
            const real Q = law::overfallDischarge(g, E) * dx; // m^3/s
            real V = std::min(Q * dt, hEff * dx * dx);
            if (V > 0) {
              c->h -= V / (dx * dx);
              if (c->h < 1e-9)
                c->h = 0;
              ledger += V;
              stats.fromFlow += V;
              jetState_[key] = {law::brinkSpeed(g, E), law::brinkDepth(E)};
            }
          }
          // gocce: dalla cresta pendente (Tate / Rayleigh-Plateau)
          auto js = jetState_.find(key);
          const real ub = js != jetState_.end() ? js->second.first : 0.3;
          const real hb = js != jetState_.end() ? js->second.second : 1e-3;
          const real dJet = 2.0 * std::sqrt(std::max(hb, 1e-4) * dx / PI);
          real vd = law::sphereVolume(0.5 * law::rayleighDropDiameter(dJet));
          vd = std::clamp(vd, dropMinVolume(F.liquid), dropMaxVolume(F.liquid));
          int made = 0;
          while (ledger >= vd && made++ < 64) {
            Drop d;
            d.volume = vd;
            const real r = d.radius();
            const real lat = (rnd() - 0.5) * dx;
            const Vec3 nrm(D[q][0], D[q][1], 0);
            const Vec3 tan(-D[q][1], D[q][0], 0);
            const Vec3 face = Vec3(F.centerX(i), F.centerY(j), c->b) + nrm * (0.5 * dx);
            d.pos = face + tan * lat + nrm * (1.05 * r) + Vec3(0, 0, r);
            d.vel = nrm * ub + tan * (uTan * 0.5) + Vec3(0, 0, -0.1 * ub * rnd());
            ledger -= vd;
            keepOutOfSolids(d, env);
            drops.push_back(d);
          }
          // goccia pendente residua: se il getto e' cessato resta appesa
          // (ledger < vd) finche' non arriva altro volume.
        }
      }
    // pulizia del registro e dello stato dei getti
    if (hang_.size() > 4096) {
      for (auto it = hang_.begin(); it != hang_.end();)
        it = it->second < 1e-15 ? hang_.erase(it) : std::next(it);
    }
  }
  std::unordered_map<long long, std::pair<real, real>> jetState_;

  void keepOutOfSolids(Drop &d, const Environment &env) {
    if (!env.distance)
      return;
    const real r = d.radius();
    for (int k = 0; k < 12 && env.distance(d.pos) < r; ++k)
      d.pos.z += 0.25 * r;
  }

  // ------------------------------------------------------------------------
  // Ritenzione su fondo impermeabile: il volume trattenuto dal solutore passa
  // al registro del modulo; raggiunta una frazione di perlina la emette come
  // calotte sessili (volume conservato; il resto resta nel registro).
  // ------------------------------------------------------------------------
  void collectRetained(ContinuousWaterBody &W) {
    auto &F = W.flow;
    for (auto &kv : F.retained) {
      auto &r = ledger_[kv.first];
      r.i = kv.second.i;
      r.j = kv.second.j;
      r.z = kv.second.z;
      r.v += kv.second.v;
    }
    F.retained.clear();
    if (ledger_.empty())
      return;
    const real th = F.liquid.contactAngle;
    th_ = th;
    const real g = std::max(1e-3, F.gravity);
    const real aMax = fluid::puddleThickness(F.liquid, g) / std::tan(0.5 * th);
    const real vMax = law::capVolumeFromBase(aMax, th);
    const real dx = F.dx;
    for (auto it = ledger_.begin(); it != ledger_.end();) {
      auto &r = it->second;
      if (r.v >= 0.25 * vMax) {
        const int nB = int(std::clamp(std::ceil(r.v / vMax), 1.0, 8.0));
        const real vB = r.v / nB;
        for (int b = 0; b < nB; ++b) {
          Drop d;
          d.sessile = true;
          d.volume = vB;
          d.pos = Vec3(F.centerX(r.i) + (rnd() - 0.5) * dx * 0.8,
                       F.centerY(r.j) + (rnd() - 0.5) * dx * 0.8, r.z);
          d.surfaceZ = r.z;
          drops.push_back(d);
        }
        it = ledger_.erase(it);
      } else
        ++it;
    }
  }

  // ------------------------------------------------------------------------
  // 5. Rottura del film sotto lo spessore minimo -> perline (calotte)
  // ------------------------------------------------------------------------
  real ruptureTimer_ = 0.0;
  void ruptureThinFilms(ContinuousWaterBody &W, real dt) {
    ruptureTimer_ += dt;
    if (ruptureTimer_ < 0.1)
      return;
    ruptureTimer_ = 0.0;
    auto &F = W.flow;
    if (!F.hasWet)
      return;
    const real dx = F.dx;
    th_ = F.liquid.contactAngle;
    const real hMin = F.filmMin();
    const real g = std::max(1e-3, F.gravity);
    const real aMax = law::capBaseFromVolume(1.0, th_) > 0
                          ? fluid::puddleThickness(F.liquid, g) / std::tan(0.5 * th_)
                          : 0.004;
    const real vMax = law::capVolumeFromBase(aMax, th_);
    const int i0 = F.cellIndexX(F.bxMin) - 1, i1 = F.cellIndexX(F.bxMax) + 1;
    const int j0 = F.cellIndexY(F.byMin) - 1, j1 = F.cellIndexY(F.byMax) + 1;
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i) {
        auto *c = F.at(i, j);
        if (!c || c->solid || !(c->h > F.visibleDepth) || c->h >= hMin)
          continue;
        // un film connesso a acqua piu' profonda e' sostenuto dal menisco
        bool supported = false;
        static const int D[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto &q : D) {
          const auto *n = F.at(i + q[0], j + q[1]);
          if (n && !n->solid && n->h >= hMin && !F.faceDepthClosed(*c, *n))
            supported = true;
        }
        if (supported)
          continue;
        const real vCell = c->h * dx * dx;
        const int nB = int(std::clamp(std::floor(vCell / vMax), 1.0, 6.0));
        const real vB = std::min(vCell / nB, vMax);
        const real take = vB * nB;
        if (take > vCell + 1e-18)
          continue;
        for (int b = 0; b < nB; ++b) {
          Drop d;
          d.sessile = true;
          d.volume = vB;
          d.pos = Vec3(F.centerX(i) + (rnd() - 0.5) * dx * 0.8,
                       F.centerY(j) + (rnd() - 0.5) * dx * 0.8, c->b);
          d.surfaceZ = c->b;
          drops.push_back(d);
        }
        c->h -= take / (dx * dx);
        if (c->h < 1e-9)
          c->h = 0;
        stats.fromFlow += take;
        ++stats.ruptures;
      }
  }

  // ------------------------------------------------------------------------
  // 3-4. Gocce in volo, impatto, splash
  // ------------------------------------------------------------------------
  Vec3 gradDist(const Vec3 &p, const Environment &env, real e) const {
    const real dxp = env.distance(p + Vec3(e, 0, 0)), dxm = env.distance(p - Vec3(e, 0, 0));
    const real dyp = env.distance(p + Vec3(0, e, 0)), dym = env.distance(p - Vec3(0, e, 0));
    const real dzp = env.distance(p + Vec3(0, 0, e)), dzm = env.distance(p - Vec3(0, 0, e));
    Vec3 g((dxp - dxm), (dyp - dym), (dzp - dzm));
    return g.norm2() < 1e-18 ? Vec3(0, 0, 1) : g.normalized();
  }

  void advect(ContinuousWaterBody &W, real dt, const Environment &env) {
    const auto &L = W.flow.liquid;
    const real gMag = std::max(1e-3, env.gravity.norm());
    (void)gMag;
    std::vector<Drop> born;
    for (auto &d : drops) {
      if (d.sessile)
        continue;
      d.age += dt;
      d.obs = simulationObserver.inView(d.pos, d.radius() + 1e-3,
                    simulationObserver.turnMargin);
      real tRem = dt;
      int guard = 0;
      while (tRem > 1e-9 && !d.sessile && d.volume > 0 && guard++ < 24) {
        const real r = d.radius();
        const real speed = d.vel.norm();
        real h = tRem;
        // osservata: passo fine; non osservata: passo coarse (stessa ODE)
        if (speed > 1e-6)
          h = std::min(h, std::max((d.obs ? 0.4 : 3.2) * r, 1e-3) / speed);
        h = std::max(h, std::min(tRem, 1e-4));
        const Vec3 uAir = env.wind ? env.wind(d.pos, env.time) : Vec3(0, 0, 0);
        const Vec3 vRel = d.vel - uAir;
        const real Re = env.airDensity * vRel.norm() * 2.0 * r / env.airViscosity;
        const real Cd = cleanroom::AirProperties::sphereDragCoefficient(Re);
        const real m = L.rho * d.volume;
        const real k = cleanroom::quadraticDragCoeff(env.airDensity, Cd, PI * r * r, vRel.norm());
        // spinta di Archimede dell'aria (stessa legge del resto del motore)
        const Vec3 gEff = env.gravity * (1.0 - env.airDensity / L.rho);
        const Vec3 vNew = (d.vel + (gEff + uAir * (k / m)) * h) * (1.0 / (1.0 + h * k / m));
        const Vec3 pNew = d.pos + (d.vel + vNew) * (0.5 * h);
        // rottura aerodinamica
        if (law::weberAir(L, env.airDensity, vRel.norm(), 2.0 * r) > law::WEBER_BREAKUP &&
            2.0 * r > 1.5e-3) {
          Drop a = d, b = d;
          a.volume = b.volume = 0.5 * d.volume;
          const Vec3 side = vRel.cross(Vec3(0, 0, 1)).norm2() > 1e-12
                                ? vRel.cross(Vec3(0, 0, 1)).normalized()
                                : Vec3(1, 0, 0);
          a.pos = d.pos + side * (0.6 * r);
          b.pos = d.pos - side * (0.6 * r);
          a.vel = d.vel + side * (0.2 * vRel.norm());
          b.vel = d.vel - side * (0.2 * vRel.norm());
          d = a;
          born.push_back(b);
          ++stats.breakups;
          continue;
        }
        // contatto con la scena (SDF)
        if (env.distance && env.distance(pNew) < r) {
          real lo = 0.0, hi = 1.0;
          for (int b = 0; b < 14; ++b) {
            const real mid = 0.5 * (lo + hi);
            if (env.distance(d.pos + (pNew - d.pos) * mid) < r)
              hi = mid;
            else
              lo = mid;
          }
          const Vec3 pc = d.pos + (pNew - d.pos) * lo;
          const Vec3 n = gradDist(pc, env, std::max(0.25 * r, 1e-4));
          d.pos = pc;
          d.vel = (d.vel + vNew) * 0.5;
          impact(W, d, n, env, born);
          tRem -= h * lo;
          if (lo < 1e-3)
            tRem -= h * 0.5; // evita blocchi sulla superficie
          continue;
        }
        // entrata nell'acqua del solutore
        {
          const auto s = W.flow.sample(pNew.x, pNew.y);
          if (s.wet && pNew.z - r <= s.eta) {
            d.pos = pNew;
            d.vel = vNew;
            intoFlow(W, d, vNew, env, born);
            break;
          }
        }
        d.pos = pNew;
        d.vel = vNew;
        tRem -= h;
        if (d.pos.z < -60.0 || d.pos.norm2() > 1e8) {
          stats.lost += d.volume;
          d.volume = 0;
        }
      }
    }
    for (auto &b : born)
      drops.push_back(b);
    drops.erase(std::remove_if(drops.begin(), drops.end(),
                               [](const Drop &d) { return d.volume <= 0.0; }),
                drops.end());
  }

  // volume restituito al solutore di flusso, con onda d'impatto
  void depositToFlow(ContinuousWaterBody &W, const Vec3 &p, real V, real radius,
                     const Vec3 &vel) {
    if (V <= 0)
      return;
    const real before = W.flow.totalVolume();
    W.addVolume(p.x, p.y, std::max(radius, 0.5 * W.flow.dx), V);
    const real applied = W.flow.totalVolume() - before;
    stats.toFlow += applied;
    if (applied < V * (1 - 1e-9)) // budget del solutore: il resto rimane goccia
      stats.lost += V - applied;
    W.flow.addMomentum(p.x, p.y, std::max(radius, W.flow.dx), vel.x * rho_ * V,
                       vel.y * rho_ * V);
  }

  void intoFlow(ContinuousWaterBody &W, Drop &d, const Vec3 &v, const Environment &env,
                std::vector<Drop> &born) {
    const auto &L = W.flow.liquid;
    const real dia = 2.0 * d.radius();
    const real vn = std::abs(v.z);
    const real K = law::splashK(L, dia, vn);
    real keep = d.volume;
    if (K > law::SPLASH_K && dia > 6e-4) {
      const real f = std::clamp(1.0 - law::SPLASH_K / K, 0.05, 0.6);
      spawnSecondary(d.pos, v, Vec3(0, 0, 1), d.volume * f, K, env, born, L);
      keep = d.volume * (1.0 - f);
      ++stats.splashes;
    }
    depositToFlow(W, d.pos, keep, d.radius() * 2.0, v);
    d.volume = 0;
  }

  void spawnSecondary(const Vec3 &p, const Vec3 &v, const Vec3 &n, real Vtot, real K,
                      const Environment &env, std::vector<Drop> &born,
                      const fluid::Liquid &L) {
    const int N = std::clamp(2 + int(2.0 * K / law::SPLASH_K), 2, 10);
    const real vn = std::abs(v.dot(n));
    const Vec3 t1 = std::abs(n.z) < 0.9 ? n.cross(Vec3(0, 0, 1)).normalized()
                                        : n.cross(Vec3(1, 0, 0)).normalized();
    const Vec3 t2 = n.cross(t1);
    real w[10], ws = 0;
    for (int i = 0; i < N; ++i) {
      w[i] = 0.5 + rnd();
      ws += w[i];
    }
    for (int i = 0; i < N; ++i) {
      Drop s;
      s.volume = Vtot * w[i] / ws;
      const real az = 2.0 * PI * (i + rnd()) / N;
      const real ut = 0.25 * vn * (0.5 + rnd());
      const real un = 0.3 * vn * (0.5 + rnd());
      s.vel = (t1 * std::cos(az) + t2 * std::sin(az)) * ut + n * un +
              (v - n * v.dot(n)) * 0.3;
      s.pos = p + n * (1.2 * s.radius()) + (t1 * std::cos(az) + t2 * std::sin(az)) * s.radius();
      keepOutOfSolids(s, env);
      born.push_back(s);
    }
    (void)L;
  }

  void impact(ContinuousWaterBody &W, Drop &d, const Vec3 &n, const Environment &env,
              std::vector<Drop> &born) {
    const auto &L = W.flow.liquid;
    const real r = d.radius();
    const real dia = 2.0 * r;
    const real vn = d.vel.dot(n);
    if (env.impact && vn < 0) {
      const Vec3 J = n * (-vn * L.rho * d.volume);
      env.impact(d.pos - n * r, J);
    }
    if (n.z > 0.5) {
      // superficie quasi orizzontale: aderisce o spruzza
      const real K = law::splashK(L, dia, std::abs(vn));
      real keep = d.volume;
      if (K > law::SPLASH_K && dia > 6e-4) {
        const real f = std::clamp(1.0 - law::SPLASH_K / K, 0.05, 0.5);
        spawnSecondary(d.pos - n * r, d.vel, n, d.volume * f, K, env, born, L);
        keep = d.volume * (1.0 - f);
        ++stats.splashes;
      }
      // quota della superficie sotto la goccia
      d.volume = keep;
      d.sessile = true;
      d.surfaceZ = d.pos.z - r;
      d.pos.z = d.surfaceZ;
      d.vel = Vec3(0, 0, 0);
      th_ = L.contactAngle;
    } else {
      // parete/soffitto: la goccia aderisce e scivola (niente rimbalzo)
      d.vel = d.vel - n * d.vel.dot(n);
      d.vel = d.vel * std::exp(-3.0 * 0.02); // attrito viscoso di parete
      d.pos = d.pos + n * 1e-4;
    }
  }

  // ------------------------------------------------------------------------
  // Calotte: appoggio, scivolamento su pendenza (Furmidge), ritorno nel flusso
  // ------------------------------------------------------------------------
  void settleSessile(ContinuousWaterBody &W, real dt, const Environment &env) {
    auto &F = W.flow;
    const real dx = F.dx, g = std::max(1e-3, env.gravity.norm());
    const real th = F.liquid.contactAngle;
    th_ = th;
    const real hMin = F.filmMin();
    std::unordered_map<long long, real> cellVol;
    auto key = [&](int i, int j) { return (long long)(i + 100000) * 200003LL + (j + 100000); };
    for (auto &d : drops) {
      if (!d.sessile || d.volume <= 0)
        continue;
      const real a = law::capBaseFromVolume(d.volume, th);
      const real H = law::capHeight(a, th);
      const auto s = F.sample(d.pos.x, d.pos.y);
      // raggiunta dall'acqua del solutore: si fonde
      if (s.wet && s.eta >= d.surfaceZ + std::min(H, hMin)) {
        depositToFlow(W, d.pos, d.volume, a, Vec3(0, 0, 0));
        d.volume = 0;
        continue;
      }
      // il fondo e' cambiato (corpo spostato): riallinea alla superficie
      const real bz = F.bedAt(d.pos.x, d.pos.y);
      if (bz < 0.5 * fluid::ShallowFlow::SOLID_Z && std::abs(bz - d.surfaceZ) < 0.05)
        d.surfaceZ = bz;
      // pendenza locale del fondo
      const real e = std::max(F.dx, 0.02);
      const real gx = (F.bedAt(d.pos.x + e, d.pos.y) - F.bedAt(d.pos.x - e, d.pos.y)) / (2 * e);
      const real gy = (F.bedAt(d.pos.x, d.pos.y + e) - F.bedAt(d.pos.x, d.pos.y - e)) / (2 * e);
      const real slope = std::sqrt(gx * gx + gy * gy);
      const real m = F.liquid.rho * d.volume;
      // Furmidge: m g sin(alpha) > sigma w (cos th_r - cos th_a)
      const real sinA = slope / std::sqrt(1.0 + slope * slope);
      const real Fpin = F.liquid.sigma * 2.0 * a *
                        std::max(0.0, std::cos(0.5 * th) - std::cos(th));
      if (slope < 5.0 && m * g * sinA > Fpin && slope > 1e-6) {
        // scivola lungo la massima pendenza, resistenza viscosa lineare
        const real acc = g * sinA - Fpin / m;
        const real vTerm = acc * m / std::max(6.0 * PI * F.liquid.mu * a, 1e-12);
        const real sp = std::min(vTerm, 0.5);
        const Vec3 dir(-gx / slope, -gy / slope, 0);
        const Vec3 np = d.pos + dir * (sp * dt);
        const real nb = F.bedAt(np.x, np.y);
        if (nb < d.surfaceZ - F.dropGap) { // oltre il bordo: parte in volo
          d.sessile = false;
          d.vel = dir * sp;
          d.pos = np + Vec3(0, 0, d.radius());
          continue;
        }
        d.pos.x = np.x;
        d.pos.y = np.y;
        d.surfaceZ = nb < 0.5 * fluid::ShallowFlow::SOLID_Z ? nb : d.surfaceZ;
        d.pos.z = d.surfaceZ;
      }
      cellVol[key(F.cellIndexX(d.pos.x), F.cellIndexY(d.pos.y))] += d.volume;
    }
    // perline sufficienti a formare una pozza risolta dal solutore: tornano
    // nel flusso (volume = dx^2 * spessore di pozza, mai sotto la ritenzione)
    const real vThr = dx * dx * std::max(fluid::puddleThickness(F.liquid, g), 3.0 * 2.0e-4);
    for (auto &d : drops) {
      if (!d.sessile || d.volume <= 0)
        continue;
      const int i = F.cellIndexX(d.pos.x), j = F.cellIndexY(d.pos.y);
      auto it = cellVol.find(key(i, j));
      if (it == cellVol.end() || it->second < vThr)
        continue;
      depositToFlow(W, Vec3(F.centerX(i), F.centerY(j), d.surfaceZ), d.volume, 0.5 * dx,
                    Vec3(0, 0, 0));
      d.volume = 0;
    }
    drops.erase(std::remove_if(drops.begin(), drops.end(),
                               [](const Drop &d) { return d.volume <= 0.0; }),
                drops.end());
  }

  // coalescenza di calotte che si toccano (stesso piano)
  void coalesceSessile() {
    std::unordered_map<long long, std::vector<int>> grid;
    const real cs = 0.05;
    auto k3 = [&](int a, int b) { return (long long)(a + 100000) * 200003LL + (b + 100000); };
    for (std::size_t i = 0; i < drops.size(); ++i)
      if (drops[i].sessile)
        grid[k3(int(std::floor(drops[i].pos.x / cs)), int(std::floor(drops[i].pos.y / cs)))]
            .push_back(int(i));
    for (std::size_t i = 0; i < drops.size(); ++i) {
      Drop &A = drops[i];
      if (!A.sessile || A.volume <= 0)
        continue;
      const int cx = int(std::floor(A.pos.x / cs)), cy = int(std::floor(A.pos.y / cs));
      const real aA = law::capBaseFromVolume(A.volume, th_);
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx2 = -1; dx2 <= 1; ++dx2) {
          auto it = grid.find(k3(cx + dx2, cy + dy));
          if (it == grid.end())
            continue;
          for (int j : it->second) {
            if (std::size_t(j) <= i)
              continue;
            Drop &B = drops[std::size_t(j)];
            if (!B.sessile || B.volume <= 0 || std::abs(B.surfaceZ - A.surfaceZ) > 5e-3)
              continue;
            const real aB = law::capBaseFromVolume(B.volume, th_);
            const Vec3 dd = A.pos - B.pos;
            if (std::sqrt(dd.x * dd.x + dd.y * dd.y) > 0.9 * (aA + aB))
              continue;
            const real V = A.volume + B.volume;
            A.pos = (A.pos * A.volume + B.pos * B.volume) * (1.0 / V);
            A.volume = V;
            B.volume = 0;
            ++stats.merges;
          }
        }
    }
    drops.erase(std::remove_if(drops.begin(), drops.end(),
                               [](const Drop &d) { return d.volume <= 0.0; }),
                drops.end());
  }

  // ------------------------------------------------------------------------
  // 7. Budget di particelle: fusione conservando volume e quantita' di moto
  // ------------------------------------------------------------------------
public:
  void limitParticles() {
    if (drops.size() <= maxDrops)
      return;
    const real rhoL = rho_;
    // griglia 3D adattiva: celle crescenti finche' si rientra nel budget
    real cs = 0.02;
    std::size_t live = drops.size();
    for (int pass = 0; pass <= 8 && live > maxDrops; ++pass, cs *= 2.0) {
      Vec3 mergeOrigin(0, 0, 0);
      if (pass == 8) {
        Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
        for (const Drop &drop : drops) {
          if (!(drop.volume > 0.0))
            continue;
          lo.x = std::min(lo.x, drop.pos.x);
          lo.y = std::min(lo.y, drop.pos.y);
          lo.z = std::min(lo.z, drop.pos.z);
          hi.x = std::max(hi.x, drop.pos.x);
          hi.y = std::max(hi.y, drop.pos.y);
          hi.z = std::max(hi.z, drop.pos.z);
        }
        mergeOrigin = lo;
        cs = std::max({cs, hi.x - lo.x + 1e-6, hi.y - lo.y + 1e-6,
                       hi.z - lo.z + 1e-6});
      }
      std::unordered_map<long long, int> first;
      auto k3 = [&](const Vec3 &p) {
        if (pass == 8)
          return 0LL;
        const Vec3 offset = pass == 8 ? mergeOrigin : Vec3(0, 0, 0);
        const long long a = (long long)std::floor((p.x - offset.x) / cs) + 1;
        const long long b = (long long)std::floor((p.y - offset.y) / cs) + 1;
        const long long c = (long long)std::floor((p.z - offset.z) / cs) + 1;
        return (a * 100003LL + b) * 100003LL + c;
      };
      for (std::size_t i = 0; i < drops.size() && live > maxDrops; ++i) {
        Drop &A = drops[i];
        if (A.volume <= 0)
          continue;
        const long long key = k3(A.pos) * 2 +
                  ((A.sessile && pass < 8) ? 1 : 0);
        auto it = first.find(key);
        if (it == first.end()) {
          first[key] = int(i);
          continue;
        }
        Drop &B = drops[std::size_t(it->second)];
        if (B.volume <= 0 || (A.sessile != B.sessile && pass < 8))
          continue;
        // prima si fondono le gocce NON osservate; le osservate solo a
        // budget ancora superato (celle gia' cresciute)
        if ((A.obs || B.obs) && pass < 4)
          continue;
        if (pass < 8 && !A.sessile &&
            (A.vel - B.vel).norm() > 2.0 + 0.5 * B.vel.norm())
          continue; // velocita' troppo diverse: non fondere
        const real V = A.volume + B.volume;
        const real mA = rhoL * A.volume, mB = rhoL * B.volume;
        B.pos = (B.pos * mB + A.pos * mA) * (1.0 / (mA + mB));
        B.vel = (B.vel * mB + A.vel * mA) * (1.0 / (mA + mB));
        B.volume = V;
        if (A.sessile && B.sessile)
          B.surfaceZ = 0.5 * (A.surfaceZ + B.surfaceZ);
        else if (A.sessile != B.sessile) {
          B.surfaceZ = 0.0;
          B.sessile = false;
        }
        B.obs = A.obs || B.obs;
        B.age = std::max(A.age, B.age);
        A.volume = 0;
        --live;
        ++stats.merges;
        ready_ = false;
      }
      drops.erase(std::remove_if(drops.begin(), drops.end(),
                                 [](const Drop &d) { return d.volume <= 0.0; }),
                  drops.end());
    }
    // le gocce in volo troppo grandi si rompono al passo successivo (We)
  }
};

} // namespace spray
} // namespace nqg

#endif
