#include "physics/nasa_rules.hpp"
// SPDX-License-Identifier: GPL-2.0-or-later
#ifndef NQG_WATER_SOLVER_HPP
#define NQG_WATER_SOLVER_HPP

#include "nqg_physics_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace nqg {
namespace fluid {

// ShallowFlow remains deterministic and conservative; observer estimates are
// read-only Monte Carlo queries over its reconstructed surface.

// Scala di scabrezza del fondo [m]: profondita' delle depressioni in cui il
// film e' trattenuto. UNA sola costante: ritenzione, soglia minima dello
// spessore nell'ancoraggio e distanza di asperita' del contatto corpo-fondo.
constexpr real ROUGHNESS = 2.0e-4;

struct Liquid {
  real rho = 998.2;
  real mu = 1.002e-3;
  real sigma = 0.0728;
  real contactAngle = 0.4363323129985824;
  real nu() const { return mu / rho; }
};

inline Liquid waterAtKelvin(real T) {
  T = std::clamp(T, 273.16, 373.0);
  const real t = T - 273.15;
  Liquid L;
  const real num = 999.83952 + 16.945176 * t - 7.9870401e-3 * t * t -
                   46.170461e-6 * t * t * t + 105.56302e-9 * std::pow(t, 4.0) -
                   280.54253e-12 * std::pow(t, 5.0);
  L.rho = num / (1.0 + 16.897850e-3 * t);
  L.mu = 2.414e-5 * std::pow(10.0, 247.8 / (T - 140.0));
  const real tau = 1.0 - T / 647.096;
  L.sigma = 0.2358 * std::pow(tau, 1.256) * (1.0 - 0.625 * tau);
  return L;
}

inline real capillaryLength(const Liquid &L, real g) {
  return std::sqrt(L.sigma / (L.rho * g));
}
inline real laplacePressure(real sigma, real r1, real r2) {
  return sigma * (1.0 / r1 + 1.0 / r2);
}
inline real capillaryGravityOmega(const Liquid &L, real g, real k, real depth) {
  return std::sqrt((g * k + L.sigma * k * k * k / L.rho) *
                   std::tanh(k * depth));
}
inline real bondNumber(const Liquid &L, real g, real len) {
  return L.rho * g * len * len / L.sigma;
}
inline real weberNumber(const Liquid &L, real v, real len) {
  return L.rho * v * v * len / L.sigma;
}
inline real puddleThickness(const Liquid &L, real g) {
  return 2.0 * capillaryLength(L, g) * std::sin(0.5 * L.contactAngle);
}
inline real contactLineThreshold(const Liquid &L) {
  return L.sigma * (1.0 - std::cos(L.contactAngle));
}
inline real meniscusRise(const Liquid &L, real g, real wallAngle) {
  return std::sqrt(2.0) * capillaryLength(L, g) *
         std::sqrt(std::max(0.0, 1.0 - std::sin(wallAngle)));
}
inline real capillaryVerticalForce(const Liquid &L, real perimeter,
                                   real bodyAngle) {
  return -L.sigma * perimeter * std::cos(bodyAngle);
}

struct BedSample {
  real z = 0;
  real manning = 0.012;
  bool solid = false;
  // Interazione col fondo (acqua che si ferma): infiltrazione (m/s, terreno
  // permeabile) e ritenzione (m, accumulo nelle depressioni di rugosita':
  // un film piu' sottile non scorre piu' ma resta intrappolato/assorbito).
  real infil = 0.0;
  real retention = ROUGHNESS;
  // Il fondo e' occupato da un corpo solido (statico o mobile, stessa regola):
  // l'acqua che occupava la colonna viene SPOSTATA nelle celle vicine, non
  // assorbita (conservazione del volume).
  bool body = false;
};

class ShallowFlow {
public:
  static constexpr int T = 16;
  static constexpr int SH = 4;
  static constexpr real SOLID_Z = 1.0e3;

  struct Cell {
    real h = 0, b = 0, u = 0, v = 0, n = 0.012;
    real tu = 0, tv = 0, fx = 0, fy = 0, s = 1, k = 0;
    real inf = 0.0, ret = ROUGHNESS;
    bool solid = false;
    bool body = false;
  };
  struct Tile {
    std::array<Cell, T * T> c;
    int ti = 0, tj = 0, mark = 0;
    bool wet = false, run = false, loaded = false;
  };
  struct Sample {
    bool wet = false;
    real eta = 0, depth = 0, bed = 0, u = 0, v = 0;
  };
  using BedFn = std::function<BedSample(real, real)>;

  real dx;
  int TX, TY;
  real x0, y0;
  Liquid liquid;
  BedFn bed;
  real gravity = 9.80665;
  real airDensity = 1.2;
  real windX = 0, windY = 0;
  real cfl = 0.45;
  real hDry = 1e-5;
  real hCap = 1e-3;
  real visibleDepth = 1e-4;
  real smagorinsky = 0.16;
  real speedLimit = 0.0; // 0 = nessun limite numerico alla velocita' fisica
  real simulatedTime = 0.0;
  real lastIntegratedDt = 0.0;
  std::size_t lastSubsteps = 0;
  real absorbRate = 4.0e-4;
  real dropGap = 0.0; // 0 = disattivato (comportamento storico)
  // Ritenzione (film sotto c.ret) su fondo IMPERMEABILE: se retainToSink e'
  // attivo (lo imposta nqg_water_spray.hpp) il volume NON viene dichiarato
  // "assorbito" ma consegnato in `retained` (cella, volume, quota fondo) a chi
  // lo trasforma in perline: nessun volume sparisce dal conto. Su terreno
  // permeabile (inf > 0) resta assorbimento reale. false = comportamento storico.
  bool retainToSink = false;
  struct Retained {
    int i = 0, j = 0;
    real v = 0, z = 0;
  };
  std::unordered_map<long long, Retained> retained;
  real volumeBudget = 60.0;
  std::size_t maxTiles = 4096;
  real absorbedVolume = 0.0;
  // Ripartizione di absorbedVolume per canale (diagnostica di bilancio):
  // absorbedVolume == somma dei cinque canali.
  struct AbsorbBreakdown {
    real infiltration = 0.0; // Darcy su fondo permeabile
    real retention = 0.0;    // film sotto c.ret (fondo senza sink)
    real compaction = 0.0;   // tile asciutti scartati da compact()
    real bedRise = 0.0;      // fondo/sabbia che sale sotto l'acqua
    real noNeighbour = 0.0;  // volume spostato senza celle bagnate vicine
  } absorbedBy;
  real lastMaxWave = 1.0, lastMaxDepth = 0.0;
  bool hasWet = false;
  real bxMin = 0, bxMax = 0, byMin = 0, byMax = 0, bzMin = 0, bzMax = 0;

  explicit ShallowFlow(real cell = 0.1, int tilesX = 128, int tilesY = 128,
                       real cx = 0, real cy = 0)
      : dx(cell), TX(tilesX), TY(tilesY) {
    x0 = cx - 0.5 * TX * T * dx;
    y0 = cy - 0.5 * TY * T * dx;
    dir.assign(std::size_t(TX) * TY, nullptr);
  }
  ShallowFlow(const ShallowFlow &o) { copyFrom(o); }
  ShallowFlow &operator=(const ShallowFlow &o) {
    if (this != &o)
      copyFrom(o);
    return *this;
  }

  Tile *tileAt(int ti, int tj) const {
    if (ti < 0 || tj < 0 || ti >= TX || tj >= TY)
      return nullptr;
    return dir[std::size_t(tj) * TX + ti];
  }
  Cell *at(int i, int j) {
    if (i < 0 || j < 0)
      return nullptr;
    Tile *t = tileAt(i >> SH, j >> SH);
    return t ? &t->c[std::size_t(j & (T - 1)) * T + (i & (T - 1))] : nullptr;
  }
  const Cell *at(int i, int j) const {
    if (i < 0 || j < 0)
      return nullptr;
    Tile *t = tileAt(i >> SH, j >> SH);
    return t ? &t->c[std::size_t(j & (T - 1)) * T + (i & (T - 1))] : nullptr;
  }
  int cellIndexX(real x) const { return int(std::floor((x - x0) / dx)); }
  int cellIndexY(real y) const { return int(std::floor((y - y0) / dx)); }
  real centerX(int i) const { return x0 + (i + 0.5) * dx; }
  real centerY(int j) const { return y0 + (j + 0.5) * dx; }

  BedSample bedSampleAt(real x, real y) const {
    return bed ? bed(x, y) : BedSample();
  }

  // Fondo di una cella: MEDIANA di 5 campioni (centro + 4 quarti di cella).
  // Un bordo (tavolo, scatola) che cade a meta' cella o esattamente sul
  // centro non dipende piu' da un confronto al limite (prima: limite
  // invisibile a +-dx/2 dal bordo vero): l'errore di posizione del bordo e'
  // <= dx/4 e la scelta e' per maggioranza.
  BedSample cellBed(real cx, real cy) const {
    if (!bed)
      return BedSample();
    const real q = 0.25 * dx;
    const BedSample s[5] = {bed(cx, cy), bed(cx - q, cy - q), bed(cx + q, cy - q),
                            bed(cx - q, cy + q), bed(cx + q, cy + q)};
    auto zz = [&](int k) { return s[k].solid ? SOLID_Z : s[k].z; };
    int idx[5] = {0, 1, 2, 3, 4};
    std::sort(idx, idx + 5, [&](int a, int b) { return zz(a) < zz(b); });
    return s[idx[2]];
  }

  Tile *ensureTile(int ti, int tj) {
    if (ti < 0 || tj < 0 || ti >= TX || tj >= TY)
      return nullptr;
    Tile *&slot = dir[std::size_t(tj) * TX + ti];
    if (slot)
      return slot;
    tiles.emplace_back();
    slot = &tiles.back();
    slot->ti = ti;
    slot->tj = tj;
    loadBed(*slot);
    return slot;
  }

  struct Rect {
    real xa, xb, ya, yb;
  };
  struct Lost {
    int i, j;
    real v;
  };
  std::vector<Lost> lostQ;

  void loadBed(Tile &t, const Rect *rect = nullptr) {
    const bool first = !t.loaded;
    t.loaded = true;
    for (int lj = 0; lj < T; ++lj)
      for (int li = 0; li < T; ++li) {
        const int gi = t.ti * T + li, gj = t.tj * T + lj;
        const real cx = centerX(gi), cy = centerY(gj);
        if (rect && (cx < rect->xa || cx > rect->xb || cy < rect->ya ||
                     cy > rect->yb))
          continue;
        Cell &c = t.c[std::size_t(lj) * T + li];
        BedSample s = cellBed(cx, cy);
        // Il fondo che SALE sotto l'acqua occupa il posto del liquido: la
        // superficie non deve saltare di colpo (onde spurie ogni refresh).
        // Sabbia: il volume e' assorbito dai pori. Corpo solido: il volume e'
        // spostato nelle celle bagnate vicine (nessuna perdita di acqua).
        if (!first && !c.solid && c.h > 0.0) {
          real lost = 0.0;
          if (s.solid)
            lost = s.body ? c.h : 0.0;
          else if (s.z > c.b)
            lost = std::min(c.h, s.z - c.b);
          if (lost > 0.0) {
            c.h -= lost;
            if (s.body)
              lostQ.push_back({gi, gj, lost * dx * dx});
            else
            {
              absorbedVolume += lost * dx * dx;
              absorbedBy.bedRise += lost * dx * dx;
            }
          }
        }
        c.solid = s.solid;
        c.body = s.body;
        c.b = s.solid ? SOLID_Z : s.z;
        c.n = s.manning;
        c.inf = s.infil;
        c.ret = s.retention;
        if (c.solid) {
          c.h = 0;
          c.u = c.v = 0;
        }
      }
  }

  // Ridistribuisce nelle celle bagnate piu' vicine (anelli crescenti) il
  // volume spostato da un corpo; se non c'e' acqua vicina e' assorbito.
  // Volume spostato non ancora ricollocato. NON e' perso: resta nel bilancio
  // (totalVolume) e viene riprovato a ogni chiamata. Coda limitata (regola
  // NASA n.2); solo l'overflow e' una perdita esplicita e contata.
  static constexpr std::size_t kMaxPendingDisplaced = 4096;
  std::vector<Lost> pendingQ;
  real pendingVolume() const {
    real v = 0.0;
    for (const Lost &q : pendingQ)
      v += q.v;
    return v;
  }

  // Deposita q.v negli anelli crescenti attorno a (q.i, q.j). `wetOnly`:
  // prima passata = solo celle gia' bagnate e libere (comportamento storico);
  // seconda passata = qualunque cella non solida (anche asciutta o su un
  // piano d'appoggio), cosi' il volume spostato non sparisce mai.
  bool depositRings(const Lost &q, bool wetOnly) {
    for (int r = 1; r <= 64; ++r) {
      std::vector<std::pair<int, int>> sel;
      for (int dj = -r; dj <= r; ++dj)
        for (int di = -r; di <= r; ++di) {
          if (std::max(std::abs(di), std::abs(dj)) != r)
            continue;
          const Cell *c = at(q.i + di, q.j + dj);
          if (!c || c->solid)
            continue;
          if (wetOnly && (c->body || c->h <= hDry))
            continue;
          sel.emplace_back(q.i + di, q.j + dj);
        }
      if (sel.empty())
        continue;
      const real dh = q.v / (real(sel.size()) * dx * dx);
      for (const auto &ij : sel) {
        Cell *c = at(ij.first, ij.second);
        c->h += dh;
        markWet(ij.first, ij.second, c->h);
      }
      return true;
    }
    return false;
  }

  void redistributeLost() {
    std::vector<Lost> work;
    work.swap(pendingQ);
    work.insert(work.end(), lostQ.begin(), lostQ.end());
    lostQ.clear();
    for (const Lost &q : work) {
      if (!(q.v > 0.0) || !std::isfinite(q.v))
        continue;
      if (depositRings(q, true) || depositRings(q, false))
        continue;
      if (pendingQ.size() < kMaxPendingDisplaced) {
        pendingQ.push_back(q); // ritenta al prossimo refresh
      } else {
        absorbedVolume += q.v; // overflow esplicito della coda
        absorbedBy.noNeighbour += q.v;
      }
    }
  }

  void refreshBed() {
    for (Tile &t : tiles)
      loadBed(t);
    redistributeLost();
  }

  // Aggiorna il fondo solo nel rettangolo (corpi mobili: costo locale).
  void refreshBedRect(real xa, real xb, real ya, real yb) {
    const Rect r{xa, xb, ya, yb};
    for (Tile &t : tiles) {
      const real tx0 = x0 + t.ti * T * dx, tx1 = tx0 + T * dx;
      const real ty0 = y0 + t.tj * T * dx, ty1 = ty0 + T * dx;
      if (tx1 < xa || tx0 > xb || ty1 < ya || ty0 > yb)
        continue;
      loadBed(t, &r);
    }
    redistributeLost();
  }

  // Sposta nelle celle bagnate vicine un volume (m^3) proveniente da (x, y).
  void pushOut(real x, real y, real volume) {
    if (!(volume > 0.0) || !std::isfinite(volume))
      return;
    lostQ.push_back({cellIndexX(x), cellIndexY(y), volume});
    redistributeLost();
  }

  // Svuota l'impronta di un corpo (acqua "fantasma" interna al solido quando
  // passa da galleggiante ad appoggiato). Ritorna il volume rimosso.
  real drainRect(real cx, real cy, real hx, real hy, bool circular) {
    const int i0 = cellIndexX(cx - hx), i1 = cellIndexX(cx + hx);
    const int j0 = cellIndexY(cy - hy), j1 = cellIndexY(cy + hy);
    real removed = 0.0;
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i) {
        const real ddx = centerX(i) - cx, ddy = centerY(j) - cy;
        const bool inside =
            circular ? (ddx * ddx / (hx * hx) + ddy * ddy / (hy * hy) <= 1.0)
                     : (std::abs(ddx) <= hx && std::abs(ddy) <= hy);
        if (!inside)
          continue;
        Cell *c = at(i, j);
        if (!c || c->solid || c->h <= 0.0)
          continue;
        removed += c->h * dx * dx;
        c->h = 0.0;
        c->u = c->v = 0.0;
      }
    return removed;
  }

  void clear() {
    for (Tile &t : tiles) {
      for (Cell &c : t.c) {
        c.h = c.u = c.v = c.tu = c.tv = c.fx = c.fy = c.k = 0;
        c.s = 1;
      }
      t.wet = false;
      t.run = false;
    }
    run.clear();
    pendingQ.clear();
    lostQ.clear();
    hasWet = false;
    lastMaxWave = 1.0;
    lastMaxDepth = 0.0;
    simulatedTime = 0.0;
    lastIntegratedDt = 0.0;
    lastSubsteps = 0;
  }

  void fillRect(real xa, real xb, real ya, real yb, real depth) {
    const int i0 = cellIndexX(xa), i1 = cellIndexX(xb);
    const int j0 = cellIndexY(ya), j1 = cellIndexY(yb);
    for (int tj = std::max(0, j0 >> SH); tj <= (j1 >> SH); ++tj)
      for (int ti = std::max(0, i0 >> SH); ti <= (i1 >> SH); ++ti)
        ensureTile(ti, tj);
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i) {
        Cell *c = at(i, j);
        if (!c || c->solid)
          continue;
        c->h = depth;
        markWet(i, j, depth);
      }
    updateBounds();
  }

  // Riempie [xa,xb]x[ya,yb] con un VOLUME (m^3): bisezione sulla quota di
  // superficie libera eta tale che sum max(0, eta - b) * dx^2 = volume.
  void fillVolume(real xa, real xb, real ya, real yb, real volume) {
    if (!(volume > 0.0) || !std::isfinite(volume))
      return;
    volume = std::min(volume, std::max(0.0, volumeBudget - totalVolume()));
    if (volume <= 0.0)
      return;
    const int i0 = cellIndexX(xa), i1 = cellIndexX(xb);
    const int j0 = cellIndexY(ya), j1 = cellIndexY(yb);
    for (int tj = std::max(0, j0 >> SH); tj <= (j1 >> SH); ++tj)
      for (int ti = std::max(0, i0 >> SH); ti <= (i1 >> SH); ++ti)
        ensureTile(ti, tj);
    std::vector<Cell *> sel;
    std::vector<std::pair<int, int>> idx;
    real bMin = 1e30, bMax = -1e30;
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i) {
        Cell *c = at(i, j);
        if (!c || c->solid)
          continue;
        sel.push_back(c);
        idx.emplace_back(i, j);
        bMin = std::min(bMin, c->b);
        bMax = std::max(bMax, c->b);
      }
    if (sel.empty())
      return;
    const real cellA = dx * dx;
    auto volumeAt = [&](real eta) {
      real v = 0;
      for (const Cell *c : sel)
        if (eta > c->b)
          v += eta - c->b;
      return v * cellA;
    };
    real lo = bMin, hi = bMax + volume / (real(sel.size()) * cellA) + 1e-3;
    for (int it = 0; it < 64; ++it) {
      const real mid = 0.5 * (lo + hi);
      if (volumeAt(mid) < volume)
        lo = mid;
      else
        hi = mid;
    }
    const real eta = hi;
    for (std::size_t q = 0; q < sel.size(); ++q) {
      const real h = std::max(0.0, eta - sel[q]->b);
      sel[q]->h = h;
      if (h > 0.0)
        markWet(idx[q].first, idx[q].second, h);
    }
    updateBounds();
  }

  void addVolume(real x, real y, real radius, real volume) {
    if (!(volume != 0.0) || !std::isfinite(volume))
      return;
    if (volume > 0.0) {
      volume = std::min(volume, volumeBudget - totalVolume());
      if (volume <= 0.0)
        return;
    }
    const int ic = cellIndexX(x), jc = cellIndexY(y);
    const int rc = int(std::ceil(radius / dx)) + 1;
    std::vector<Cell *> sel;
    std::vector<std::pair<int, int>> idx;
    for (int j = jc - rc; j <= jc + rc; ++j)
      for (int i = ic - rc; i <= ic + rc; ++i) {
        const real ddx = centerX(i) - x, ddy = centerY(j) - y;
        if (ddx * ddx + ddy * ddy > radius * radius && !(i == ic && j == jc))
          continue;
        ensureTile(i >> SH, j >> SH);
        Cell *c = at(i, j);
        if (!c || c->solid)
          continue;
        sel.push_back(c);
        idx.emplace_back(i, j);
      }
    if (sel.empty())
      return;
    const real dh = volume / (real(sel.size()) * dx * dx);
    for (std::size_t q = 0; q < sel.size(); ++q) {
      sel[q]->h = std::max(0.0, sel[q]->h + dh);
      markWet(idx[q].first, idx[q].second, sel[q]->h);
    }
  }

  // Ritorna il volume realmente applicato alle celle (con segno).
  real addVolumeRing(real cx, real cy, real hx, real hy, bool circular,
                     real margin, real volume) {
    if (!(volume != 0.0) || !std::isfinite(volume))
      return 0.0;
    if (volume > 0.0) {
      volume = std::min(volume, volumeBudget - totalVolume());
      if (volume <= 0.0)
        return 0.0;
    }
    const real ox = hx + margin, oy = hy + margin;
    const int i0 = cellIndexX(cx - ox), i1 = cellIndexX(cx + ox);
    const int j0 = cellIndexY(cy - oy), j1 = cellIndexY(cy + oy);
    std::vector<Cell *> sel;
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i) {
        const real ddx = centerX(i) - cx, ddy = centerY(j) - cy;
        bool inner, outer;
        if (circular) {
          const real r2 = ddx * ddx + ddy * ddy;
          inner = r2 <= hx * hx;
          outer = r2 <= ox * ox;
        } else {
          inner = std::abs(ddx) <= hx && std::abs(ddy) <= hy;
          outer = std::abs(ddx) <= ox && std::abs(ddy) <= oy;
        }
        if (inner || !outer)
          continue;
        Cell *c = at(i, j);
        if (!c || c->solid || c->h <= hDry)
          continue;
        sel.push_back(c);
      }
    if (sel.empty())
      return 0.0;
    const real dh = volume / (real(sel.size()) * dx * dx);
    real applied = 0.0;
    for (Cell *c : sel) {
      const real h0 = c->h;
      c->h = std::max(0.0, c->h + std::max(dh, -0.5 * c->h));
      applied += c->h - h0;
    }
    return applied * dx * dx;
  }

  // Perturbazione localizzata della superficie (goccia, tuffo): profilo
  // "cappello messicano" a integrale nullo -> volume conservato. L'onda poi
  // si propaga con le equazioni del flusso (c = sqrt(g h)), si riflette su
  // pareti e solidi e si smorza per attrito: nessuna onda procedurale.
  void addDisturbance(real x, real y, real sigma, real amp) {
    if (!std::isfinite(amp) || amp == 0.0)
      return;
    sigma = std::max(sigma, dx);
    const int ic = cellIndexX(x), jc = cellIndexY(y);
    const int rc = int(std::ceil(4.0 * sigma / dx)) + 1;
    struct WeightedCell {
      Cell *cell;
      int i, j;
      real profile;
    };
    std::vector<WeightedCell> affected;
    affected.reserve(std::size_t(2 * rc + 1) * std::size_t(2 * rc + 1));
    real profileSum = 0.0;
    for (int j = jc - rc; j <= jc + rc; ++j)
      for (int i = ic - rc; i <= ic + rc; ++i) {
        Cell *c = at(i, j);
        if (!c || c->solid || c->h <= hDry)
          continue;
        const real ddx = centerX(i) - x, ddy = centerY(j) - y;
        const real q = (ddx * ddx + ddy * ddy) / (sigma * sigma);
        const real profile = (2.0 - q) * std::exp(-0.5 * q);
        affected.push_back({c, i, j, profile});
        profileSum += profile;
      }
    if (affected.size() < 2)
      return;

    const real profileMean = profileSum / real(affected.size());
    real amplitudeScale = 1.0;
    for (const WeightedCell &sample : affected) {
      const real delta = amp * (sample.profile - profileMean);
      if (delta < 0.0)
        amplitudeScale =
            std::min(amplitudeScale, sample.cell->h / -delta);
    }
    for (const WeightedCell &sample : affected) {
      Cell *c = sample.cell;
      c->h += amplitudeScale * amp * (sample.profile - profileMean);
      if (c->h < 0.0)
        c->h = 0.0;
      if (c->h > hDry)
        markWet(sample.i, sample.j, c->h);
      }
  }

  void addMomentum(real x, real y, real radius, real px, real py) {
    if (!std::isfinite(px) || !std::isfinite(py))
      return;
    const int ic = cellIndexX(x), jc = cellIndexY(y);
    const int rc = int(std::ceil(radius / dx)) + 1;
    std::vector<Cell *> sel;
    real mass = 0;
    for (int j = jc - rc; j <= jc + rc; ++j)
      for (int i = ic - rc; i <= ic + rc; ++i) {
        const real ddx = centerX(i) - x, ddy = centerY(j) - y;
        if (ddx * ddx + ddy * ddy > radius * radius)
          continue;
        Cell *c = at(i, j);
        if (!c || c->solid || c->h <= hDry)
          continue;
        sel.push_back(c);
        mass += liquid.rho * c->h * dx * dx;
      }
    if (sel.empty() || mass < 1e-6)
      return;
    real du = px / mass, dv = py / mass;
    const real mag = std::sqrt(du * du + dv * dv);
    if (mag > 2.0) {
      du *= 2.0 / mag;
      dv *= 2.0 / mag;
    }
    for (Cell *c : sel) {
      c->u += du;
      c->v += dv;
    }
  }

  // Strapiombo (dropGap > 0, impostato da nqg_water_spray.hpp): la faccia fra
  // una cella alta e una cella la cui superficie libera sta SOTTO il fondo
  // della prima non conduce: l'acqua non "teletrasporta" la colonna sul
  // pavimento, ma resta al bordo e ne esce come getto/gocce (modulo spray).
  bool faceDepthClosed(const Cell &L, const Cell &R) const {
    if (!(dropGap > 0.0) || std::abs(L.b - R.b) <= dropGap)
      return false;
    const Cell &lo = L.b < R.b ? L : R;
    const Cell &hi = L.b < R.b ? R : L;
    return lo.b + lo.h < hi.b;
  }
  // Due celle adiacenti sono idraulicamente CONNESSE se il salto di fondo e'
  // una pendenza praticabile (<= ~1.25 dx) oppure se l'acqua della cella piu'
  // bassa raggiunge il fondo di quella piu' alta (sommersione). Un dirupo
  // (bordo di tavolo/scatola) NON e' connesso: la superficie libera non deve
  // essere interpolata attraverso di esso (lama d'acqua "appesa" in aria).
  static bool hydraulicallyConnected(const Cell &a, const Cell &b, real dx) {
    const real db = std::abs(a.b - b.b);
    if (db <= 1.25 * dx)
      return true;
    const Cell &lo = a.b < b.b ? a : b;
    const Cell &hi = a.b < b.b ? b : a;
    return lo.h > hDryConst() && lo.b + lo.h >= hi.b;
  }
  static constexpr real hDryConst() { return 1e-5; }

  // Ricostruzione bilineare CONTINUA su tutte le celle fluide (asciutte
  // incluse, h = 0, eta = fondo), ma limitata alle celle connesse alla cella
  // di riferimento (quella col peso maggiore): attraverso un dirupo si usa
  // solo il lato "proprio". Pesi rinormalizzati.
  struct Recon {
    real w = 0, eta = 0, dep = 0, bd = 0, uu = 0, vv = 0;
  };
  Recon reconstruct(real x, real y) const {
    const real fi = (x - x0) / dx - 0.5, fj = (y - y0) / dx - 0.5;
    const int i0 = int(std::floor(fi)), j0 = int(std::floor(fj));
    const real fx = fi - i0, fy = fj - j0;
    const Cell *cc[4];
    real wt[4];
    const Cell *ref = nullptr;
    real wref = -1.0;
    for (int dj = 0; dj < 2; ++dj)
      for (int di = 0; di < 2; ++di) {
        const int q = dj * 2 + di;
        const Cell *c = at(i0 + di, j0 + dj);
        cc[q] = (c && !c->solid) ? c : nullptr;
        wt[q] = (di ? fx : 1 - fx) * (dj ? fy : 1 - fy);
        if (cc[q] && wt[q] > wref) {
          wref = wt[q];
          ref = cc[q];
        }
      }
    Recon r;
    if (!ref)
      return r;
    for (int dj = 0; dj < 2; ++dj)
      for (int di = 0; di < 2; ++di) {
        const int q = dj * 2 + di;
        const Cell *c = cc[q];
        if (!c || !(c == ref || hydraulicallyConnected(*c, *ref, dx)))
          continue;
        const Cell *e = at(i0 + di + 1, j0 + dj);
        const Cell *n = at(i0 + di, j0 + dj + 1);
        const real cu = 0.5 * (c->u + (e ? e->u : c->u));
        const real cv = 0.5 * (c->v + (n ? n->v : c->v));
        r.w += wt[q];
        r.eta += wt[q] * (c->b + c->h);
        r.dep += wt[q] * c->h;
        r.bd += wt[q] * c->b;
        r.uu += wt[q] * cu;
        r.vv += wt[q] * cv;
      }
    return r;
  }

  Sample sample(real x, real y) const {
    Sample s;
    const Recon r = reconstruct(x, y);
    if (r.w < 0.5 * 0.5) // almeno meta' del peso del vicinato "proprio"
      return s;
    s.depth = r.dep / r.w;
    if (!(s.depth > visibleDepth))
      return s;
    s.wet = true;
    s.eta = r.eta / r.w;
    s.bed = r.bd / r.w;
    s.u = r.uu / r.w;
    s.v = r.vv / r.w;
    return s;
  }

  Sample samplePhysical(real x, real y) const {
    Sample s;
    const Recon r = reconstruct(x, y);
    if (r.w < 0.25)
      return s;
    s.depth = r.dep / r.w;
    if (!(s.depth > hDry))
      return s;
    s.wet = true;
    s.eta = r.eta / r.w;
    s.bed = r.bd / r.w;
    s.u = r.uu / r.w;
    s.v = r.vv / r.w;
    return s;
  }

  // Superficie CONTINUA: stessa ricostruzione di sample() ma senza soglia di
  // visibilita' (eta = fondo dove l'acqua e' assente). Serve al ray marching.
  struct Raw {
    bool valid = false;
    real eta = 0, depth = 0;
  };
  Raw sampleRaw(real x, real y) const {
    Raw out;
    const Recon r = reconstruct(x, y);
    if (r.w < 0.25)
      return out;
    out.valid = true;
    out.eta = r.eta / r.w;
    out.depth = r.dep / r.w;
    return out;
  }

  real depthAt(real x, real y) const {
    const Cell *c = at(cellIndexX(x), cellIndexY(y));
    return (c && !c->solid) ? c->h : 0.0;
  }

  real bedAt(real x, real y) const {
    const Cell *c = at(cellIndexX(x), cellIndexY(y));
    if (c && !c->solid)
      return c->b;
    return bedSampleAt(x, y).z;
  }

  bool bodyBedAt(real x, real y) const {
    const Cell *c = at(cellIndexX(x), cellIndexY(y));
    return c && !c->solid && c->body;
  }

  real totalVolume() const {
    real v = 0;
    for (const Tile &t : tiles)
      if (t.wet)
        for (const Cell &c : t.c)
          v += c.h;
    return v * dx * dx + pendingVolume();
  }
  std::size_t wetCells() const {
    std::size_t n = 0;
    for (const Tile &t : tiles)
      if (t.wet)
        for (const Cell &c : t.c)
          if (c.h > visibleDepth)
            ++n;
    return n;
  }
  real maxDepth() const { return lastMaxDepth; }

  void step(real dtTotal) {
    lastIntegratedDt = 0.0;
    lastSubsteps = 0;
    if (!(dtTotal > 0) || !std::isfinite(dtTotal) || tiles.empty() ||
        !(dx > 0.0) || !(liquid.rho > 0.0) || !(cfl > 0.0))
      return;
    real t = 0;
    while (t < dtTotal) {
      const real remaining = dtTotal - t;
      const real maxStableDt = stableDt();
      if (!(maxStableDt > 0.0) || !std::isfinite(maxStableDt))
        break;
      const real dt = std::min(remaining, maxStableDt);
      if (!(dt > 0.0) || !std::isfinite(dt))
        break;
      substep(dt);
      const real next = t + dt;
      if (!(next > t))
        break;
      t = next;
      ++lastSubsteps;
    }
    lastIntegratedDt = t;
    simulatedTime += t;
    if (++pruneCounter >= 30) {
      pruneCounter = 0;
      compact();
    }
    updateBounds();
  }

  real filmMin() const { return 0.5 * puddleThickness(liquid, gravity); }

  // Isteresi dell'angolo di contatto: un film sottile si muove solo se la
  // spinta (gravita' + capillarita' + inerzia) supera la forza di
  // ancoraggio della linea di contatto, per unita' di lunghezza
  //   F_pin = sigma (cos(theta_rec) - cos(theta_adv)),  theta_rec = theta/2.
  // Accelerazione di soglia a_pin = F_pin / (rho h dx): cresce al diminuire
  // dello spessore, svanisce sopra ~1 cm (acqua profonda: onde libere).
  // E' l'attrito statico del liquido: cosi' pozze e fronti SI FERMANO.
  real pinAccel(real hf) const {
    constexpr real HFADE = 0.01;
    if (hf >= HFADE)
      return 0.0;
    const real th = liquid.contactAngle;
    const real hyst =
        liquid.sigma * std::max(0.0, std::cos(0.5 * th) - std::cos(th));
    const real fade = 1.0 - hf / HFADE;
    return fade * hyst / (liquid.rho * std::max(hf, ROUGHNESS) * dx);
  }

private:
  int pruneCounter = 0;
  std::deque<Tile> tiles;
  std::vector<Tile *> dir;
  std::vector<Tile *> run;
  int epoch = 0;

  void copyFrom(const ShallowFlow &o) {
    dx = o.dx;
    TX = o.TX;
    TY = o.TY;
    x0 = o.x0;
    y0 = o.y0;
    liquid = o.liquid;
    bed = o.bed;
    gravity = o.gravity;
    airDensity = o.airDensity;
    windX = o.windX;
    windY = o.windY;
    cfl = o.cfl;
    hDry = o.hDry;
    hCap = o.hCap;
    visibleDepth = o.visibleDepth;
    smagorinsky = o.smagorinsky;
    speedLimit = o.speedLimit;
    simulatedTime = o.simulatedTime;
    lastIntegratedDt = o.lastIntegratedDt;
    lastSubsteps = o.lastSubsteps;
    absorbRate = o.absorbRate;
    dropGap = o.dropGap;
    retainToSink = o.retainToSink;
    retained = o.retained;
    volumeBudget = o.volumeBudget;
    maxTiles = o.maxTiles;
    absorbedVolume = o.absorbedVolume;
    absorbedBy = o.absorbedBy;
    pendingQ = o.pendingQ;
    pruneCounter = 0;
    lastMaxWave = o.lastMaxWave;
    lastMaxDepth = o.lastMaxDepth;
    hasWet = o.hasWet;
    bxMin = o.bxMin;
    bxMax = o.bxMax;
    byMin = o.byMin;
    byMax = o.byMax;
    bzMin = o.bzMin;
    bzMax = o.bzMax;
    tiles = o.tiles;
    run.clear();
    epoch = 0;
    dir.assign(std::size_t(TX) * TY, nullptr);
    for (Tile &t : tiles) {
      t.run = false;
      dir[std::size_t(t.tj) * TX + t.ti] = &t;
    }
  }

  void markWet(int i, int j, real h) {
    Tile *t = tileAt(i >> SH, j >> SH);
    if (t)
      t->wet = true;
    lastMaxDepth = std::max(lastMaxDepth, h);
    lastMaxWave = std::max(lastMaxWave, std::sqrt(gravity * h));
  }

  real faceDepth(const Cell &L, const Cell &R) const {
    if (L.solid || R.solid)
      return 0.0;
    if (faceDepthClosed(L, R))
      return 0.0;
    const real eL = L.b + L.h, eR = R.b + R.h;
    const real hf = std::max(eL, eR) - std::max(L.b, R.b);
    return hf > 0 ? hf : 0.0;
  }

  const Cell *nb(int i, int j, const Cell &ref, Cell &tmp) const {
    const Cell *p = at(i, j);
    if (p)
      return p;
    tmp = Cell();
    tmp.b = ref.b;
    tmp.n = ref.n;
    return &tmp;
  }

  template <class F> void each(F &&f) {
    for (Tile *t : run)
      for (int lj = 0; lj < T; ++lj)
        for (int li = 0; li < T; ++li)
          f(t->ti * T + li, t->tj * T + lj, t->c[std::size_t(lj) * T + li]);
  }

  static void zeroTile(Tile &t) {
    for (Cell &c : t.c) {
      c.u = c.v = c.tu = c.tv = c.fx = c.fy = c.k = 0;
      c.s = 1;
    }
  }

  real stableDt() const {
    real maxSignalSpeed = 0.3;
    for (const Tile &tile : tiles) {
      if (!tile.wet)
        continue;
      for (const Cell &cell : tile.c) {
        if (cell.solid || cell.h <= hDry)
          continue;
        const real waveSpeed = std::sqrt(gravity * cell.h);
        const real signalSpeed =
            std::max(std::abs(cell.u), std::abs(cell.v)) + waveSpeed;
        maxSignalSpeed = std::max(maxSignalSpeed, signalSpeed);
      }
    }
    real dt = cfl * dx / maxSignalSpeed;
    const real om =
        std::sqrt(liquid.sigma / liquid.rho * std::max(lastMaxDepth, hCap)) *
        8.0 / (dx * dx);
    dt = std::min(dt, 1.0 / (om + 1e-9));
    return dt;
  }

  void growHalo() {
    if (tiles.size() >= maxTiles)
      return;
    const std::size_t n0 = tiles.size();
    for (std::size_t k = 0; k < n0; ++k) {
      if (!tiles[k].wet)
        continue;
      const int ti = tiles[k].ti, tj = tiles[k].tj;
      bool W = false, E = false, S = false, N = false;
      for (int q = 0; q < T; ++q) {
        W = W || tiles[k].c[std::size_t(q) * T].h > hDry;
        E = E || tiles[k].c[std::size_t(q) * T + T - 1].h > hDry;
        S = S || tiles[k].c[std::size_t(q)].h > hDry;
        N = N || tiles[k].c[std::size_t(T - 1) * T + q].h > hDry;
      }
      if (W)
        ensureTile(ti - 1, tj);
      if (E)
        ensureTile(ti + 1, tj);
      if (S)
        ensureTile(ti, tj - 1);
      if (N)
        ensureTile(ti, tj + 1);
      if (W && S)
        ensureTile(ti - 1, tj - 1);
      if (E && S)
        ensureTile(ti + 1, tj - 1);
      if (W && N)
        ensureTile(ti - 1, tj + 1);
      if (E && N)
        ensureTile(ti + 1, tj + 1);
    }
  }

  void compact() {
    std::deque<Tile> keep;
    bool dropped = false;
    for (Tile &t : tiles) {
      if (t.wet || t.run) {
        keep.push_back(std::move(t));
        continue;
      }
      dropped = true;
      for (const Cell &c : t.c)
        if (!c.solid) {
          absorbedVolume += c.h * dx * dx;
          absorbedBy.compaction += c.h * dx * dx;
        }
    }
    if (!dropped)
      return;
    tiles.swap(keep);
    run.clear();
    dir.assign(std::size_t(TX) * TY, nullptr);
    for (Tile &t : tiles) {
      t.run = false;
      dir[std::size_t(t.tj) * TX + t.ti] = &t;
    }
  }

  void buildRunList() {
    ++epoch;
    for (Tile &t : tiles) {
      if (!t.wet)
        continue;
      for (int dj = -1; dj <= 1; ++dj)
        for (int di = -1; di <= 1; ++di) {
          Tile *n = tileAt(t.ti + di, t.tj + dj);
          if (n)
            n->mark = epoch;
        }
    }
    run.clear();
    for (Tile &t : tiles) {
      if (t.mark == epoch) {
        t.run = true;
        run.push_back(&t);
      } else if (t.run) {
        zeroTile(t);
        t.run = false;
      }
    }
  }

  void updateFlags() {
    real mw = 0, md = 0;
    for (Tile *t : run) {
      bool w = false;
      for (const Cell &c : t->c)
        if (c.h > hDry) {
          w = true;
          md = std::max(md, c.h);
          const real sp = std::max(std::abs(c.u), std::abs(c.v));
          mw = std::max(mw, sp + std::sqrt(gravity * c.h));
        }
      t->wet = w;
    }
    lastMaxWave = mw;
    lastMaxDepth = md;
  }

  void updateBounds() {
    bool any = false;
    real xa = 1e30, xb = -1e30, ya = 1e30, yb = -1e30, za = 1e30, zb = -1e30;
    for (const Tile &t : tiles) {
      if (!t.wet)
        continue;
      for (int lj = 0; lj < T; ++lj)
        for (int li = 0; li < T; ++li) {
          const Cell &c = t.c[std::size_t(lj) * T + li];
          if (c.h <= visibleDepth)
            continue;
          any = true;
          const real cx = centerX(t.ti * T + li), cy = centerY(t.tj * T + lj);
          xa = std::min(xa, cx - dx);
          xb = std::max(xb, cx + dx);
          ya = std::min(ya, cy - dx);
          yb = std::max(yb, cy + dx);
          za = std::min(za, c.b);
          zb = std::max(zb, c.b + c.h);
        }
    }
    hasWet = any;
    if (any) {
      bxMin = xa;
      bxMax = xb;
      byMin = ya;
      byMax = yb;
      bzMin = za - 0.01;
      bzMax = zb + 0.02;
    }
  }

  real U(int i, int j) const {
    const Cell *p = at(i, j);
    return p ? p->u : 0.0;
  }
  real V(int i, int j) const {
    const Cell *p = at(i, j);
    return p ? p->v : 0.0;
  }
  real sampleU(real x, real y) const {
    const real fi = (x - x0) / dx, fj = (y - y0) / dx - 0.5;
    const int i0 = int(std::floor(fi)), j0 = int(std::floor(fj));
    const real fx = fi - i0, fy = fj - j0;
    return (U(i0, j0) * (1 - fx) + U(i0 + 1, j0) * fx) * (1 - fy) +
           (U(i0, j0 + 1) * (1 - fx) + U(i0 + 1, j0 + 1) * fx) * fy;
  }
  real sampleV(real x, real y) const {
    const real fi = (x - x0) / dx - 0.5, fj = (y - y0) / dx;
    const int i0 = int(std::floor(fi)), j0 = int(std::floor(fj));
    const real fx = fi - i0, fy = fj - j0;
    return (V(i0, j0) * (1 - fx) + V(i0 + 1, j0) * fx) * (1 - fy) +
           (V(i0, j0 + 1) * (1 - fx) + V(i0 + 1, j0 + 1) * fx) * fy;
  }

  void substep(real dt) {
    NQG_REQUIRE(dt > 0 && dt < 1.0);
    NQG_REQUIRE(TX > 0 && TY > 0);

    growHalo();
    buildRunList();
    if (run.empty())
      return;

    each([&](int i, int j, Cell &c) {
      if (c.solid) {
        c.tu = c.tv = 0;
        return;
      }
      Cell ta, tb;
      const Cell *w = nb(i - 1, j, c, ta);
      const Cell *s = nb(i, j - 1, c, tb);
      if (faceDepth(*w, c) < hDry) {
        c.tu = 0;
      } else {
        const real vav =
            0.25 * (V(i - 1, j) + V(i, j) + V(i - 1, j + 1) + V(i, j + 1));
        c.tu = sampleU(x0 + i * dx - c.u * dt, y0 + (j + 0.5) * dx - vav * dt);
      }
      if (faceDepth(*s, c) < hDry) {
        c.tv = 0;
      } else {
        const real uav =
            0.25 * (U(i, j - 1) + U(i + 1, j - 1) + U(i, j) + U(i + 1, j));
        c.tv = sampleV(x0 + (i + 0.5) * dx - uav * dt, y0 + j * dx - c.v * dt);
      }
    });

    each([&](int i, int j, Cell &c) {
      c.k = 0;
      if (c.solid || c.h <= hCap)
        return;
      const real e = c.b + c.h;
      real sum = 0;
      static const int d[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
      for (const auto &q : d) {
        const Cell *n = at(i + q[0], j + q[1]);
        real en = e;
        if (n && !n->solid && n->h > hCap)
          en = n->b + n->h;
        sum += en - e;
      }
      c.k = sum / (dx * dx);
    });

    const real capC = liquid.sigma / liquid.rho;
    const real nu0 = liquid.nu();
    const real rho = liquid.rho;
    const real pinThr = liquid.sigma * (1.0 - std::cos(liquid.contactAngle));
    const real idx = 1.0 / dx;
    auto TU = [&](int i, int j, real fb) {
      const Cell *p = at(i, j);
      return p ? p->tu : fb;
    };
    auto TV = [&](int i, int j, real fb) {
      const Cell *p = at(i, j);
      return p ? p->tv : fb;
    };

    each([&](int i, int j, Cell &c) {
      if (c.solid) {
        c.u = c.v = 0;
        return;
      }
      Cell ta, tb;
      const Cell *w = nb(i - 1, j, c, ta);
      const Cell *s = nb(i, j - 1, c, tb);

      real hf = faceDepth(*w, c);
      if (hf < hDry) {
        c.u = 0;
      } else {
        real a = -gravity * ((c.b + c.h) - (w->b + w->h)) * idx;
        if (c.h > hCap && w->h > hCap)
          a += capC * (c.k - w->k) * idx;
        const real tu = c.tu;
        const real lap = (TU(i + 1, j, tu) + TU(i - 1, j, tu) +
                          TU(i, j + 1, tu) + TU(i, j - 1, tu) - 4 * tu) *
                         idx * idx;
        const real gx = (TU(i + 1, j, tu) - TU(i - 1, j, tu)) * 0.5 * idx;
        const real gy = (TU(i, j + 1, tu) - TU(i, j - 1, tu)) * 0.5 * idx;
        const real S = std::sqrt(gx * gx + gy * gy);
        const real cs = smagorinsky * dx;
        const real nue = std::min(nu0 + cs * cs * S, 0.2 * dx * dx / dt);
        a += nue * lap;
        const real vav =
            0.25 * (V(i - 1, j) + V(i, j) + V(i - 1, j + 1) + V(i, j + 1));
        const real rx = windX - tu, ry = windY - vav;
        const real sp = std::sqrt(rx * rx + ry * ry);
        if (sp > 1e-6) {
          const real cd = std::clamp(1.2e-3 + 0.065e-3 * sp, 1.2e-3, 2.4e-3);
          a += airDensity * cd * sp * rx / (rho * std::max(hf, 0.005));
        }
        real un = tu + dt * a;
        {
          const real dvPin = dt * pinAccel(hf);
          un = std::abs(un) <= dvPin ? 0.0 : un - std::copysign(dvPin, un);
        }
        const real nm = 0.5 * (c.n + w->n);
        const real hp = std::max(hf, 1e-4);
        const real fr = 3.0 * nu0 / (hp * hp) + gravity * nm * nm *
                                                    std::abs(un) /
                                                    std::pow(hp, 4.0 / 3.0);
        un /= 1.0 + dt * fr;
        if (un != 0.0) {
          const Cell *up = un > 0 ? w : &c;
          const Cell *dn = un > 0 ? &c : w;
          if (dn->h < hDry && !dn->solid && up->h > hDry) {
            const real drive =
                0.5 * rho * gravity * hf * hf + 0.5 * rho * hf * un * un;
            if (drive < pinThr)
              un = 0.0;
          }
        }
        c.u = speedLimit > 0.0
            ? std::clamp(un, -speedLimit, speedLimit)
            : un;
      }

      hf = faceDepth(*s, c);
      if (hf < hDry) {
        c.v = 0;
      } else {
        real a = -gravity * ((c.b + c.h) - (s->b + s->h)) * idx;
        if (c.h > hCap && s->h > hCap)
          a += capC * (c.k - s->k) * idx;
        const real tv = c.tv;
        const real lap = (TV(i + 1, j, tv) + TV(i - 1, j, tv) +
                          TV(i, j + 1, tv) + TV(i, j - 1, tv) - 4 * tv) *
                         idx * idx;
        const real gx = (TV(i + 1, j, tv) - TV(i - 1, j, tv)) * 0.5 * idx;
        const real gy = (TV(i, j + 1, tv) - TV(i, j - 1, tv)) * 0.5 * idx;
        const real S = std::sqrt(gx * gx + gy * gy);
        const real cs = smagorinsky * dx;
        const real nue = std::min(nu0 + cs * cs * S, 0.2 * dx * dx / dt);
        a += nue * lap;
        const real uav =
            0.25 * (U(i, j - 1) + U(i + 1, j - 1) + U(i, j) + U(i + 1, j));
        const real rx = windX - uav, ry = windY - tv;
        const real sp = std::sqrt(rx * rx + ry * ry);
        if (sp > 1e-6) {
          const real cd = std::clamp(1.2e-3 + 0.065e-3 * sp, 1.2e-3, 2.4e-3);
          a += airDensity * cd * sp * ry / (rho * std::max(hf, 0.005));
        }
        real vn = tv + dt * a;
        {
          const real dvPin = dt * pinAccel(hf);
          vn = std::abs(vn) <= dvPin ? 0.0 : vn - std::copysign(dvPin, vn);
        }
        const real nm = 0.5 * (c.n + s->n);
        const real hp = std::max(hf, 1e-4);
        const real fr = 3.0 * nu0 / (hp * hp) + gravity * nm * nm *
                                                    std::abs(vn) /
                                                    std::pow(hp, 4.0 / 3.0);
        vn /= 1.0 + dt * fr;
        if (vn != 0.0) {
          const Cell *up = vn > 0 ? s : &c;
          const Cell *dn = vn > 0 ? &c : s;
          if (dn->h < hDry && !dn->solid && up->h > hDry) {
            const real drive =
                0.5 * rho * gravity * hf * hf + 0.5 * rho * hf * vn * vn;
            if (drive < pinThr)
              vn = 0.0;
          }
        }
        c.v = speedLimit > 0.0
            ? std::clamp(vn, -speedLimit, speedLimit)
            : vn;
      }
    });

    each([&](int i, int j, Cell &c) {
      Cell ta, tb;
      const Cell *w = nb(i - 1, j, c, ta);
      const Cell *s = nb(i, j - 1, c, tb);
      c.fx = faceDepth(*w, c) * c.u;
      c.fy = faceDepth(*s, c) * c.v;
    });

    each([&](int i, int j, Cell &c) {
      real out = 0;
      if (c.fx < 0)
        out -= c.fx;
      if (c.fy < 0)
        out -= c.fy;
      const Cell *e = at(i + 1, j);
      const Cell *n = at(i, j + 1);
      if (e && e->fx > 0)
        out += e->fx;
      if (n && n->fy > 0)
        out += n->fy;
      const real vol = dt / dx * out;
      c.s = (vol > c.h && vol > 0) ? c.h / vol : 1.0;
    });

    each([&](int i, int j, Cell &c) {
      const Cell *w = at(i - 1, j);
      const Cell *s = at(i, j - 1);
      if (c.fx > 0) {
        if (w) {
          c.fx *= w->s;
          c.u *= w->s;
        }
      } else if (c.fx < 0) {
        c.fx *= c.s;
        c.u *= c.s;
      }
      if (c.fy > 0) {
        if (s) {
          c.fy *= s->s;
          c.v *= s->s;
        }
      } else if (c.fy < 0) {
        c.fy *= c.s;
        c.v *= c.s;
      }
    });

    each([&](int i, int j, Cell &c) {
      const Cell *e = at(i + 1, j);
      const Cell *n = at(i, j + 1);
      const real fxe = e ? e->fx : 0.0;
      const real fyn = n ? n->fy : 0.0;
      c.h -= dt / dx * ((fxe - c.fx) + (fyn - c.fy));
      if (c.h < 1e-9 || c.solid)
        c.h = 0;
    });

    const real dAbs = absorbRate * dt;
    each([&](int i, int j, Cell &c) {
      if (c.solid || c.h <= 0.0)
        return;
      // infiltrazione nel terreno permeabile (Darcy, tasso costante)
      if (c.inf > 0.0) {
        const real di = std::min(c.h, c.inf * dt);
        c.h -= di;
        absorbedVolume += di * dx * dx;
        absorbedBy.infiltration += di * dx * dx;
      }
      // film sotto la ritenzione di superficie (scabrezza/depressioni del
      // fondo): intrappolato/assorbito. Lo spessore minimo di pozza NON e' piu'
      // una perdita di volume: lo tiene l'ancoraggio della linea di contatto.
      const real fm = c.ret;
      if (c.h <= 0.0 || c.h >= fm)
        return;
      const real d = std::min(c.h, dAbs);
      c.h -= d;
      real rem = d;
      if (c.h < hDry) {
        rem += c.h;
        c.h = 0.0;
      }
      if (retainToSink && c.inf <= 0.0) {
        Retained &r =
            retained[(long long)(i + 100000) * 200003LL + (j + 100000)];
        r.i = i;
        r.j = j;
        r.v += rem * dx * dx;
        r.z = c.b;
      } else {
        absorbedVolume += rem * dx * dx;
        absorbedBy.retention += rem * dx * dx;
      }
    });

    updateFlags();
  }
};

struct ObserverSurfaceEstimate {
  real wetProbability = 0.0;
  real meanEta = 0.0;
  real etaVariance = 0.0;
  real meanDepth = 0.0;
  std::size_t sampleCount = 0;
};

inline std::uint64_t observerSampleHash(std::uint64_t value) {
  value += 0x9E3779B97F4A7C15ull;
  value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ull;
  value = (value ^ (value >> 27)) * 0x94D049BB133111EBull;
  return value ^ (value >> 31);
}

inline real observerSampleUnit(std::uint64_t seed, std::uint64_t sample,
                               std::uint64_t axis) {
  const std::uint64_t bits =
      observerSampleHash(seed ^ (sample * 2 + axis));
  return real(bits >> 11) * (1.0 / 9007199254740992.0);
}

inline ObserverSurfaceEstimate estimateObserverSurface(
    const ShallowFlow &flow, real x, real y, real footprintX, real footprintY,
    std::size_t samples, std::uint64_t seed) {
  ObserverSurfaceEstimate estimate;
  estimate.sampleCount = samples;
  if (samples == 0)
    return estimate;

  real etaMean = 0.0;
  real etaM2 = 0.0;
  real depthSum = 0.0;
  std::size_t wetSamples = 0;
  for (std::size_t i = 0; i < samples; ++i) {
    const real sx = x + (observerSampleUnit(seed, i, 0) - 0.5) * footprintX;
    const real sy = y + (observerSampleUnit(seed, i, 1) - 0.5) * footprintY;
    const ShallowFlow::Raw sample = flow.sampleRaw(sx, sy);
    if (!sample.valid || sample.depth <= flow.visibleDepth)
      continue;

    ++wetSamples;
    const real delta = sample.eta - etaMean;
    etaMean += delta / real(wetSamples);
    etaM2 += delta * (sample.eta - etaMean);
    depthSum += sample.depth;
  }

  estimate.wetProbability = real(wetSamples) / real(samples);
  if (wetSamples > 0) {
    estimate.meanEta = etaMean;
    estimate.etaVariance = etaM2 / real(wetSamples);
    estimate.meanDepth = depthSum / real(wetSamples);
  }
  return estimate;
}


} // namespace fluid
} // namespace nqg

#endif // NQG_WATER_SOLVER_HPP
