// ============================================================================
//  nqg_engine3d.hpp  -  motore grafico 3D + livello di gioco (C++17, solo STL)
//  Richiede nqg_physics_core.hpp (stessa cartella).
//
//  IDEA: la camera NON e' una pinhole. E' un OSSERVATORE nel senso dei
//  documenti:
//        C = (f_s, B_s, R, C_ops, M, L)  con  f_s*K <= C_ops
//        Il sensore ha tempo di esposizione Te, quantizzazione Q, rumore eta.
//        Il suo orologio e' il tempo proprio:  dtau = alpha(r) dt
//        (Schwarzschild). Quello che il motore puo' mostrare e' deciso dalla
//        capacita' dell'osservatore.
//
//  RENDERING: ray tracing di geodetiche nulle di Schwarzschild, equazione di
//  Binet
//        u'' = -u + 3 M u^2,  u = 1/r      [F]
//  OTTIMIZZAZIONE CHIAVE: il raggio e' planare e la sua forma dipende solo
//  dall'angolo psi rispetto alla radiale. Si tabula u(phi) per ~3000 valori di
//  psi una volta per raggio della camera; poi ogni pixel costa O(1) (2 lookup
//  bilineari
//  + eventuali attraversamenti del disco) invece di ~10^3 passi di
//  integrazione. Il piano del disco taglia la traiettoria planare a phi = phi0
//  + k*pi (esatto).
//
//  STATO EPISTEMICO: geodetiche, redshift gravitazionale e Doppler = [F].
//  Emissione del disco, texture, stelle = stilizzati (artistici).
//  Meccanica di gioco con livelli/V11 = [T] ipotesi del modello NQG, non fisica
//  provata. Il ritardo di propagazione della luce (tempo di volo) e'
//  trascurato.
// ============================================================================
#ifndef NQG_ENGINE3D_HPP
#define NQG_ENGINE3D_HPP

#include "nqg_physics_core.hpp"
#include <atomic>
#include <cstdio>
#include <set>
#include <string>
#include <thread>

namespace nqg {
namespace engine {

// ------------------------------------------------------------------ vettori
struct Vec3 {
  real x = 0, y = 0, z = 0;
  Vec3() = default;
  Vec3(real a, real b, real c) : x(a), y(b), z(c) {}
  Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(real s) const { return {x * s, y * s, z * s}; }
  real dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3 &o) const {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  real norm() const { return std::sqrt(dot(*this)); }
  Vec3 normalized() const {
    real n = norm();
    return n > 0 ? *this * (1 / n) : Vec3(0, 0, 1);
  }
};
inline Vec3 rotateAbout(const Vec3 &v, const Vec3 &axisUnit,
                        real a) { // Rodrigues
  const real c = std::cos(a), s = std::sin(a);
  return v * c + axisUnit.cross(v) * s + axisUnit * (axisUnit.dot(v) * (1 - c));
}

struct Rgb {
  float r = 0, g = 0, b = 0;
};
struct Image {
  int w = 0, h = 0;
  std::vector<float> px; // RGB lineare->display [0,1]
  Image() = default;
  Image(int w_, int h_) : w(w_), h(h_), px(std::size_t(w_) * h_ * 3, 0.f) {}
  float *at(int x, int y) { return &px[(std::size_t(y) * w + x) * 3]; }
  const float *at(int x, int y) const {
    return &px[(std::size_t(y) * w + x) * 3];
  }
  bool writePPM(const std::string &path) const {
    std::FILE *f = std::fopen(path.c_str(), "wb");
    if (!f)
      return false;
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (float v : px) {
      unsigned char c =
          (unsigned char)std::lround(std::clamp(v, 0.f, 1.f) * 255.f);
      std::fputc(c, f);
    }
    std::fclose(f);
    return true;
  }
  void bar(int x0, int y0, int bw, int bh, real frac,
           Rgb col) { // HUD senza font
    for (int y = y0; y < y0 + bh && y < h; ++y)
      for (int x = x0; x < x0 + bw && x < w; ++x) {
        const bool edge =
            (y == y0 || y == y0 + bh - 1 || x == x0 || x == x0 + bw - 1);
        const bool fill = x - x0 < frac * bw;
        float *p = at(x, y);
        p[0] = edge ? 1.f : fill ? col.r : p[0] * 0.35f;
        p[1] = edge ? 1.f : fill ? col.g : p[1] * 0.35f;
        p[2] = edge ? 1.f : fill ? col.b : p[2] * 0.35f;
      }
  }
};

// -------------------------------------------------------------- hash e rumore
inline std::uint32_t hash32(std::uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}
inline real h01(std::uint32_t a, std::uint32_t b = 0, std::uint32_t c = 0) {
  return hash32(a * 0x9E3779B1U ^ hash32(b + 0x85ebca6bU) ^
                hash32(c * 0xC2B2AE35U + 7U)) /
         4294967296.0;
}
inline real smooth(real t) { return t * t * (3 - 2 * t); }
// value noise 2D, periodico di periodo NA nella seconda coordinata (angolo)
inline real noisePeriodic(real x, real a, int NA) {
  const int xi = int(std::floor(x)), ai = int(std::floor(a));
  const real fx = smooth(x - xi), fa = smooth(a - ai);
  auto V = [&](int i, int j) {
    int jj = ((j % NA) + NA) % NA;
    return h01(std::uint32_t(i + 4096), std::uint32_t(jj));
  };
  return (V(xi, ai) * (1 - fx) + V(xi + 1, ai) * fx) * (1 - fa) +
         (V(xi, ai + 1) * (1 - fx) + V(xi + 1, ai + 1) * fx) * fa;
}
// corpo nero -> RGB (approssimazione di Tanner Helland, T in kelvin)
inline Rgb blackbody(real T) {
  const real t = std::clamp(T, 1000.0, 40000.0) / 100.0;
  real r, g, b;
  r = t <= 66 ? 255 : 329.698727446 * std::pow(t - 60, -0.1332047592);
  g = t <= 66 ? 99.4708025861 * std::log(t) - 161.1195681661
              : 288.1221695283 * std::pow(t - 60, -0.0755148492);
  b = t >= 66
          ? 255
          : (t <= 19 ? 0 : 138.5177312231 * std::log(t - 10) - 305.0447927307);
  auto cl = [](real v) { return float(std::clamp(v, 0.0, 255.0) / 255.0); };
  return {cl(r), cl(g), cl(b)};
}

// ===================================================== capacita' osservatore
// C = (f_s, B_s, R, C_ops, M, L)  [dossie.txt §11] + parametri del sensore
struct ObserverCapacity {
  real fs = 30;  // Hz, frequenza di campionamento (tempo PROPRIO)
  real Bs = 4e8; // bit/s, banda di uscita
  int width = 1280, height = 720; // R richiesta
  real Cops = 5e10;               // operazioni/s
  real memBytes = 64e6;           // M
  real latency = 0.10;            // L, secondi propri
  real exposure = 0.012;          // Te, secondi propri
  int bits = 8;                   // Q
  real readNoise = 0.004;         // eta (sigma, in unita' di segnale)
  int subsamples = 3;             // campioni temporali dentro Te
  real opsPerPixel = 600;         // costo K per pixel e frame (modello)
  real gain = 2.2;                // guadagno di esposizione
};
struct Plan { // risultato della negoziazione capacita' <-> richiesta
  int width = 0, height = 0;
  real fsEff = 0, pixelBudget = 0, usage = 0; // usage = fs*K/Cops
  std::string limitedBy;
  int latencyFrames = 0;
};
// f_s*K <= C_ops, K = pixel*opsPerPixel*(1+0.15(sub-1)); banda e memoria
// limitano i pixel.
inline Plan negotiate(const ObserverCapacity &c) {
  Plan p;
  const real kSub = 1 + 0.15 * (c.subsamples - 1);
  const real pOps = c.Cops / (c.fs * c.opsPerPixel * kSub);
  const real pBw = c.Bs / (c.fs * c.bits * 3);
  const int lf = int(std::ceil(c.latency * c.fs));
  p.latencyFrames = lf;
  const real pMem = c.memBytes / (12.0 * (lf + 2));
  const real req = real(c.width) * c.height;
  p.pixelBudget = std::min({pOps, pBw, pMem});
  p.limitedBy = req <= p.pixelBudget ? "nessuno"
                                     : (p.pixelBudget == pOps  ? "C_ops"
                                        : p.pixelBudget == pBw ? "B_s"
                                                               : "memoria");
  real s = req <= p.pixelBudget ? 1.0 : std::sqrt(p.pixelBudget / req);
  p.width = std::max(16, int(c.width * s));
  p.height = std::max(9, int(c.height * s));
  p.fsEff = c.fs;
  const real K = real(p.width) * p.height * c.opsPerPixel * kSub;
  if (c.fs * K > c.Cops) {
    p.fsEff = c.Cops / K;
    p.limitedBy = "C_ops (f_s ridotta)";
  }
  p.usage = p.fsEff * K / c.Cops;
  return p;
}
// il segnale e' risolvibile se f_s > 2 nu_obs (Nyquist); altrimenti alias
struct ObservationReport {
  real alpha = 1, nuObs = 0, nyquist = 0, fs = 0, aliasFreq = 0;
  bool aliased = false;
};
inline ObservationReport observeBeacon(real M, real rObs, real nuCoord,
                                       real fs) {
  ObservationReport r;
  r.alpha = schw::lapse(M, rObs);
  r.nuObs = nuCoord / r.alpha;
  r.fs = fs;
  r.nyquist = schw::nyquistMin(r.nuObs);
  r.aliased = !(fs > r.nyquist);
  r.aliasFreq = std::abs(r.nuObs - fs * std::round(r.nuObs / fs));
  return r;
}

// ================================================================== camera
struct Camera {
  real r = 30, theta = 1.35, phi = 0, yaw = 0, pitch = 0,
       fovY = 60.0 * PI / 180.0;
  Vec3 radial() const {
    return {std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi),
            std::cos(theta)};
  }
  Vec3 position() const { return radial() * r; }
  void basis(Vec3 &f, Vec3 &right, Vec3 &up) const {
    const Vec3 rh = radial(), zh(0, 0, 1);
    f = rh * -1.0;
    right = f.cross(zh);
    if (right.norm() < 1e-9)
      right = Vec3(0, 1, 0);
    right = right.normalized();
    up = right.cross(f).normalized();
    f = rotateAbout(f, up, yaw);
    right = rotateAbout(right, up, yaw);
    f = rotateAbout(f, right, pitch);
    up = rotateAbout(up, right, pitch);
  }
};

// ============================================== tabella dei raggi (u(phi)|psi)
class RayTable {
public:
  RayTable(real M, real rc, int nPsi = 3072, real dphi = 0.01,
           real Rfar = 400.0, real phiMax = 10 * PI)
      : M_(M), rc_(rc), n_(nPsi), dphi_(dphi), Rfar_(Rfar), phiMax_(phiMax),
        rays_(std::size_t(nPsi)) {
    std::atomic<int> next{0};
    const unsigned T = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> th;
    for (unsigned t = 0; t < T; ++t)
      th.emplace_back([&] {
        for (int i; (i = next++) < n_;)
          trace(rays_[std::size_t(i)], (i + 0.5) * PI / n_);
      });
    for (auto &x : th)
      x.join();
  }
  real radius() const { return rc_; }
  struct Ray {
    std::vector<float> u;
    real phiEnd = 0, uEnd = 0, phiInf = 0;
    bool captured = false;
  };
  const Ray &ray(int i) const { return rays_[std::size_t(i)]; }
  int size() const { return n_; }
  real dphi() const { return dphi_; }
  bool uAt(int i, real phi, real &u) const {
    const Ray &R = rays_[std::size_t(i)];
    if (phi >= R.phiEnd || phi < 0)
      return false;
    const std::size_t J = R.u.size();
    std::size_t j = std::size_t(phi / dphi_);
    if (j >= J)
      j = J - 1;
    const real p0 = j * dphi_, u0 = R.u[j];
    real p1, u1;
    if (j + 1 < J) {
      p1 = p0 + dphi_;
      u1 = R.u[j + 1];
    } else {
      p1 = R.phiEnd;
      u1 = R.uEnd;
    }
    const real t = (p1 > p0) ? (phi - p0) / (p1 - p0) : 0;
    u = u0 + (u1 - u0) * t;
    return true;
  }
  // bilineare in psi; se i due raggi vicini divergono (bordo ombra) usa il piu'
  // vicino
  bool sample(real psi, real phi, real &u) const {
    real s = psi / PI * n_ - 0.5;
    int i0 = int(std::floor(s));
    real w = s - i0;
    i0 = std::clamp(i0, 0, n_ - 1);
    const int i1 = std::clamp(i0 + 1, 0, n_ - 1);
    real a, b;
    const bool va = uAt(i0, phi, a), vb = uAt(i1, phi, b);
    if (va && vb) {
      u = a * (1 - w) + b * w;
      return true;
    }
    if (va && w < 0.5) {
      u = a;
      return true;
    }
    if (vb && w >= 0.5) {
      u = b;
      return true;
    }
    return false;
  }
  int nearest(real psi) const {
    return std::clamp(int(std::lround(psi / PI * n_ - 0.5)), 0, n_ - 1);
  }
  // stato finale interpolato: captured? phiEnd, phiInf
  void terminal(real psi, bool &captured, real &phiEnd, real &phiInf) const {
    real s = psi / PI * n_ - 0.5;
    int i0 = std::clamp(int(std::floor(s)), 0, n_ - 1);
    const int i1 = std::clamp(i0 + 1, 0, n_ - 1);
    const real w = std::clamp(s - std::floor(s), 0.0, 1.0);
    const Ray &A = rays_[std::size_t(i0)], &B = rays_[std::size_t(i1)];
    const Ray &N = w < 0.5 ? A : B;
    captured = N.captured;
    if (A.captured == B.captured) {
      phiEnd = A.phiEnd * (1 - w) + B.phiEnd * w;
      phiInf = A.phiInf * (1 - w) + B.phiInf * w;
    } else {
      phiEnd = N.phiEnd;
      phiInf = N.phiInf;
    }
  }
  // riferimento diretto (per i test): u(phi) integrato senza tabella
  static bool direct(real M, real rc, real psi, real phi, real &uOut) {
    real u = 1 / rc,
         v = -u * std::sqrt(1 - 2 * M / rc) * std::cos(psi) / std::sin(psi),
         p = 0;
    const real h = 1e-4;
    const real uh = 1 / (2 * M);
    while (p < phi) {
      step(u, v, h, M);
      p += h;
      if (u >= uh)
        return false;
    }
    uOut = u;
    return true;
  }

private:
  static void step(real &u, real &v, real h, real M) {
    auto f = [&](real uu) { return -uu + 3 * M * uu * uu; };
    const real k1u = v, k1v = f(u), k2u = v + h / 2 * k1v,
               k2v = f(u + h / 2 * k1u);
    const real k3u = v + h / 2 * k2v, k3v = f(u + h / 2 * k2u),
               k4u = v + h * k3v, k4v = f(u + h * k3u);
    u += h / 6 * (k1u + 2 * k2u + 2 * k3u + k4u);
    v += h / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
  }
  void trace(Ray &R, real psi) const {
    const real u0 = 1 / rc_, a0 = std::sqrt(1 - 2 * M_ / rc_),
               uh = 1 / (2 * M_), uf = 1 / Rfar_;
    real u = u0, v = -u0 * a0 * std::cos(psi) / std::sin(psi), phi = 0;
    R.u.push_back(float(u));
    for (;;) {
      if (phi >= phiMax_) {
        R.captured = true;
        R.phiEnd = phi;
        R.uEnd = u;
        return;
      }
      const int n = std::clamp(
          int(std::ceil(dphi_ * std::abs(v) / (0.05 * std::max(u, 1e-6)))), 1,
          400);
      const real h = dphi_ / n;
      for (int s = 0; s < n; ++s) {
        const real up = u, pp = phi;
        step(u, v, h, M_);
        phi += h;
        if (u >= uh) {
          const real f = (uh - up) / (u - up);
          R.captured = true;
          R.phiEnd = pp + f * h;
          R.uEnd = uh;
          return;
        }
        if (u <= uf && v < 0) {
          const real f = (uf - up) / (u - up);
          R.phiEnd = pp + f * h;
          R.uEnd = uf;
          const real b = rc_ * std::sin(psi) / a0;
          R.phiInf = R.phiEnd + std::asin(std::min(1.0, b * uf));
          return;
        }
      }
      R.u.push_back(float(u));
    }
  }
  real M_, rc_;
  int n_;
  real dphi_, Rfar_, phiMax_;
  std::vector<Ray> rays_;
};

// ================================================================ scena/disco
struct Scene {
  real M = 1;
  real rIn = 6, rOut = 22; // disco (ISCO = 6M)
  real Tin = 9000;         // K, temperatura interna (stilizzata)
  real beaconR = 9, beaconPhi0 = 0.0, beaconSigma = 0.9,
       beaconNu = 40; // faro: nu in tempo coordinato [Hz]
  real beaconBoost = 7.0;
};
// fattore g = E_oss / E_em per materia in orbita circolare, fotone con momento
// angolare Lz (E_loc=1).
//  g = sqrt(1-3M/r) / (alpha_o - Omega*Lz)       [F]   (verificato in test:
//  facciata Lz=0)
inline real diskG(real M, real rEm, real alphaObs, real Lz) {
  const real Om = std::sqrt(M / (rEm * rEm * rEm));
  return std::sqrt(1 - 3 * M / rEm) / (alphaObs - Om * Lz);
}

struct Hit {
  real r, phiDisk, g;
};
struct PixelGeom {
  int nh = 0;
  Hit h[6];
  bool sky = false;
  Vec3 skyDir;
};

class Renderer {
public:
  explicit Renderer(Scene s = {}) : S(s) {}
  Scene S;
  unsigned threads = std::max(1u, std::thread::hardware_concurrency());

  const RayTable &table(real rc) {
    if (!tab_ || std::abs(tab_->radius() - rc) > 1e-9)
      tab_ = std::make_unique<RayTable>(S.M, rc);
    return *tab_;
  }
  // geometria di un pixel: tutti gli attraversamenti del disco e l'eventuale
  // cielo
  PixelGeom trace(const RayTable &T, const Camera &cam, const Vec3 &n,
                  real alphaObs) const {
    PixelGeom G;
    const Vec3 e1 = cam.radial();
    const real cp = std::clamp(n.dot(e1), -1.0, 1.0), psi = std::acos(cp),
               sp = std::sin(psi);
    Vec3 e2 = sp > 1e-9 ? (n - e1 * cp) * (1 / sp)
                        : (std::abs(e1.z) < 0.9
                               ? Vec3(0, 0, 1).cross(e1).normalized()
                               : Vec3(1, 0, 0).cross(e1).normalized());
    bool cap;
    real pEnd, pInf;
    T.terminal(psi, cap, pEnd, pInf);
    const real Lz = -cam.r * sp * e1.cross(e2).z; // fotone fisico = -n
    if (std::abs(e1.z) > 1e-9 || std::abs(e2.z) > 1e-9) {
      real p0 = std::atan2(-e1.z, e2.z);
      if (p0 < 0)
        p0 += PI;
      if (p0 < 1e-7)
        p0 += PI;
      for (real ph = p0; ph < pEnd && G.nh < 6; ph += PI) {
        real u;
        if (!T.sample(psi, ph, u) || u <= 0)
          continue;
        const real r = 1 / u;
        if (r < S.rIn || r > S.rOut)
          continue;
        const Vec3 pos = (e1 * std::cos(ph) + e2 * std::sin(ph)) * r;
        G.h[G.nh++] = {r, std::atan2(pos.y, pos.x),
                       diskG(S.M, r, alphaObs, Lz)};
      }
    }
    if (!cap) {
      G.sky = true;
      G.skyDir = e1 * std::cos(pInf) + e2 * std::sin(pInf);
    }
    return G;
  }
  Rgb sky(const Vec3 &d, real shift) const {
    const real lat = std::asin(std::clamp(d.z, -1.0, 1.0)),
               lon = std::atan2(d.y, d.x), D = 0.045;
    Rgb o;
    const Vec3 nG = Vec3(0.25, 0.35, 0.9).normalized();
    const real ga = d.dot(nG);
    const real gl =
        std::exp(-(ga / 0.2) * (ga / 0.2)) * 0.10 *
        (0.5 + noisePeriodic(lat * 9 + 50, lon * 3.0 + 40, 1 << 20));
    o.r = float(gl * 0.9);
    o.g = float(gl * 0.8);
    o.b = float(gl * 1.1);
    const int lc0 = int(std::floor(lat / D));
    for (int dl = -1; dl <= 1; ++dl) {
      const int lc = lc0 + dl;
      const real latc = (lc + 0.5) * D;
      const int nlon =
          std::max(1, int(std::lround(2 * PI * std::cos(latc) / D)));
      if (std::cos(latc) <= 0)
        continue;
      const int lo = int(std::floor((lon + PI) / (2 * PI) * nlon));
      for (int dn = -1; dn <= 1; ++dn) {
        const int ln = ((lo + dn) % nlon + nlon) % nlon;
        const std::uint32_t a = std::uint32_t(lc + 5000), b = std::uint32_t(ln);
        if (h01(a, b, 1) > 0.42)
          continue;
        const real sl = (lc + h01(a, b, 2)) * D,
                   so = -PI + (ln + h01(a, b, 3)) / nlon * 2 * PI;
        const Vec3 s(std::cos(sl) * std::cos(so), std::cos(sl) * std::sin(so),
                     std::sin(sl));
        const Vec3 df = d - s;
        const real a2 = df.dot(df), sg = 0.0024;
        const real br = std::pow(h01(a, b, 4), 3.0) * 6.0 + 0.15;
        const real I = br * std::exp(-a2 / (2 * sg * sg));
        if (I < 1e-3)
          continue;
        const Rgb c = blackbody((3000 + 9000 * h01(a, b, 5)) * shift);
        o.r += float(I * c.r);
        o.g += float(I * c.g);
        o.b += float(I * c.b);
      }
    }
    return o;
  }
  // emissione del disco in un dato istante di tempo COORDINATO
  Rgb diskEmission(const Hit &h, real tCoord, real alphaObs) const {
    (void)alphaObs;
    const real Om = std::sqrt(S.M / (h.r * h.r * h.r)),
               rot = h.phiDisk - Om * tCoord;
    const int NA = 64;
    const real rho = std::log(h.r / S.rIn) * 7.0, ang = rot / (2 * PI) * NA;
    const real n = 0.6 * noisePeriodic(rho, ang, NA) +
                   0.4 * noisePeriodic(rho * 2.3 + 11, ang * 2.0, 2 * NA);
    const real tex = 0.45 + 1.1 * n;
    const real Tem = S.Tin * std::pow(S.rIn / h.r, 0.75);
    const real I = std::pow(h.g, 3.5) * std::pow(S.rIn / h.r, 1.5) * tex;
    Rgb c = blackbody(Tem * h.g);
    // faro: sorgente puntiforme che orbita e pulsa a nu (tempo coordinato)
    const real bphi =
        S.beaconPhi0 +
        std::sqrt(S.M / (S.beaconR * S.beaconR * S.beaconR)) * tCoord;
    const real d2 = h.r * h.r + S.beaconR * S.beaconR -
                    2 * h.r * S.beaconR * std::cos(h.phiDisk - bphi);
    const real w = std::exp(-d2 / (2 * S.beaconSigma * S.beaconSigma));
    const real blink = 0.5 * (1 + std::sin(2 * PI * S.beaconNu * tCoord));
    const real B = S.beaconBoost * w * blink * std::pow(h.g, 3.5);
    return {float(I * c.r + B), float(I * c.g + B * 0.95),
            float(I * c.b + B * 0.8)};
  }
  // un frame: tau = tempo proprio dell'osservatore all'inizio dell'esposizione
  Image render(const Camera &cam, const ObserverCapacity &cap, const Plan &plan,
               real tau, std::uint32_t frame = 0) {
    const real alpha = schw::lapse(S.M, cam.r);
    const RayTable &T = table(cam.r);
    Image img(plan.width, plan.height);
    Vec3 f, right, up;
    cam.basis(f, right, up);
    const real th = std::tan(cam.fovY / 2),
               asp = real(plan.width) / plan.height;
    const int ns = std::max(1, cap.subsamples), W = plan.width, H = plan.height;
    const real levels = std::pow(2.0, cap.bits) - 1;
    std::atomic<int> next{0};
    auto work = [&] {
      for (int y; (y = next++) < H;)
        for (int x = 0; x < W; ++x) {
          const real sx = ((x + 0.5) / W * 2 - 1) * asp * th,
                     sy = (1 - (y + 0.5) / H * 2) * th;
          const Vec3 n = (f + right * sx + up * sy).normalized();
          const PixelGeom G = trace(T, cam, n, alpha);
          Rgb acc;
          const Rgb skyc = G.sky ? sky(G.skyDir, 1.0 / alpha) : Rgb{};
          for (int k = 0; k < ns;
               ++k) { // integrazione temporale sul Te del sensore (tempo
                      // proprio -> coordinato)
            const real tc = (tau + (k + 0.5) / ns * cap.exposure) / alpha;
            real Tr = 1;
            Rgb s;
            for (int i = 0; i < G.nh; ++i) {
              const Rgb e = diskEmission(G.h[i], tc, alpha);
              s.r += float(Tr * e.r);
              s.g += float(Tr * e.g);
              s.b += float(Tr * e.b);
              Tr *= 0.4;
            }
            s.r += float(Tr * skyc.r);
            s.g += float(Tr * skyc.g);
            s.b += float(Tr * skyc.b);
            acc.r += s.r / ns;
            acc.g += s.g / ns;
            acc.b += s.b / ns;
          }
          float *o = img.at(x, y);
          const float c[3] = {acc.r, acc.g, acc.b};
          for (int ch = 0; ch < 3;
               ++ch) { // sensore: guadagno, tone map, rumore di lettura,
                       // quantizzazione Q
            real v = c[ch] * cap.gain;
            v = v / (1 + v);
            v = std::pow(std::max(v, 0.0), 1 / 2.2);
            v += cap.readNoise *
                 (h01(std::uint32_t(x), std::uint32_t(y) * 3 + ch, frame) * 2 -
                  1) *
                 1.7;
            v = std::clamp(v, 0.0, 1.0);
            o[ch] = float(std::round(v * levels) / levels);
          }
        }
    };
    std::vector<std::thread> threads_;
    for (unsigned t = 0; t < threads; ++t)
      threads_.emplace_back(work);
    for (auto &t : threads_)
      t.join();
    return img;
  }

private:
  std::unique_ptr<RayTable> tab_;
};

// ================================================================== gioco
// [T] Meccanica NQG: l'osservatore ha conoscenza p su d ipotesi a ciascun
// livello. La conoscenza collassa col flusso V11 con lambda che avanza col
// tempo PROPRIO moltiplicato dall'uso di capacita' (f_s*K/C_ops). Meno
// capacita' usata -> meno informazione.
struct Input {
  real thrustR = 0, yawRate = 0, pitchRate = 0, orbitRate = 0;
};
struct Game {
  Scene scene;
  Camera cam;
  ObserverCapacity cap;
  Plan plan;
  Renderer rend;
  real vr = 0, fuel = 100, hull = 100, tau = 0, tCoord = 0, score = 0,
       lambdaRate = 0.35;
  bool over = false;
  std::string status = "in orbita statica";
  hypothesis::NestedLevels levels;
  std::vector<real> S0;
  std::uint32_t frameIdx = 0;
  explicit Game(Scene s = {}, ObserverCapacity c = {})
      : scene(s), cap(c), rend(s) {
    plan = negotiate(cap);
    cam.r = 30;
    levels.addLevel(10, {.30, .25, .20, .15, .10});
    levels.addLevel(40, {.28, .26, .20, .16, .10});
    levels.addLevel(160, {.24, .23, .21, .17, .15});
    for (std::size_t i = 0; i < levels.size(); ++i)
      S0.push_back(levels.entropy(i));
  }
  real alpha() const { return schw::lapse(scene.M, cam.r); }
  real hoverAcceleration() const {
    return scene.M / (cam.r * cam.r * alpha());
  } // accel. propria per restare fermi [F]
  real tidal() const { return std::sqrt(info::kretschmann(scene.M, cam.r)); }
  real knowledge() const {
    real s = 0;
    for (std::size_t i = 0; i < levels.size(); ++i)
      s += S0[i] - levels.entropy(i);
    return s;
  }
  // avanza di dtau secondi PROPRI
  void step(real dtau, const Input &in) {
    if (over)
      return;
    const real a = alpha();
    tau += dtau;
    tCoord += dtau / a;
    vr += in.thrustR * 0.8 * dtau;
    cam.r += vr * dtau;
    vr *= (1 - 0.8 * dtau);
    if (cam.r <= 2.1 * scene.M) {
      over = true;
      status = "oltre l'orizzonte: segnale perso";
      cam.r = 2.1 * scene.M;
      return;
    }
    cam.yaw += in.yawRate * dtau;
    cam.pitch = std::clamp(cam.pitch + in.pitchRate * dtau, -1.2, 1.2);
    cam.phi += in.orbitRate * dtau;
    fuel -=
        (std::abs(in.thrustR) * 0.5 + 0.02 * hoverAcceleration() * 25) * dtau;
    const real tid = tidal();
    if (tid > 0.02)
      hull -= (tid - 0.02) * 400 * dtau;
    if (fuel <= 0) {
      fuel = 0;
      over = true;
      status = "carburante esaurito";
    }
    if (hull <= 0) {
      hull = 0;
      over = true;
      status = "scafo distrutto dalle maree";
    }
    plan = negotiate(cap);
    const real dl = lambdaRate * plan.usage * dtau;
    for (std::size_t i = 0; i < levels.size(); ++i)
      levels.setLambda(
          i, levels.entropy(i) >= 0 ? lambdaOf(i) + dl * (1.0 + 0.5 * i) : 0);
    score = knowledge();
  }
  Image frame() {
    rend.S = scene;
    Image im = rend.render(cam, cap, plan, tau, frameIdx++);
    im.bar(6, im.h - 30, im.w / 3, 6, fuel / 100.0, {0.2f, 0.9f, 0.3f});
    im.bar(6, im.h - 21, im.w / 3, 6, hull / 100.0, {0.9f, 0.3f, 0.2f});
    im.bar(6, im.h - 12, im.w / 3, 6, std::min(1.0, score / 3.0),
           {0.3f, 0.5f, 1.0f});
    im.bar(im.w - im.w / 3 - 6, im.h - 12, im.w / 3, 6, plan.usage,
           {0.9f, 0.8f, 0.2f});
    return im;
  }
  real lambdaOf(std::size_t i) const { return lam_.size() > i ? lam_[i] : 0; }

private:
  std::vector<real> lam_ = std::vector<real>(3, 0.0);

public:
  void advanceLevels(real dl) {
    for (std::size_t i = 0; i < lam_.size(); ++i) {
      lam_[i] += dl * (1.0 + 0.5 * i);
      levels.setLambda(i, lam_[i]);
    }
  }
};

} // namespace engine
} // namespace nqg
#endif