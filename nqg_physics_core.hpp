// ============================================================================
//  nqg_physics_core.hpp  -  "physics core" autonomo (solo STL, C++17)
//  Nested Quantum Geometry (NQG) + GR di Schwarzschild + LQG/LQC + osservatore
//
//  CRONOLOGIA RICOSTRUITA (i timestamp dei file sono identici: l'ordine e'
//  dedotto dai riferimenti interni tra documenti)
//   1. livelli-annidati-LQG_2.pdf / originale_trascritto.txt   paper unificato
//   V1-V12, X1-X10
//   2. dossie.txt (+ plus.txt = 1+2 concatenati) +
//   redshift_observer_simulator.py
//   3. geometria_stereografica.txt                              sfera di
//   Riemann, test spettrale
//   4. addendum1.txt                                            V11 = flusso
//   Fisher-Rao, LQC, redshift
//   5. verifica_completa.py                                     verifica
//   indipendente (trova errori)
//   6. addendum2.txt                                            consolidato (ha
//   ancora 2(N+1)/(N+4))
//   7. addendum3.txt                                            corregge in
//   4(N+1)/(N+4), principio di chiusura
//   8. verifica_2_modello_minimale.py                           T1-T4: V11 <=>
//   famiglia Gibbs/TFD
//
//  CLASSI EPISTEMICHE (dai documenti):  [M] matematica  [N] numerica
//  [F] fisica standard  [T] ipotesi del modello  [D] da derivare.
//  Tutto cio' che e' [T]/[D] sta in namespace nqg::hypothesis ed e' separato.
//
//  OTTIMIZZAZIONI ALGORITMICHE (misurate in test_nqg_core.cpp):
//   - V11 in forma chiusa nel dominio log: O(n), nessuna ODE, nessuna deriva,
//     composizione esatta (semigruppo), stabile per e^lambda enorme.
//   - Stimatore di frequenza lag-1: O(n), senza unwrap della fase e senza fit.
//   - Entropia di stato puro bipartito dalla matrice di Gram del lato piu'
//     piccolo: costo O(min(dA,dB)^3) invece di O(max^3).
//   - Propagatore spettrale: una diagonalizzazione, poi ogni t costa O(n^2).
//   - Integrale del tempo proprio con sostituzione r=2M sin^2(theta):
//     toglie la singolarita' integrabile, Simpson converge senza adattivita'.
//   - Catalan esatto con riduzione per gcd (niente overflow intermedio).
//   - Dormand-Prince 5(4) adattivo con FSAL per le geodetiche.
// ============================================================================
#ifndef NQG_PHYSICS_CORE_HPP
#define NQG_PHYSICS_CORE_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

namespace nqg {

using real = double;
using cplx = std::complex<double>;
constexpr real PI = 3.14159265358979323846264338327950288;
constexpr real NEG_INF = -std::numeric_limits<real>::infinity();

// ---------------------------------------------------------------- utilita'
struct KahanSum { // somma compensata
  real s = 0, c = 0;
  void add(real x) {
    real y = x - c, t = s + y;
    c = (t - s) - y;
    s = t;
  }
  real value() const { return s; }
};

inline real logSumExp(const real *x, std::size_t n) {
  real m = NEG_INF;
  for (std::size_t i = 0; i < n; ++i)
    m = std::max(m, x[i]);
  if (m == NEG_INF)
    return NEG_INF;
  KahanSum k;
  for (std::size_t i = 0; i < n; ++i)
    k.add(std::exp(x[i] - m));
  return m + std::log(k.value());
}

// ---------------------------------------------------------------- costanti
namespace consts {
constexpr real G = 6.67430e-11, c = 299792458.0, hbar = 1.054571817e-34;
constexpr real Mpc = 3.0857e22;           // valore usato nei verify.py
constexpr real H0_default = 67.4e3 / Mpc; // [s^-1]  ASSUNTO dal paper
constexpr real gamma_BI = 0.2375;         // Barbero-Immirzi (entropia LQG)
inline real planckLength() { return std::sqrt(G * hbar / (c * c * c)); }
inline real planckMass() { return std::sqrt(hbar * c / G); }
} // namespace consts

// =========================================================== V1-V3, V8: info
namespace info {
using namespace consts;
inline real schwarzschildRadiusE(real E) { return 2 * G * E / std::pow(c, 4); }
inline real entropyBH(real E) {
  return 4 * PI * G * E * E / (hbar * std::pow(c, 5));
}
inline real entropyBekenstein(real R, real E) {
  return 2 * PI * R * E / (hbar * c);
}
inline real capacityK(real I, real R, real E) {
  return I * hbar * c / (2 * PI * R * E);
}
inline real radiusAtK1(real I, real E) { return I * hbar * c / (2 * PI * E); }
// [M] R(K=1)/Rs = I/S_BH : per I<S_BH l'orizzonte "precede" la saturazione
inline real horizonPrecedence(real I, real E) { return I / entropyBH(E); }
inline real comptonRadius(real M) { return hbar / (M * c); }
inline real schwarzschildRadiusM(real M) { return 2 * G * M / (c * c); }
inline real crossingMass() { return planckMass() / std::sqrt(2.0); }
inline real crossingRadius() { return std::sqrt(2.0) * planckLength(); }
inline real kretschmann(real M, real r) {
  return 48 * M * M / std::pow(r, 6);
} // G=c=1
} // namespace info

// =========================================== V4-V7, redshift: Schwarzschild
namespace schw { // unita' geometriche G=c=1 [F]
inline real lapse(real M, real r) {
  if (r <= 2 * M)
    throw std::domain_error("osservatore statico: serve r>2M");
  return std::sqrt(1 - 2 * M / r);
}
inline real observedFrequency(real nu0, real alpha) { return nu0 / alpha; }
// 1+z = alpha_o/alpha_e  (statici)
inline real onePlusZ(real M, real r_emit, real r_obs) {
  return lapse(M, r_obs) / lapse(M, r_emit);
}
inline real nyquistMin(real nu) { return 2 * nu; }

// tau = int_0^{2M} dr / sqrt(2M/r-1) = pi M  [M]; forma chiusa e quadratura.
inline real properTimeHorizonToSingularity(real M) { return PI * M; }
// r = 2M sin^2 t  =>  integrando 4M sin^2 t (liscio): Simpson composito.
inline real properTimeQuadrature(real M, int n = 512) {
  if (n % 2)
    ++n;
  const real a = 0, b = PI / 2, h = (b - a) / n;
  auto f = [&](real t) {
    real s = std::sin(t);
    return 4 * M * s * s;
  };
  real s = f(a) + f(b);
  for (int i = 1; i < n; ++i)
    s += f(a + i * h) * (i % 2 ? 4 : 2);
  return s * h / 3;
}
// caduta radiale libera da fermo in r0 (cicloide), tempo proprio fino a r<=r0.
// r0=2M, r=0 -> pi M (coincide con V6).
inline real freeFallProperTime(real M, real r0, real r) {
  const real eta = std::acos(std::clamp(2 * r / r0 - 1, -1.0, 1.0));
  return std::sqrt(r0 * r0 * r0 / (8 * M)) * (eta + std::sin(eta));
}
inline real energyPerMass(real M, real r, real vr, real L) {
  return std::sqrt(vr * vr + (1 - 2 * M / r) * (1 + L * L / (r * r)));
}
inline real circularL(real M, real r) {
  return std::sqrt(M * r * r / (r - 3 * M));
}
} // namespace schw

// ------------------------------------------------ Dormand-Prince 5(4), FSAL
template <std::size_t N> struct DPResult {
  std::array<real, N> y;
  std::size_t accepted = 0, rejected = 0;
};

template <std::size_t N, class F>
DPResult<N> integrateDP45(F &&f, std::array<real, N> y, real t0, real t1,
                          real rtol = 1e-10, real atol = 1e-12, real h = 0,
                          std::size_t maxSteps = 50'000'000) {
  using V = std::array<real, N>;
  const real dir = t1 >= t0 ? 1 : -1;
  DPResult<N> out;
  real t = t0;
  if (h == 0)
    h = std::abs(t1 - t0) * 1e-3;
  h *= dir;
  V k1 = f(t, y), k2, k3, k4, k5, k6, k7, yn;
  while ((t1 - t) * dir > 0 && out.accepted + out.rejected < maxSteps) {
    if ((t + h - t1) * dir > 0)
      h = t1 - t;
    auto st = [&](const V &a, real c1, const V &b1, real c2 = 0,
                  const V *b2 = nullptr, real c3 = 0, const V *b3 = nullptr,
                  real c4 = 0, const V *b4 = nullptr, real c5 = 0,
                  const V *b5 = nullptr) {
      V r = y;
      for (std::size_t i = 0; i < N; ++i) {
        real s = c1 * b1[i];
        if (b2)
          s += c2 * (*b2)[i];
        if (b3)
          s += c3 * (*b3)[i];
        if (b4)
          s += c4 * (*b4)[i];
        if (b5)
          s += c5 * (*b5)[i];
        r[i] = a[i] + h * s;
      }
      return r;
    };
    k2 = f(t + h / 5, st(y, 1.0 / 5, k1));
    k3 = f(t + 3 * h / 10, st(y, 3.0 / 40, k1, 9.0 / 40, &k2));
    k4 = f(t + 4 * h / 5, st(y, 44.0 / 45, k1, -56.0 / 15, &k2, 32.0 / 9, &k3));
    k5 = f(t + 8 * h / 9, st(y, 19372.0 / 6561, k1, -25360.0 / 2187, &k2,
                             64448.0 / 6561, &k3, -212.0 / 729, &k4));
    k6 = f(t + h, st(y, 9017.0 / 3168, k1, -355.0 / 33, &k2, 46732.0 / 5247,
                     &k3, 49.0 / 176, &k4, -5103.0 / 18656, &k5));
    for (std::size_t i = 0; i < N; ++i)
      yn[i] = y[i] + h * (35.0 / 384 * k1[i] + 500.0 / 1113 * k3[i] +
                          125.0 / 192 * k4[i] - 2187.0 / 6784 * k5[i] +
                          11.0 / 84 * k6[i]);
    k7 = f(t + h, yn);
    real err = 0;
    for (std::size_t i = 0; i < N; ++i) {
      real e = h * ((35.0 / 384 - 5179.0 / 57600) * k1[i] +
                    (500.0 / 1113 - 7571.0 / 16695) * k3[i] +
                    (125.0 / 192 - 393.0 / 640) * k4[i] +
                    (-2187.0 / 6784 + 92097.0 / 339200) * k5[i] +
                    (11.0 / 84 - 187.0 / 2100) * k6[i] - 1.0 / 40 * k7[i]);
      err = std::max(
          err, std::abs(e) /
                   (atol + rtol * std::max(std::abs(y[i]), std::abs(yn[i]))));
    }
    if (err <= 1) {
      t += h;
      y = yn;
      k1 = k7;
      ++out.accepted;
    } else
      ++out.rejected;
    h *= std::min(5.0,
                  std::max(0.2, 0.9 * std::pow(std::max(err, 1e-10), -0.2)));
  }
  out.y = y;
  return out;
}

namespace schw {
// geodetica timelike equatoriale, parametro = tempo proprio. y={r, vr, phi}.
// d2r/dtau2 = -M/r^2 + L^2/r^3 - 3 M L^2/r^4 (regolare anche oltre r=2M).
inline std::array<real, 3> geodesicRhs(real M, real L,
                                       const std::array<real, 3> &y) {
  const real r = y[0], r2 = r * r;
  return {y[1], -M / r2 + L * L / (r2 * r) - 3 * M * L * L / (r2 * r2), L / r2};
}
inline DPResult<3> geodesic(real M, real L, std::array<real, 3> y0, real tau,
                            real rtol = 1e-11) {
  return integrateDP45<3>(
      [&](real, const std::array<real, 3> &y) { return geodesicRhs(M, L, y); },
      y0, 0, tau, rtol, 1e-13);
}
} // namespace schw

// ====================================================== sfera di Riemann
namespace stereo {
struct Vec3 {
  real x, y, z;
};
inline Vec3 project(cplx z) { // Pi(z), sfera unitaria, polo nord = infinito
  const real m2 = std::norm(z), d = 1 + m2;
  return {2 * z.real() / d, 2 * z.imag() / d, (m2 - 1) / d};
}
inline cplx unproject(const Vec3 &n) {
  return {n.x / (1 - n.z), n.y / (1 - n.z)};
}
inline real conformalFactor(cplx z) {
  return 2 / (1 + std::norm(z));
} // ds = f |dz|
inline real distanceToNorth(cplx z) { return 2 * std::atan(1 / std::abs(z)); }
// carta ADIMENSIONALE; il raggio fisico entra solo come r_N^2 (correzione T4):
inline real areaElementPhysical(real rN, cplx z) {
  real f = conformalFactor(z);
  return rN * rN * f * f;
}
} // namespace stereo

// ================================================================== LQG
namespace lqg {
using consts::gamma_BI;
inline real areaOfPunctures(const std::vector<real> &js,
                            real gamma = gamma_BI) { // unita' lP^2
  KahanSum k;
  for (real j : js)
    k.add(std::sqrt(j * (j + 1)));
  return 8 * PI * gamma * k.value();
}
inline real areaHalf(real gamma = gamma_BI) {
  return 4 * PI * std::sqrt(3.0) * gamma;
}
inline real radiusFromN(real N, real gamma = gamma_BI) {
  return std::sqrt(std::sqrt(3.0) * gamma * N);
}
inline real
rhoCrit(real gamma = gamma_BI) { // in rho_Pl [N] dipende dallo schema
  return std::sqrt(3.0) / (32 * PI * PI * gamma * gamma * gamma);
}
inline real rhoCrossing() { return 3 / (16 * PI); }
inline real puncturesInArea(real A, real gamma = gamma_BI) {
  return A / areaHalf(gamma);
}
inline real entropyFractionHalf(real gamma = gamma_BI) {
  return std::log(2.0) / (PI * std::sqrt(3.0) * gamma);
}

inline real logCatalan(std::size_t k) {
  return std::lgamma(2.0 * k + 1) - std::lgamma(k + 1.0) - std::lgamma(k + 2.0);
}
inline std::uint64_t
catalanExact(std::size_t k) { // C_{n+1}=C_n*(4n+2)/(n+2), con gcd
  if (k > 35)
    throw std::overflow_error("Catalan(k>35) non entra in uint64");
  std::uint64_t a = 1;
  for (std::size_t n = 0; n < k; ++n) {
    std::uint64_t b = 4 * n + 2, d = n + 2;
    std::uint64_t g1 = std::gcd(a, d);
    a /= g1;
    d /= g1;
    std::uint64_t g2 = std::gcd(b, d);
    b /= g2;
    d /= g2;
    a *= b; // d==1 per costruzione
  }
  return a;
}
inline std::uint64_t
singletDimension(std::size_t N) { // N spin-1/2 -> Catalan(N/2) [M]
  if (N % 2)
    return 0;
  return catalanExact(N / 2);
}
inline real singletRatio(real N) {
  return 4 * (N + 1) / (N + 4);
} // D_{N+2}/D_N (addendum 3)
inline real logDimAsymptotic(real N) {
  return N * std::log(2.0) - 1.5 * std::log(N) + 1.5 * std::log(2.0) -
         0.5 * std::log(PI);
}
} // namespace lqg

// ============================================================ algebra lineare
namespace linalg {
struct CMat {
  std::size_t r = 0, c = 0;
  std::vector<cplx> a;
  CMat() = default;
  CMat(std::size_t r_, std::size_t c_) : r(r_), c(c_), a(r_ * c_) {}
  cplx &operator()(std::size_t i, std::size_t j) { return a[i * c + j]; }
  const cplx &operator()(std::size_t i, std::size_t j) const {
    return a[i * c + j];
  }
  static CMat identity(std::size_t n) {
    CMat m(n, n);
    for (std::size_t i = 0; i < n; ++i)
      m(i, i) = 1;
    return m;
  }
};
inline CMat dagger(const CMat &A) {
  CMat B(A.c, A.r);
  for (std::size_t i = 0; i < A.r; ++i)
    for (std::size_t j = 0; j < A.c; ++j)
      B(j, i) = std::conj(A(i, j));
  return B;
}
inline CMat mul(const CMat &A, const CMat &B) { // ordine i-k-j (cache friendly)
  CMat C(A.r, B.c);
  for (std::size_t i = 0; i < A.r; ++i)
    for (std::size_t k = 0; k < A.c; ++k) {
      const cplx aik = A(i, k);
      if (aik == cplx(0))
        continue;
      for (std::size_t j = 0; j < B.c; ++j)
        C(i, j) += aik * B(k, j);
    }
  return C;
}
// Jacobi ciclico per matrici hermitiane (fase + rotazione reale). A = V diag(w)
// V^dagger.
inline void hermitianEigen(CMat A, std::vector<real> &w, CMat &V,
                           real tol = 1e-14, int maxSweeps = 80) {
  const std::size_t n = A.r;
  if (A.c != n)
    throw std::invalid_argument("matrice non quadrata");
  V = CMat::identity(n);
  for (int sweep = 0; sweep < maxSweeps; ++sweep) {
    real off = 0, dg = 0;
    for (std::size_t p = 0; p < n; ++p) {
      dg += std::norm(A(p, p));
      for (std::size_t q = p + 1; q < n; ++q)
        off += std::norm(A(p, q));
    }
    if (off == 0 || off <= tol * tol * (dg + off))
      break;
    for (std::size_t p = 0; p + 1 < n; ++p)
      for (std::size_t q = p + 1; q < n; ++q) {
        const cplx b = A(p, q);
        const real ab = std::abs(b);
        if (ab < 1e-300)
          continue;
        const real th = (A(q, q).real() - A(p, p).real()) / (2 * ab);
        const real t =
            (th >= 0 ? 1.0 : -1.0) / (std::abs(th) + std::sqrt(th * th + 1));
        const real c = 1 / std::sqrt(t * t + 1), s = t * c;
        const cplx em = std::conj(b) / ab;
        const cplx Upp = c, Upq = s, Uqp = -s * em, Uqq = c * em;
        for (std::size_t k = 0; k < n; ++k) {
          const cplx x = A(k, p), y = A(k, q);
          A(k, p) = x * Upp + y * Uqp;
          A(k, q) = x * Upq + y * Uqq;
          const cplx vx = V(k, p), vy = V(k, q);
          V(k, p) = vx * Upp + vy * Uqp;
          V(k, q) = vx * Upq + vy * Uqq;
        }
        for (std::size_t k = 0; k < n; ++k) {
          const cplx x = A(p, k), y = A(q, k);
          A(p, k) = std::conj(Upp) * x + std::conj(Uqp) * y;
          A(q, k) = std::conj(Upq) * x + std::conj(Uqq) * y;
        }
      }
  }
  std::vector<std::size_t> idx(n);
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&](std::size_t i, std::size_t j) {
    return A(i, i).real() < A(j, j).real();
  });
  w.resize(n);
  CMat Vs(n, n);
  for (std::size_t j = 0; j < n; ++j) {
    w[j] = A(idx[j], idx[j]).real();
    for (std::size_t i = 0; i < n; ++i)
      Vs(i, j) = V(i, idx[j]);
  }
  V = std::move(Vs);
}
} // namespace linalg

// ====================================================== stati quantistici
namespace qm {
using linalg::CMat;
inline real vonNeumann(const std::vector<real> &w) {
  KahanSum k;
  for (real x : w)
    if (x > 0)
      k.add(-x * std::log(x));
  return k.value();
}
// spettro di Schmidt (autovalori di rho_A) di |psi> in H_A (x) H_B, ordine
// decrescente. Usa la Gram del lato piu' piccolo: S(rho_A)=S(rho_B) per stati
// puri [V10].
inline std::vector<real> schmidtSpectrum(const std::vector<cplx> &psi,
                                         std::size_t dA, std::size_t dB) {
  if (psi.size() != dA * dB)
    throw std::invalid_argument("dimensioni incoerenti");
  const bool useA = dA <= dB;
  const std::size_t d = useA ? dA : dB;
  CMat rho(d, d);
  for (std::size_t i = 0; i < d; ++i)
    for (std::size_t j = i; j < d; ++j) {
      cplx s = 0;
      if (useA)
        for (std::size_t k = 0; k < dB; ++k)
          s += psi[i * dB + k] * std::conj(psi[j * dB + k]);
      else
        for (std::size_t k = 0; k < dA; ++k)
          s += psi[k * dB + i] * std::conj(psi[k * dB + j]);
      rho(i, j) = s;
      rho(j, i) = std::conj(s);
    }
  std::vector<real> w;
  CMat V;
  linalg::hermitianEigen(rho, w, V);
  for (real &x : w)
    x = std::max(x, 0.0);
  std::reverse(w.begin(), w.end());
  return w;
}
inline real entanglementEntropy(const std::vector<cplx> &psi, std::size_t dA,
                                std::size_t dB) {
  return vonNeumann(schmidtSpectrum(psi, dA, dB));
}
// rho_A completa (quando serve la matrice, non solo lo spettro)
inline CMat reducedDensityA(const std::vector<cplx> &psi, std::size_t dA,
                            std::size_t dB) {
  CMat rho(dA, dA);
  for (std::size_t i = 0; i < dA; ++i)
    for (std::size_t j = 0; j < dA; ++j) {
      cplx s = 0;
      for (std::size_t k = 0; k < dB; ++k)
        s += psi[i * dB + k] * std::conj(psi[j * dB + k]);
      rho(i, j) = s;
    }
  return rho;
}
// U = exp(-i H t) = V e^{-iEt} V^dagger, dopo una sola diagonalizzazione: ogni
// t costa O(n^2)
class SpectralPropagator {
public:
  explicit SpectralPropagator(const CMat &H) {
    linalg::hermitianEigen(H, E_, V_);
  }
  void prepare(const std::vector<cplx> &psi0) {
    const std::size_t n = E_.size();
    c_.assign(n, 0);
    for (std::size_t j = 0; j < n; ++j)
      for (std::size_t k = 0; k < n; ++k)
        c_[j] += std::conj(V_(k, j)) * psi0[k];
  }
  std::vector<cplx> state(real t) const {
    const std::size_t n = E_.size();
    std::vector<cplx> ph(n), out(n, 0);
    for (std::size_t j = 0; j < n; ++j)
      ph[j] = c_[j] * std::polar(1.0, -E_[j] * t);
    for (std::size_t k = 0; k < n; ++k)
      for (std::size_t j = 0; j < n; ++j)
        out[k] += V_(k, j) * ph[j];
    return out;
  }
  const std::vector<real> &energies() const { return E_; }

private:
  std::vector<real> E_;
  CMat V_;
  std::vector<cplx> c_;
};
// Page-Wootters discreto V9: |Psi> = N^{-1/2} sum_k |k>_C (x) U(k dt)|psi0>_S,
// E_j = 2 pi j/(N dt)
struct PageWootters {
  std::size_t N, m;
  real dt;
  std::vector<cplx> psi; // indice k*m+j
  PageWootters(std::size_t N_, std::size_t m_, real dt_ = 1)
      : N(N_), m(m_), dt(dt_), psi(N_ * m_) {
    for (std::size_t k = 0; k < N; ++k)
      for (std::size_t j = 0; j < m; ++j)
        psi[k * m + j] =
            std::polar(1.0, -energy(j) * k * dt) / std::sqrt(real(N) * real(m));
  }
  real energy(std::size_t j) const { return 2 * PI * j / (N * dt); }
  real fixedPointResidual() const { // || (T (x) U(dt)) Psi - Psi ||
    KahanSum s;
    for (std::size_t k = 0; k < N; ++k)
      for (std::size_t j = 0; j < m; ++j)
        s.add(std::norm(std::polar(1.0, -energy(j) * dt) *
                            psi[((k + N - 1) % N) * m + j] -
                        psi[k * m + j]));
    return std::sqrt(s.value());
  }
  real entropySystem() const {
    return entanglementEntropy(psi, N, m);
  } // = ln m
};
} // namespace qm

// ================================================================ V11
namespace v11 {
inline real entropy(const std::vector<real> &p) {
  KahanSum k;
  for (real x : p)
    if (x > 0)
      k.add(-x * std::log(x));
  return k.value();
}
inline real entropyFromLog(const std::vector<real> &lnp) {
  KahanSum k;
  for (real l : lnp)
    if (l > NEG_INF)
      k.add(-std::exp(l) * l);
  return k.value();
}
// dp_i/dlambda = sign * p_i (ln p_i + S)
inline std::vector<real> rhs(const std::vector<real> &p, int sign = +1) {
  const real S = entropy(p);
  std::vector<real> d(p.size(), 0);
  for (std::size_t i = 0; i < p.size(); ++i)
    if (p[i] > 0)
      d[i] = sign * p[i] * (std::log(p[i]) + S);
  return d;
}
inline real dSdLambda(const std::vector<real> &p) { // = -Var_p(ln p) <= 0 [M]
  const real S = entropy(p);
  KahanSum k;
  for (real x : p)
    if (x > 0) {
      real d = std::log(x) + S;
      k.add(x * d * d);
    }
  return -k.value();
}
// SOLUZIONE CHIUSA nel dominio log: ln p_i(l) = e^{sign*l} ln p_i(0) - lnZ.
// O(n), esatta.
inline void flowLog(std::vector<real> &lnp, real lambda, int sign = +1) {
  // FIX: clamp per evitare overflow di std::exp quando |lambda| e' grande.
  const real sc = std::exp(std::clamp(sign * lambda, -700.0, 700.0));
  for (real &l : lnp)
    if (l > NEG_INF)
      l *= sc;
  const real lz = logSumExp(lnp.data(), lnp.size());
  for (real &l : lnp)
    if (l > NEG_INF)
      l -= lz;
}
inline std::vector<real> toLog(const std::vector<real> &p) {
  std::vector<real> l(p.size());
  for (std::size_t i = 0; i < p.size(); ++i)
    l[i] = p[i] > 0 ? std::log(p[i]) : NEG_INF;
  return l;
}
inline std::vector<real> fromLog(const std::vector<real> &l) {
  std::vector<real> p(l.size());
  for (std::size_t i = 0; i < l.size(); ++i)
    p[i] = l[i] > NEG_INF ? std::exp(l[i]) : 0.0;
  return p;
}
inline std::vector<real> flow(const std::vector<real> &p0, real lambda,
                              int sign = +1) {
  auto l = toLog(p0);
  flowLog(l, lambda, sign);
  return fromLog(l);
}
// riferimento lento (RK4) usato solo per validare/benchmarkare la forma chiusa
inline std::vector<real> flowRK4(std::vector<real> p, real lambda,
                                 std::size_t steps, int sign = +1) {
  const real h = lambda / steps;
  const std::size_t n = p.size();
  std::vector<real> t(n);
  auto add = [&](const std::vector<real> &a, const std::vector<real> &k,
                 real s) {
    for (std::size_t i = 0; i < n; ++i)
      t[i] = a[i] + s * k[i];
    return t;
  };
  for (std::size_t s = 0; s < steps; ++s) {
    auto k1 = rhs(p, sign);
    auto k2 = rhs(add(p, k1, h / 2), sign);
    auto k3 = rhs(add(p, k2, h / 2), sign);
    auto k4 = rhs(add(p, k3, h), sign);
    for (std::size_t i = 0; i < n; ++i)
      p[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  }
  return p;
}
// V11 sugli autovalori di una matrice densita': autovettori fissi, spettro
// trasformato [spettrale]
inline linalg::CMat flowDensity(const linalg::CMat &rho, real lambda) {
  std::vector<real> w;
  linalg::CMat V;
  linalg::hermitianEigen(rho, w, V);
  for (real &x : w)
    x = std::max(x, 0.0);
  auto q = flow(w, lambda);
  const std::size_t n = w.size();
  linalg::CMat out(n, n);
  for (std::size_t k = 0; k < n; ++k)
    for (std::size_t i = 0; i < n; ++i) {
      const cplx a = q[k] * V(i, k);
      for (std::size_t j = 0; j < n; ++j)
        out(i, j) += a * std::conj(V(j, k));
    }
  return out;
}
// ln p(b) = s ln p(a) + c  (T1/T2): se s>0 con residuo~0 -> orbita V11 con
// e^{dLambda}=s
struct OrbitFit {
  real slope = NAN, intercept = NAN, maxResidual = NAN;
  std::size_t used = 0;
  bool ok = false;
};
inline OrbitFit fitOrbit(const std::vector<real> &pa,
                         const std::vector<real> &pb, real eps = 1e-10) {
  std::vector<real> x, y;
  for (std::size_t i = 0; i < pa.size(); ++i)
    if (pa[i] > eps && pb[i] > eps) {
      x.push_back(std::log(pa[i]));
      y.push_back(std::log(pb[i]));
    }
  OrbitFit f;
  f.used = x.size();
  if (x.size() < 2)
    return f;
  const real n = x.size(), mx = std::accumulate(x.begin(), x.end(), 0.0) / n,
             my = std::accumulate(y.begin(), y.end(), 0.0) / n;
  real sxx = 0, sxy = 0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    sxx += (x[i] - mx) * (x[i] - mx);
    sxy += (x[i] - mx) * (y[i] - my);
  }
  if (sxx < 1e-300)
    return f; // distribuzione uniforme: parametro non identificabile
  f.slope = sxy / sxx;
  f.intercept = my - f.slope * mx;
  f.maxResidual = 0;
  for (std::size_t i = 0; i < x.size(); ++i)
    f.maxResidual =
        std::max(f.maxResidual, std::abs(f.intercept + f.slope * x[i] - y[i]));
  f.ok = f.slope > 0;
  return f;
}
// residuo spettrale epsilon_n (geometria_stereografica §7), pesi w_ij uniformi
// se vuoti; rif = autovalore massimo
struct LevelResidual {
  real a = NAN, eps = NAN;
  bool valid = false;
};
inline LevelResidual levelResidual(const std::vector<real> &pn,
                                   const std::vector<real> &pn1,
                                   const std::vector<real> &w = {}) {
  LevelResidual r;
  if (pn.size() != pn1.size() || pn.size() < 3)
    return r;
  const std::size_t ref =
      std::size_t(std::max_element(pn.begin(), pn.end()) - pn.begin());
  real num = 0, den = 0, n1 = 0;
  for (std::size_t i = 0; i < pn.size(); ++i) {
    if (i == ref || pn[i] <= 0 || pn1[i] <= 0 || pn[ref] <= 0 || pn1[ref] <= 0)
      continue;
    const real wi = w.empty() ? 1.0 : w[i];
    const real ln = std::log(pn[i] / pn[ref]), l1 = std::log(pn1[i] / pn1[ref]);
    num += wi * ln * l1;
    den += wi * ln * ln;
    n1 += wi * l1 * l1;
  }
  if (den < 1e-300 || n1 < 1e-300)
    return r;
  r.a = num / den;
  real e2 = 0;
  for (std::size_t i = 0; i < pn.size(); ++i) {
    if (i == ref || pn[i] <= 0 || pn1[i] <= 0)
      continue;
    const real wi = w.empty() ? 1.0 : w[i];
    const real ln = std::log(pn[i] / pn[ref]), l1 = std::log(pn1[i] / pn1[ref]);
    e2 += wi * (l1 - r.a * ln) * (l1 - r.a * ln);
  }
  r.eps = std::sqrt(e2 / n1);
  r.valid = r.a > 0;
  return r;
}
// famiglia di Gibbs = orbita V11 con e^lambda = beta/beta0 (T2, H=-ln p0 fisso)
inline std::vector<real> gibbs(const std::vector<real> &E, real beta) {
  std::vector<real> l(E.size());
  for (std::size_t i = 0; i < E.size(); ++i)
    l[i] = -beta * E[i];
  const real lz = logSumExp(l.data(), l.size());
  for (real &x : l)
    x = std::exp(x - lz);
  return l;
}
inline real lambdaForBeta(real beta, real beta0) {
  return std::log(beta / beta0);
}
} // namespace v11

// ============================================================ osservatore
namespace observer {
struct Capacity { // C=(f_s,..., C_ops, K): f_s*K <= C_ops
  real f_sensor, C_ops, K_ops_per_sample;
  real fmax() const { return std::min(f_sensor, C_ops / K_ops_per_sample); }
  bool canResolve(real nu) const { return fmax() > schw::nyquistMin(nu); }
};
// campioni di un oscillatore coordinato nu0 letti da un clock locale di passo
// fisso dtau=1/fs
inline std::vector<cplx> sampleLocalClock(real alpha, real nu0, real fs,
                                          std::size_t n, real sigmaPhase = 0,
                                          std::uint64_t seed = 0) {
  std::mt19937_64 g(seed);
  std::normal_distribution<real> N01(0, 1);
  const real cyc = (nu0 / alpha) / fs; // cicli per campione (t_n = tau_n/alpha)
  std::vector<cplx> y(n);
  for (std::size_t k = 0; k < n; ++k) {
    real x = cyc * k;
    x -= std::floor(x); // riduzione di fase: niente perdita di precisione
    y[k] = std::polar(1.0, 2 * PI * x +
                               (sigmaPhase > 0 ? sigmaPhase * N01(g) : 0.0));
  }
  return y;
}
// stimatore lag-1: nu = arg(sum y[k+1] conj(y[k])) / (2 pi dtau). O(n), valido
// per |nu|<fs/2.
inline real lag1Frequency(const std::vector<cplx> &y, real dtau) {
  KahanSum re, im;
  for (std::size_t k = 0; k + 1 < y.size(); ++k) {
    cplx z = y[k + 1] * std::conj(y[k]);
    re.add(z.real());
    im.add(z.imag());
  }
  return std::atan2(im.value(), re.value()) / (2 * PI * dtau);
}
} // namespace observer

// ============================================ [T]/[D]: ipotesi del modello
namespace hypothesis {
// [T] alpha^2 = 1-K, K=Rs/r  (NON derivato dai livelli: reinserisce
// Schwarzschild per scelta)
inline real lapseFromK(real K) {
  if (K >= 1)
    throw std::domain_error("K>=1: oltre l'orizzonte");
  return std::sqrt(1 - K);
}
// [T] livelli annidati: ogni livello = N punture j=1/2 con spettro iniziale p0
// e parametro di flusso lambda. Stato a lambda calcolato dalla forma chiusa
// partendo da p0 (nessun errore di integrazione accumulato).
class NestedLevels {
public:
  explicit NestedLevels(real gamma = consts::gamma_BI) : gamma_(gamma) {}
  std::size_t addLevel(std::size_t N, const std::vector<real> &p0,
                       real lambda0 = 0) {
    lnp0_.push_back(v11::toLog(p0));
    N_.push_back(N);
    lam_.push_back(lambda0);
    return N_.size() - 1;
  }
  std::size_t size() const { return N_.size(); }
  std::size_t N(std::size_t i) const { return N_.at(i); }
  void setLambda(std::size_t i, real l) { lam_.at(i) = l; }
  void advanceAll(real dl) {
    for (real &l : lam_)
      l += dl;
  } // O(L), indipendente da dl
  std::vector<real> spectrum(std::size_t i) const {
    auto l = lnp0_.at(i);
    v11::flowLog(l, lam_[i]);
    return v11::fromLog(l);
  }
  real entropy(std::size_t i) const {
    auto l = lnp0_.at(i);
    v11::flowLog(l, lam_[i]);
    return v11::entropyFromLog(l);
  }
  real area(std::size_t i) const {
    return lqg::areaHalf(gamma_) * N_.at(i);
  } // lP^2
  real radius(std::size_t i) const {
    return lqg::radiusFromN(real(N_.at(i)), gamma_);
  } // lP
  v11::OrbitFit compatibility(std::size_t i, std::size_t j) const {
    return v11::fitOrbit(spectrum(i), spectrum(j));
  }

private:
  real gamma_;
  std::vector<std::vector<real>> lnp0_;
  std::vector<std::size_t> N_;
  std::vector<real> lam_;
};
} // namespace hypothesis

// ============================================================ contatto e
// attrito
namespace contact {
inline real combineFriction(real a, real b) {
  return std::sqrt(std::max(a, 0.0) * std::max(b, 0.0));
}
inline real coulombImpulse(real stick, real normal, real muS, real muK) {
  stick = std::max(stick, 0.0);
  normal = std::max(normal, 0.0);
  return stick <= muS * normal ? stick : muK * normal;
}
inline real stokesNumber(real rhoBody, real speed, real length, real mu) {
  return rhoBody * std::abs(speed) * length / (9.0 * std::max(mu, 1e-12));
}
inline real wetRestitution(real eDry, real St, real Stc = 10.0) {
  if (St <= Stc)
    return 0.0;
  return eDry * (1.0 - Stc / St);
}
inline real wetFrictionCoefficient(real muDry, real film, real roughness,
                                   real viscosity, real speed, real pressure,
                                   real length) {
  if (film <= 0.0 || speed <= 1e-9)
    return muDry;
  const real p = std::max(pressure, 1.0);
  const real hHd = std::sqrt(viscosity * speed * std::max(length, 1e-6) / p);
  const real lam = std::min(film, hHd) / std::max(roughness, 1e-9);
  const real phi = 1.0 / (1.0 + std::pow(lam / 2.0, 4.0));
  const real muHyd =
      std::min(muDry, viscosity * speed / (std::max(hHd, 1e-9) * p));
  return phi * muDry + (1.0 - phi) * muHyd;
}
inline real squeezeDiscCoefficient(real mu, real R, real h) {
  return 1.5 * PI * mu * std::pow(R, 4.0) / std::pow(std::max(h, 1e-9), 3.0);
}
inline real squeezeSphereCoefficient(real mu, real R, real h) {
  return 6.0 * PI * mu * R * R / std::max(h, 1e-9);
}
inline real rollingDeceleration(real crr, real g) { return crr * g; }
} // namespace contact

// ================================================================ fluidi
namespace fluid {

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
};

class ShallowFlow {
public:
  static constexpr int T = 16;
  static constexpr int SH = 4;
  static constexpr real SOLID_Z = 1.0e3;

  struct Cell {
    real h = 0, b = 0, u = 0, v = 0, n = 0.012;
    real tu = 0, tv = 0, fx = 0, fy = 0, s = 1, k = 0;
    bool solid = false;
  };
  struct Tile {
    std::array<Cell, T * T> c;
    int ti = 0, tj = 0, mark = 0;
    bool wet = false, run = false;
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
  real speedLimit = 30.0;
  int maxSubsteps = 24;
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

  void loadBed(Tile &t) {
    for (int lj = 0; lj < T; ++lj)
      for (int li = 0; li < T; ++li) {
        Cell &c = t.c[std::size_t(lj) * T + li];
        BedSample s =
            bedSampleAt(centerX(t.ti * T + li), centerY(t.tj * T + lj));
        c.solid = s.solid;
        c.b = s.solid ? SOLID_Z : s.z;
        c.n = s.manning;
        if (c.solid) {
          c.h = 0;
          c.u = c.v = 0;
        }
      }
  }

  void refreshBed() {
    for (Tile &t : tiles)
      loadBed(t);
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
    hasWet = false;
    lastMaxWave = 1.0;
    lastMaxDepth = 0.0;
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
  }

  void addVolume(real x, real y, real radius, real volume) {
    if (!(volume != 0.0) || !std::isfinite(volume))
      return;
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

  void addVolumeRing(real cx, real cy, real hx, real hy, bool circular,
                     real margin, real volume) {
    if (!(volume != 0.0) || !std::isfinite(volume))
      return;
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
      return;
    const real dh = volume / (real(sel.size()) * dx * dx);
    for (Cell *c : sel)
      c->h = std::max(0.0, c->h + std::max(dh, -0.5 * c->h));
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

  Sample sample(real x, real y) const {
    const real fi = (x - x0) / dx - 0.5, fj = (y - y0) / dx - 0.5;
    const int i0 = int(std::floor(fi)), j0 = int(std::floor(fj));
    const real fx = fi - i0, fy = fj - j0;
    Sample s;
    real w = 0, eta = 0, dep = 0, bd = 0, uu = 0, vv = 0;
    for (int dj = 0; dj < 2; ++dj)
      for (int di = 0; di < 2; ++di) {
        const Cell *c = at(i0 + di, j0 + dj);
        if (!c || c->solid || c->h <= visibleDepth)
          continue;
        const real wt = (di ? fx : 1 - fx) * (dj ? fy : 1 - fy);
        const Cell *e = at(i0 + di + 1, j0 + dj);
        const Cell *n = at(i0 + di, j0 + dj + 1);
        const real cu = 0.5 * (c->u + (e ? e->u : c->u));
        const real cv = 0.5 * (c->v + (n ? n->v : c->v));
        w += wt;
        eta += wt * (c->b + c->h);
        dep += wt * c->h;
        bd += wt * c->b;
        uu += wt * cu;
        vv += wt * cv;
      }
    if (w < 0.5)
      return s;
    s.wet = true;
    s.eta = eta / w;
    s.depth = dep / w;
    s.bed = bd / w;
    s.u = uu / w;
    s.v = vv / w;
    return s;
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

  real totalVolume() const {
    real v = 0;
    for (const Tile &t : tiles)
      if (t.wet)
        for (const Cell &c : t.c)
          v += c.h;
    return v * dx * dx;
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
    if (!(dtTotal > 0) || !std::isfinite(dtTotal) || tiles.empty())
      return;
    real t = 0;
    int n = 0;
    while (t < dtTotal - 1e-12) {
      if (n++ >= maxSubsteps)
        break;
      const real dt = std::min(dtTotal - t, stableDt());
      substep(dt);
      t += dt;
    }
    updateBounds();
  }

private:
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
    maxSubsteps = o.maxSubsteps;
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

  static real faceDepth(const Cell &L, const Cell &R) {
    if (L.solid || R.solid)
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
    const real c = std::max(lastMaxWave, 0.3);
    real dt = cfl * dx / c;
    const real om =
        std::sqrt(liquid.sigma / liquid.rho * std::max(lastMaxDepth, hCap)) *
        8.0 / (dx * dx);
    dt = std::min(dt, 1.0 / (om + 1e-9));
    return std::max(dt, 1e-4);
  }

  void growHalo() {
    for (std::size_t k = 0; k < tiles.size(); ++k) {
      if (!tiles[k].wet)
        continue;
      const int ti = tiles[k].ti, tj = tiles[k].tj;
      for (int dj = -1; dj <= 1; ++dj)
        for (int di = -1; di <= 1; ++di)
          ensureTile(ti + di, tj + dj);
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
        c.u = std::clamp(un, -speedLimit, speedLimit);
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
        c.v = std::clamp(vn, -speedLimit, speedLimit);
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

    updateFlags();
  }
};

} // namespace fluid

} // namespace nqg
#endif // NQG_PHYSICS_CORE_HPP