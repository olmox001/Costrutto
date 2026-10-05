// ============================================================================
//  nqg_matter_physics.hpp  -  Fisica della Materia Multi-Fase ed Elementare
//  FIX 2025 (dinamica):
//   - DEM solo per coppie sabbia-sabbia.
//   - Accoppiamento acqua-sabbia: buoyancy + drag.
//  FIX 2025c (performance, invarianti dinamici):
//   - ParticleSpatialHash (CSR, cellSize = sphRadius_) -> SPH/DEM/Coulomb
//     da O(N^2) a O(N*k). Stessa coppia-forza, stesso segno, stessa dinamica.
//   - Coulomb ristretto alle sole particelle cariche (C<<N).
//   - Precompute alpha, 1/h di SPH; pow(x,7) via exp(7 ln x) solo su ramo
//     water/wet.
// ============================================================================
#ifndef NQG_MATTER_PHYSICS_HPP
#define NQG_MATTER_PHYSICS_HPP

#include "nqg_air_physics.hpp"
#include "nqg_engine3d.hpp"
#include "nqg_physics_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

namespace nqg {
namespace matter {

using cleanroom::AirProperties;
using engine::Rgb;
using engine::Vec3;

enum class MatterType {
  Water,
  Sand,
  MolecularComplex,
  Electron,
  Positron,
  Nucleus
};

struct Particle {
  MatterType type = MatterType::Water;
  Vec3 pos = Vec3(0, 0, 1);
  Vec3 vel = Vec3(0, 0, 0);
  Vec3 force = Vec3(0, 0, 0);

  real mass = 0.01;
  real radius = 0.05;
  real charge = 0.0;
  Rgb color = {0.2f, 0.6f, 1.0f};

  real density = 1000.0;
  real pressure = 0.0;

  int complexId = -1;

  real comptonRadius() const {
    if (mass <= 1e-35)
      return 0.0;
    return info::comptonRadius(mass);
  }
};

struct MolecularBond {
  std::size_t i, j;
  real restLength = 0.15;
  real stiffness = 250.0;
  real damping = 4.0;
  real breakStress = 50.0;
  bool intact = true;
};

// ---------------------------------------------------------------------------
// Spatial hash compatto (CSR). Deterministico, nessuna allocazione per-query.
// La cella e' un cubo di lato cs; per ogni particella si visitano le 27 celle
// adiacenti. copre ogni interazione entro raggio cs (SPH h, DEM r_i+r_j << cs).
// ---------------------------------------------------------------------------
class ParticleSpatialHash {
public:
  void build(const std::vector<Particle> &ps, real cellSize) {
    cs_ = cellSize;
    if (ps.empty()) {
      nx_ = ny_ = nz_ = 0;
      cellOf_.clear();
      sorted_.clear();
      cellStart_.clear();
      return;
    }
    Vec3 mn = ps[0].pos, mx = ps[0].pos;
    for (const auto &p : ps) {
      if (p.pos.x < mn.x)
        mn.x = p.pos.x;
      if (p.pos.y < mn.y)
        mn.y = p.pos.y;
      if (p.pos.z < mn.z)
        mn.z = p.pos.z;
      if (p.pos.x > mx.x)
        mx.x = p.pos.x;
      if (p.pos.y > mx.y)
        mx.y = p.pos.y;
      if (p.pos.z > mx.z)
        mx.z = p.pos.z;
    }
    lo_ = mn - Vec3(cs_, cs_, cs_);
    nx_ = std::max(1, int((mx.x - mn.x) / cs_) + 3);
    ny_ = std::max(1, int((mx.y - mn.y) / cs_) + 3);
    nz_ = std::max(1, int((mx.z - mn.z) / cs_) + 3);
    const int nCells = nx_ * ny_ * nz_;

    cellOf_.resize(ps.size());
    std::vector<int> counts(nCells, 0);
    for (std::size_t i = 0; i < ps.size(); ++i) {
      const int c = cellIndexOf(ps[i].pos);
      cellOf_[i] = c;
      ++counts[c];
    }
    cellStart_.assign(nCells + 1, 0);
    for (int c = 0; c < nCells; ++c)
      cellStart_[c + 1] = cellStart_[c] + counts[c];
    sorted_.resize(ps.size());
    std::vector<int> cursor(cellStart_.begin(), cellStart_.end() - 1);
    for (std::size_t i = 0; i < ps.size(); ++i)
      sorted_[cursor[cellOf_[i]]++] = int(i);
  }

  // Tutti i vicini di idx tranne idx stesso.
  template <class F> void forAll(int idx, F &&fn) const {
    const int c = cellOf_[idx];
    const int iz = c / (nx_ * ny_);
    const int iy = (c / nx_) % ny_;
    const int ix = c % nx_;
    for (int dz = -1; dz <= 1; ++dz) {
      const int jz = iz + dz;
      if (jz < 0 || jz >= nz_)
        continue;
      for (int dy = -1; dy <= 1; ++dy) {
        const int jy = iy + dy;
        if (jy < 0 || jy >= ny_)
          continue;
        for (int dx = -1; dx <= 1; ++dx) {
          const int jx = ix + dx;
          if (jx < 0 || jx >= nx_)
            continue;
          const int cc = (jz * ny_ + jy) * nx_ + jx;
          for (int k = cellStart_[cc]; k < cellStart_[cc + 1]; ++k) {
            const int j = sorted_[k];
            if (j != idx)
              fn(j);
          }
        }
      }
    }
  }

  // Solo j > idx (coppie contate una volta).
  template <class F> void forHalf(int idx, F &&fn) const {
    const int c = cellOf_[idx];
    const int iz = c / (nx_ * ny_);
    const int iy = (c / nx_) % ny_;
    const int ix = c % nx_;
    for (int dz = -1; dz <= 1; ++dz) {
      const int jz = iz + dz;
      if (jz < 0 || jz >= nz_)
        continue;
      for (int dy = -1; dy <= 1; ++dy) {
        const int jy = iy + dy;
        if (jy < 0 || jy >= ny_)
          continue;
        for (int dx = -1; dx <= 1; ++dx) {
          const int jx = ix + dx;
          if (jx < 0 || jx >= nx_)
            continue;
          const int cc = (jz * ny_ + jy) * nx_ + jx;
          for (int k = cellStart_[cc]; k < cellStart_[cc + 1]; ++k) {
            const int j = sorted_[k];
            if (j > idx)
              fn(j);
          }
        }
      }
    }
  }

private:
  int cellIndexOf(const Vec3 &p) const {
    int ix = int((p.x - lo_.x) / cs_);
    if (ix < 0)
      ix = 0;
    else if (ix >= nx_)
      ix = nx_ - 1;
    int iy = int((p.y - lo_.y) / cs_);
    if (iy < 0)
      iy = 0;
    else if (iy >= ny_)
      iy = ny_ - 1;
    int iz = int((p.z - lo_.z) / cs_);
    if (iz < 0)
      iz = 0;
    else if (iz >= nz_)
      iz = nz_ - 1;
    return (iz * ny_ + iy) * nx_ + ix;
  }
  real cs_ = 0.25;
  Vec3 lo_;
  int nx_ = 0, ny_ = 0, nz_ = 0;
  std::vector<int> cellOf_;
  std::vector<int> cellStart_;
  std::vector<int> sorted_;
};

class MatterSimulator {
public:
  std::vector<Particle> particles;
  std::vector<MolecularBond> bonds;

  real sphRadius_ = 0.22;
  real waterRestDensity_ = 1000.0;
  real waterBulkModulus_ = 2000.0;
  real waterViscosity_ = 0.045;
  real waterSurfaceTension_ = 0.15;

  real sandFrictionCoeff_ = 0.65;
  real sandStiffness_ = 3500.0;
  real sandDamping_ = 12.0;

  real coulombConstant_ = 50.0;

  Vec3 gravity = Vec3(0, 0, -9.80665);

  struct AudioEvent {
    enum class Kind { Splash, SandClick, SolidImpact } kind;
    real intensity;
    real frequency;
  };
  std::vector<AudioEvent> frameAudioEvents;

  void clear() {
    particles.clear();
    bonds.clear();
  }

  void spawnWaterCluster(const Vec3 &origin, int count = 40,
                         real spread = 0.3) {
    std::mt19937 rng(1337 + particles.size());
    std::uniform_real_distribution<real> dist(-spread, spread);
    for (int i = 0; i < count; ++i) {
      Particle p;
      p.type = MatterType::Water;
      p.pos = origin + Vec3(dist(rng), dist(rng), dist(rng) * 0.5);
      p.vel = Vec3(dist(rng) * 0.5, dist(rng) * 0.5, -0.5);
      p.mass = 0.025;
      p.radius = 0.055;
      p.density = waterRestDensity_;
      p.color = {0.18f, 0.55f, 0.95f};
      particles.push_back(p);
    }
  }

  void spawnSandCluster(const Vec3 &origin, int count = 50,
                        real spread = 0.25) {
    std::mt19937 rng(42 + particles.size());
    std::uniform_real_distribution<real> dist(-spread, spread);
    for (int i = 0; i < count; ++i) {
      Particle p;
      p.type = MatterType::Sand;
      p.pos = origin + Vec3(dist(rng), dist(rng), dist(rng) * 0.5);
      p.vel = Vec3(dist(rng) * 0.2, dist(rng) * 0.2, -0.2);
      p.mass = 0.04;
      p.radius = 0.045;
      p.color = {0.88f, 0.74f, 0.42f};
      particles.push_back(p);
    }
  }

  void spawnMolecularComplex(const Vec3 &origin, int nx = 3, int ny = 3,
                             int nz = 3, real spacing = 0.14) {
    std::size_t baseIdx = particles.size();
    static int complexCounter = 0;
    int cId = complexCounter++;

    for (int iz = 0; iz < nz; ++iz)
      for (int iy = 0; iy < ny; ++iy)
        for (int ix = 0; ix < nx; ++ix) {
          Particle p;
          p.type = MatterType::MolecularComplex;
          p.pos = origin + Vec3(ix * spacing, iy * spacing, iz * spacing);
          p.vel = Vec3(0, 0, 0);
          p.mass = 0.08;
          p.radius = 0.048;
          p.complexId = cId;
          p.color = {0.25f, 0.85f, 0.45f};
          particles.push_back(p);
        }

    for (int iz = 0; iz < nz; ++iz)
      for (int iy = 0; iy < ny; ++iy)
        for (int ix = 0; ix < nx; ++ix) {
          std::size_t i = baseIdx + iz * (nx * ny) + iy * nx + ix;
          auto connect = [&](int dx, int dy, int dz) {
            int jx = ix + dx, jy = iy + dy, jz = iz + dz;
            if (jx < nx && jy < ny && jz < nz) {
              std::size_t j = baseIdx + jz * (nx * ny) + jy * nx + jx;
              MolecularBond b;
              b.i = i;
              b.j = j;
              b.restLength = (particles[i].pos - particles[j].pos).norm();
              b.stiffness = 450.0;
              b.damping = 6.0;
              bonds.push_back(b);
            }
          };
          connect(1, 0, 0);
          connect(0, 1, 0);
          connect(0, 0, 1);
          connect(1, 1, 0);
          connect(1, 0, 1);
        }
  }

  void spawnElementaryDipole(const Vec3 &origin) {
    Particle e;
    e.type = MatterType::Electron;
    e.pos = origin + Vec3(-0.2, 0, 0);
    e.vel = Vec3(0, 1.2, 0);
    e.mass = 0.01;
    e.radius = 0.035;
    e.charge = -1.0;
    e.color = {0.2f, 0.4f, 1.0f};
    particles.push_back(e);

    Particle p;
    p.type = MatterType::Positron;
    p.pos = origin + Vec3(0.2, 0, 0);
    p.vel = Vec3(0, -1.2, 0);
    p.mass = 0.01;
    p.radius = 0.035;
    p.charge = +1.0;
    p.color = {1.0f, 0.25f, 0.25f};
    particles.push_back(p);
  }

  real sphKernel(real r, real h) const {
    if (r >= h)
      return 0.0;
    const real q = r / h;
    const real term = 1.0 - q;
    const real alpha = 21.0 / (2.0 * PI * h * h * h);
    return alpha * term * term * term * term * (1.0 + 4.0 * q);
  }

  Vec3 sphKernelGradient(const Vec3 &rVec, real dist, real h) const {
    if (dist >= h || dist < 1e-9)
      return Vec3(0, 0, 0);
    const real q = dist / h;
    const real term = 1.0 - q;
    const real alpha = 21.0 / (2.0 * PI * h * h * h);
    const real dWdr = -alpha * (20.0 * q / h) * term * term * term;
    const real s = dWdr / dist;
    return Vec3(rVec.x * s, rVec.y * s, rVec.z * s);
  }

  void computeForces(const AirProperties &air) {
    const std::size_t N = particles.size();
    frameAudioEvents.clear();

    // 1. Reset + gravita' + aerodinamica (O(N))
    for (std::size_t i = 0; i < N; ++i) {
      auto &p = particles[i];
      p.force = gravity * p.mass;
      Vec3 fDrag, fBuoyancy;
      real re;
      air.computeAerodynamicForces(p.radius, p.mass, p.pos, p.vel, fDrag,
                                   fBuoyancy, re, gravity.norm());
      p.force = p.force + fDrag + fBuoyancy;
    }
    if (N < 2)
      return;

    // Costruzione spatial hash (una volta per substep)
    const real h = sphRadius_;
    grid_.build(particles, h);

    // 2. Densita' e pressione SPH (O(N*k))
    for (std::size_t i = 0; i < N; ++i) {
      if (particles[i].type != MatterType::Water)
        continue;
      const real self = sphKernel(0.0, h) * particles[i].mass;
      real rho = self;
      const Vec3 pi = particles[i].pos;
      grid_.forAll(int(i), [&](int j) {
        if (particles[j].type != MatterType::Water)
          return;
        const Vec3 d = pi - particles[j].pos;
        const real dist = d.norm();
        if (dist < h)
          rho += particles[j].mass * sphKernel(dist, h);
      });
      const real rhoMin = waterRestDensity_ * 0.5;
      particles[i].density = rho > rhoMin ? rho : rhoMin;
      const real ratio = particles[i].density / waterRestDensity_;
      // pow(ratio,7) = exp(7*ln(ratio))
      const real p7 = std::exp(7.0 * std::log(ratio));
      const real pv = waterBulkModulus_ * (p7 - 1.0);
      particles[i].pressure = pv > 0.0 ? pv : 0.0;
    }

    // 3. Coppie (SPH, DEM, coulomb, acqua-sabbia) via forHalf -> O(N*k)
    const real gMag = gravity.norm();

    // Pre-lista particelle cariche (per Coulomb C<<N)
    std::vector<int> charged;
    charged.reserve(8);
    for (std::size_t i = 0; i < N; ++i)
      if (std::abs(particles[i].charge) > 1e-4)
        charged.push_back(int(i));

    // Accumulatori Coulomb (applicati dopo, l'ordine tra cariche resta C^2)
    for (std::size_t a = 0; a < charged.size(); ++a) {
      const int i = charged[a];
      for (std::size_t b = a + 1; b < charged.size(); ++b) {
        const int j = charged[b];
        auto &pi = particles[i];
        auto &pj = particles[j];
        Vec3 rVec = pi.pos - pj.pos;
        real dist = rVec.norm();
        if (dist < 1e-6)
          continue;
        Vec3 n = rVec * (1.0 / dist);
        real fC = coulombConstant_ * (pi.charge * pj.charge) /
                  std::max(dist * dist, 0.005);
        Vec3 fCoulomb = n * fC;
        pi.force = pi.force + fCoulomb;
        pj.force = pj.force - fCoulomb;
      }
    }

    for (std::size_t i = 0; i < N; ++i) {
      auto &pi = particles[i];
      const Vec3 pi_pos = pi.pos;
      grid_.forHalf(int(i), [&](int j) {
        auto &pj = particles[j];
        Vec3 rVec = pi_pos - pj.pos;
        real dist = rVec.norm();
        if (dist < 1e-6)
          return;
        const real sumR = pi.radius + pj.radius;
        Vec3 n = rVec * (1.0 / dist);

        // B. SPH acqua-acqua
        if (pi.type == MatterType::Water && pj.type == MatterType::Water &&
            dist < h) {
          Vec3 gradW = sphKernelGradient(rVec, dist, h);
          real pTerm = (pi.pressure / (pi.density * pi.density)) +
                       (pj.pressure / (pj.density * pj.density));
          Vec3 fPress = gradW * (-pi.mass * pj.mass * pTerm);

          Vec3 vDiff = pi.vel - pj.vel;
          real vDotR = vDiff.dot(rVec);
          Vec3 fVisc(0, 0, 0);
          if (vDotR < 0) {
            real muIJ = (h * vDotR) / (dist * dist + 0.01 * h * h);
            real piIJ =
                -waterViscosity_ * muIJ / (0.5 * (pi.density + pj.density));
            fVisc = gradW * (-pi.mass * pj.mass * piIJ);
          }
          real surfCoeff = waterSurfaceTension_ * (1.0 - dist / h);
          Vec3 fSurf = n * (-surfCoeff * pi.mass * pj.mass);
          Vec3 fWaterTotal = fPress + fVisc + fSurf;
          pi.force = pi.force + fWaterTotal;
          pj.force = pj.force - fWaterTotal;
        }

        // C. DEM sabbia-sabbia
        if (pi.type == MatterType::Sand && pj.type == MatterType::Sand) {
          real overlap = sumR - dist;
          if (overlap > 0) {
            real vn = (pi.vel - pj.vel).dot(n);
            real fnMag = sandStiffness_ * overlap - sandDamping_ * vn;
            if (fnMag < 0)
              fnMag = 0;
            Vec3 fn = n * fnMag;
            Vec3 vt = (pi.vel - pj.vel) - n * vn;
            real vtMag = vt.norm();
            Vec3 ft(0, 0, 0);
            if (vtMag > 1e-6) {
              real maxFt = sandFrictionCoeff_ * fnMag;
              real ftMag = maxFt < 80.0 * overlap ? maxFt : 80.0 * overlap;
              ft = vt * (-ftMag / vtMag);
            }
            Vec3 fSandTotal = fn + ft;
            pi.force = pi.force + fSandTotal;
            pj.force = pj.force - fSandTotal;

            if (overlap > 0.005 && std::abs(vn) > 0.15) {
              AudioEvent ev;
              ev.kind = AudioEvent::Kind::SandClick;
              real iv = std::abs(vn) * 0.1;
              ev.intensity = iv < 0.0 ? 0.0 : (iv > 0.4 ? 0.4 : iv);
              ev.frequency = 2200.0;
              frameAudioEvents.push_back(ev);
            }
          }
        }

        // D. Acqua-sabbia (buoyancy + drag sul granello)
        if ((pi.type == MatterType::Sand && pj.type == MatterType::Water) ||
            (pi.type == MatterType::Water && pj.type == MatterType::Sand)) {
          Particle &sand = (pi.type == MatterType::Sand) ? pi : pj;
          Particle &water = (pi.type == MatterType::Water) ? pi : pj;
          if (dist < h && gMag > 1e-9) {
            const real vol =
                (4.0 / 3.0) * PI * sand.radius * sand.radius * sand.radius;
            const real Fb = water.density * gMag * vol / 8.0;
            sand.force.z += Fb;
            const Vec3 vRel = sand.vel - water.vel;
            const real vMag = vRel.norm();
            if (vMag > 1e-6) {
              const real A = PI * sand.radius * sand.radius;
              const real dragMag = 0.5 * water.density * 0.5 * A * vMag * vMag;
              const real kf = -dragMag / (8.0 * vMag);
              sand.force =
                  sand.force + Vec3(vRel.x * kf, vRel.y * kf, vRel.z * kf);
            }
          }
        }
      });
    }

    // 4. Legami molecolari (O(B))
    for (auto &bond : bonds) {
      if (!bond.intact)
        continue;
      auto &pA = particles[bond.i];
      auto &pB = particles[bond.j];
      Vec3 dVec = pA.pos - pB.pos;
      real dist = dVec.norm();
      if (dist < 1e-7)
        continue;
      Vec3 dir = dVec * (1.0 / dist);
      real delta = dist - bond.restLength;
      real relV = (pA.vel - pB.vel).dot(dir);
      real forceMag = -bond.stiffness * delta - bond.damping * relV;
      if (std::abs(forceMag) > bond.breakStress) {
        bond.intact = false;
        continue;
      }
      Vec3 fBond = dir * forceMag;
      pA.force = pA.force + fBond;
      pB.force = pB.force - fBond;
    }
  }

  void step(real dt, const AirProperties &air) {
    if (dt <= 0 || particles.empty())
      return;
    const int subSteps = 3;
    real subDt = dt / subSteps;

    for (int s = 0; s < subSteps; ++s) {
      computeForces(air);
      for (auto &p : particles) {
        Vec3 accel = p.force * (1.0 / p.mass);
        p.vel = p.vel + accel * subDt;
        p.pos = p.pos + p.vel * subDt;

        if (p.pos.z - p.radius < 0.0) {
          p.pos.z = p.radius;
          real restitution = (p.type == MatterType::Water)  ? 0.05
                             : (p.type == MatterType::Sand) ? 0.15
                                                            : 0.70;
          if (p.vel.z < 0) {
            if (std::abs(p.vel.z) > 0.3) {
              AudioEvent ev;
              ev.kind = (p.type == MatterType::Water) ? AudioEvent::Kind::Splash
                        : (p.type == MatterType::Sand)
                            ? AudioEvent::Kind::SandClick
                            : AudioEvent::Kind::SolidImpact;
              real iv = std::abs(p.vel.z) * 0.12;
              ev.intensity = iv < 0.0 ? 0.0 : (iv > 0.5 ? 0.5 : iv);
              ev.frequency = (ev.kind == AudioEvent::Kind::Splash)      ? 480.0
                             : (ev.kind == AudioEvent::Kind::SandClick) ? 1800.0
                                                                        : 280.0;
              frameAudioEvents.push_back(ev);
            }
            p.vel.z = -p.vel.z * restitution;
            real friction = (p.type == MatterType::Sand) ? 0.85 : 0.95;
            p.vel.x *= (1.0 - friction * subDt * 10.0);
            p.vel.y *= (1.0 - friction * subDt * 10.0);
            if (std::abs(p.vel.z) < 0.02)
              p.vel.z = 0.0;
          }
        }
      }
    }
  }

  real computeKineticEntropy() const {
    if (particles.empty())
      return 0.0;
    std::vector<real> energies;
    energies.reserve(particles.size());
    for (const auto &p : particles)
      energies.push_back(0.5 * p.mass * p.vel.dot(p.vel) + 1e-9);
    real totalE = std::accumulate(energies.begin(), energies.end(), 0.0);
    if (totalE <= 0)
      return 0.0;
    std::vector<real> probs(energies.size());
    for (std::size_t i = 0; i < energies.size(); ++i)
      probs[i] = energies[i] / totalE;
    return v11::entropy(probs);
  }

private:
  ParticleSpatialHash grid_;
};

} // namespace matter
} // namespace nqg

#endif // NQG_MATTER_PHYSICS_HPP