// ============================================================================
//  nqg_matter_physics.hpp  -  Fisica della Materia Multi-Fase ed Elementare
//  Integra:
//    - Particelle Elementari (Elettroni, Positroni, Fotoni, Nuclei) con Compton & Coulomb
//    - Fluidodinamica SPH per Acqua (Navier-Stokes lagrangiano, equazione di Tait, tensione sup.)
//    - Meccanica Granulare DEM per Sabbia (Contatti Hertziani, attrito di Coulomb, angolo di riposo)
//    - Complessi Molecolari (Legami armonici/Lennard-Jones, solidi deformabili ed elastici)
//    - Accoppiamento con l'Aria e Gravita' (nqg_cleanroom_engine.hpp & nqg_physics_core.hpp)
// ============================================================================
#ifndef NQG_MATTER_PHYSICS_HPP
#define NQG_MATTER_PHYSICS_HPP

#include "nqg_physics_core.hpp"
#include "nqg_engine3d.hpp"
#include "nqg_air_physics.hpp"

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <array>
#include <iostream>

namespace nqg {
namespace matter {

using engine::Vec3;
using engine::Rgb;
using cleanroom::AirProperties;

// ----------------------------------------------------------------------------
// Tipi di Materia / Particelle
// ----------------------------------------------------------------------------
enum class MatterType {
  Water,            // Liquido SPH (Tait, viscosita', coesione)
  Sand,             // Granulare DEM (Frizione di Coulomb, angolo di riposo)
  MolecularComplex, // Atomi legati con potenziale Lennard-Jones/armonico
  Electron,         // Particella elementare q = -1, m_e, Compton radius
  Positron,         // Antiparticella q = +1, m_e
  Nucleus           // Ione/nucleo positivo pesante
};

// ----------------------------------------------------------------------------
// Singola Particella Fisica Multi-Fase
// ----------------------------------------------------------------------------
struct Particle {
  MatterType type = MatterType::Water;
  Vec3 pos = Vec3(0, 0, 1);
  Vec3 vel = Vec3(0, 0, 0);
  Vec3 force = Vec3(0, 0, 0);

  real mass = 0.01;      // kg
  real radius = 0.05;    // m
  real charge = 0.0;     // Unita' elementari e
  Rgb color = {0.2f, 0.6f, 1.0f};

  // Proprieta' fluidodinamiche SPH (Acqua)
  real density = 1000.0; // kg/m^3
  real pressure = 0.0;   // Pa

  // Indice complesso molecolare (-1 se libero)
  int complexId = -1;

  // Raggio di Compton quantistico calcolato dal core fisico: lambda_C = hbar / (m * c)
  real comptonRadius() const {
    if (mass <= 1e-35) return 0.0;
    return info::comptonRadius(mass);
  }
};

// Legame elastico/molecolare tra particelle in un complesso
struct MolecularBond {
  std::size_t i, j;
  real restLength = 0.15; // m
  real stiffness = 250.0; // N/m
  real damping = 4.0;     // N*s/m
  real breakStress = 50.0;// Soglia di rottura legame
  bool intact = true;
};

// ----------------------------------------------------------------------------
// Simulatore di Materia e Interazioni Fondamentali
// ----------------------------------------------------------------------------
class MatterSimulator {
public:
  std::vector<Particle> particles;
  std::vector<MolecularBond> bonds;

  // Parametri SPH Acqua
  real sphRadius_ = 0.22;       // Smoothing kernel h (m)
  real waterRestDensity_ = 1000.0; // kg/m^3
  real waterBulkModulus_ = 2000.0; // B per equazione di Tait
  real waterViscosity_ = 0.045;    // Viscosita' di taglio
  real waterSurfaceTension_ = 0.15;// Coefficiente di coesione superficiale

  // Parametri Granulari Sabbia (DEM)
  real sandFrictionCoeff_ = 0.65;  // tan(33°) -> Angolo di riposo naturale sabbia
  real sandStiffness_ = 3500.0;    // Rigidezza contatto Hertziano
  real sandDamping_ = 12.0;        // Dissipazione energia cinetica

  // Parametri Elettrostatici / Quantistici
  real coulombConstant_ = 50.0;    // Costante k_e scalata per simulazione

  // Gravita' attiva
  Vec3 gravity = Vec3(0, 0, -9.80665);

  // Buffer audio eventi (per sintesi senza ronzio)
  struct AudioEvent {
    enum class Kind { Splash, SandClick, SolidImpact } kind;
    real intensity;
    real frequency;
  };
  std::vector<AudioEvent> frameAudioEvents;

  // Pulisce tutte le particelle
  void clear() {
    particles.clear();
    bonds.clear();
  }

  // 1. Spawna gocce d'Acqua (Flusso SPH)
  void spawnWaterCluster(const Vec3 &origin, int count = 40, real spread = 0.3) {
    std::mt19937 rng(1337 + particles.size());
    std::uniform_real_distribution<real> dist(-spread, spread);

    for (int i = 0; i < count; ++i) {
      Particle p;
      p.type = MatterType::Water;
      p.pos = origin + Vec3(dist(rng), dist(rng), dist(rng) * 0.5);
      p.vel = Vec3(dist(rng) * 0.5, dist(rng) * 0.5, -0.5);
      p.mass = 0.025; // 25 grammi
      p.radius = 0.055;
      p.density = waterRestDensity_;
      p.color = {0.18f, 0.55f, 0.95f}; // Azzurro rifrangente
      particles.push_back(p);
    }
  }

  // 2. Spawna Sabbia (Granulare DEM con frizione interna)
  void spawnSandCluster(const Vec3 &origin, int count = 50, real spread = 0.25) {
    std::mt19937 rng(42 + particles.size());
    std::uniform_real_distribution<real> dist(-spread, spread);

    for (int i = 0; i < count; ++i) {
      Particle p;
      p.type = MatterType::Sand;
      p.pos = origin + Vec3(dist(rng), dist(rng), dist(rng) * 0.5);
      p.vel = Vec3(dist(rng) * 0.2, dist(rng) * 0.2, -0.2);
      p.mass = 0.04;
      p.radius = 0.045;
      p.color = {0.88f, 0.74f, 0.42f}; // Sabbia dorata
      particles.push_back(p);
    }
  }

  // 3. Spawna Complesso Molecolare (Reticolo elastico legato con legami armonici)
  void spawnMolecularComplex(const Vec3 &origin, int nx = 3, int ny = 3, int nz = 3, real spacing = 0.14) {
    std::size_t baseIdx = particles.size();
    static int complexCounter = 0;
    int cId = complexCounter++;

    for (int iz = 0; iz < nz; ++iz) {
      for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
          Particle p;
          p.type = MatterType::MolecularComplex;
          p.pos = origin + Vec3(ix * spacing, iy * spacing, iz * spacing);
          p.vel = Vec3(0, 0, 0);
          p.mass = 0.08;
          p.radius = 0.048;
          p.complexId = cId;
          p.color = {0.25f, 0.85f, 0.45f}; // Verde smeraldo molecolare
          particles.push_back(p);
        }
      }
    }

    // Crea legami reticolari tra primi vicini
    for (int iz = 0; iz < nz; ++iz) {
      for (int iy = 0; iy < ny; ++iy) {
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
    }
  }

  // 4. Spawna Particelle Elementari (Elettroni, Positroni, Nuclei con carica e Compton)
  void spawnElementaryDipole(const Vec3 &origin) {
    // Elettrone
    Particle e;
    e.type = MatterType::Electron;
    e.pos = origin + Vec3(-0.2, 0, 0);
    e.vel = Vec3(0, 1.2, 0);
    e.mass = 0.01;
    e.radius = 0.035;
    e.charge = -1.0;
    e.color = {0.2f, 0.4f, 1.0f}; // Blu elettrico
    particles.push_back(e);

    // Positrone / Nucleo
    Particle p;
    p.type = MatterType::Positron;
    p.pos = origin + Vec3(0.2, 0, 0);
    p.vel = Vec3(0, -1.2, 0);
    p.mass = 0.01;
    p.radius = 0.035;
    p.charge = +1.0;
    p.color = {1.0f, 0.25f, 0.25f}; // Rosso positivo
    particles.push_back(p);
  }

  // --------------------------------------------------------------------------
  // Kernel di Smoothing Wendland C2 per SPH Acqua
  // W(r, h) = alpha * (1 - r/h)^4 * (1 + 4*r/h)
  // --------------------------------------------------------------------------
  real sphKernel(real r, real h) const {
    if (r >= h) return 0.0;
    real q = r / h;
    real term = 1.0 - q;
    // Fattore di normalizzazione in 3D: alpha = 21 / (2 * pi * h^3)
    real alpha = 21.0 / (2.0 * PI * std::pow(h, 3.0));
    return alpha * std::pow(term, 4.0) * (1.0 + 4.0 * q);
  }

  Vec3 sphKernelGradient(const Vec3 &rVec, real dist, real h) const {
    if (dist >= h || dist < 1e-9) return Vec3(0, 0, 0);
    real q = dist / h;
    real term = 1.0 - q;
    // dW/dr = - alpha * (20 * q / h) * (1 - q)^3
    real alpha = 21.0 / (2.0 * PI * std::pow(h, 3.0));
    real dWdr = -alpha * (20.0 * q / h) * std::pow(term, 3.0);
    return rVec * (dWdr / dist);
  }

  // --------------------------------------------------------------------------
  // Calcolo delle Forze Fisiche Complete
  // --------------------------------------------------------------------------
  void computeForces(const AirProperties &air) {
    const std::size_t N = particles.size();
    frameAudioEvents.clear();

    // 1. Reset delle forze e applicazione gravita' + aerodinamica aria
    for (std::size_t i = 0; i < N; ++i) {
      auto &p = particles[i];
      p.force = gravity * p.mass;

      // Accoppiamento con l'aria: spinta d'Archimede + attrito viscoso
      Vec3 fDrag, fBuoyancy;
      real re;
      air.computeAerodynamicForces(p.radius, p.mass, p.pos, p.vel, fDrag, fBuoyancy, re);
      p.force = p.force + fDrag + fBuoyancy;
    }

    // 2. Calcolo densita' e pressione SPH per le particelle d'acqua
    const real h = sphRadius_;
    for (std::size_t i = 0; i < N; ++i) {
      if (particles[i].type != MatterType::Water) continue;
      real rho = 0.0;
      for (std::size_t j = 0; j < N; ++j) {
        if (particles[j].type != MatterType::Water) continue;
        real dist = (particles[i].pos - particles[j].pos).norm();
        rho += particles[j].mass * sphKernel(dist, h);
      }
      particles[i].density = std::max(waterRestDensity_ * 0.5, rho);
      // Equazione di Tait: P = B * [ (rho / rho0)^7 - 1 ]
      real ratio = particles[i].density / waterRestDensity_;
      particles[i].pressure = std::max(0.0, waterBulkModulus_ * (std::pow(ratio, 7.0) - 1.0));
    }

    // 3. Interazioni a Coppie (SPH Acqua, DEM Sabbia, Coulomb Elettrostatico)
    for (std::size_t i = 0; i < N; ++i) {
      for (std::size_t j = i + 1; j < N; ++j) {
        auto &pi = particles[i];
        auto &pj = particles[j];

        Vec3 rVec = pi.pos - pj.pos;
        real dist = rVec.norm();
        if (dist < 1e-6) continue;
        Vec3 n = rVec * (1.0 / dist);
        real sumR = pi.radius + pj.radius;

        // A. Interazione Elettrostatica di Coulomb (Particelle Elementari)
        if (std::abs(pi.charge) > 1e-4 && std::abs(pj.charge) > 1e-4) {
          real fC = coulombConstant_ * (pi.charge * pj.charge) / std::max(dist * dist, 0.005);
          Vec3 fCoulomb = n * fC;
          pi.force = pi.force + fCoulomb;
          pj.force = pj.force - fCoulomb;
        }

        // B. Fluidodinamica SPH per Acqua (Pressione + Viscosita' + Coesione)
        if (pi.type == MatterType::Water && pj.type == MatterType::Water && dist < h) {
          Vec3 gradW = sphKernelGradient(rVec, dist, h);

          // Pressione simmetrizzata Navier-Stokes
          real pTerm = (pi.pressure / (pi.density * pi.density)) +
                       (pj.pressure / (pj.density * pj.density));
          Vec3 fPress = gradW * (-pi.mass * pj.mass * pTerm);

          // Viscosita' di taglio artificiale
          Vec3 vDiff = pi.vel - pj.vel;
          real vDotR = vDiff.dot(rVec);
          Vec3 fVisc(0, 0, 0);
          if (vDotR < 0) {
            real muIJ = (h * vDotR) / (dist * dist + 0.01 * h * h);
            real piIJ = -waterViscosity_ * muIJ / (0.5 * (pi.density + pj.density));
            fVisc = gradW * (-pi.mass * pj.mass * piIJ);
          }

          // Coesione superficiale (tensione superficiale per formare gocce)
          real surfCoeff = waterSurfaceTension_ * (1.0 - dist / h);
          Vec3 fSurf = n * (-surfCoeff * pi.mass * pj.mass);

          Vec3 fWaterTotal = fPress + fVisc + fSurf;
          pi.force = pi.force + fWaterTotal;
          pj.force = pj.force - fWaterTotal;
        }

        // C. Meccanica Granulare DEM per Sabbia (Contatto Hertziano + Attrito di Coulomb)
        if ((pi.type == MatterType::Sand && pj.type == MatterType::Sand) ||
            dist < sumR) {
          real overlap = sumR - dist;
          if (overlap > 0) {
            // Forza normale Hertziana: F_n = k * delta - gamma * v_n
            real vn = (pi.vel - pj.vel).dot(n);
            real fnMag = sandStiffness_ * overlap - sandDamping_ * vn;
            fnMag = std::max(0.0, fnMag);
            Vec3 fn = n * fnMag;

            // Forza tangenziale con attrito di Coulomb: |F_t| <= mu * |F_n|
            Vec3 vt = (pi.vel - pj.vel) - n * vn;
            real vtMag = vt.norm();
            Vec3 ft(0, 0, 0);
            if (vtMag > 1e-6) {
              real maxFt = sandFrictionCoeff_ * fnMag;
              real ftMag = std::min(maxFt, 80.0 * overlap);
              ft = vt * (-ftMag / vtMag);
            }

            Vec3 fSandTotal = fn + ft;
            pi.force = pi.force + fSandTotal;
            pj.force = pj.force - fSandTotal;

            // Evento audio di contatto tra granelli (senza ronzio continuo)
            if (overlap > 0.005 && std::abs(vn) > 0.15) {
              AudioEvent ev;
              ev.kind = (pi.type == MatterType::Water || pj.type == MatterType::Water)
                            ? AudioEvent::Kind::Splash
                            : AudioEvent::Kind::SandClick;
              ev.intensity = std::clamp(static_cast<real>(std::abs(vn) * 0.1), 0.0, 0.4);
              ev.frequency = (ev.kind == AudioEvent::Kind::Splash) ? 550.0 : 2200.0;
              frameAudioEvents.push_back(ev);
            }
          }
        }
      }
    }

    // 4. Forze dei Legami Molecolari (Complessi Molecolari)
    for (auto &bond : bonds) {
      if (!bond.intact) continue;
      auto &pA = particles[bond.i];
      auto &pB = particles[bond.j];

      Vec3 dVec = pA.pos - pB.pos;
      real dist = dVec.norm();
      if (dist < 1e-7) continue;

      Vec3 dir = dVec * (1.0 / dist);
      real delta = dist - bond.restLength;

      // Legame armonico con smorzamento
      real relV = (pA.vel - pB.vel).dot(dir);
      real forceMag = -bond.stiffness * delta - bond.damping * relV;

      // Rottura del legame se si supera la resistenza meccanica
      if (std::abs(forceMag) > bond.breakStress) {
        bond.intact = false;
        continue;
      }

      Vec3 fBond = dir * forceMag;
      pA.force = pA.force + fBond;
      pB.force = pB.force - fBond;
    }
  }

  // --------------------------------------------------------------------------
  // Avanzamento Temporale tramite Integrazione Euler-Verlet / DP45
  // --------------------------------------------------------------------------
  void step(real dt, const AirProperties &air) {
    if (dt <= 0 || particles.empty()) return;

    // Sub-stepping per stabilita' SPH e granulare
    const int subSteps = 3;
    real subDt = dt / subSteps;

    for (int s = 0; s < subSteps; ++s) {
      computeForces(air);

      for (auto &p : particles) {
        // Accelerazione a = F / m
        Vec3 accel = p.force * (1.0 / p.mass);
        p.vel = p.vel + accel * subDt;
        p.pos = p.pos + p.vel * subDt;

        // Collisione con il pavimento infinito della clean room (z = 0)
        if (p.pos.z - p.radius < 0.0) {
          p.pos.z = p.radius;
          real restitution = (p.type == MatterType::Water) ? 0.05
                            : (p.type == MatterType::Sand)  ? 0.15
                            : 0.70;
          if (p.vel.z < 0) {
            // Evento audio impatto pavimento
            if (std::abs(p.vel.z) > 0.3) {
              AudioEvent ev;
              ev.kind = (p.type == MatterType::Water) ? AudioEvent::Kind::Splash
                        : (p.type == MatterType::Sand) ? AudioEvent::Kind::SandClick
                        : AudioEvent::Kind::SolidImpact;
              ev.intensity = std::clamp(static_cast<real>(std::abs(p.vel.z) * 0.12), 0.0, 0.5);
              ev.frequency = (ev.kind == AudioEvent::Kind::Splash) ? 480.0
                            : (ev.kind == AudioEvent::Kind::SandClick) ? 1800.0
                            : 280.0;
              frameAudioEvents.push_back(ev);
            }

            p.vel.z = -p.vel.z * restitution;

            // Attrito statico/dinamico con il pavimento
            real friction = (p.type == MatterType::Sand) ? 0.85 : 0.95;
            p.vel.x *= (1.0 - friction * subDt * 10.0);
            p.vel.y *= (1.0 - friction * subDt * 10.0);

            // Arresto oscillazioni residue
            if (std::abs(p.vel.z) < 0.02) p.vel.z = 0.0;
          }
        }
      }
    }
  }

  // --------------------------------------------------------------------------
  // Entropia Termodinamica della Configurazione (Core Fisico V11)
  // Calcola la distribuzione di energia cinetica e l'entropia statistica S
  // --------------------------------------------------------------------------
  real computeKineticEntropy() const {
    if (particles.empty()) return 0.0;
    std::vector<real> energies;
    energies.reserve(particles.size());
    for (const auto &p : particles) {
      energies.push_back(0.5 * p.mass * p.vel.dot(p.vel) + 1e-9);
    }
    real totalE = std::accumulate(energies.begin(), energies.end(), 0.0);
    if (totalE <= 0) return 0.0;
    std::vector<real> probs(energies.size());
    for (std::size_t i = 0; i < energies.size(); ++i) {
      probs[i] = energies[i] / totalE;
    }
    return v11::entropy(probs);
  }
};

} // namespace matter
} // namespace nqg

#endif // NQG_MATTER_PHYSICS_HPP
