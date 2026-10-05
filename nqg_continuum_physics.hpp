// ============================================================================
//  nqg_continuum_physics.hpp
//  FIX 2025e (dinamica + stabilita' + EM):
//   - RigidSolidElement: sleeping (asleep/sleepTimer/wake()).
//     I corpi a riposo non si integrano -> niente jitter perpetuo.
//   - ElectromagneticField::resetForces(): azzera lastEMForce di TUTTI i
//     solidi non-statici ogni frame (prima i solidi senza carica
//     mantenevano la forza del frame precedente).
//   - ElectromagneticField::applyToSolids(): accumula (+=) invece di
//     sovrascrivere -> Coulomb e Lorentz coesistono.
// ============================================================================
#ifndef NQG_CONTINUUM_PHYSICS_HPP
#define NQG_CONTINUUM_PHYSICS_HPP

#include "nqg_air_physics.hpp"
#include "nqg_engine3d.hpp"
#include "nqg_physics_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

namespace nqg {
namespace continuum {

using cleanroom::AirProperties;
using engine::Rgb;
using engine::Vec3;

namespace phys {
using nqg::consts::c;
using nqg::consts::G;
using nqg::consts::hbar;
constexpr real k_coulomb = 8.98755179e9;
constexpr real e_charge = 1.602176634e-19;
constexpr real m_electron = 9.1093837015e-31;
constexpr real mu_0 = 4.0 * PI * 1e-7;
inline real epsilon_0() { return 1.0 / (mu_0 * c * c); }
constexpr real g_earth = 9.80665;
} // namespace phys

// ============================================================================
// 1. ACQUA
// ============================================================================
struct HydroResult {
  Vec3 force = Vec3(0, 0, 0);
  Vec3 dragCoeff = Vec3(0, 0, 0);
  Vec3 waterVel = Vec3(0, 0, 0);
  real submergedVolume = 0;
  real submergedFraction = 0;
  real addedMass = 0;
  real buoyancy = 0;
  real squeeze = 0;
  real level = 0;
  real film = 0;
  real bedZ = 0;
  bool wet = false;
};

class ContinuousWaterBody {
public:
  fluid::ShallowFlow flow;

  real refractiveIndex = 1.333;
  Vec3 absorptionCoeff = Vec3(2.0, 0.25, 0.05);
  real bodyContactAngle = 1.1;
  real addedMassCoeffBox = 0.6;
  real addedMassCoeffSphere = 0.5;

  Vec3 basinCenter = Vec3(0, 0, 0);
  real waterVolume = 0.0;
  real simTime_ = 0.0;

  struct SurfaceWave {
    Vec3 origin{0, 0, 0};
    real amplitude = 0.0;
    real omega = 0.0;
    real phase = 0.0;
    real birth = 0.0;
  };
  struct Ripple {
    Vec3 origin{0, 0, 0};
    real birth = 0.0;
    real amplitude = 0.0;
  };
  std::vector<SurfaceWave> waves;
  std::vector<Ripple> ripples;

  ContinuousWaterBody() : flow(0.1, 128, 128, 0.0, 0.0) {
    const real half = 3.0;
    initialFill(basinCenter.x - half, basinCenter.x + half,
                basinCenter.y - half, basinCenter.y + half, 0.45);
  }

  void setBedProvider(fluid::ShallowFlow::BedFn fn) {
    flow.bed = std::move(fn);
  }
  void refreshBed() { flow.refreshBed(); }
  void setLiquidTemperature(real kelvin) {
    const real angle = flow.liquid.contactAngle;
    flow.liquid = fluid::waterAtKelvin(kelvin);
    flow.liquid.contactAngle = angle;
  }
  void setContactAngle(real radians) { flow.liquid.contactAngle = radians; }
  void setWind(const Vec3 &w) {
    flow.windX = w.x;
    flow.windY = w.y;
  }
  void setGravity(real g) { flow.gravity = std::max(1e-3, g); }

  void step(real dt) {
    flow.step(dt);
    waterVolume = flow.totalVolume();
    simTime_ += dt;
    if (!ripples.empty()) {
      const real tLim = simTime_ - 3.0;
      ripples.erase(
          std::remove_if(ripples.begin(), ripples.end(),
                         [&](const Ripple &r) { return r.birth < tLim; }),
          ripples.end());
    }
    if (!waves.empty()) {
      const real tLim = simTime_ - 6.0;
      waves.erase(
          std::remove_if(waves.begin(), waves.end(),
                         [&](const SurfaceWave &w) { return w.birth < tLim; }),
          waves.end());
    }
  }
  void clear() {
    flow.clear();
    waves.clear();
    ripples.clear();
    waterVolume = 0.0;
    simTime_ = 0.0;
  }
  void initialFill(real xa, real xb, real ya, real yb, real depth) {
    flow.fillRect(xa, xb, ya, yb, depth);
    waterVolume = flow.totalVolume();
  }
  void addVolume(real x, real y, real radius, real volume) {
    flow.addVolume(x, y, radius, volume);
    waterVolume = flow.totalVolume();
  }

  real density() const { return flow.liquid.rho; }
  real totalVolume() const { return flow.totalVolume(); }
  std::size_t wetCells() const { return flow.wetCells(); }
  real maxDepth() const { return flow.maxDepth(); }
  real depthAt(real x, real y) const { return flow.depthAt(x, y); }
  bool isInsideBasin(real x, real y) const { return flow.depthAt(x, y) > 1e-4; }

  real surfaceHeight(real x, real y) const {
    auto s = flow.sample(x, y);
    return s.wet ? s.eta : -1e9;
  }

  real waveElevation(real x, real y, real time) const {
    if (ripples.empty() && waves.empty())
      return 0.0;
    real dh = 0.0;
    for (const auto &r : ripples) {
      const real age = time - r.birth;
      if (age < 0.0 || age > 3.0)
        continue;
      const real dx = x - r.origin.x;
      const real dy = y - r.origin.y;
      const real dist = std::sqrt(dx * dx + dy * dy);
      const real front = age * 1.6;
      const real d = dist - front;
      const real sigma = 0.10 + 0.08 * age;
      const real env =
          std::exp(-(d * d) / (2.0 * sigma * sigma)) * std::exp(-1.5 * age);
      dh += r.amplitude * env * std::cos(d * 28.0 - age * 6.0);
    }
    for (const auto &w : waves) {
      const real age = time - w.birth;
      if (age < 0.0 || age > 6.0)
        continue;
      const real dx = x - w.origin.x;
      const real dy = y - w.origin.y;
      const real dist = std::sqrt(dx * dx + dy * dy);
      const real env = std::exp(-dist * dist / 4.0) * std::exp(-0.4 * age);
      dh += w.amplitude * env * std::sin(w.omega * age - dist * 3.5 + w.phase);
    }
    return dh;
  }

  real surfaceHeightVisual(real x, real y, real time) const {
    auto s = flow.sample(x, y);
    if (!s.wet)
      return -1e9;
    return s.eta + waveElevation(x, y, time);
  }
  real evaluateHeight(real x, real y, real time) const {
    return surfaceHeightVisual(x, y, time);
  }

  Vec3 velocityAt(real x, real y) const {
    auto s = flow.sample(x, y);
    return s.wet ? Vec3(s.u, s.v, 0) : Vec3(0, 0, 0);
  }

  Vec3 evaluateNormal(real x, real y, real) const {
    auto c = flow.sample(x, y);
    if (!c.wet)
      return Vec3(0, 0, 1);
    const real e = 0.5 * flow.dx;
    const real inv2e = 1.0 / (2.0 * e);
    auto sx1 = flow.sample(x + e, y), sx0 = flow.sample(x - e, y);
    auto sy1 = flow.sample(x, y + e), sy0 = flow.sample(x, y - e);
    real ex1 = sx1.wet ? sx1.eta : c.eta, ex0 = sx0.wet ? sx0.eta : c.eta;
    real ey1 = sy1.wet ? sy1.eta : c.eta, ey0 = sy0.wet ? sy0.eta : c.eta;
    real dhdx = (ex1 - ex0) * inv2e;
    real dhdy = (ey1 - ey0) * inv2e;
    return Vec3(-dhdx, -dhdy, 1.0).normalized();
  }

  Vec3 evaluateNormalVisual(real x, real y, real time) const {
    const real eps = std::max(0.03, 0.5 * flow.dx);
    const real inv2e = 1.0 / (2.0 * eps);
    const real hL = surfaceHeightVisual(x - eps, y, time);
    const real hR = surfaceHeightVisual(x + eps, y, time);
    const real hD = surfaceHeightVisual(x, y - eps, time);
    const real hU = surfaceHeightVisual(x, y + eps, time);
    if (hL < -1e8 || hR < -1e8 || hD < -1e8 || hU < -1e8)
      return evaluateNormal(x, y, time);
    const real dhdx = (hR - hL) * inv2e;
    const real dhdy = (hU - hD) * inv2e;
    return Vec3(-dhdx, -dhdy, 1.0).normalized();
  }

  real currentLevel() const {
    auto s = flow.sample(basinCenter.x, basinCenter.y);
    return s.wet ? s.depth : 0.0;
  }
  bool isInsideSpillway(real x, real y) const {
    const real half = 3.0;
    return !(std::abs(x - basinCenter.x) <= half &&
             std::abs(y - basinCenter.y) <= half);
  }

  void addImpulse(const Vec3 &pos, real time, real amplitude) {
    if (!std::isfinite(amplitude) || amplitude <= 0.0)
      return;
    auto s = flow.sample(pos.x, pos.y);
    if (!s.wet)
      return;
    const real depth = std::max(0.01, s.depth);
    const real amp = std::min(amplitude, 0.5 * depth);

    SurfaceWave w;
    w.origin = pos;
    w.amplitude = amp;
    w.omega = std::sqrt(flow.gravity / depth);
    w.phase = 0.0;
    w.birth = time;
    waves.push_back(w);

    Ripple r;
    r.origin = pos;
    r.birth = time;
    r.amplitude = amp;
    ripples.push_back(r);

    if (waves.size() > 24)
      waves.erase(waves.begin(), waves.begin() + (waves.size() - 24));
    if (ripples.size() > 32)
      ripples.erase(ripples.begin(), ripples.begin() + (ripples.size() - 32));
  }

  bool intersectWater(const Vec3 &ro, const Vec3 &rd, real time, real &tOut,
                      Vec3 &nOut, real &depthOut) const {
    if (!flow.hasWet)
      return false;
    real tN = 0.001, tF = 1e30;
    auto slab = [&](real o, real d, real lo, real hi) {
      if (std::abs(d) < 1e-12)
        return o >= lo && o <= hi;
      real t1 = (lo - o) / d, t2 = (hi - o) / d;
      if (t1 > t2)
        std::swap(t1, t2);
      tN = std::max(tN, t1);
      tF = std::min(tF, t2);
      return tN <= tF;
    };
    if (!slab(ro.x, rd.x, flow.bxMin, flow.bxMax) ||
        !slab(ro.y, rd.y, flow.byMin, flow.byMax) ||
        !slab(ro.z, rd.z, flow.bzMin - 0.5, flow.bzMax + 0.5))
      return false;

    auto inside = [&](real t, fluid::ShallowFlow::Sample &s, real &etaUsed) {
      const Vec3 p = ro + rd * t;
      s = flow.sample(p.x, p.y);
      if (!s.wet)
        return false;
      etaUsed = s.eta + waveElevation(p.x, p.y, time);
      return p.z < etaUsed;
    };

    fluid::ShallowFlow::Sample s;
    real etaUsed = 0;
    if (inside(tN, s, etaUsed))
      return false;
    real tPrev = tN, t = tN;
    const real dx = flow.dx;
    for (int it = 0; it < 4000 && t < tF; ++it) {
      real step = dx;
      if (s.wet) {
        const real gap = (ro + rd * t).z - etaUsed;
        const real half = 0.5 * std::abs(gap);
        step = half < 0.5 * dx ? 0.5 * dx : (half > 2.0 * dx ? 2.0 * dx : half);
      }
      tPrev = t;
      t = std::min(t + step, tF);
      if (inside(t, s, etaUsed)) {
        real lo = tPrev, hi = t;
        for (int b = 0; b < 10; ++b) {
          real mid = 0.5 * (lo + hi);
          fluid::ShallowFlow::Sample sm;
          real em;
          if (inside(mid, sm, em))
            hi = mid;
          else
            lo = mid;
        }
        fluid::ShallowFlow::Sample sh;
        real eu;
        inside(hi, sh, eu);
        const Vec3 p = ro + rd * hi;
        tOut = hi;
        nOut = evaluateNormalVisual(p.x, p.y, time);
        depthOut = std::max(1e-4, sh.depth);
        return true;
      }
      if (t >= tF)
        break;
    }
    return false;
  }

  static bool refractRay(const Vec3 &I, const Vec3 &N, real eta,
                         Vec3 &refracted) {
    real nDotI = N.dot(I);
    real cosThetaI = -nDotI;
    Vec3 norm = N;
    if (cosThetaI < 0) {
      cosThetaI = -cosThetaI;
      norm = N * (-1.0);
      eta = 1.0 / eta;
    }
    real sin2ThetaT = eta * eta * (1.0 - cosThetaI * cosThetaI);
    if (sin2ThetaT > 1.0)
      return false;
    real cosThetaT = std::sqrt(1.0 - sin2ThetaT);
    refracted = (I * eta + norm * (eta * cosThetaI - cosThetaT)).normalized();
    return true;
  }

  static real fresnelDielectric(real cosTheta, real n1 = 1.0, real n2 = 1.333) {
    real r0 = (n1 - n2) / (n1 + n2);
    r0 = r0 * r0;
    return r0 +
           (1.0 - r0) * std::pow(1.0 - std::clamp(cosTheta, 0.0, 1.0), 5.0);
  }

  Vec3 beerLambertTransmission(real pathLength) const {
    return Vec3(std::exp(-absorptionCoeff.x * pathLength),
                std::exp(-absorptionCoeff.y * pathLength),
                std::exp(-absorptionCoeff.z * pathLength));
  }

  static real segmentArea(real r, real h) {
    h = std::clamp(h, 0.0, 2.0 * r);
    if (h <= 0)
      return 0;
    if (h >= 2.0 * r)
      return PI * r * r;
    const real d = r - h;
    return r * r * std::acos(std::clamp(d / r, -1.0, 1.0)) -
           d * std::sqrt(std::max(0.0, 2.0 * r * h - h * h));
  }

  HydroResult boxForces(const Vec3 &pos, const Vec3 &size, const Vec3 &vel,
                        real g) const {
    HydroResult r;
    const real bottom = pos.z - 0.5 * size.z;
    const real top = pos.z + 0.5 * size.z;
    const real hx = 0.5 * size.x * 0.9, hy = 0.5 * size.y * 0.9;
    const int N = 3;
    int wetCount = 0;
    real etaSum = 0, subSum = 0, bedSum = 0;
    Vec3 vsum(0, 0, 0);
    for (int a = 0; a < N; ++a)
      for (int b = 0; b < N; ++b) {
        const real fx = ((a + 0.5) / N) * 2.0 - 1.0;
        const real fy = ((b + 0.5) / N) * 2.0 - 1.0;
        auto s = flow.sample(pos.x + fx * hx, pos.y + fy * hy);
        if (!s.wet)
          continue;
        ++wetCount;
        etaSum += s.eta;
        bedSum += s.bed;
        vsum.x += s.u;
        vsum.y += s.v;
        real dsub = s.eta - bottom;
        if (dsub < 0)
          dsub = 0;
        else if (dsub > size.z)
          dsub = size.z;
        subSum += dsub;
      }
    if (wetCount == 0)
      return r;
    const real inv = 1.0 / wetCount;
    r.wet = true;
    r.level = etaSum * inv;
    r.waterVel = vsum * inv;
    r.bedZ = bedSum * inv;
    r.film = std::max(0.0, r.level - r.bedZ);
    const real rho = flow.liquid.rho, mu = flow.liquid.mu;
    const real dsub = subSum / (N * N);
    r.submergedVolume = size.x * size.y * dsub;
    r.submergedFraction = size.z > 0 ? dsub / size.z : 0.0;
    if (r.submergedVolume <= 0)
      return r;
    r.buoyancy = rho * g * r.submergedVolume;
    r.force.z += r.buoyancy;
    r.addedMass = addedMassCoeffBox * rho * r.submergedVolume;
    const Vec3 vr = vel - r.waterVel;
    const real sp = vr.norm();
    const real Re = rho * sp * std::max(size.x, size.y) / mu;
    const real Cf = Re < 5e5 ? 1.328 / std::sqrt(std::max(Re, 1.0))
                             : 0.074 * std::pow(Re, -0.2);
    const real wetArea = 2.0 * size.x * size.y + 2.0 * (size.x + size.y) * dsub;
    const real skin = 0.5 * rho * std::min(Cf, 0.2) * wetArea * sp;
    r.dragCoeff.x = 0.5 * rho * 1.05 * size.y * dsub * std::abs(vr.x) + skin;
    r.dragCoeff.y = 0.5 * rho * 1.05 * size.x * dsub * std::abs(vr.y) + skin;
    r.dragCoeff.z = 0.5 * rho * 1.2 * size.x * size.y * std::abs(vr.z) + skin;
    if (r.level > bottom && r.level < top) {
      const real perimeter = 2.0 * (size.x + size.y);
      r.force.z += fluid::capillaryVerticalForce(flow.liquid, perimeter,
                                                 bodyContactAngle);
    }
    const real gap = bottom - r.bedZ;
    const real Req = std::sqrt(size.x * size.y / PI);
    if (gap < 0.2 * Req && gap < r.film)
      r.squeeze = contact::squeezeDiscCoefficient(mu, Req, std::max(gap, 2e-5));
    return r;
  }

  HydroResult sphereForces(const Vec3 &pos, real radius, const Vec3 &vel,
                           real g) const {
    HydroResult r;
    auto s = flow.sample(pos.x, pos.y);
    if (!s.wet)
      return r;
    r.wet = true;
    r.level = s.eta;
    r.waterVel = Vec3(s.u, s.v, 0);
    r.bedZ = s.bed;
    r.film = std::max(0.0, s.eta - s.bed);
    const real rho = flow.liquid.rho, mu = flow.liquid.mu;
    const real bottom = pos.z - radius;
    const real hc = std::clamp(s.eta - bottom, 0.0, 2.0 * radius);
    if (hc <= 0)
      return r;
    r.submergedVolume = PI * hc * hc * (3.0 * radius - hc) / 3.0;
    r.submergedFraction =
        r.submergedVolume / ((4.0 / 3.0) * PI * radius * radius * radius);
    r.buoyancy = rho * g * r.submergedVolume;
    r.force.z += r.buoyancy;
    r.addedMass = addedMassCoeffSphere * rho * r.submergedVolume;
    const Vec3 vr = vel - r.waterVel;
    const real sp = vr.norm();
    const real Re = rho * sp * 2.0 * radius / mu;
    const real Cd = AirProperties::sphereDragCoefficient(Re);
    const real area = segmentArea(radius, hc);
    const real c = 0.5 * rho * Cd * area * sp;
    r.dragCoeff = Vec3(c, c, c);
    if (hc < 2.0 * radius) {
      const real a = std::sqrt(std::max(0.0, 2.0 * radius * hc - hc * hc));
      r.force.z += fluid::capillaryVerticalForce(flow.liquid, 2.0 * PI * a,
                                                 bodyContactAngle);
    }
    const real gap = bottom - r.bedZ;
    if (gap < 0.25 * radius && gap < r.film)
      r.squeeze =
          contact::squeezeSphereCoefficient(mu, radius, std::max(gap, 2e-5));
    return r;
  }

  void couple(real cx, real cy, real hx, real hy, bool circular, real dVolume,
              real px, real py) {
    const real margin = 1.5 * flow.dx;
    flow.addVolumeRing(cx, cy, hx, hy, circular, margin, dVolume);
    flow.addMomentum(cx, cy, std::max(hx, hy) + 2.0 * flow.dx, px, py);
    waterVolume = flow.totalVolume();
  }

  real couplePlayer(const Vec3 &pos, real radius, real footZ, real height,
                    const Vec3 &vel, real dt, real prevSubmerged) {
    auto s = flow.sample(pos.x, pos.y);
    real sub = 0, px = 0, py = 0;
    if (s.wet) {
      const real d = std::clamp(s.eta - footZ, 0.0, height);
      sub = PI * radius * radius * d;
      const real rx = vel.x - s.u, ry = vel.y - s.v;
      const real sp = std::sqrt(rx * rx + ry * ry);
      const real area = 2.0 * radius * d;
      const real F = 0.5 * flow.liquid.rho * 1.0 * area * sp;
      px = F * rx * dt;
      py = F * ry * dt;
    }
    couple(pos.x, pos.y, radius, radius, true, sub - prevSubmerged, px, py);
    return sub;
  }
};

// ============================================================================
// 2. VENTO
// ============================================================================
class ContinuousWindField {
public:
  Vec3 baseDrift = Vec3(1.2, 0.4, 0.0);
  real turbulenceIntensity = 2.4;
  real spatialScale = 0.6;

  static Vec3 vectorPotential(const Vec3 &p, real t) {
    real px = p.x * 0.7 + t * 0.4;
    real py = p.y * 0.7 - t * 0.3;
    real pz = p.z * 0.7 + t * 0.2;
    real psiX = std::sin(py * 1.5 + 0.4) * std::cos(pz * 1.2);
    real psiY = std::sin(pz * 1.5 + 0.9) * std::cos(px * 1.2);
    real psiZ = std::sin(px * 1.5 + 1.2) * std::cos(py * 1.2);
    return Vec3(psiX, psiY, psiZ);
  }

  Vec3 evaluateVelocity(const Vec3 &p, real t) const {
    const real e = 0.01;
    const real inv2e = 1.0 / (2.0 * e);
    Vec3 pXp = vectorPotential(p + Vec3(e, 0, 0), t);
    Vec3 pXm = vectorPotential(p - Vec3(e, 0, 0), t);
    Vec3 pYp = vectorPotential(p + Vec3(0, e, 0), t);
    Vec3 pYm = vectorPotential(p - Vec3(0, e, 0), t);
    Vec3 pZp = vectorPotential(p + Vec3(0, 0, e), t);
    Vec3 pZm = vectorPotential(p - Vec3(0, 0, e), t);
    const real dPz_dy = (pYp.z - pYm.z) * inv2e;
    const real dPy_dz = (pZp.y - pZm.y) * inv2e;
    const real dPx_dz = (pZp.x - pZm.x) * inv2e;
    const real dPz_dx = (pXp.z - pXm.z) * inv2e;
    const real dPy_dx = (pXp.y - pXm.y) * inv2e;
    const real dPx_dy = (pYp.x - pYm.x) * inv2e;
    Vec3 turbulent(dPz_dy - dPy_dz, dPx_dz - dPz_dx, dPy_dx - dPx_dy);
    return baseDrift + turbulent * turbulenceIntensity;
  }

  Vec3 windForceOnBody(const Vec3 &bodyPos, const Vec3 &bodyVel,
                       const Vec3 &bodySize, real airDensity,
                       real simTime) const {
    Vec3 windVel = evaluateVelocity(bodyPos, simTime);
    Vec3 vRel = windVel - bodyVel;
    const real v2 = vRel.dot(vRel);
    if (v2 < 1e-10)
      return Vec3(0, 0, 0);
    const real vMag = std::sqrt(v2);
    const real area =
        std::max(bodySize.y * bodySize.z, bodySize.x * bodySize.z);
    const real forceMag = 0.5 * airDensity * 1.05 * area * v2;
    const real k = forceMag / vMag;
    return Vec3(vRel.x * k, vRel.y * k, vRel.z * k);
  }
};

// ============================================================================
// 3. SABBIA
// ============================================================================
class ContinuousSandDuneField {
public:
  Vec3 sandCenter = Vec3(3.2, 2.5, 0.0);
  real sandRadius = 2.2;
  real criticalSlope = 0.65;

  static constexpr int GRID_N = 64;
  std::array<std::array<real, GRID_N>, GRID_N> height{};
  real gridSpacing = 0.08;

  ContinuousSandDuneField() {
    for (int j = 0; j < GRID_N; ++j)
      for (int i = 0; i < GRID_N; ++i) {
        real x = (i - GRID_N / 2) * gridSpacing;
        real y = (j - GRID_N / 2) * gridSpacing;
        real r = std::sqrt(x * x + y * y);
        height[j][i] = (r < 1.4) ? std::max(0.0, (1.4 - r) * 0.58) : 0.0;
      }
  }

  void pourSand(real worldX, real worldY, real amount = 0.02) {
    real localX = worldX - sandCenter.x;
    real localY = worldY - sandCenter.y;
    int i = int(localX / gridSpacing) + GRID_N / 2;
    int j = int(localY / gridSpacing) + GRID_N / 2;
    if (i >= 2 && i < GRID_N - 2 && j >= 2 && j < GRID_N - 2) {
      height[j][i] += amount;
      relaxAvalanche();
    }
  }

  void relaxAvalanche(int iterations = 4) {
    const real inv_dx = 1.0 / gridSpacing;
    static const int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (int iter = 0; iter < iterations; ++iter)
      for (int j = 1; j < GRID_N - 1; ++j)
        for (int i = 1; i < GRID_N - 1; ++i) {
          real hC = height[j][i];
          real maxDiff = 0.0;
          int bestDi = 0, bestDj = 0;
          for (auto &d : dirs) {
            real hN = height[j + d[1]][i + d[0]];
            real diff = hC - hN;
            if (diff > maxDiff) {
              maxDiff = diff;
              bestDi = d[0];
              bestDj = d[1];
            }
          }
          real slope = maxDiff * inv_dx;
          if (slope > criticalSlope) {
            real transfer = (slope - criticalSlope) * gridSpacing * 0.25;
            height[j][i] -= transfer;
            height[j + bestDj][i + bestDi] += transfer;
          }
        }
  }

  real sampleHeight(real worldX, real worldY) const {
    real localX = worldX - sandCenter.x;
    real localY = worldY - sandCenter.y;
    real fi = localX / gridSpacing + GRID_N / 2.0;
    real fj = localY / gridSpacing + GRID_N / 2.0;
    int i0 = int(std::floor(fi));
    int j0 = int(std::floor(fj));
    if (i0 < 0 || i0 >= GRID_N - 1 || j0 < 0 || j0 >= GRID_N - 1)
      return 0.0;
    real u = fi - i0, v = fj - j0;
    real h00 = height[j0][i0];
    real h10 = height[j0][i0 + 1];
    real h01 = height[j0 + 1][i0];
    real h11 = height[j0 + 1][i0 + 1];
    return (h00 * (1 - u) + h10 * u) * (1 - v) + (h01 * (1 - u) + h11 * u) * v;
  }

  Vec3 evaluateNormal(real worldX, real worldY) const {
    const real eps = 0.04;
    const real inv2e = 1.0 / (2.0 * eps);
    real hL = sampleHeight(worldX - eps, worldY);
    real hR = sampleHeight(worldX + eps, worldY);
    real hD = sampleHeight(worldX, worldY - eps);
    real hU = sampleHeight(worldX, worldY + eps);
    real dhdx = (hR - hL) * inv2e;
    real dhdy = (hU - hD) * inv2e;
    return Vec3(-dhdx, -dhdy, 1.0).normalized();
  }

  bool intersectSand(const Vec3 &ro, const Vec3 &rd, real &tOut,
                     Vec3 &nOut) const {
    if (std::abs(rd.z) < 1e-6)
      return false;
    real tFloor = -ro.z / rd.z;
    if (tFloor < 0.01)
      return false;
    real tTop = (0.9 - ro.z) / rd.z;
    real tMin = std::max(0.01, std::min(tFloor, tTop));
    real tMax = std::max(0.01, std::max(tFloor, tTop));
    const int STEPS = 8;
    real dt = (tMax - tMin) / STEPS;
    real tPrev = tMin;
    Vec3 pPrev = ro + rd * tPrev;
    real hPrev = sampleHeight(pPrev.x, pPrev.y);
    real diffPrev = pPrev.z - hPrev;
    for (int s = 1; s <= STEPS; ++s) {
      real tCurr = tMin + s * dt;
      Vec3 pCurr = ro + rd * tCurr;
      real hCurr = sampleHeight(pCurr.x, pCurr.y);
      real diffCurr = pCurr.z - hCurr;
      if (diffPrev >= 0.0 && diffCurr <= 0.0 && hCurr > 0.005) {
        real frac = diffPrev / (diffPrev - diffCurr + 1e-9);
        real tExact = tPrev + frac * (tCurr - tPrev);
        Vec3 pHit = ro + rd * tExact;
        tOut = tExact;
        nOut = evaluateNormal(pHit.x, pHit.y);
        return true;
      }
      tPrev = tCurr;
      diffPrev = diffCurr;
    }
    return false;
  }
};

// ============================================================================
// 4. SOLIDI RIGIDI
// ============================================================================
struct RigidSolidElement {
  enum class Shape { Box, Cylinder, Sphere };
  Shape shape = Shape::Box;
  Vec3 pos = Vec3(-1.2, 3.2, 0.4);
  Vec3 vel = Vec3(0, 0, 0);
  Vec3 size = Vec3(0.4, 0.4, 0.4);
  real mass = 4.5;
  real density = 2400.0;
  real restitution = 0.35;
  real charge = 0.0;
  Rgb albedo = {0.85f, 0.82f, 0.78f};
  real metallic = 0.1;
  real roughness = 0.2;
  bool isStatic = false;
  bool isRoomSlab = false;
  real mu_s = 0.55;
  real mu_k = 0.40;
  real rolling = 0.015;
  real prevSubmerged = 0;
  real prevAddedMass = 0;

  // Sleeping (anti-jitter)
  bool asleep = false;
  real sleepTimer = 0.0;
  static constexpr real SLEEP_LIN_THRESH = 0.03; // m/s
  static constexpr real SLEEP_ANG_THRESH = 0.08; // rad/s
  static constexpr real SLEEP_TIME = 0.6;        // s

  Vec3 lastGravity = Vec3(0, 0, 0);
  Vec3 lastBuoyancy = Vec3(0, 0, 0);
  Vec3 lastWindForce = Vec3(0, 0, 0);
  Vec3 lastEMForce = Vec3(0, 0, 0);

  Vec3 ex = Vec3(1, 0, 0);
  Vec3 ey = Vec3(0, 1, 0);
  Vec3 ez = Vec3(0, 0, 1);
  Vec3 angVel = Vec3(0, 0, 0);

  mutable Vec3 cachedHalfW = Vec3(0, 0, 0);
  mutable bool halfWDirty = true;

  using GroundHeightFn = std::function<real(real, real)>;

  real volume() const { return size.x * size.y * size.z; }
  Vec3 halfExtents() const { return size * 0.5; }
  real boundRadius() const { return halfExtents().norm(); }

  void wake() {
    asleep = false;
    sleepTimer = 0.0;
  }

  Vec3 localInertiaDiag() const {
    if (mass <= 0.0)
      return Vec3(1, 1, 1);
    const real a = size.x, b = size.y, c = size.z;
    return Vec3(mass * (b * b + c * c) / 12.0, mass * (a * a + c * c) / 12.0,
                mass * (a * a + b * b) / 12.0);
  }

  Vec3 applyInvInertia(const Vec3 &t) const {
    if (isStatic || mass <= 0.0)
      return Vec3(0, 0, 0);
    const Vec3 il = localInertiaDiag();
    if (il.x < 1e-12 || il.y < 1e-12 || il.z < 1e-12)
      return Vec3(0, 0, 0);
    const real tx = t.dot(ex) / il.x;
    const real ty = t.dot(ey) / il.y;
    const real tz = t.dot(ez) / il.z;
    return ex * tx + ey * ty + ez * tz;
  }

  std::array<Vec3, 8> verticesWorld() const {
    std::array<Vec3, 8> v;
    const Vec3 h = halfExtents();
    const Vec3 exH = ex * h.x, eyH = ey * h.y, ezH = ez * h.z;
    int k = 0;
    for (int sx = -1; sx <= 1; sx += 2)
      for (int sy = -1; sy <= 1; sy += 2)
        for (int sz = -1; sz <= 1; sz += 2)
          v[k++] = pos + exH * real(sx) + eyH * real(sy) + ezH * real(sz);
    return v;
  }

  bool containsPoint(const Vec3 &p) const {
    const Vec3 d = p - pos;
    const Vec3 h = halfExtents();
    return std::abs(d.dot(ex)) <= h.x && std::abs(d.dot(ey)) <= h.y &&
           std::abs(d.dot(ez)) <= h.z;
  }

  void integrateOrientation(real dt) {
    const real w2 = angVel.norm2();
    if (w2 < 1e-24)
      return;
    const real w = std::sqrt(w2);
    const Vec3 axis = angVel * (1.0 / w);
    const real a = w * dt;
    ex = engine::rotateAbout(ex, axis, a);
    ey = engine::rotateAbout(ey, axis, a);
    ez = engine::rotateAbout(ez, axis, a);
    ex = ex.normalized();
    ey = (ey - ex * ey.dot(ex)).normalized();
    ez = ex.cross(ey);
    halfWDirty = true;
  }

  Vec3 pointVelocity(const Vec3 &p) const {
    const Vec3 r = p - pos;
    return vel + angVel.cross(r);
  }

  void applyImpulseAtPoint(const Vec3 &J, const Vec3 &p) {
    if (isStatic)
      return;
    const Vec3 r = p - pos;
    vel = vel + J * (1.0 / mass);
    angVel = angVel + applyInvInertia(r.cross(J));
  }

  Vec3 worldAlignedHalfExtents() const {
    const Vec3 h = halfExtents();
    return Vec3(
        std::abs(ex.x) * h.x + std::abs(ey.x) * h.y + std::abs(ez.x) * h.z,
        std::abs(ex.y) * h.x + std::abs(ey.y) * h.y + std::abs(ez.y) * h.z,
        std::abs(ex.z) * h.x + std::abs(ey.z) * h.y + std::abs(ez.z) * h.z);
  }

  void refreshWorldHalf() {
    cachedHalfW = worldAlignedHalfExtents();
    halfWDirty = false;
  }
  const Vec3 &halfW() const {
    if (halfWDirty) {
      cachedHalfW = worldAlignedHalfExtents();
      halfWDirty = false;
    }
    return cachedHalfW;
  }

  static bool intersectBox(const Vec3 &ro, const Vec3 &rd, const Vec3 &center,
                           const Vec3 &halfExt, real &tOut, Vec3 &nOut) {
    Vec3 bMin = center - halfExt;
    Vec3 bMax = center + halfExt;
    auto safeDiv = [](real num, real den) {
      if (std::abs(den) < 1e-9)
        return num * (den >= 0 ? 1e9 : -1e9);
      return num / den;
    };
    real t1 = safeDiv(bMin.x - ro.x, rd.x);
    real t2 = safeDiv(bMax.x - ro.x, rd.x);
    real t3 = safeDiv(bMin.y - ro.y, rd.y);
    real t4 = safeDiv(bMax.y - ro.y, rd.y);
    real t5 = safeDiv(bMin.z - ro.z, rd.z);
    real t6 = safeDiv(bMax.z - ro.z, rd.z);
    real tN = std::max({std::min(t1, t2), std::min(t3, t4), std::min(t5, t6)});
    real tF = std::min({std::max(t1, t2), std::max(t3, t4), std::max(t5, t6)});
    if (tN > tF || tF < 0.001)
      return false;
    tOut = tN > 0.001 ? tN : tF;
    if (tOut < 0.001)
      return false;
    Vec3 hitP = ro + rd * tOut - center;
    real dx = std::abs(hitP.x) / halfExt.x;
    real dy = std::abs(hitP.y) / halfExt.y;
    real dz = std::abs(hitP.z) / halfExt.z;
    if (dx >= dy && dx >= dz)
      nOut = Vec3(hitP.x > 0 ? 1.0 : -1.0, 0, 0);
    else if (dy >= dz)
      nOut = Vec3(0, hitP.y > 0 ? 1.0 : -1.0, 0);
    else
      nOut = Vec3(0, 0, hitP.z > 0 ? 1.0 : -1.0);
    return true;
  }

  // Pre-check ray-sphere (bounding). Veloce, sovra-inclusivo.
  bool raySphereCull(const Vec3 &ro, const Vec3 &rd) const {
    const Vec3 oc = ro - pos;
    const real b = oc.dot(rd);
    const real k = oc.dot(oc) - boundRadius() * boundRadius();
    const real disc = b * b - k;
    if (disc < 0)
      return true; // sicuramente miss
    const real sq = std::sqrt(disc);
    return (-b + sq) <= 0; // sfera interamente dietro
  }

  bool intersectOBB(const Vec3 &ro, const Vec3 &rd, real &tOut,
                    Vec3 &nOut) const {
    const Vec3 h = halfExtents();
    const Vec3 d = ro - pos;
    const Vec3 roL(d.dot(ex), d.dot(ey), d.dot(ez));
    const Vec3 rdL(rd.dot(ex), rd.dot(ey), rd.dot(ez));
    auto safeDiv = [](real num, real den) {
      if (std::abs(den) < 1e-9)
        return num * (den >= 0 ? 1e9 : -1e9);
      return num / den;
    };
    real t1 = safeDiv(-h.x - roL.x, rdL.x);
    real t2 = safeDiv(h.x - roL.x, rdL.x);
    real t3 = safeDiv(-h.y - roL.y, rdL.y);
    real t4 = safeDiv(h.y - roL.y, rdL.y);
    real t5 = safeDiv(-h.z - roL.z, rdL.z);
    real t6 = safeDiv(h.z - roL.z, rdL.z);
    real tN = std::max({std::min(t1, t2), std::min(t3, t4), std::min(t5, t6)});
    real tF = std::min({std::max(t1, t2), std::max(t3, t4), std::max(t5, t6)});
    if (tN > tF || tF < 0.001)
      return false;
    tOut = tN > 0.001 ? tN : tF;
    if (tOut < 0.001)
      return false;
    const Vec3 hitL = roL + rdL * tOut;
    const real dx = std::abs(hitL.x) / h.x;
    const real dy = std::abs(hitL.y) / h.y;
    const real dz = std::abs(hitL.z) / h.z;
    Vec3 nL;
    if (dx >= dy && dx >= dz)
      nL = Vec3(hitL.x > 0 ? 1.0 : -1.0, 0, 0);
    else if (dy >= dz)
      nL = Vec3(0, hitL.y > 0 ? 1.0 : -1.0, 0);
    else
      nL = Vec3(0, 0, hitL.z > 0 ? 1.0 : -1.0);
    nOut = ex * nL.x + ey * nL.y + ez * nL.z;
    return true;
  }

  static bool obbOverlapSAT(const RigidSolidElement &A,
                            const RigidSolidElement &B, Vec3 &colNormal,
                            real &overlap) {
    const Vec3 hA = A.halfExtents();
    const Vec3 hB = B.halfExtents();
    const Vec3 ax[3] = {A.ex, A.ey, A.ez};
    const Vec3 bx[3] = {B.ex, B.ey, B.ez};
    const real ha[3] = {hA.x, hA.y, hA.z};
    const real hb[3] = {hB.x, hB.y, hB.z};
    const Vec3 d = A.pos - B.pos;
    const real tA[3] = {d.dot(A.ex), d.dot(A.ey), d.dot(A.ez)};

    real R[3][3], AbsR[3][3];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        R[i][j] = ax[i].dot(bx[j]);
        AbsR[i][j] = std::abs(R[i][j]) + 1e-9;
      }

    real minPen = 1e30;
    Vec3 minAxis(0, 0, 1);

    for (int i = 0; i < 3; ++i) {
      const real ra = ha[i];
      const real rb =
          hb[0] * AbsR[i][0] + hb[1] * AbsR[i][1] + hb[2] * AbsR[i][2];
      const real pen = ra + rb - std::abs(tA[i]);
      if (pen <= 0)
        return false;
      if (pen < minPen) {
        minPen = pen;
        minAxis = ax[i] * (tA[i] >= 0 ? 1.0 : -1.0);
      }
    }
    for (int j = 0; j < 3; ++j) {
      const real ra =
          ha[0] * AbsR[0][j] + ha[1] * AbsR[1][j] + ha[2] * AbsR[2][j];
      const real rb = hb[j];
      const real tB = d.dot(bx[j]);
      const real pen = ra + rb - std::abs(tB);
      if (pen <= 0)
        return false;
      if (pen < minPen) {
        minPen = pen;
        minAxis = bx[j] * (tB >= 0 ? 1.0 : -1.0);
      }
    }
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        Vec3 axis = ax[i].cross(bx[j]);
        const real len2 = axis.norm2();
        if (len2 < 1e-12)
          continue;
        const real invLen = 1.0 / std::sqrt(len2);
        axis = axis * invLen;
        const real ra = ha[0] * std::abs(axis.dot(ax[0])) +
                        ha[1] * std::abs(axis.dot(ax[1])) +
                        ha[2] * std::abs(axis.dot(ax[2]));
        const real rb = hb[0] * std::abs(axis.dot(bx[0])) +
                        hb[1] * std::abs(axis.dot(bx[1])) +
                        hb[2] * std::abs(axis.dot(bx[2]));
        const real tP = d.dot(axis);
        const real pen = ra + rb - std::abs(tP);
        if (pen <= 0)
          return false;
        if (pen < minPen) {
          minPen = pen;
          minAxis = axis * (tP >= 0 ? 1.0 : -1.0);
        }
      }
    colNormal = minAxis;
    overlap = minPen;
    return true;
  }

  struct Manifold {
    Vec3 points[4];
    int count = 0;
  };

  static Manifold buildManifold(const RigidSolidElement &A,
                                const RigidSolidElement &B) {
    Manifold m;
    Vec3 buf[16];
    int n = 0;
    for (const Vec3 &v : A.verticesWorld())
      if (B.containsPoint(v) && n < 16)
        buf[n++] = v;
    for (const Vec3 &v : B.verticesWorld())
      if (A.containsPoint(v) && n < 16)
        buf[n++] = v;
    if (n == 0) {
      m.points[0] = (A.pos + B.pos) * 0.5;
      m.count = 1;
      return m;
    }
    Vec3 uniq[16];
    int un = 0;
    for (int i = 0; i < n; ++i) {
      bool dup = false;
      for (int k = 0; k < un; ++k)
        if ((buf[i] - uniq[k]).norm2() < 9e-6) {
          dup = true;
          break;
        }
      if (!dup)
        uniq[un++] = buf[i];
    }
    Vec3 c(0, 0, 0);
    for (int i = 0; i < un; ++i)
      c = c + uniq[i];
    c = c * (1.0 / real(un));
    for (int i = 0; i < un && i < 4; ++i) {
      int best = i;
      real bd = (uniq[i] - c).norm2();
      for (int k = i + 1; k < un; ++k) {
        real dk = (uniq[k] - c).norm2();
        if (dk > bd) {
          bd = dk;
          best = k;
        }
      }
      std::swap(uniq[i], uniq[best]);
    }
    m.count = std::min(un, 4);
    for (int i = 0; i < m.count; ++i)
      m.points[i] = uniq[i];
    return m;
  }

  void stepDynamics(real dt, const Vec3 &gravityVec, ContinuousWaterBody &water,
                    const ContinuousWindField &wind, real airDensity,
                    real simTime, const GroundHeightFn &groundHeight = {}) {
    if (isStatic || dt <= 0)
      return;

    // --- Sleeping: corpi a riposo non si integrano -> niente jitter ---
    if (asleep) {
      vel = Vec3(0, 0, 0);
      angVel = Vec3(0, 0, 0);
      return;
    }

    lastGravity = gravityVec * mass;
    lastWindForce = wind.windForceOnBody(pos, vel, size, airDensity, simTime);
    const real g = std::max(1e-6, gravityVec.norm());
    const int n = std::clamp(int(std::ceil(dt / 0.004)), 1, 24);
    const real h = dt / n;
    const real area = std::max(size.x * size.z, size.y * size.z);
    const real airBeta = airDensity * 0.5 * 1.05 * area;
    const Vec3 pos0 = pos, vel0 = vel;
    const Vec3 ex0 = ex, ey0 = ey, ez0 = ez, av0 = angVel;
    Vec3 reaction(0, 0, 0);

    for (int k = 0; k < n; ++k) {
      HydroResult H = water.boxForces(pos, size, vel, g);
      if (H.addedMass > prevAddedMass + 1e-12) {
        const real f = (mass + prevAddedMass) / (mass + H.addedMass);
        const Vec3 vr = vel - H.waterVel;
        const Vec3 nv = H.waterVel + vr * f;
        reaction = reaction + (vel - nv) * (mass + prevAddedMass);
        vel = nv;
      }
      prevAddedMass = H.addedMass;
      const real meff = mass + H.addedMass;
      lastBuoyancy = Vec3(0, 0, H.buoyancy);

      // Somma forze: gravita' (CoM), buoyancy+drag (acqua),
      // vento, campo EM (Lorentz + Coulomb tra solidi).
      const Vec3 F = lastGravity + H.force + lastWindForce + lastEMForce;
      vel = vel + F * (h / meff);
      vel = vel * (1.0 / (1.0 + airBeta * h / mass));

      if (H.wet) {
        const Vec3 vr = vel - H.waterVel;
        Vec3 vn(vr.x / (1.0 + H.dragCoeff.x * h / meff),
                vr.y / (1.0 + H.dragCoeff.y * h / meff),
                vr.z / (1.0 + H.dragCoeff.z * h / meff));
        if (H.squeeze > 0 && vn.z < 0)
          vn.z /= 1.0 + H.squeeze * h / meff;
        reaction = reaction + (vr - vn) * meff;
        vel = H.waterVel + vn;
        const real frac = std::clamp(H.submergedFraction, 0.0, 1.0);
        angVel = angVel * (1.0 / (1.0 + 4.0 * frac * h));
      }

      pos = pos + vel * h;
      integrateOrientation(h);
      angVel = angVel * (1.0 - 0.05 * h);
      resolveFloor(h, H, F.z, water, groundHeight);
    }

    if (!(std::isfinite(pos.x) && std::isfinite(pos.y) &&
          std::isfinite(pos.z) && std::isfinite(vel.x) &&
          std::isfinite(vel.y) && std::isfinite(vel.z) &&
          std::isfinite(angVel.x) && std::isfinite(angVel.y) &&
          std::isfinite(angVel.z))) {
      pos = pos0;
      vel = vel0 * 0.0;
      angVel = Vec3(0, 0, 0);
      ex = ex0;
      ey = ey0;
      ez = ez0;
      angVel = av0;
      prevAddedMass = 0;
      return;
    }

    const real maxSpeed = 200.0;
    if (vel.norm2() > maxSpeed * maxSpeed)
      vel = vel.normalized() * maxSpeed;
    const real maxOmega = 40.0;
    if (angVel.norm2() > maxOmega * maxOmega)
      angVel = angVel.normalized() * maxOmega;

    // --- Sleep detection ---
    const real linMag = vel.norm();
    const real angMag = angVel.norm();
    if (linMag < SLEEP_LIN_THRESH && angMag < SLEEP_ANG_THRESH) {
      sleepTimer += dt;
      if (sleepTimer > SLEEP_TIME) {
        asleep = true;
        vel = Vec3(0, 0, 0);
        angVel = Vec3(0, 0, 0);
      }
    } else {
      sleepTimer = 0.0;
    }

    HydroResult Hf = water.boxForces(pos, size, vel, g);
    water.couple(pos.x, pos.y, 0.5 * size.x, 0.5 * size.y, false,
                 Hf.submergedVolume - prevSubmerged, reaction.x, reaction.y);
    prevSubmerged = Hf.submergedVolume;
  }

private:
  void resolveFloor(real h, const HydroResult &H, real netFz,
                    const ContinuousWaterBody &water,
                    const GroundHeightFn &groundHeight) {
    real groundZ = 0.0;
    if (groundHeight) {
      real gz = groundHeight(pos.x, pos.y);
      if (std::isfinite(gz) && gz > groundZ)
        groundZ = gz;
    }
    const Vec3 half = halfExtents();
    const real hZ = std::abs(ex.z) * half.x + std::abs(ey.z) * half.y +
                    std::abs(ez.z) * half.z;
    const real bottom = pos.z - hZ;
    if (bottom > groundZ + 1e-4)
      return;
    pos.z += (groundZ - bottom);

    const real mu = water.flow.liquid.mu;
    const real film = H.wet ? std::max(0.0, H.level) : 0.0;
    const real rhoB = mass / std::max(volume(), 1e-9);
    const real D = std::sqrt(std::max(size.x * size.y, 1e-6));

    auto verts = verticesWorld();
    Vec3 contacts[8];
    int nc = 0;
    const real contactTol = 2e-3;
    for (const auto &v : verts)
      if (v.z <= groundZ + contactTol)
        contacts[nc++] = v;
    if (nc == 0) {
      Vec3 lo = verts[0];
      for (const auto &v : verts)
        if (v.z < lo.z)
          lo = v;
      contacts[nc++] = lo;
    }

    const Vec3 n(0, 0, 1);
    const real inv_nc = 1.0 / real(nc);
    const real jGravShare = std::max(0.0, -netFz) * h * inv_nc;

    const int ITER = 4;
    for (int it = 0; it < ITER; ++it) {
      for (int k = 0; k < nc; ++k) {
        const Vec3 r = contacts[k] - pos;
        const Vec3 vC = vel + angVel.cross(r);
        const real vn = vC.z;
        const Vec3 rxn = r.cross(n);
        const Vec3 irxn = applyInvInertia(rxn);
        const real K = (mass > 0.0 ? 1.0 / mass : 0.0) + n.dot(irxn.cross(r));

        real jn = 0;
        if (vn < 0 && K > 1e-12) {
          real e = restitution;
          if (-vn < 0.05)
            e = 0;
          if (film > 0 && vn < 0) {
            const real St = contact::stokesNumber(rhoB, vn, D, mu);
            const real ew = contact::wetRestitution(restitution, St);
            const real w = std::clamp(film / 1e-3, 0.0, 1.0);
            e = restitution * (1.0 - w) + ew * w;
          }
          jn = -(1.0 + e) * vn / K;
        }
        jn += jGravShare;

        if (jn > 0) {
          const Vec3 Jn = n * jn;
          vel = vel + Jn * (1.0 / mass);
          angVel = angVel + applyInvInertia(r.cross(Jn));
        }

        const Vec3 vC2 = vel + angVel.cross(r);
        const real vts = std::sqrt(vC2.x * vC2.x + vC2.y * vC2.y);
        if (vts > 1e-7 && jn > 0) {
          real muS = mu_s, muK = mu_k;
          if (film > 0) {
            const real A = std::max(size.x * size.y, 1e-6);
            const real p = jn / (h * A);
            const real mw = contact::wetFrictionCoefficient(
                mu_s, film, 2e-5, mu, vts, p, std::max(size.x, size.y));
            const real f = mu_s > 0 ? mw / mu_s : 1.0;
            muS *= f;
            muK *= f;
          }
          const Vec3 t(-vC2.x / vts, -vC2.y / vts, 0);
          const Vec3 rxt = r.cross(t);
          const Vec3 irxt = applyInvInertia(rxt);
          const real Kt =
              (mass > 0.0 ? 1.0 / mass : 0.0) + t.dot(irxt.cross(r));
          if (Kt > 1e-12) {
            const real jtStick = vts / Kt;
            const real jt = contact::coulombImpulse(jtStick, jn, muS, muK);
            const Vec3 Jt = t * jt;
            vel = vel + Jt * (1.0 / mass);
            angVel = angVel + applyInvInertia(r.cross(Jt));
          }
        }
      }
    }

    if (rolling > 0.0 && mass > 0.0) {
      const real decel =
          contact::rollingDeceleration(rolling, std::abs(netFz) / mass);
      const real wMag = std::sqrt(angVel.x * angVel.x + angVel.y * angVel.y);
      if (wMag > 1e-9) {
        const real dw = decel * h / std::max(hZ, 0.05);
        const real f = std::max(0.0, 1.0 - dw / wMag);
        angVel.x *= f;
        angVel.y *= f;
      }
    }
  }
};

// ============================================================================
// 5-6. EM e Quantistico
// ============================================================================
class ElectromagneticField {
public:
  struct PointCharge {
    Vec3 position;
    real charge = 0.0;
  };
  struct MagneticDipole {
    Vec3 position;
    Vec3 moment = Vec3(0, 0, 1.0);
    real strength = 1.0;
  };

  std::vector<PointCharge> charges;
  std::vector<MagneticDipole> dipoles;
  Vec3 backgroundE = Vec3(0, 0, 0);

  Vec3 electricField(const Vec3 &p) const {
    Vec3 E = backgroundE;
    for (const auto &ch : charges) {
      Vec3 rVec = p - ch.position;
      real r = rVec.norm();
      if (r < 0.01)
        continue;
      E = E + rVec * (phys::k_coulomb * ch.charge / (r * r * r));
    }
    return E;
  }
  Vec3 magneticField(const Vec3 &p) const {
    Vec3 B(0, 0, 0);
    for (const auto &dip : dipoles) {
      Vec3 rVec = p - dip.position;
      real r = rVec.norm();
      if (r < 0.01)
        continue;
      Vec3 rHat = rVec * (1.0 / r);
      Vec3 m = dip.moment * dip.strength;
      real mDotR = m.dot(rHat);
      real r3 = r * r * r;
      real coeff = phys::mu_0 / (4.0 * PI);
      B = B + (rHat * (3.0 * mDotR) - m) * (coeff / r3);
    }
    return B;
  }
  Vec3 lorentzForce(const Vec3 &p, const Vec3 &v, real q) const {
    if (std::abs(q) < 1e-15)
      return Vec3(0, 0, 0);
    Vec3 E = electricField(p);
    Vec3 B = magneticField(p);
    return (E + v.cross(B)) * q;
  }
  real electricPotential(const Vec3 &p) const {
    real phi = 0;
    for (const auto &ch : charges) {
      real r = (p - ch.position).norm();
      if (r < 0.01)
        continue;
      phi += phys::k_coulomb * ch.charge / r;
    }
    return phi;
  }
  real totalPotentialEnergy() const {
    real U = 0;
    for (std::size_t i = 0; i < charges.size(); ++i)
      for (std::size_t j = i + 1; j < charges.size(); ++j) {
        real r = (charges[i].position - charges[j].position).norm();
        if (r < 0.01)
          continue;
        U += phys::k_coulomb * charges[i].charge * charges[j].charge / r;
      }
    return U;
  }

  // Azzera lastEMForce per TUTTI i solidi dinamici. Da chiamare ad inizio
  // frame prima di accumulare Coulomb + Lorentz.
  void resetForces(std::vector<RigidSolidElement> &solids) const {
    for (auto &s : solids) {
      if (s.isStatic)
        continue;
      s.lastEMForce = Vec3(0, 0, 0);
    }
  }

  // Lorentz: campo E/B sui solidi carichi. ACCUMULA (+=).
  void applyToSolids(std::vector<RigidSolidElement> &solids) const {
    for (auto &s : solids) {
      if (s.isStatic || std::abs(s.charge) < 1e-15)
        continue;
      s.lastEMForce = s.lastEMForce + lorentzForce(s.pos, s.vel, s.charge);
    }
  }

  // Coulomb mutuo tra solidi carichi. ACCUMULA (+=).
  void applyCoulombBetweenSolids(std::vector<RigidSolidElement> &solids) const {
    for (std::size_t i = 0; i < solids.size(); ++i) {
      if (solids[i].isStatic)
        continue;
      for (std::size_t j = i + 1; j < solids.size(); ++j) {
        real qi = solids[i].charge, qj = solids[j].charge;
        if (std::abs(qi) < 1e-15 || std::abs(qj) < 1e-15)
          continue;
        Vec3 rVec = solids[i].pos - solids[j].pos;
        real r = rVec.norm();
        if (r < 0.05)
          r = 0.05;
        Vec3 rHat = rVec * (1.0 / r);
        real fMag = phys::k_coulomb * qi * qj / (r * r);
        Vec3 force = rHat * fMag;
        if (!solids[i].isStatic)
          solids[i].lastEMForce = solids[i].lastEMForce + force;
        if (!solids[j].isStatic)
          solids[j].lastEMForce = solids[j].lastEMForce - force;
      }
    }
  }
};

struct QuantumWavepacketField {
  Vec3 center = Vec3(-2.5, 2.0, 1.2);
  real sigma = 0.28;
  real particleMass = phys::m_electron;
  real energyLevel = 1.0;
  Rgb glowColor = {0.3f, 0.85f, 1.0f};

  real comptonWavelength() const {
    return phys::hbar / (particleMass * phys::c);
  }
  real schwarzschildRadius() const {
    return nqg::info::schwarzschildRadiusM(particleMass);
  }
  static real crossingMass() { return nqg::info::crossingMass(); }

  real evaluateDensity(const Vec3 &p) const {
    Vec3 diff = p - center;
    real r2 = diff.dot(diff);
    real normFactor = 1.0 / (std::pow(PI * sigma * sigma, 1.5));
    return normFactor * std::exp(-r2 / (sigma * sigma));
  }
  real heisenbergKineticEnergy() const {
    real deltap = phys::hbar / (2.0 * sigma);
    return deltap * deltap / (2.0 * particleMass);
  }
};

} // namespace continuum
} // namespace nqg

#endif // NQG_CONTINUUM_PHYSICS_HPP