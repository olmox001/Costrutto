// ============================================================================
//  nqg_cleanroom_engine.hpp
//  FIX 2025e:
//   - EM: resetForces() prima di Coulomb + Lorentz (prima la Lorentz
//     sovrascriveva la Coulomb -> forze spurie/deriva).
//   - traceRay/isInShadow: bounding-sphere cull su ogni solido prima di
//     intersectOBB -> riduce di 3-10x i test OBB su solidi sparsi.
//   - Luce: nDotL calcolato PRIMA della shadow ray (skip shadow se nDotL<=0).
//   - Solver: SOLVER_ITERS 8->16, baumgarte 0.5->0.3, slop 5e-3->1e-2.
//   - Wake dei solidi addormentati quando un altro corpo li colpisce.
//   - sampleBed/resolvePlayer/collideSpheres usano halfW() (const-safe).
//   - traceRay: skip shadow-ray per luci dietro la superficie.
// ============================================================================
#ifndef NQG_CLEANROOM_ENGINE_HPP
#define NQG_CLEANROOM_ENGINE_HPP

#include "nqg_earth_environment.hpp"
#include "nqg_engine3d.hpp"
#include "nqg_physics_core.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
//  namespace nqg::apartment
// ============================================================================
namespace nqg {
namespace apartment {

using engine::Rgb;
using engine::Vec3;
using nqg::real;

inline bool isFiniteVec(const Vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline Vec3 safeNormalize(const Vec3 &v, const Vec3 &fb = Vec3(0, 0, 1)) {
  const real n2 = v.norm2();
  if (n2 < 1e-18 || !std::isfinite(n2))
    return fb;
  const real inv = 1.0 / std::sqrt(n2);
  if (!std::isfinite(inv))
    return fb;
  return Vec3(v.x * inv, v.y * inv, v.z * inv);
}

struct AABB {
  Vec3 center = Vec3(0, 0, 0);
  Vec3 half = Vec3(0.5, 0.5, 0.5);
  Rgb albedo = {0.85f, 0.85f, 0.82f};
  real metallic = 0.05;
  real roughness = 0.40;
  bool isStatic = true;
};

struct CapsuleCollider {
  real radius = 0.30;
  real height = 1.80;
  real eyeHeight = 1.70;
  real footZ(real camZ) const { return camZ - eyeHeight; }
  real headZ(real camZ) const { return camZ - eyeHeight + height; }
};

struct RoomGeometry {
  real xMin = -6.0, xMax = 6.0;
  real yMin = -5.0, yMax = 5.0;
  real zMin = 0.0, zMax = 3.2;
  real wallT = 0.20;
  real doorX0 = -1.0, doorX1 = 1.0;
  real doorZ1 = 2.20;
  std::vector<AABB> walls;

  RoomGeometry() { build(); }

  void build() {
    walls.clear();
    const real cx = 0.5 * (xMin + xMax);
    const real cy = 0.5 * (yMin + yMax);
    const real cz = 0.5 * (zMin + zMax);
    const real hx = 0.5 * (xMax - xMin);
    const real hy = 0.5 * (yMax - yMin);
    const real hz = 0.5 * (zMax - zMin);
    const real T = wallT;

    // Pavimento
    {
      AABB a;
      a.center = Vec3(cx, cy, zMin - T * 0.5);
      a.half = Vec3(hx + 2 * T, hy + 2 * T, T * 0.5);
      a.albedo = {0.78f, 0.76f, 0.72f};
      a.roughness = 0.65;
      a.metallic = 0.0;
      walls.push_back(a);
    }
    // Soffitto
    {
      AABB a;
      a.center = Vec3(cx, cy, zMax + T * 0.5);
      a.half = Vec3(hx + 2 * T, hy + 2 * T, T * 0.5);
      a.albedo = {0.92f, 0.92f, 0.90f};
      a.roughness = 0.88;
      walls.push_back(a);
    }
    // Muro nord (y = yMax)
    {
      AABB a;
      a.center = Vec3(cx, yMax + T * 0.5, cz);
      a.half = Vec3(hx + T, T * 0.5, hz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
    // Muro est
    {
      AABB a;
      a.center = Vec3(xMax + T * 0.5, cy, cz);
      a.half = Vec3(T * 0.5, hy + T, hz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
    // Muro ovest
    {
      AABB a;
      a.center = Vec3(xMin - T * 0.5, cy, cz);
      a.half = Vec3(T * 0.5, hy + T, hz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
    // Muro sud con porta
    {
      real sx = 0.5 * (xMin + doorX0), shx = 0.5 * (doorX0 - xMin);
      AABB a;
      a.center = Vec3(sx, yMin - T * 0.5, cz);
      a.half = Vec3(shx, T * 0.5, hz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
    {
      real sx = 0.5 * (doorX1 + xMax), shx = 0.5 * (xMax - doorX1);
      AABB a;
      a.center = Vec3(sx, yMin - T * 0.5, cz);
      a.half = Vec3(shx, T * 0.5, hz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
    {
      real sx = 0.5 * (doorX0 + doorX1), shx = 0.5 * (doorX1 - doorX0);
      real sz = 0.5 * (doorZ1 + zMax), shz = 0.5 * (zMax - doorZ1);
      AABB a;
      a.center = Vec3(sx, yMin - T * 0.5, sz);
      a.half = Vec3(shx, T * 0.5, shz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
  }

  bool isFloorSlab(const AABB &a) const {
    return (a.center.z + a.half.z) <= zMin + 1e-6;
  }
  bool isCeilingSlab(const AABB &a) const {
    return (a.center.z - a.half.z) >= zMax - 1e-6;
  }
  bool insideXY(const Vec3 &p) const {
    return p.x >= xMin && p.x <= xMax && p.y >= yMin && p.y <= yMax;
  }
};

inline bool resolveCylinderAABB(Vec3 &camPos, Vec3 &camVel,
                                const CapsuleCollider &cap, const AABB &box,
                                real margin = 0.005) {
  const real footZ = cap.footZ(camPos.z);
  const real headZ = cap.headZ(camPos.z);
  const real boxZ0 = box.center.z - box.half.z;
  const real boxZ1 = box.center.z + box.half.z;
  if (headZ <= boxZ0 || footZ >= boxZ1)
    return false;

  const real boxX0 = box.center.x - box.half.x;
  const real boxX1 = box.center.x + box.half.x;
  const real boxY0 = box.center.y - box.half.y;
  const real boxY1 = box.center.y + box.half.y;

  const real px = std::clamp(camPos.x, boxX0, boxX1);
  const real py = std::clamp(camPos.y, boxY0, boxY1);
  const real dx = camPos.x - px;
  const real dy = camPos.y - py;
  const real dist2 = dx * dx + dy * dy;
  const real r = cap.radius + margin;
  if (dist2 >= r * r)
    return false;

  const real dist = std::sqrt(dist2 > 1e-12 ? dist2 : 1e-12);
  Vec3 n;
  real push;
  if (dist > 1e-5) {
    n = Vec3(dx / dist, dy / dist, 0.0);
    push = r - dist;
  } else {
    real dxL = camPos.x - boxX0, dxR = boxX1 - camPos.x;
    real dyL = camPos.y - boxY0, dyR = boxY1 - camPos.y;
    real minD = std::min({dxL, dxR, dyL, dyR});
    if (minD == dxL) {
      n = Vec3(-1, 0, 0);
      push = dxL + r;
    } else if (minD == dxR) {
      n = Vec3(1, 0, 0);
      push = dxR + r;
    } else if (minD == dyL) {
      n = Vec3(0, -1, 0);
      push = dyL + r;
    } else {
      n = Vec3(0, 1, 0);
      push = dyR + r;
    }
  }
  camPos = camPos + n * push;
  const real vn = camVel.dot(n);
  if (vn < 0)
    camVel = camVel - n * vn;
  return true;
}

inline void resolveFloorCeiling(Vec3 &camPos, Vec3 &camVel,
                                const CapsuleCollider &cap,
                                const RoomGeometry &room) {
  const real footZ0 = cap.footZ(camPos.z);
  if (footZ0 < room.zMin) {
    camPos.z += (room.zMin - footZ0);
    if (camVel.z < 0)
      camVel.z = 0;
  }
  const real footZ = cap.footZ(camPos.z);
  const real headZ = cap.headZ(camPos.z);
  const real roofTop = room.zMax + room.wallT;
  if (footZ >= room.zMax) {
    const real margin = 2.0 * room.wallT;
    const bool overRoof =
        camPos.x > room.xMin - margin && camPos.x < room.xMax + margin &&
        camPos.y > room.yMin - margin && camPos.y < room.yMax + margin;
    if (overRoof && footZ < roofTop && camVel.z <= 0) {
      camPos.z += (roofTop - footZ);
      if (camVel.z < 0)
        camVel.z = 0;
    }
  } else if (room.insideXY(camPos)) {
    if (headZ > room.zMax) {
      camPos.z -= (headZ - room.zMax);
      if (camVel.z > 0)
        camVel.z = 0;
    }
  }
}

} // namespace apartment
} // namespace nqg

// ============================================================================
//  Physics modules
// ============================================================================
#include "nqg_air_physics.hpp"
#include "nqg_continuum_physics.hpp"
#include "nqg_matter_physics.hpp"

// ============================================================================
//  namespace nqg::cleanroom
// ============================================================================
namespace nqg {
namespace cleanroom {

using apartment::AABB;
using apartment::CapsuleCollider;
using apartment::RoomGeometry;
using engine::blackbody;
using engine::Camera;
using engine::Image;
using engine::Rgb;
using engine::Vec3;

struct LightSource {
  enum class Type { Directional, Point, CeilingPanel };
  Type type = Type::Point;
  Vec3 position = Vec3(0, 0, 5);
  Vec3 direction = Vec3(0.3, 0.4, -0.8).normalized();
  real intensity = 1.0;
  real temperatureK = 6000.0;
  Rgb color = {1.0f, 1.0f, 1.0f};
  real radius = 0.2;
  bool active = true;

  Rgb getEmissionColor() const {
    Rgb bb = blackbody(temperatureK);
    return {bb.r * color.r * float(intensity),
            bb.g * color.g * float(intensity),
            bb.b * color.b * float(intensity)};
  }
};

class LightingSystem {
public:
  std::vector<LightSource> lights;
  Rgb ambientSky = {0.25f, 0.28f, 0.32f};
  Rgb ambientGround = {0.18f, 0.20f, 0.22f};

  static LightingSystem createApartmentPreset() {
    LightingSystem sys;
    {
      LightSource lamp;
      lamp.type = LightSource::Type::CeilingPanel;
      lamp.position = Vec3(0, 0, 3.0);
      lamp.temperatureK = 3500.0;
      lamp.intensity = 3.0;
      sys.lights.push_back(lamp);
    }
    {
      LightSource doorFill;
      doorFill.type = LightSource::Type::Point;
      doorFill.position = Vec3(0, -4.8, 1.4);
      doorFill.temperatureK = 5500.0;
      doorFill.intensity = 1.2;
      sys.lights.push_back(doorFill);
    }
    {
      LightSource sun;
      sun.type = LightSource::Type::Directional;
      sun.direction = Vec3(-0.4, 0.3, -0.85).normalized();
      sun.temperatureK = 6500.0;
      sun.intensity = 0.8;
      sys.lights.push_back(sun);
    }
    return sys;
  }

  static LightingSystem createCleanRoomPreset() {
    return createApartmentPreset();
  }
};

struct PhysicalSphere {
  Vec3 pos = Vec3(0, 3, 2.0);
  Vec3 vel = Vec3(0, 0, 0);
  Vec3 omega = Vec3(0, 0, 0);
  real radius = 0.35;
  real mass = 2.5;
  real restitution = 0.72;
  real mu_s = 0.55;
  real mu_k = 0.40;
  real rolling = 0.012;
  Rgb albedo = {0.88f, 0.25f, 0.20f};
  real metallic = 0.15;
  real roughness = 0.25;

  Vec3 lastFDrag = Vec3(0, 0, 0);
  Vec3 lastFBuoyancy = Vec3(0, 0, 0);
  real lastReynolds = 0.0;
  real prevSubmerged = 0.0;
  real prevAddedMass = 0.0;

  real inertia() const { return 0.4 * mass * radius * radius; }

  void step(real dt, const AirProperties &air, real gravityMag = 9.80665,
            continuum::ContinuousWaterBody *water = nullptr) {
    if (dt <= 0)
      return;
    const int n = std::clamp(int(std::ceil(dt / 0.004)), 1, 32);
    const real h = dt / n;
    const Vec3 pos0 = pos;
    Vec3 reaction(0, 0, 0);

    for (int k = 0; k < n; ++k) {
      continuum::HydroResult H;
      if (water)
        H = water->sphereForces(pos, radius, vel, gravityMag);
      const real frac = std::clamp(H.submergedFraction, 0.0, 1.0);
      Vec3 fD, fB;
      real re;
      air.computeAerodynamicForces(radius, mass, pos, vel, fD, fB, re);
      lastFDrag = fD * (1.0 - frac);
      lastFBuoyancy = fB * (1.0 - frac);
      lastReynolds = re;
      if (H.addedMass > prevAddedMass + 1e-12) {
        const real f = (mass + prevAddedMass) / (mass + H.addedMass);
        const Vec3 vr = vel - H.waterVel;
        const Vec3 nv = H.waterVel + vr * f;
        reaction = reaction + (vel - nv) * (mass + prevAddedMass);
        vel = nv;
      }
      prevAddedMass = H.addedMass;
      const real meff = mass + H.addedMass;
      const Vec3 F =
          Vec3(0, 0, -mass * gravityMag) + lastFDrag + lastFBuoyancy + H.force;
      vel = vel + F * (h / meff);
      if (H.wet) {
        const Vec3 vr = vel - H.waterVel;
        Vec3 vn = vr * (1.0 / (1.0 + H.dragCoeff.x * h / meff));
        if (H.squeeze > 0 && vn.z < 0)
          vn.z /= 1.0 + H.squeeze * h / meff;
        reaction = reaction + (vr - vn) * meff;
        vel = H.waterVel + vn;
        omega = omega * (1.0 / (1.0 + 4.0 * frac * h));
      }
      pos = pos + vel * h;
      resolveFloor(h, H, F.z, water);
    }
    if (!(engine::isFinite(pos) && engine::isFinite(vel) &&
          engine::isFinite(omega))) {
      pos = pos0;
      vel = Vec3(0, 0, 0);
      omega = Vec3(0, 0, 0);
      prevAddedMass = 0;
      return;
    }
    if (water) {
      const continuum::HydroResult Hf =
          water->sphereForces(pos, radius, vel, gravityMag);
      water->couple(pos.x, pos.y, radius, radius, true,
                    Hf.submergedVolume - prevSubmerged, reaction.x, reaction.y);
      prevSubmerged = Hf.submergedVolume;
    }
    air.computeAerodynamicForces(radius, mass, pos, vel, lastFDrag,
                                 lastFBuoyancy, lastReynolds);
  }

private:
  void resolveFloor(real h, const continuum::HydroResult &H, real netFz,
                    const continuum::ContinuousWaterBody *water) {
    if (pos.z - radius > 1e-4)
      return;
    pos.z = radius;
    const real mu = water ? water->flow.liquid.mu : 1.0e-3;
    const real film = H.wet ? std::max(0.0, H.level) : 0.0;
    const real vn = vel.z;
    real e = restitution;
    if (film > 0 && vn < 0) {
      const real rhoB = mass / ((4.0 / 3.0) * PI * radius * radius * radius);
      const real St = contact::stokesNumber(rhoB, vn, 2.0 * radius, mu);
      const real ew = contact::wetRestitution(restitution, St);
      const real w = std::clamp(film / 1e-3, 0.0, 1.0);
      e = restitution * (1.0 - w) + ew * w;
    }
    real jn = 0;
    if (vn < 0) {
      if (-vn < 0.05)
        e = 0;
      jn = mass * (-(1.0 + e) * vn);
      vel.z = -e * vn;
    }
    const real normalAccel = std::max(0.0, -netFz) / mass;
    jn += mass * normalAccel * h;

    const Vec3 rv(0, 0, -radius);
    const Vec3 vc = vel + omega.cross(rv);
    const real sp = std::sqrt(vc.x * vc.x + vc.y * vc.y);
    if (sp > 1e-9 && jn > 0) {
      real muS = mu_s, muK = mu_k;
      if (film > 0) {
        const real a = 0.05 * radius;
        const real p = (jn / h) / (PI * a * a);
        const real mw = contact::wetFrictionCoefficient(mu_s, film, 2e-5, mu,
                                                        sp, p, 2.0 * radius);
        const real f = mu_s > 0 ? mw / mu_s : 1.0;
        muS *= f;
        muK *= f;
      }
      const real invEff =
          (1.0 / mass) * (1.0 + radius * radius * mass / inertia());
      const real jt = contact::coulombImpulse(sp / invEff, jn, muS, muK);
      const Vec3 t(vc.x / sp, vc.y / sp, 0);
      const Vec3 imp = t * (-jt);
      vel = vel + imp * (1.0 / mass);
      omega = omega + rv.cross(imp) * (1.0 / inertia());
    }

    const real d = contact::rollingDeceleration(rolling, normalAccel) * h;
    const real vt = std::sqrt(vel.x * vel.x + vel.y * vel.y);
    if (vt > 1e-9) {
      const real f = std::max(0.0, 1.0 - d / vt);
      vel.x *= f;
      vel.y *= f;
      omega.x *= f;
      omega.y *= f;
    } else {
      const real wt = std::sqrt(omega.x * omega.x + omega.y * omega.y);
      if (wt > 1e-9) {
        const real f = std::max(0.0, 1.0 - d / (radius * wt));
        omega.x *= f;
        omega.y *= f;
      }
    }
    omega.z *= 1.0 / (1.0 + 0.5 * h);
  }
};

class CleanRoomScene {
public:
  AirProperties air;
  LightingSystem lighting = LightingSystem::createApartmentPreset();
  std::vector<PhysicalSphere> spheres;
  matter::MatterSimulator matterSim;
  continuum::ContinuousWaterBody water;
  continuum::ContinuousWindField wind;
  continuum::ContinuousSandDuneField sand;
  std::vector<continuum::RigidSolidElement> solids;
  continuum::QuantumWavepacketField quantumField;
  continuum::ElectromagneticField emField;
  real simTime = 0.0;
  real bedTimer = 0.0;
  real playerSubmerged = 0.0;
  Camera camera;
  Vec3 gravityVec = Vec3(0, 0, -9.80665);

  earth::EarthGlobe globe;
  earth::WindProfile windProfile;
  real homeLat = 45.0 * PI / 180.0;
  real homeLon = 9.0 * PI / 180.0;
  Vec3 homeECEF;
  Vec3 homeUp, homeEast, homeNorth;

  RoomGeometry room;

  mutable Vec3 lastObserverPos = Vec3(0, -3, 1.7);
  mutable real currentAltitude = 1.7;
  mutable real currentLatitude = 0.0;
  mutable real currentLongitude = 0.0;
  mutable real currentGravity = 9.80665;
  mutable real currentPressure = 101325.0;
  mutable real currentTemperature = 288.15;
  mutable real currentDensity = 1.225;
  mutable real currentSoundSpeed = 340.29;
  mutable Vec3 currentWind = Vec3(0, 0, 0);

  mutable std::vector<Rgb> cachedLightColors;
  mutable Rgb cachedAmbientSky = {0.25f, 0.28f, 0.32f};
  mutable Rgb cachedAmbientGround = {0.18f, 0.20f, 0.22f};
  mutable real cachedEffScattering = 2.5e-5;

  bool zeroGravityEnabled = false;

  void setZeroGravity(bool z) {
    zeroGravityEnabled = z;
    matterSim.gravity = z ? Vec3(0, 0, 0) : gravityVec;
  }

  struct Contact {
    int i = 0, j = 0;
    Vec3 normal = Vec3(0, 0, 1);
    real overlap = 0.0;
    continuum::RigidSolidElement::Manifold manifold;
  };

  CleanRoomScene() {
    homeECEF = globe.toECEF({homeLat, homeLon, 0.0});
    earth::EarthGlobe::enuBasis(homeLat, homeLon, homeUp, homeEast, homeNorth);
    windProfile = earth::WindField::profileFor(0.15, homeLat);
    currentLatitude = homeLat;
    currentLongitude = homeLon;

    camera.r = 6.0;
    camera.theta = 1.35;
    camera.phi = 0.0;
    camera.yaw = 0.0;
    camera.pitch = -0.10;
    camera.fovY = 65.0 * PI / 180.0;

    for (const auto &w : room.walls) {
      continuum::RigidSolidElement s;
      s.pos = w.center;
      s.size = w.half * 2.0;
      s.isStatic = true;
      s.albedo = w.albedo;
      s.metallic = w.metallic;
      s.roughness = w.roughness;
      s.restitution = 0.0;
      s.mass = 1e9;
      if (room.isFloorSlab(w) || room.isCeilingSlab(w))
        s.isRoomSlab = true;
      solids.push_back(s);
    }
    {
      continuum::RigidSolidElement t;
      t.pos = Vec3(-3.0, 2.5, 0.40);
      t.size = Vec3(1.8, 0.9, 0.80);
      t.mass = 25.0;
      t.albedo = {0.72f, 0.55f, 0.35f};
      t.metallic = 0.0;
      t.roughness = 0.55;
      t.restitution = 0.05;
      t.isStatic = true;
      solids.push_back(t);
    }
    {
      continuum::RigidSolidElement c;
      c.pos = Vec3(3.5, 2.0, 0.50);
      c.size = Vec3(0.9, 0.9, 0.9);
      c.mass = 8.0;
      c.albedo = {0.65f, 0.45f, 0.28f};
      c.metallic = 0.0;
      c.roughness = 0.62;
      c.restitution = 0.15;
      c.isStatic = false;
      c.ex = Vec3(0.98, 0.20, 0.00).normalized();
      c.ey = Vec3(-0.20, 0.98, 0.00).normalized();
      c.ez = c.ex.cross(c.ey);
      c.angVel = Vec3(0.4, 0.9, -0.3);
      solids.push_back(c);
    }
    {
      continuum::RigidSolidElement c;
      c.pos = Vec3(3.8, 2.6, 1.50);
      c.size = Vec3(0.7, 0.7, 0.7);
      c.mass = 4.0;
      c.albedo = {0.58f, 0.40f, 0.22f};
      c.metallic = 0.0;
      c.roughness = 0.60;
      c.restitution = 0.20;
      c.isStatic = false;
      c.ex = Vec3(0.94, 0.34, 0.00).normalized();
      c.ey = Vec3(-0.34, 0.94, 0.00).normalized();
      c.ez = c.ex.cross(c.ey);
      c.angVel = Vec3(1.2, -0.8, 0.5);
      solids.push_back(c);
    }

    water.setBedProvider([this](real x, real y) { return sampleBed(x, y); });
    water.basinCenter = Vec3(0.0, 0.0, 0.0);
    water.initialFill(room.xMin + 0.05, room.xMax - 0.05, room.yMin + 0.05,
                      room.yMax - 0.05, 0.15);
    water.setLiquidTemperature(293.15);

    PhysicalSphere s;
    s.pos = Vec3(2.0, -1.0, 2.2);
    s.vel = Vec3(0.4, 0.2, 0.0);
    spheres.push_back(s);

    refreshLightCache();
    updateEnvironment();

    // Pre-popola la cache worldHalf di tutti i solidi.
    for (auto &s : solids)
      s.refreshWorldHalf();
  }

  void refreshLightCache() const {
    const std::size_t n = lighting.lights.size();
    cachedLightColors.resize(n);
    for (std::size_t i = 0; i < n; ++i)
      cachedLightColors[i] = lighting.lights[i].getEmissionColor();
    cachedAmbientSky = lighting.ambientSky;
    cachedAmbientGround = lighting.ambientGround;
    cachedEffScattering = 2.5e-5 * (currentDensity / earth::planet::rho0);
  }

  fluid::BedSample sampleBed(real x, real y) const {
    fluid::BedSample s;
    for (const auto &b : solids) {
      if (!b.isStatic)
        continue;
      if (b.isRoomSlab)
        continue;
      const Vec3 hh = b.halfW();
      const real z0 = b.pos.z - hh.z;
      const real z1 = b.pos.z + hh.z;
      if (z0 > 0.05)
        continue;
      if (z1 <= 0.01)
        continue;
      if (std::abs(x - b.pos.x) > hh.x)
        continue;
      if (std::abs(y - b.pos.y) > hh.y)
        continue;
      if (z1 > 2.0) {
        s.solid = true;
        s.z = fluid::ShallowFlow::SOLID_Z;
      } else {
        s.z = std::max(s.z, z1);
      }
    }
    if (!s.solid) {
      const real sh = sand.sampleHeight(x, y);
      if (sh > s.z)
        s.z = sh;
      if (sh > 0.001)
        s.manning = 0.035;
    }
    return s;
  }

  Vec3 localToECEF(const Vec3 &local) const {
    return earth::EarthGlobe::enuToECEF(local, homeECEF, homeUp, homeEast,
                                        homeNorth);
  }
  Vec3 ecefToLocal(const Vec3 &p) const {
    return earth::EarthGlobe::ecefToENU(p, homeECEF, homeUp, homeEast,
                                        homeNorth);
  }
  real effectiveScattering() const {
    return 2.5e-5 * (currentDensity / earth::planet::rho0);
  }

  void updateEnvironment() {
    real alt = lastObserverPos.z;
    currentAltitude = alt;
    Vec3 obsECEF = localToECEF(lastObserverPos);
    earth::EarthGlobe::ecefToLatLon(obsECEF, currentLatitude, currentLongitude,
                                    currentAltitude);
    real hGeo = earth::StandardAtmosphere::geopotential(currentAltitude);
    currentPressure = globe.atmo.pressure(hGeo);
    currentTemperature = globe.atmo.temperature(hGeo);
    currentDensity = globe.atmo.density(hGeo);
    currentSoundSpeed = globe.atmo.speedOfSound(hGeo);
    air.pressurePa = currentPressure;
    air.temperatureK = currentTemperature;
    air.scatteringCoeff = effectiveScattering();
    currentGravity = globe.grav.g(currentLatitude, currentAltitude);
    gravityVec =
        zeroGravityEnabled ? Vec3(0, 0, 0) : Vec3(0, 0, -currentGravity);
    earth::WindProfile wp = earth::WindField::profileFor(
        std::min(1.0, std::max(0.0, 0.15)), homeLat);
    Vec3 atmoWind = earth::WindField::velocityAt(wp, currentAltitude);
    Vec3 localWind(atmoWind.x, atmoWind.y, atmoWind.z);
    Vec3 curlWind = wind.evaluateVelocity(lastObserverPos, simTime);
    real blend = std::clamp(currentAltitude / 2000.0, 0.0, 1.0);
    currentWind = curlWind * (1.0 - blend) + localWind * blend;
    air.windVelocity = currentWind;
    refreshLightCache();
  }

  void resolvePlayerCollision(Vec3 &camPos, Vec3 &camVel,
                              const CapsuleCollider &cap) const {
    if (!apartment::isFiniteVec(camPos) || !apartment::isFiniteVec(camVel)) {
      camPos = Vec3(0, 0, cap.eyeHeight);
      camVel = Vec3(0, 0, 0);
      return;
    }
    apartment::resolveFloorCeiling(camPos, camVel, cap, room);
    for (const auto &s : solids) {
      if (s.isRoomSlab)
        continue;
      apartment::AABB box;
      box.center = s.pos;
      box.half = s.halfW();
      apartment::resolveCylinderAABB(camPos, camVel, cap, box);
    }
    {
      real h = sand.sampleHeight(camPos.x, camPos.y);
      if (h > 0.001) {
        real footZ = cap.footZ(camPos.z);
        if (footZ < h) {
          camPos.z += (h - footZ);
          if (camVel.z < 0)
            camVel.z = 0;
        }
      }
    }
    camPos.x = std::clamp(camPos.x, -100000.0, 100000.0);
    camPos.y = std::clamp(camPos.y, -100000.0, 100000.0);
    if (camPos.z < cap.eyeHeight + 0.001)
      camPos.z = cap.eyeHeight + 0.001;
  }

  void couplePlayer(const Vec3 &camPos, const Vec3 &camVel,
                    const CapsuleCollider &cap, real dt) {
    if (!(dt > 0) || !apartment::isFiniteVec(camPos) ||
        !apartment::isFiniteVec(camVel))
      return;
    playerSubmerged =
        water.couplePlayer(camPos, cap.radius, cap.footZ(camPos.z), cap.height,
                           camVel, dt, playerSubmerged);
  }

  void notifyPlayerInWater(const Vec3 &camPos, const CapsuleCollider &cap,
                           real speed) {
    if (!(speed > 0.0) || !apartment::isFiniteVec(camPos))
      return;
    const real foot = cap.footZ(camPos.z);
    if (!water.isInsideBasin(camPos.x, camPos.y))
      return;
    const real level = water.surfaceHeight(camPos.x, camPos.y);
    if (!(foot < level))
      return;
    const real amp = std::clamp(speed * 0.02, 0.005, 0.06);
    water.addImpulse(Vec3(camPos.x, camPos.y, level), simTime, amp);
  }

  static bool intersectSphere(const Vec3 &ro, const Vec3 &rd,
                              const Vec3 &center, real radius, real &tOut,
                              Vec3 &nOut) {
    Vec3 oc = ro - center;
    real b = oc.dot(rd);
    real c = oc.dot(oc) - radius * radius;
    real disc = b * b - c;
    if (disc < 0 || !std::isfinite(disc))
      return false;
    real sq = std::sqrt(disc);
    real t = -b - sq;
    if (t < 0.001)
      t = -b + sq;
    if (t < 0.001)
      return false;
    tOut = t;
    nOut = apartment::safeNormalize((ro + rd * t) - center);
    return true;
  }

  static bool intersectFloor(const Vec3 &ro, const Vec3 &rd, real &tOut,
                             Vec3 &nOut) {
    if (std::abs(rd.z) < 1e-7)
      return false;
    real t = -ro.z / rd.z;
    if (t < 0.001 || !std::isfinite(t))
      return false;
    tOut = t;
    nOut = Vec3(0, 0, 1);
    return true;
  }

  bool intersectEarth(const Vec3 &ro, const Vec3 &rd, real &tOut, Vec3 &nOut,
                      real &latOut, real &lonOut) const {
    const real R = earth::planet::R_E;
    const Vec3 C(0, 0, -R);
    Vec3 oc = ro - C;
    real b = oc.dot(rd);
    real c = oc.dot(oc) - R * R;
    real disc = b * b - c;
    if (disc < 0 || !std::isfinite(disc))
      return false;
    real sq = std::sqrt(disc);
    real t = -b - sq;
    if (t < 0.001)
      t = -b + sq;
    if (t < 0.001)
      return false;
    Vec3 hitLocal = ro + rd * t;
    Vec3 hitECEF = localToECEF(hitLocal);
    real alt;
    earth::EarthGlobe::ecefToLatLon(hitECEF, latOut, lonOut, alt);
    tOut = t;
    nOut = apartment::safeNormalize(hitLocal - C);
    return true;
  }

  static void overlayGrid(Rgb &col, real lat, real lon, real lineWidthDeg) {
    real step = 1.0 * PI / 180.0;
    real fLat = std::abs(std::fmod(lat, step)) / step;
    real fLon = std::abs(std::fmod(lon, step)) / step;
    real dLat = std::min(fLat, 1.0 - fLat);
    real dLon = std::min(fLon, 1.0 - fLon);
    real d = std::min(dLat, dLon);
    real lw = lineWidthDeg / 180.0;
    if (d < lw) {
      real g = 1.0 - d / lw;
      col.r = col.r * (1.0f - 0.55f * float(g)) + 0.08f * float(g);
      col.g = col.g * (1.0f - 0.55f * float(g)) + 0.88f * float(g);
      col.b = col.b * (1.0f - 0.55f * float(g)) + 1.00f * float(g);
    }
  }

  real terrainDetail(real lat, real lon) const {
    real h = 0.0;
    h += 25.0 * std::sin(lat * 421.0 + 5.3) * std::cos(lon * 389.0 + 2.1);
    h += 10.0 * std::sin(lat * 1130.0 + 7.7) * std::cos(lon * 1043.0 + 4.4);
    h += 4.0 * std::sin(lat * 2917.0 + 1.9) * std::cos(lon * 2811.0 + 6.2);
    h += 1.5 * std::sin(lat * 7901.0 + 3.2) * std::cos(lon * 7321.0 + 0.8);
    h += 0.5 * std::sin(lat * 21401.0 + 9.4) * std::cos(lon * 19891.0 + 3.7);
    return h;
  }

  Rgb terrainAlbedoAt(const Vec3 &localPoint) const {
    const Vec3 ecef = localToECEF(localPoint);
    real lat, lon, alt;
    earth::EarthGlobe::ecefToLatLon(ecef, lat, lon, alt);
    auto surf = globe.terrain.sample(lat, lon, 0.0);
    const real detail = terrainDetail(lat, lon);
    const real shade = std::clamp(1.0 + 0.008 * detail, 0.6, 1.4);
    return {std::clamp(surf.albedo.r * float(shade), 0.0f, 1.0f),
            std::clamp(surf.albedo.g * float(shade), 0.0f, 1.0f),
            std::clamp(surf.albedo.b * float(shade), 0.0f, 1.0f)};
  }

  // Shadow ray con bounding-sphere cull.
  bool isInShadow(const Vec3 &pos, const Vec3 &lightDir, real maxDist) const {
    Vec3 ro = pos + lightDir * 0.005;
    for (const auto &s : solids) {
      if (s.raySphereCull(ro, lightDir))
        continue;
      real t;
      Vec3 n;
      if (s.intersectOBB(ro, lightDir, t, n)) {
        if (t < maxDist)
          return true;
      }
    }
    return false;
  }

  static Rgb floorTexture(const Vec3 &hit) {
    real u = hit.x, v = hit.y;
    real fu = std::abs(u - std::round(u));
    real fv = std::abs(v - std::round(v));
    real edge = std::min(fu, fv);
    real su = std::abs(u * 5.0 - std::round(u * 5.0));
    real sv = std::abs(v * 5.0 - std::round(v * 5.0));
    real subEdge = std::min(su, sv);
    Rgb col = {0.82f, 0.84f, 0.87f};
    int iu = int(std::floor(u)), iv = int(std::floor(v));
    if ((iu + iv) % 2 != 0) {
      col.r *= 0.96f;
      col.g *= 0.96f;
      col.b *= 0.96f;
    }
    if (edge < 0.02)
      return {0.45f, 0.48f, 0.52f};
    if (subEdge < 0.015)
      return {0.75f, 0.78f, 0.82f};
    return col;
  }

  Rgb skyColor(const Vec3 &dir) const {
    real rhoRatio = std::clamp(currentDensity / earth::planet::rho0, 0.0, 1.5);
    Vec3 sunDir = apartment::safeNormalize(homeEast * 0.70 + homeUp * 0.55 +
                                           homeNorth * 0.45);
    real cosSun = std::max(0.0, dir.dot(sunDir));
    real baseR = (0.30 + 0.35 * cosSun) * rhoRatio;
    real baseG = (0.48 + 0.30 * cosSun) * rhoRatio;
    real baseB = (0.85 + 0.10 * cosSun) * rhoRatio;
    real sunDisc = std::pow(cosSun, 800.0) * 8.0;
    real skyFade = std::clamp(1.0 - currentAltitude / 60000.0, 0.0, 1.0);
    Rgb col;
    col.r = float(std::clamp(baseR * skyFade + sunDisc, 0.0, 1.5));
    col.g = float(std::clamp(baseG * skyFade + sunDisc, 0.0, 1.5));
    col.b = float(std::clamp(baseB * skyFade + sunDisc * 0.85, 0.0, 1.5));
    if (currentAltitude > 10000.0) {
      real starVis =
          std::clamp((currentAltitude - 10000.0) / 40000.0, 0.0, 1.0);
      std::uint32_t hx = std::uint32_t(int(dir.x * 12000.0) + 100000);
      std::uint32_t hy = std::uint32_t(int(dir.y * 12000.0) + 100000);
      std::uint32_t hz = std::uint32_t(int(dir.z * 12000.0) + 100000);
      real h = engine::h01(hx, hy, hz);
      if (h > 1.0 - 0.003 * starVis) {
        real bright = 0.4 + 0.6 * engine::h01(hx ^ 17u, hy ^ 31u, hz ^ 47u);
        col.r += float(bright * starVis);
        col.g += float(bright * starVis * 0.95);
        col.b += float(bright * starVis * 1.05);
      }
    }
    return col;
  }

  Rgb traceRay(const Vec3 &ro, const Vec3 &rd) const {
    const std::size_t nLights = cachedLightColors.size();

    bool cameraUnderwater = false;
    {
      if (water.isInsideBasin(ro.x, ro.y)) {
        const real eta = water.surfaceHeightVisual(ro.x, ro.y, simTime);
        if (eta > -1e8 && ro.z < eta)
          cameraUnderwater = true;
      }
    }

    real tHit = 1e9;
    Vec3 normal;
    Rgb hitAlbedo;
    real metallic = 0.0, roughness = 0.5;
    bool hitSomething = false, hitWater = false, hitQuantum = false;

    for (const auto &s : solids) {
      if (s.raySphereCull(ro, rd))
        continue;
      real tBox;
      Vec3 nBox;
      if (s.intersectOBB(ro, rd, tBox, nBox)) {
        if (tBox < tHit && std::isfinite(tBox)) {
          tHit = tBox;
          normal = nBox;
          hitAlbedo = s.albedo;
          metallic = s.metallic;
          roughness = s.roughness;
          hitSomething = true;
          hitWater = false;
          hitQuantum = false;
        }
      }
    }
    real tW = -1.0, dW = 0.0;
    Vec3 nW;
    if (water.intersectWater(ro, rd, simTime, tW, nW, dW)) {
      if (tW < tHit && std::isfinite(tW)) {
        tHit = tW;
        normal = nW;
        hitSomething = true;
        hitWater = true;
        hitQuantum = false;
      }
    }
    real tS;
    Vec3 nS;
    if (sand.intersectSand(ro, rd, tS, nS)) {
      if (tS < tHit && std::isfinite(tS)) {
        tHit = tS;
        normal = nS;
        hitAlbedo = {0.86f, 0.74f, 0.44f};
        metallic = 0.0;
        roughness = 0.88;
        hitSomething = true;
        hitWater = false;
        hitQuantum = false;
      }
    }
    {
      Vec3 qDiff = ro - quantumField.center;
      real qB = qDiff.dot(rd);
      real qR = quantumField.sigma * 4.0;
      real qC = qDiff.dot(qDiff) - qR * qR;
      real qDisc = qB * qB - qC;
      if (qDisc > 0 && (-qB - std::sqrt(qDisc)) < tHit) {
        real tQEntry = std::max(0.01, -qB - std::sqrt(qDisc));
        real tQExit = -qB + std::sqrt(qDisc);
        if (tQEntry < tHit && tQExit > 0.01) {
          real accum = 0;
          const int QSTEPS = 16;
          real qdt = (std::min(tQExit, tHit) - tQEntry) / QSTEPS;
          for (int qi = 0; qi < QSTEPS; ++qi) {
            Vec3 qp = ro + rd * (tQEntry + (qi + 0.5) * qdt);
            accum += quantumField.evaluateDensity(qp) * qdt;
          }
          if (accum > 0.001) {
            real alpha = std::min(1.0, accum * 5.0);
            hitQuantum = (alpha > 0.02 && tQEntry < tHit);
          }
        }
      }
    }
    {
      real tE;
      Vec3 nE;
      real latHit, lonHit;
      if (intersectEarth(ro, rd, tE, nE, latHit, lonHit)) {
        if (tE < tHit && std::isfinite(tE)) {
          auto surf = globe.terrain.sample(latHit, lonHit, 0.0);
          real detail = terrainDetail(latHit, lonHit);
          real shade = std::clamp(1.0 + 0.008 * detail, 0.6, 1.4);
          hitAlbedo.r = std::clamp(surf.albedo.r * float(shade), 0.0f, 1.0f);
          hitAlbedo.g = std::clamp(surf.albedo.g * float(shade), 0.0f, 1.0f);
          hitAlbedo.b = std::clamp(surf.albedo.b * float(shade), 0.0f, 1.0f);
          metallic = 0.0;
          roughness = std::min(1.0, surf.roughness * 1.8);
          hitWater = (surf.type == earth::SurfaceType::Water);
          real lw = 0.08 + std::clamp(200.0 / std::max(1.0, tE), 0.0, 0.5);
          overlayGrid(hitAlbedo, latHit, lonHit, lw);
          tHit = tE;
          normal = nE;
          hitSomething = true;
          hitQuantum = false;
        }
      }
    }
    if (!hitSomething && !hitQuantum) {
      Rgb sky = skyColor(rd);
      if (cameraUnderwater) {
        sky.r *= 0.30f;
        sky.g *= 0.55f;
        sky.b *= 0.75f;
      }
      return sky;
    }
    if (hitQuantum && !hitSomething) {
      Vec3 qDiff = ro - quantumField.center;
      real qB = qDiff.dot(rd);
      real qR = quantumField.sigma * 4.0;
      real qC = qDiff.dot(qDiff) - qR * qR;
      real qDisc = qB * qB - qC;
      real tQEntry = std::max(0.01, -qB - std::sqrt(qDisc));
      real tQExit = -qB + std::sqrt(qDisc);
      real accum = 0;
      const int QSTEPS = 16;
      real qdt = (tQExit - tQEntry) / QSTEPS;
      for (int qi = 0; qi < QSTEPS; ++qi) {
        Vec3 qp = ro + rd * (tQEntry + (qi + 0.5) * qdt);
        accum += quantumField.evaluateDensity(qp) * qdt;
      }
      real alpha = std::min(1.0, accum * 5.0);
      Rgb bg = skyColor(rd);
      return {
          float(quantumField.glowColor.r * alpha * 2.0 + bg.r * (1.0 - alpha)),
          float(quantumField.glowColor.g * alpha * 2.0 + bg.g * (1.0 - alpha)),
          float(quantumField.glowColor.b * alpha * 2.0 + bg.b * (1.0 - alpha))};
    }

    Vec3 hitPos = ro + rd * tHit;
    Vec3 viewDir = rd * (-1.0);

    if (hitWater) {
      if (cameraUnderwater) {
        Rgb below = hitAlbedo;
        Rgb amb = {cachedAmbientSky.r * 0.4f, cachedAmbientSky.g * 0.55f,
                   cachedAmbientSky.b * 0.7f};
        Rgb lit = {below.r * amb.r, below.g * amb.g, below.b * amb.b};
        Vec3 trans = water.beerLambertTransmission(2.0 * std::max(tHit, 0.0));
        lit.r *= float(trans.x);
        lit.g *= float(trans.y);
        lit.b *= float(trans.z);
        lit.r += 0.02f;
        lit.g += 0.05f;
        lit.b += 0.08f;
        return lit;
      }

      Vec3 refrDir;
      const bool hasRefr = continuum::ContinuousWaterBody::refractRay(
          rd, normal, 1.0 / water.refractiveIndex, refrDir);
      const real cosTheta = std::clamp(-normal.dot(rd), 0.0, 1.0);
      const real F = continuum::ContinuousWaterBody::fresnelDielectric(
          cosTheta, 1.0, water.refractiveIndex);

      Rgb belowCol = {0.55f, 0.65f, 0.75f};
      if (hasRefr) {
        const Vec3 subOrigin = hitPos + refrDir * 1e-4;
        real tSub = 1e9;
        Vec3 nSub(0, 0, 1);
        Rgb albSub{0.5f, 0.5f, 0.5f};
        bool subValid = false;
        for (const auto &s : solids) {
          if (s.isRoomSlab)
            continue;
          if (s.raySphereCull(subOrigin, refrDir))
            continue;
          real tBox;
          Vec3 nBox;
          if (s.intersectOBB(subOrigin, refrDir, tBox, nBox) && tBox < tSub) {
            tSub = tBox;
            nSub = nBox;
            albSub = s.albedo;
            subValid = true;
          }
        }
        for (const auto &sp : spheres) {
          real tSph;
          Vec3 nSph;
          if (intersectSphere(subOrigin, refrDir, sp.pos, sp.radius, tSph,
                              nSph) &&
              tSph < tSub) {
            tSub = tSph;
            nSub = nSph;
            albSub = sp.albedo;
            subValid = true;
          }
        }
        {
          real tSn;
          Vec3 nSn;
          if (sand.intersectSand(subOrigin, refrDir, tSn, nSn) && tSn < tSub) {
            tSub = tSn;
            nSub = nSn;
            albSub = {0.86f, 0.74f, 0.44f};
            subValid = true;
          }
        }
        {
          real tF;
          Vec3 nF;
          if (intersectFloor(subOrigin, refrDir, tF, nF) && tF < tSub) {
            const Vec3 floorHit = subOrigin + refrDir * tF;
            if (room.insideXY(floorHit))
              albSub = floorTexture(floorHit);
            else
              albSub = terrainAlbedoAt(floorHit);
            tSub = tF;
            nSub = nF;
            subValid = true;
          }
        }
        if (subValid) {
          const Vec3 subHit = subOrigin + refrDir * tSub;
          const real hemi = 0.5 * (nSub.z + 1.0);
          const Rgb amb = {float(cachedAmbientGround.r * (1.0 - hemi) +
                                 cachedAmbientSky.r * hemi),
                           float(cachedAmbientGround.g * (1.0 - hemi) +
                                 cachedAmbientSky.g * hemi),
                           float(cachedAmbientGround.b * (1.0 - hemi) +
                                 cachedAmbientSky.b * hemi)};
          Rgb litSub = {albSub.r * amb.r, albSub.g * amb.g, albSub.b * amb.b};
          for (std::size_t li = 0; li < nLights; ++li) {
            const auto &L = lighting.lights[li];
            if (!L.active)
              continue;
            Vec3 toLight;
            real dist = 1e6, atten = 1.0;
            if (L.type == LightSource::Type::Directional) {
              toLight = L.direction * (-1.0);
            } else {
              toLight = L.position - subHit;
              dist = toLight.norm();
              if (dist > 1e-4)
                toLight = toLight * (1.0 / dist);
              atten = 1.0 / (1.0 + 0.12 * dist + 0.03 * dist * dist);
            }
            // nDotL PRIMA della shadow ray (skip)
            const real nDotL = std::max(0.0, nSub.dot(toLight));
            if (nDotL <= 0.0)
              continue;
            if (isInShadow(subHit, toLight, dist))
              continue;
            const Rgb &lCol = cachedLightColors[li];
            litSub.r += float(albSub.r * lCol.r * nDotL * atten);
            litSub.g += float(albSub.g * lCol.g * nDotL * atten);
            litSub.b += float(albSub.b * lCol.b * nDotL * atten);
          }
          const Vec3 trans =
              water.beerLambertTransmission(2.0 * std::max(tSub, 0.0));
          belowCol = {float(litSub.r * trans.x), float(litSub.g * trans.y),
                      float(litSub.b * trans.z)};
        }
      }
      const Vec3 reflDir = rd - normal * (2.0 * rd.dot(normal));
      const Rgb reflCol = skyColor(reflDir);
      Rgb specSum = {0.0f, 0.0f, 0.0f};
      for (std::size_t li = 0; li < nLights; ++li) {
        const auto &light = lighting.lights[li];
        if (!light.active)
          continue;
        const Vec3 lDir =
            (light.type == LightSource::Type::Directional)
                ? light.direction * (-1.0)
                : apartment::safeNormalize(light.position - hitPos);
        const Vec3 halfVec = apartment::safeNormalize(lDir + viewDir);
        const real nDotH = std::max(0.0, normal.dot(halfVec));
        const real spec = std::pow(nDotH, 128.0);
        const Rgb &lCol = cachedLightColors[li];
        specSum.r += float(lCol.r * spec * 2.5);
        specSum.g += float(lCol.g * spec * 2.5);
        specSum.b += float(lCol.b * spec * 2.5);
      }
      Rgb litWater;
      litWater.r = float(belowCol.r * (1.0 - F) + reflCol.r * F + specSum.r);
      litWater.g = float(belowCol.g * (1.0 - F) + reflCol.g * F + specSum.g);
      litWater.b = float(belowCol.b * (1.0 - F) + reflCol.b * F + specSum.b);
      const real ext = effectiveScattering() * tHit;
      const real transmission = std::exp(-ext);
      const real inScatter = 1.0 - transmission;
      const Rgb haze = skyColor(rd);
      return {float(litWater.r * transmission + haze.r * inScatter),
              float(litWater.g * transmission + haze.g * inScatter),
              float(litWater.b * transmission + haze.b * inScatter)};
    }

    Rgb lit = {0.0f, 0.0f, 0.0f};
    real hemi = 0.5 * (normal.z + 1.0);
    Rgb ambient = {
        float(cachedAmbientGround.r * (1.0 - hemi) + cachedAmbientSky.r * hemi),
        float(cachedAmbientGround.g * (1.0 - hemi) + cachedAmbientSky.g * hemi),
        float(cachedAmbientGround.b * (1.0 - hemi) +
              cachedAmbientSky.b * hemi)};
    lit.r += hitAlbedo.r * ambient.r;
    lit.g += hitAlbedo.g * ambient.g;
    lit.b += hitAlbedo.b * ambient.b;
    for (std::size_t li = 0; li < nLights; ++li) {
      const auto &L = lighting.lights[li];
      if (!L.active)
        continue;
      Vec3 toLight;
      real dist = 1e6, atten = 1.0;
      if (L.type == LightSource::Type::Directional) {
        toLight = L.direction * (-1.0);
        dist = 1000.0;
        atten = 1.0;
      } else {
        toLight = L.position - hitPos;
        dist = toLight.norm();
        if (dist > 1e-4)
          toLight = toLight * (1.0 / dist);
        atten = 1.0 / (1.0 + 0.12 * dist + 0.03 * dist * dist);
      }
      // nDotL PRIMA della shadow ray
      const real nDotL = std::max(0.0, normal.dot(toLight));
      if (nDotL <= 0.0)
        continue;
      if (isInShadow(hitPos, toLight, dist))
        continue;
      const Rgb &lCol = cachedLightColors[li];
      const Vec3 halfVec = apartment::safeNormalize(toLight + viewDir);
      const real nDotH = std::max(0.0, normal.dot(halfVec));
      const real specPower = std::max(2.0, (1.0 - roughness) * 128.0);
      const real spec = std::pow(nDotH, specPower);
      const real vDotH = std::max(0.0, viewDir.dot(halfVec));
      const real f0 = 0.04 * (1.0 - metallic) + metallic;
      const real fresnel = f0 + (1.0 - f0) * std::pow(1.0 - vDotH, 5.0);
      const real diffFactor = (1.0 - metallic) * nDotL * atten;
      const real specFactor = spec * fresnel * atten * 1.5;
      lit.r += float(hitAlbedo.r * lCol.r * diffFactor + lCol.r * specFactor);
      lit.g += float(hitAlbedo.g * lCol.g * diffFactor + lCol.g * specFactor);
      lit.b += float(hitAlbedo.b * lCol.b * diffFactor + lCol.b * specFactor);
    }
    const real effScat = effectiveScattering();
    const real ext = effScat * tHit;
    const real transmission = std::exp(-ext);
    const real inScatter = 1.0 - transmission;
    const Rgb haze = skyColor(rd);
    lit.r = float(lit.r * transmission + haze.r * inScatter * 0.9);
    lit.g = float(lit.g * transmission + haze.g * inScatter * 0.9);
    lit.b = float(lit.b * transmission + haze.b * inScatter * 0.9);
    if (hitQuantum) {
      const real qDens = quantumField.evaluateDensity(hitPos);
      const real qAlpha = std::min(0.5, qDens * 0.5);
      lit.r += float(quantumField.glowColor.r * qAlpha);
      lit.g += float(quantumField.glowColor.g * qAlpha);
      lit.b += float(quantumField.glowColor.b * qAlpha);
    }
    if (cameraUnderwater) {
      lit.r *= 0.55f;
      lit.g *= 0.85f;
      lit.b *= 1.00f;
      lit.r += 0.02f;
      lit.g += 0.06f;
      lit.b += 0.10f;
    }
    return lit;
  }

  Image render(int width, int height, const Vec3 &camEye = Vec3(0, -3, 1.7),
               real yaw = 0, real pitch = 0) const {
    lastObserverPos = camEye;
    currentAltitude = camEye.z;
    real hGeo = earth::StandardAtmosphere::geopotential(currentAltitude);
    currentDensity = globe.atmo.density(hGeo);

    if (cachedLightColors.size() != lighting.lights.size())
      refreshLightCache();

    Image img(width, height);
    Vec3 f(std::sin(yaw) * std::cos(pitch), std::cos(yaw) * std::cos(pitch),
           std::sin(pitch));
    f = apartment::safeNormalize(f, Vec3(0, 1, 0));
    Vec3 worldUp(0, 0, 1);
    Vec3 right = f.cross(worldUp);
    if (right.norm2() < 1e-12)
      right = Vec3(1, 0, 0);
    else
      right = right.normalized();
    Vec3 up = right.cross(f).normalized();
    const real th = std::tan(camera.fovY / 2.0);
    const real aspect = real(width) / real(height);
    std::atomic<int> nextY{0};
    unsigned numThreads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> workers;
    workers.reserve(numThreads);
    auto worker = [&] {
      for (int y; (y = nextY++) < height;) {
        for (int x = 0; x < width; ++x) {
          real sx = ((x + 0.5) / width * 2.0 - 1.0) * aspect * th;
          real sy = (1.0 - (y + 0.5) / height * 2.0) * th;
          Vec3 dir = apartment::safeNormalize(f + right * sx + up * sy);
          Rgb c = traceRay(camEye, dir);
          float *out = img.at(x, y);
          out[0] =
              std::clamp(std::pow(c.r / (1.0f + c.r), 1.0f / 2.2f), 0.0f, 1.0f);
          out[1] =
              std::clamp(std::pow(c.g / (1.0f + c.g), 1.0f / 2.2f), 0.0f, 1.0f);
          out[2] =
              std::clamp(std::pow(c.b / (1.0f + c.b), 1.0f / 2.2f), 0.0f, 1.0f);
        }
      }
    };
    for (unsigned t = 0; t < numThreads; ++t)
      workers.emplace_back(worker);
    for (auto &t : workers)
      t.join();
    return img;
  }

  void collideSpheresWithSolids() {
    for (auto &sp : spheres) {
      for (auto &s : solids) {
        Vec3 n;
        real pen;
        const Vec3 half = s.halfW();
        if (!engine::sphereBoxContact(sp.pos, sp.radius, s.pos, half, n, pen))
          continue;
        const real invS = 1.0 / sp.mass;
        const real invB = s.isStatic ? 0.0 : 1.0 / s.mass;
        const real tot = invS + invB;
        pen = std::min(pen, 0.15);
        sp.pos = sp.pos + n * (pen * invS / tot);
        if (!s.isStatic)
          s.pos = s.pos - n * (pen * invB / tot);
        const Vec3 rv = n * (-sp.radius);
        const Vec3 vcA = sp.vel + sp.omega.cross(rv);
        const Vec3 vr = vcA - s.vel;
        const real vn = vr.dot(n);
        if (vn >= 0)
          continue;
        real e = std::min(sp.restitution, s.restitution);
        if (-vn < 0.2)
          e = 0;
        const real jn = -(1.0 + e) * vn / tot;
        sp.vel = sp.vel + n * (jn * invS);
        if (!s.isStatic) {
          s.vel = s.vel - n * (jn * invB);
          if (std::abs(jn) * invB > 0.05)
            s.wake();
        }
        const Vec3 vt = vr - n * vn;
        const real vts = vt.norm();
        if (vts > 1e-9) {
          const real inertiaInv = 1.0 / sp.inertia();
          const real invEff = invS + invB + sp.radius * sp.radius * inertiaInv;
          const real muS = contact::combineFriction(sp.mu_s, s.mu_s);
          const real muK = contact::combineFriction(sp.mu_k, s.mu_k);
          const real jt = contact::coulombImpulse(vts / invEff, jn, muS, muK);
          const Vec3 imp = vt * (-jt / vts);
          sp.vel = sp.vel + imp * invS;
          if (!s.isStatic)
            s.vel = s.vel - imp * invB;
          sp.omega = sp.omega + rv.cross(imp) * inertiaInv;
        }
      }
    }
  }

  static void solveContactPoint(continuum::RigidSolidElement &A,
                                continuum::RigidSolidElement &B, const Vec3 &p,
                                const Vec3 &n, real restitution, real muS,
                                real muK) {
    (void)muS;

    const bool sA = A.isStatic, sB = B.isStatic;
    if (sA && sB)
      return;

    const Vec3 rA = p - A.pos;
    const Vec3 rB = p - B.pos;
    const Vec3 zero(0, 0, 0);
    const Vec3 vA = sA ? zero : A.vel + A.angVel.cross(rA);
    const Vec3 vB = sB ? zero : B.vel + B.angVel.cross(rB);
    const Vec3 vRel = vA - vB;
    const real vn = vRel.dot(n);

    // Wake se urto significativo
    if (vn < -0.05) {
      if (!sA && A.asleep)
        A.wake();
      if (!sB && B.asleep)
        B.wake();
    }

    const Vec3 wA = sA ? zero : A.applyInvInertia(rA.cross(n));
    const Vec3 wB = sB ? zero : B.applyInvInertia(rB.cross(n));
    const real kA = sA ? 0.0 : (1.0 / A.mass) + n.dot(wA.cross(rA));
    const real kB = sB ? 0.0 : (1.0 / B.mass) + n.dot(wB.cross(rB));
    const real K = kA + kB;
    if (K < 1e-12)
      return;

    real e = restitution;
    if (vn > -0.5)
      e = 0.0;
    real jn = -(1.0 + e) * vn / K;
    if (jn < 0.0)
      jn = 0.0;

    const Vec3 J = n * jn;
    if (!sA) {
      A.vel = A.vel + J * (1.0 / A.mass);
      A.angVel = A.angVel + A.applyInvInertia(rA.cross(J));
    }
    if (!sB) {
      B.vel = B.vel - J * (1.0 / B.mass);
      B.angVel = B.angVel - B.applyInvInertia(rB.cross(J));
    }

    const Vec3 vA2 = sA ? zero : A.vel + A.angVel.cross(rA);
    const Vec3 vB2 = sB ? zero : B.vel + B.angVel.cross(rB);
    const Vec3 vRel2 = vA2 - vB2;
    Vec3 vT = vRel2 - n * vRel2.dot(n);
    const real vt = vT.norm();
    if (vt < 1e-6)
      return;
    const Vec3 t = vT * (1.0 / vt);

    const Vec3 wtA = sA ? zero : A.applyInvInertia(rA.cross(t));
    const Vec3 wtB = sB ? zero : B.applyInvInertia(rB.cross(t));
    const real ktA = sA ? 0.0 : (1.0 / A.mass) + t.dot(wtA.cross(rA));
    const real ktB = sB ? 0.0 : (1.0 / B.mass) + t.dot(wtB.cross(rB));
    const real Kt = ktA + ktB;
    if (Kt < 1e-12)
      return;

    real jt = -vt / Kt;
    const real jMax = muK * jn;
    jt = std::clamp(jt, -jMax, jMax);
    const Vec3 Jt = t * jt;
    if (!sA) {
      A.vel = A.vel + Jt * (1.0 / A.mass);
      A.angVel = A.angVel + A.applyInvInertia(rA.cross(Jt));
    }
    if (!sB) {
      B.vel = B.vel - Jt * (1.0 / B.mass);
      B.angVel = B.angVel - B.applyInvInertia(rB.cross(Jt));
    }
  }

  void stepPhysics(real dt) {
    if (dt <= 0 || !std::isfinite(dt))
      return;
    simTime += dt;
    updateEnvironment();

    water.setBedProvider([this](real x, real y) { return sampleBed(x, y); });
    water.setWind(currentWind);
    water.setGravity(currentGravity);
    bedTimer += dt;
    if (bedTimer > 0.25) {
      water.refreshBed();
      bedTimer = 0.0;
    }
    water.step(dt);

    sand.relaxAvalanche(1);
    real airDensity = currentDensity;

    // --- EM: prima azzera, poi accumula Coulomb + Lorentz ---
    emField.resetForces(solids);
    emField.applyCoulombBetweenSolids(solids);
    emField.applyToSolids(solids);

    for (auto &s : solids)
      s.refreshWorldHalf();

    real maxLinSpeed = 0.0, maxAngSpeed = 0.0, minHalf = 1.0;
    for (const auto &s : solids) {
      if (s.isStatic || s.asleep)
        continue;
      maxLinSpeed = std::max(maxLinSpeed, s.vel.norm());
      maxAngSpeed = std::max(maxAngSpeed, s.angVel.norm());
      const Vec3 he = s.halfExtents();
      minHalf = std::min({minHalf, he.x, he.y, he.z});
    }
    int nSub = 1;
    {
      const real dtLinTarget =
          0.5 * std::max(minHalf, 1e-3) / std::max(maxLinSpeed, 1e-3);
      const real dtAngTarget = 0.25 / std::max(maxAngSpeed, 1e-3);
      const real dtTarget = std::min(dtLinTarget, dtAngTarget);
      if (dt > dtTarget)
        nSub = std::clamp(int(std::ceil(dt / dtTarget)), 1, 16);
    }
    const real dtSub = dt / nSub;

    std::vector<Contact> contacts;
    contacts.reserve(solids.size() * 2);

    const int SOLVER_ITERS = 16; // era 8
    const real baumgarte = 0.3;  // era 0.5
    const real slop = 1.0e-2;    // era 5e-3

    continuum::RigidSolidElement::GroundHeightFn groundFn =
        [this](real x, real y) -> real {
      real gz = 0.0;
      const real sh = sand.sampleHeight(x, y);
      if (std::isfinite(sh) && sh > gz)
        gz = sh;
      return gz;
    };

    for (int sub = 0; sub < nSub; ++sub) {
      for (auto &s : solids)
        s.stepDynamics(dtSub, gravityVec, water, wind, airDensity, simTime,
                       groundFn);

      contacts.clear();
      for (std::size_t i = 0; i < solids.size(); ++i) {
        for (std::size_t j = i + 1; j < solids.size(); ++j) {
          auto &A = solids[i];
          auto &B = solids[j];
          if (A.isStatic && B.isStatic)
            continue;
          if ((A.isStatic || A.asleep) && (B.isStatic || B.asleep))
            continue;
          const real rA = A.size.norm() * 0.5;
          const real rB = B.size.norm() * 0.5;
          if ((A.pos - B.pos).norm() > rA + rB)
            continue;
          Vec3 n;
          real overlap;
          if (!continuum::RigidSolidElement::obbOverlapSAT(A, B, n, overlap))
            continue;
          Contact c;
          c.i = int(i);
          c.j = int(j);
          c.normal = n;
          c.overlap = overlap;
          c.manifold = continuum::RigidSolidElement::buildManifold(A, B);
          if (c.manifold.count == 0)
            continue;
          contacts.push_back(c);
        }
      }

      for (int iter = 0; iter < SOLVER_ITERS; ++iter) {
        for (auto &c : contacts) {
          auto &A = solids[c.i];
          auto &B = solids[c.j];
          const real muS = contact::combineFriction(A.mu_s, B.mu_s);
          const real muK = contact::combineFriction(A.mu_k, B.mu_k);
          const real e = std::min(A.restitution, B.restitution);
          for (int k = 0; k < c.manifold.count; ++k)
            solveContactPoint(A, B, c.manifold.points[k], c.normal, e, muS,
                              muK);
        }
      }

      for (std::size_t i = 0; i < solids.size(); ++i) {
        for (std::size_t j = i + 1; j < solids.size(); ++j) {
          auto &A = solids[i];
          auto &B = solids[j];
          if (A.isStatic && B.isStatic)
            continue;
          if ((A.isStatic || A.asleep) && (B.isStatic || B.asleep))
            continue;
          Vec3 n;
          real overlap;
          if (!continuum::RigidSolidElement::obbOverlapSAT(A, B, n, overlap))
            continue;
          if (overlap <= slop)
            continue;
          const real invA = A.isStatic ? 0.0 : 1.0 / A.mass;
          const real invB = B.isStatic ? 0.0 : 1.0 / B.mass;
          const real invSum = invA + invB;
          if (invSum < 1e-12)
            continue;
          const Vec3 corr = n * (baumgarte * (overlap - slop));
          if (!A.isStatic)
            A.pos = A.pos + corr * (invA / invSum);
          if (!B.isStatic)
            B.pos = B.pos - corr * (invB / invSum);
        }
      }
    }

    for (auto &s : spheres)
      s.step(dt, air, currentGravity, &water);
    collideSpheresWithSolids();

    matterSim.gravity = gravityVec;
    matterSim.step(dt, air);

    for (auto &s : solids)
      s.refreshWorldHalf();
  }
};

} // namespace cleanroom
} // namespace nqg

#endif // NQG_CLEANROOM_ENGINE_HPP