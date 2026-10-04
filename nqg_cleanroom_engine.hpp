// ============================================================================
//  nqg_cleanroom_engine.hpp  -  Appartamento su Globo Terrestre
//  (SELF-CONTAINED)
//  ---------------------------------------------------------------------------
//  TUTTO IN UN SOLO FILE. Non dipende da nqg_apartment.hpp.
//
//  Contenuto:
//    namespace nqg::apartment  -> utility + geometria stanza + collisione
//    namespace nqg::cleanroom  -> illuminazione + scena + ray tracing
//
//  FIX applicati:
//    - Polo: lat/lon via ECEF + atan2 (nessuna distorsione)
//    - Soffitto: collisione SOLO dentro il footprint XY della stanza
//    - Hardening: guardie NaN/Inf, safe-normalize, clamp posizioni
//    - Scattering atmosferico scala planetaria (visibilita' ~50 km)
// ============================================================================
#ifndef NQG_CLEANROOM_ENGINE_HPP
#define NQG_CLEANROOM_ENGINE_HPP

#include "nqg_earth_environment.hpp"
#include "nqg_engine3d.hpp"
#include "nqg_physics_core.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
//  namespace nqg::apartment  -  geometria stanza + collisione capsula
// ============================================================================
namespace nqg {
namespace apartment {

using engine::Rgb;
using engine::Vec3;
using nqg::real;

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------
inline bool isFiniteVec(const Vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline Vec3 safeNormalize(const Vec3 &v, const Vec3 &fb = Vec3(0, 0, 1)) {
  real n2 = v.dot(v);
  if (n2 < 1e-18 || !std::isfinite(n2))
    return fb;
  real inv = 1.0 / std::sqrt(n2);
  if (!std::isfinite(inv))
    return fb;
  return v * inv;
}

// ---------------------------------------------------------------------------
// AABB
// ---------------------------------------------------------------------------
struct AABB {
  Vec3 center = Vec3(0, 0, 0);
  Vec3 half = Vec3(0.5, 0.5, 0.5);
  Rgb albedo = {0.85f, 0.85f, 0.82f};
  real metallic = 0.05;
  real roughness = 0.40;
  bool isStatic = true;
};

// ---------------------------------------------------------------------------
// Capsule collider (verticale, in piedi)
// ---------------------------------------------------------------------------
struct CapsuleCollider {
  real radius = 0.30;
  real height = 1.80;
  real eyeHeight = 1.70;

  real footZ(real camZ) const { return camZ - eyeHeight; }
  real headZ(real camZ) const { return camZ - eyeHeight + height; }
};

// ---------------------------------------------------------------------------
// RoomGeometry: 12m x 10m x 3.2m, porta sul lato Sud (y = yMin)
// ---------------------------------------------------------------------------
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
    // Muro Nord
    {
      AABB a;
      a.center = Vec3(cx, yMax + T * 0.5, cz);
      a.half = Vec3(hx + T, T * 0.5, hz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
    // Muro Est
    {
      AABB a;
      a.center = Vec3(xMax + T * 0.5, cy, cz);
      a.half = Vec3(T * 0.5, hy + T, hz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
    // Muro Ovest
    {
      AABB a;
      a.center = Vec3(xMin - T * 0.5, cy, cz);
      a.half = Vec3(T * 0.5, hy + T, hz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
    // Muro Sud sinistro (xMin -> doorX0)
    {
      real sx = 0.5 * (xMin + doorX0), shx = 0.5 * (doorX0 - xMin);
      AABB a;
      a.center = Vec3(sx, yMin - T * 0.5, cz);
      a.half = Vec3(shx, T * 0.5, hz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
    // Muro Sud destro (doorX1 -> xMax)
    {
      real sx = 0.5 * (doorX1 + xMax), shx = 0.5 * (xMax - doorX1);
      AABB a;
      a.center = Vec3(sx, yMin - T * 0.5, cz);
      a.half = Vec3(shx, T * 0.5, hz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
    // Architrave sopra la porta
    {
      real sx = 0.5 * (doorX0 + doorX1);
      real shx = 0.5 * (doorX1 - doorX0);
      real sz = 0.5 * (doorZ1 + zMax);
      real shz = 0.5 * (zMax - doorZ1);
      AABB a;
      a.center = Vec3(sx, yMin - T * 0.5, sz);
      a.half = Vec3(shx, T * 0.5, shz);
      a.albedo = {0.88f, 0.86f, 0.82f};
      a.roughness = 0.80;
      walls.push_back(a);
    }
  }

  bool isFloorSlab(const AABB &a) const {
    real bz1 = a.center.z + a.half.z;
    return bz1 <= zMin + 1e-6;
  }
  bool isCeilingSlab(const AABB &a) const {
    real bz0 = a.center.z - a.half.z;
    return bz0 >= zMax - 1e-6;
  }
  bool insideXY(const Vec3 &p) const {
    return p.x >= xMin && p.x <= xMax && p.y >= yMin && p.y <= yMax;
  }
};

// ---------------------------------------------------------------------------
// Cilindro verticale vs AABB (pareti) - collisione orizzontale
// ---------------------------------------------------------------------------
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
  const real r2 = r * r;

  if (dist2 >= r2)
    return false;

  const real dist = std::sqrt(std::max(dist2, 1e-12));
  Vec3 n;
  real push;

  if (dist > 1e-5) {
    n = Vec3(dx / dist, dy / dist, 0.0);
    push = r - dist;
  } else {
    real dxL = camPos.x - boxX0;
    real dxR = boxX1 - camPos.x;
    real dyL = camPos.y - boxY0;
    real dyR = boxY1 - camPos.y;
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

// ---------------------------------------------------------------------------
// FIX: pavimento sempre, soffitto SOLO dentro footprint XY della stanza
// ---------------------------------------------------------------------------
inline void resolveFloorCeiling(Vec3 &camPos, Vec3 &camVel,
                                const CapsuleCollider &cap,
                                const RoomGeometry &room) {
  const real footZ = cap.footZ(camPos.z);
  if (footZ < room.zMin) {
    camPos.z += (room.zMin - footZ);
    if (camVel.z < 0)
      camVel.z = 0;
  }

  if (room.insideXY(camPos)) {
    const real headZ = cap.headZ(camPos.z);
    if (headZ > room.zMax) {
      camPos.z -= (headZ - room.zMax);
      if (camVel.z > 0)
        camVel.z = 0;
    }
  }
}

inline void resolveRoomCollision(Vec3 &camPos, Vec3 &camVel,
                                 const CapsuleCollider &cap,
                                 const RoomGeometry &room) {
  if (!isFiniteVec(camPos) || !isFiniteVec(camVel)) {
    camPos = Vec3(0, 0, cap.eyeHeight);
    camVel = Vec3(0, 0, 0);
    return;
  }
  resolveFloorCeiling(camPos, camVel, cap, room);

  for (const auto &w : room.walls) {
    if (room.isFloorSlab(w) || room.isCeilingSlab(w))
      continue;
    resolveCylinderAABB(camPos, camVel, cap, w);
  }

  camPos.x = std::clamp(camPos.x, -100000.0, 100000.0);
  camPos.y = std::clamp(camPos.y, -100000.0, 100000.0);
  camPos.z =
      std::clamp(camPos.z, cap.eyeHeight + 0.001, cap.eyeHeight + 100000.0);
}

} // namespace apartment
} // namespace nqg

// ============================================================================
//  Physics modules (inclusi dopo le definizioni geometria)
// ============================================================================
#include "nqg_air_physics.hpp"
#include "nqg_continuum_physics.hpp"
#include "nqg_matter_physics.hpp"

// ============================================================================
//  namespace nqg::cleanroom  -  illuminazione + scena + ray tracing
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

// ----------------------------------------------------------------------------
// Sistema di Illuminazione
// ----------------------------------------------------------------------------
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
    LightSource lamp;
    lamp.type = LightSource::Type::CeilingPanel;
    lamp.position = Vec3(0, 0, 3.0);
    lamp.temperatureK = 3500.0;
    lamp.intensity = 3.0;
    sys.lights.push_back(lamp);

    LightSource doorFill;
    doorFill.type = LightSource::Type::Point;
    doorFill.position = Vec3(0, -4.8, 1.4);
    doorFill.temperatureK = 5500.0;
    doorFill.intensity = 1.2;
    sys.lights.push_back(doorFill);

    LightSource sun;
    sun.type = LightSource::Type::Directional;
    sun.direction = Vec3(-0.4, 0.3, -0.85).normalized();
    sun.temperatureK = 6500.0;
    sun.intensity = 0.8;
    sys.lights.push_back(sun);
    return sys;
  }

  static LightingSystem createCleanRoomPreset() {
    return createApartmentPreset();
  }
};

// ----------------------------------------------------------------------------
// Sfera di prova
// ----------------------------------------------------------------------------
struct PhysicalSphere {
  Vec3 pos = Vec3(0, 3, 2.0);
  Vec3 vel = Vec3(0, 0, 0);
  real radius = 0.35;
  real mass = 2.5;
  real restitution = 0.72;
  Rgb albedo = {0.88f, 0.25f, 0.20f};
  real metallic = 0.15;
  real roughness = 0.25;

  Vec3 lastFDrag = Vec3(0, 0, 0);
  Vec3 lastFBuoyancy = Vec3(0, 0, 0);
  real lastReynolds = 0.0;

  void step(real dt, const AirProperties &air, real gravityMag = 9.80665) {
    if (dt <= 0)
      return;
    std::array<real, 6> y0 = {pos.x, pos.y, pos.z, vel.x, vel.y, vel.z};

    auto rhs = [&](real, const std::array<real, 6> &y) -> std::array<real, 6> {
      Vec3 p(y[0], y[1], y[2]);
      Vec3 v(y[3], y[4], y[5]);
      Vec3 fD, fB;
      real re;
      air.computeAerodynamicForces(radius, mass, p, v, fD, fB, re);
      Vec3 fGravity(0, 0, -mass * gravityMag);
      Vec3 fTotal = fGravity + fB + fD;
      Vec3 accel = fTotal * (1.0 / mass);
      return {v.x, v.y, v.z, accel.x, accel.y, accel.z};
    };

    auto res = nqg::integrateDP45<6>(rhs, y0, 0.0, dt, 1e-6, 1e-8);
    Vec3 newPos(res.y[0], res.y[1], res.y[2]);
    Vec3 newVel(res.y[3], res.y[4], res.y[5]);
    if (std::isfinite(newPos.x) && std::isfinite(newPos.y) &&
        std::isfinite(newPos.z) && std::isfinite(newVel.x) &&
        std::isfinite(newVel.y) && std::isfinite(newVel.z)) {
      pos = newPos;
      vel = newVel;
    } else {
      vel = Vec3(0, 0, 0);
    }

    if (pos.z - radius < 0.0) {
      pos.z = radius;
      if (vel.z < 0) {
        vel.z = -vel.z * restitution;
        vel.x *= 0.94;
        vel.y *= 0.94;
        if (std::abs(vel.z) < 0.05)
          vel.z = 0;
      }
    }
    air.computeAerodynamicForces(radius, mass, pos, vel, lastFDrag,
                                 lastFBuoyancy, lastReynolds);
  }
};

// ----------------------------------------------------------------------------
// SCENA
// ----------------------------------------------------------------------------
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
  Camera camera;
  Vec3 gravityVec = Vec3(0, 0, -9.80665);

  // Globo
  earth::EarthGlobe globe;
  earth::WindProfile windProfile;
  real homeLat = 45.0 * PI / 180.0;
  real homeLon = 9.0 * PI / 180.0;
  Vec3 homeECEF;
  Vec3 homeUp, homeEast, homeNorth;

  // Appartamento
  RoomGeometry room;

  // Stato osservatore
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

    // Popola solids con pareti della stanza
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
      solids.push_back(s);
    }

    // Tavolo statico
    {
      continuum::RigidSolidElement table;
      table.pos = Vec3(-3.0, 2.5, 0.40);
      table.size = Vec3(1.8, 0.9, 0.80);
      table.mass = 25.0;
      table.albedo = {0.72f, 0.55f, 0.35f};
      table.metallic = 0.0;
      table.roughness = 0.55;
      table.restitution = 0.05;
      table.isStatic = true;
      solids.push_back(table);
    }
    // Casse dinamiche
    {
      continuum::RigidSolidElement crate;
      crate.pos = Vec3(3.5, 2.0, 0.50);
      crate.size = Vec3(0.9, 0.9, 0.9);
      crate.mass = 8.0;
      crate.albedo = {0.65f, 0.45f, 0.28f};
      crate.metallic = 0.0;
      crate.roughness = 0.62;
      crate.restitution = 0.15;
      crate.isStatic = false;
      solids.push_back(crate);
    }
    {
      continuum::RigidSolidElement crate;
      crate.pos = Vec3(3.8, 2.6, 1.50);
      crate.size = Vec3(0.7, 0.7, 0.7);
      crate.mass = 4.0;
      crate.albedo = {0.58f, 0.40f, 0.22f};
      crate.metallic = 0.0;
      crate.roughness = 0.60;
      crate.restitution = 0.20;
      crate.isStatic = false;
      solids.push_back(crate);
    }

    PhysicalSphere s;
    s.pos = Vec3(2.0, -1.0, 2.2);
    s.vel = Vec3(0.4, 0.2, 0.0);
    spheres.push_back(s);

    updateEnvironment();
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
    gravityVec = Vec3(0, 0, -currentGravity);

    earth::WindProfile wp = earth::WindField::profileFor(
        std::min(1.0, std::max(0.0, 0.15)), homeLat);
    Vec3 atmoWind = earth::WindField::velocityAt(wp, currentAltitude);
    Vec3 localWind(atmoWind.x, atmoWind.y, atmoWind.z);

    Vec3 curlWind = wind.evaluateVelocity(lastObserverPos, simTime);
    real blend = std::clamp(currentAltitude / 2000.0, 0.0, 1.0);
    currentWind = curlWind * (1.0 - blend) + localWind * blend;
    air.windVelocity = currentWind;
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

  bool isInShadow(const Vec3 &pos, const Vec3 &lightDir, real maxDist) const {
    Vec3 ro = pos + lightDir * 0.005;
    for (const auto &s : solids) {
      real t;
      Vec3 n;
      if (continuum::RigidSolidElement::intersectBox(ro, lightDir, s.pos,
                                                     s.halfExtents(), t, n)) {
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
    real tHit = 1e9;
    Vec3 normal;
    Rgb hitAlbedo;
    real metallic = 0.0, roughness = 0.5;
    bool hitSomething = false, hitWater = false, hitQuantum = false;

    // 1. Solidi
    for (const auto &s : solids) {
      real tBox;
      Vec3 nBox;
      if (continuum::RigidSolidElement::intersectBox(
              ro, rd, s.pos, s.halfExtents(), tBox, nBox)) {
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

    // 2. Acqua locale
    real tW, dW;
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

    // 3. Sabbia locale
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

    // 4. Nube quantistica
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

    // 5. Superficie terrestre
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

    if (!hitSomething && !hitQuantum)
      return skyColor(rd);
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

    // Ottica acqua
    if (hitWater) {
      Vec3 refrDir;
      bool hasRefr = continuum::ContinuousWaterBody::refractRay(
          rd, normal, 1.0 / water.refractiveIndex, refrDir);
      Rgb bottomCol = {0.80f, 0.84f, 0.88f};
      if (hasRefr && refrDir.z < -1e-4) {
        real dToFloor = hitPos.z / (-refrDir.z);
        Vec3 floorPos = hitPos + refrDir * dToFloor;
        bottomCol = floorTexture(floorPos);
        Vec3 trans = water.beerLambertTransmission(dToFloor);
        bottomCol.r = float(bottomCol.r * trans.x * 0.40);
        bottomCol.g = float(bottomCol.g * trans.y * 0.88);
        bottomCol.b = float(bottomCol.b * trans.z * 1.10);
      }
      real cosTheta = std::clamp(-normal.dot(rd), 0.0, 1.0);
      real F = continuum::ContinuousWaterBody::fresnelDielectric(
          cosTheta, 1.0, water.refractiveIndex);
      Vec3 reflDir = rd - normal * (2.0 * rd.dot(normal));
      Rgb reflCol = skyColor(reflDir);

      Rgb specSum = {0.0f, 0.0f, 0.0f};
      for (const auto &light : lighting.lights) {
        if (!light.active)
          continue;
        Vec3 lDir = (light.type == LightSource::Type::Directional)
                        ? light.direction * (-1.0)
                        : apartment::safeNormalize(light.position - hitPos);
        Vec3 halfVec = apartment::safeNormalize(lDir + viewDir);
        real nDotH = std::max(0.0, normal.dot(halfVec));
        real spec = std::pow(nDotH, 128.0);
        Rgb lCol = light.getEmissionColor();
        specSum.r += float(lCol.r * spec * 2.5);
        specSum.g += float(lCol.g * spec * 2.5);
        specSum.b += float(lCol.b * spec * 2.5);
      }
      Rgb litWater;
      litWater.r = float(bottomCol.r * (1.0 - F) + reflCol.r * F + specSum.r);
      litWater.g = float(bottomCol.g * (1.0 - F) + reflCol.g * F + specSum.g);
      litWater.b = float(bottomCol.b * (1.0 - F) + reflCol.b * F + specSum.b);
      real ext = effectiveScattering() * tHit;
      real transmission = std::exp(-ext);
      real inScatter = 1.0 - transmission;
      Rgb haze = skyColor(rd);
      return {float(litWater.r * transmission + haze.r * inScatter),
              float(litWater.g * transmission + haze.g * inScatter),
              float(litWater.b * transmission + haze.b * inScatter)};
    }

    // Illuminazione fisica
    Rgb lit = {0.0f, 0.0f, 0.0f};
    real hemi = 0.5 * (normal.z + 1.0);
    Rgb ambient = {float(lighting.ambientGround.r * (1.0 - hemi) +
                         lighting.ambientSky.r * hemi),
                   float(lighting.ambientGround.g * (1.0 - hemi) +
                         lighting.ambientSky.g * hemi),
                   float(lighting.ambientGround.b * (1.0 - hemi) +
                         lighting.ambientSky.b * hemi)};
    lit.r += hitAlbedo.r * ambient.r;
    lit.g += hitAlbedo.g * ambient.g;
    lit.b += hitAlbedo.b * ambient.b;

    for (const auto &L : lighting.lights) {
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
      if (isInShadow(hitPos, toLight, dist))
        continue;
      real nDotL = std::max(0.0, normal.dot(toLight));
      if (nDotL <= 0.0)
        continue;
      Rgb lCol = L.getEmissionColor();
      Vec3 halfVec = apartment::safeNormalize(toLight + viewDir);
      real nDotH = std::max(0.0, normal.dot(halfVec));
      real specPower = std::max(2.0, (1.0 - roughness) * 128.0);
      real spec = std::pow(nDotH, specPower);
      real vDotH = std::max(0.0, viewDir.dot(halfVec));
      real f0 = 0.04 * (1.0 - metallic) + metallic;
      real fresnel = f0 + (1.0 - f0) * std::pow(1.0 - vDotH, 5.0);
      real diffFactor = (1.0 - metallic) * nDotL * atten;
      real specFactor = spec * fresnel * atten * 1.5;
      lit.r += float(hitAlbedo.r * lCol.r * diffFactor + lCol.r * specFactor);
      lit.g += float(hitAlbedo.g * lCol.g * diffFactor + lCol.g * specFactor);
      lit.b += float(hitAlbedo.b * lCol.b * diffFactor + lCol.b * specFactor);
    }

    real effScat = effectiveScattering();
    real ext = effScat * tHit;
    real transmission = std::exp(-ext);
    real inScatter = 1.0 - transmission;
    Rgb haze = skyColor(rd);
    lit.r = float(lit.r * transmission + haze.r * inScatter * 0.9);
    lit.g = float(lit.g * transmission + haze.g * inScatter * 0.9);
    lit.b = float(lit.b * transmission + haze.b * inScatter * 0.9);

    if (hitQuantum) {
      real qDens = quantumField.evaluateDensity(hitPos);
      real qAlpha = std::min(0.5, qDens * 0.5);
      lit.r += float(quantumField.glowColor.r * qAlpha);
      lit.g += float(quantumField.glowColor.g * qAlpha);
      lit.b += float(quantumField.glowColor.b * qAlpha);
    }
    return lit;
  }

  Image render(int width, int height, const Vec3 &camEye = Vec3(0, -3, 1.7),
               real yaw = 0, real pitch = 0) const {
    lastObserverPos = camEye;
    currentAltitude = camEye.z;
    real hGeo = earth::StandardAtmosphere::geopotential(currentAltitude);
    currentDensity = globe.atmo.density(hGeo);

    Image img(width, height);
    Vec3 f(std::sin(yaw) * std::cos(pitch), std::cos(yaw) * std::cos(pitch),
           std::sin(pitch));
    f = apartment::safeNormalize(f, Vec3(0, 1, 0));
    Vec3 worldUp(0, 0, 1);
    Vec3 right = f.cross(worldUp);
    if (right.norm() < 1e-6)
      right = Vec3(1, 0, 0);
    else
      right = right.normalized();
    Vec3 up = right.cross(f).normalized();

    const real th = std::tan(camera.fovY / 2.0);
    const real aspect = real(width) / real(height);

    std::atomic<int> nextY{0};
    unsigned numThreads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> workers;

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

  void stepPhysics(real dt) {
    if (dt <= 0 || !std::isfinite(dt))
      return;
    simTime += dt;
    updateEnvironment();

    real wSpeed = currentWind.norm();
    for (auto &w : water.waves) {
      w.amplitude = 0.015 + wSpeed * 0.006;
    }

    sand.relaxAvalanche(1);
    real airDensity = currentDensity;

    emField.applyCoulombBetweenSolids(solids);
    emField.applyToSolids(solids);

    for (auto &s : solids) {
      s.stepDynamics(dt, gravityVec, water, wind, airDensity, simTime);
    }

    for (std::size_t i = 0; i < solids.size(); ++i) {
      for (std::size_t j = i + 1; j < solids.size(); ++j) {
        Vec3 colNormal;
        real overlap;
        if (continuum::RigidSolidElement::aabbOverlap(
                solids[i].pos, solids[i].halfExtents(), solids[j].pos,
                solids[j].halfExtents(), colNormal, overlap)) {
          real totalMass = solids[i].mass + solids[j].mass;
          if (!solids[i].isStatic && !solids[j].isStatic) {
            solids[i].pos = solids[i].pos +
                            colNormal * (overlap * solids[j].mass / totalMass);
            solids[j].pos = solids[j].pos -
                            colNormal * (overlap * solids[i].mass / totalMass);
          } else if (!solids[i].isStatic) {
            solids[i].pos = solids[i].pos + colNormal * overlap;
          } else if (!solids[j].isStatic) {
            solids[j].pos = solids[j].pos - colNormal * overlap;
          }
          real vRel = (solids[i].vel - solids[j].vel).dot(colNormal);
          if (vRel < 0) {
            real e = std::min(solids[i].restitution, solids[j].restitution);
            real jImpulse = -(1.0 + e) * vRel;
            if (solids[i].isStatic) {
              jImpulse /= (1.0 / solids[j].mass);
              solids[j].vel =
                  solids[j].vel - colNormal * (jImpulse / solids[j].mass);
            } else if (solids[j].isStatic) {
              jImpulse /= (1.0 / solids[i].mass);
              solids[i].vel =
                  solids[i].vel + colNormal * (jImpulse / solids[i].mass);
            } else {
              jImpulse /= (1.0 / solids[i].mass + 1.0 / solids[j].mass);
              solids[i].vel =
                  solids[i].vel + colNormal * (jImpulse / solids[i].mass);
              solids[j].vel =
                  solids[j].vel - colNormal * (jImpulse / solids[j].mass);
            }
          }
        }
      }
    }

    for (auto &s : spheres) {
      s.step(dt, air, currentGravity);
    }

    matterSim.gravity = gravityVec;
    matterSim.step(dt, air);

    water.ripples.erase(
        std::remove_if(water.ripples.begin(), water.ripples.end(),
                       [this](const continuum::ContinuousWaterBody::Ripple &r) {
                         return (simTime - r.startTime) > 5.0;
                       }),
        water.ripples.end());
  }
};

} // namespace cleanroom
} // namespace nqg

#endif // NQG_CLEANROOM_ENGINE_HPP