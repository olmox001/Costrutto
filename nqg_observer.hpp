/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2026 olmox001
 *
 * nqg_observer.hpp — Capsule: the only host/client data object.
 *
 * Architecture
 * ------------
 *   Engine  = kernel (rules, simulation). Never talks to SDL.
 *   Capsule = the entity every external API connects to.
 *             It is both HOST (owns physical/sensory state) and
 *             CLIENT (receives commands, exposes raw sensory data).
 *
 * All of the following live ON the capsule — no bypass:
 *   • Commands   (input written by the client interface)
 *   • Framebuffer (what this capsule sees — engine fills it)
 *   • AudioOut   (what this capsule hears — engine fills it)
 *   • Observation (HUD / telemetry metrics for this capsule)
 *   • Body + kinematics (position, velocity, orientation)
 *
 * Multiple capsules may coexist; each has its own sensory streams.
 * The client (SDL, network, AI…) only reads/writes capsule data.
 */
#pragma once

#include "nqg_commands.hpp"
#include "nqg_cleanroom_engine.hpp"
#include "nqg_earth_environment.hpp"
#include "physics/physics_atmosphere.hpp"
#include "physics/physics_geodesy.hpp"
#include "nqg_hud.hpp"
#include "engine/perf_profiler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace nqg {
namespace observer {

using apartment::CapsuleCollider;
using cleanroom::CleanRoomScene;
using cmd::Commands;
using earth::GlobeResolver;
using engine::Image;
using engine::Vec3;

struct AudioSample {
  float freqHz = 440.0f;
  float intensity = 0.3f;
  float decay = 20.0f;
};

struct Observation {
  int id = 0;
  bool freefall = false;
  bool zeroGravity = false;
  std::string modeLabel = "WALK";
  Vec3 position{0, 0, 0};
  Vec3 velocity{0, 0, 0};
  real speed = 0;
  real pitch = 0;
  real altitude = 0;
  real revolutionFraction = 0;
  long turns = 0;
  real footZ = 0;
  real headZ = 0;
  real gravity = 9.81;
  real density = 1.225;
  real windMag = 0;
  bool insideRoom = false; // legacy name; prefer inside_structure()
  bool inside_structure() const { return insideRoom; }
  bool insideSpillway = false;
  int renderW = 0;
  int renderH = 0;
  double fps = 0;
  std::vector<std::string> lines;
};

struct Capsule {
  int id = 0;
  std::string name = "capsule0"; // formal default (was "appartamento")

  // CLIENT → HOST
  Commands commands;

  // HOST → CLIENT
  Image framebuffer;
  std::vector<AudioSample> audioOut;
  Observation observation;

  // Physical body
  CapsuleCollider body;
  GlobeResolver globe;
  Vec3 velocity{0, 0, 0};
  real pitch = 0.0;
  bool freefall = false;
  bool zeroGravity = false;

  double controlTime = 0.0;
  bool jumpWasDown = false;
  double jumpPressTime = 0.0;
  double lastTapTime = -100.0;

  real baseMoveSpeed = 10.0;
  real baseFlySpeed = 15.0;
  real jumpImpulse = 5.5;
  real freefallThrust = 20.0;
  real shiftBoost = 10.0;
  real collisionStep = 0.05;
  double tapMaxDuration = 0.25;
  double doubleTapWindow = 0.40;

  int viewW = 480;
  int viewH = 360;
  bool renderEnabled = true;

  Capsule() {
    body.radius = 0.30;
    body.height = 1.80;
    body.eyeHeight = 1.70;
    body.spawnClearance = 0.05;
  }

  void spawnAt(const Vec3 &pos) {
    globe.setPosition(pos);
    velocity = Vec3(0, 0, 0);
    pitch = 0.0;
    freefall = false;
    audioOut.clear();
    observation = Observation{};
    observation.id = id;
  }

  Vec3 eyePosition() const { return globe.pos; }
  Vec3 lookDirection() const {
    return globe.fwd * std::cos(pitch) + globe.up * std::sin(pitch);
  }
  Vec3 viewUp() const {
    return globe.up * std::cos(pitch) - globe.fwd * std::sin(pitch);
  }
  Vec3 right() const { return globe.right(); }

  void beginFrame() {
    audioOut.clear();
  }

  void hear(float freq, float intensity, float decay = 20.f) {
    audioOut.push_back({freq, intensity, decay});
  }

  Vec3 integrateMotion(real dt, CleanRoomScene &scene) {
    const Commands &c = commands;
    controlTime += dt;

    if (c.hasLook()) {
      globe.turn(c.lookYaw);
      pitch -= c.lookPitch;
      constexpr real PITCH_LIM = 1.45;
      pitch = std::clamp(pitch, -PITCH_LIM, PITCH_LIM);
    }

    const Vec3 up = globe.up;
    const Vec3 forward = globe.fwd;
    const Vec3 rgt = globe.right();

    real altitudeFactor =
        std::sqrt(1.0 + std::max(0.0, globe.altitude()) / 10.0);
    real shiftMult = c.boost ? shiftBoost : 1.0;
    real moveSpeed = baseMoveSpeed * altitudeFactor * shiftMult;
    // Aerodynamic authority from ambient density (no hard speed clamp).
    {
      const real rho = observation.density > 0 ? observation.density
          : nqg::physics::atmosphere::density(observation.altitude);
      const real area = 0.55;
      const real mass = 80.0;
      const real v_term = nqg::physics::atmosphere::terminal_speed(mass, rho, 1.0, area);
      const real spd = velocity.norm();
      moveSpeed *= nqg::physics::atmosphere::control_authority(spd, v_term);
    }
    real flySpeed = baseFlySpeed * altitudeFactor * shiftMult;
    real thrustScaled = freefallThrust * altitudeFactor;

    const bool jumpDown = c.jumpHeld;
    const bool jumpJustPressed = c.jumpPressed || (jumpDown && !jumpWasDown);
    const bool jumpJustReleased = c.jumpReleased || (!jumpDown && jumpWasDown);
    const bool jumpHeldLong =
        jumpDown && (controlTime - jumpPressTime) > tapMaxDuration;

    if (jumpJustPressed)
      jumpPressTime = controlTime;
    if (jumpJustReleased) {
      const double held = controlTime - jumpPressTime;
      if (held < tapMaxDuration) {
        if (controlTime - lastTapTime < doubleTapWindow) {
          freefall = !freefall;
          lastTapTime = -100.0;
          hear(freefall ? 220.f : 660.f, 0.5f, 45.f);
        } else {
          velocity = velocity + up * (jumpImpulse - velocity.dot(up));
          lastTapTime = controlTime;
          hear(440.f, 0.35f, 22.f);
        }
      }
    }
    jumpWasDown = jumpDown;

    Vec3 oldPos = globe.pos;
    Vec3 disp(0, 0, 0);

    if (freefall) {
      Vec3 acc(0, 0, 0);
      if (!zeroGravity)
        acc = acc - up * scene.currentGravity;
      acc = acc - (velocity - scene.currentWind) * (scene.currentDensity * 0.4);
      if (jumpHeldLong)
        acc = acc + up * thrustScaled;
      const real airCtrl = 3.0 * altitudeFactor;
      if (c.hasMove()) {
        acc = acc + forward * (c.moveY * airCtrl);
        acc = acc + rgt * (c.moveX * airCtrl);
      }
      velocity = velocity + acc * dt;
      disp = velocity * dt;
    } else {
      Vec3 wish(0, 0, 0);
      if (c.hasMove()) {
        wish = wish + forward * c.moveY;
        wish = wish + rgt * c.moveX;
      }
      real vUp = velocity.dot(up);
      if (jumpHeldLong)
        vUp = std::max(vUp, flySpeed);
      else if (!zeroGravity && globe.altitude() > body.eyeHeight + 1e-3)
        vUp -= scene.currentGravity * dt;
      if (c.crouch) {
        vUp = 0;
        disp = up * (-moveSpeed * dt);
      } else {
        disp = up * (vUp * dt);
      }
      disp = disp + wish * (moveSpeed * dt);
      velocity = up * vUp;
    }

    {
      const Vec3 hd = disp - up * disp.dot(up);
      scene.playerMoveVel = (!freefall && std::isfinite(hd.norm()))
                                ? hd * (1.0 / std::max(1e-3, dt))
                                : Vec3(0, 0, 0);
    }

    const real moveLen = disp.norm();
    int nSub = 1;
    if (std::isfinite(moveLen) && moveLen > collisionStep &&
        globe.pos.norm2() < 300.0 * 300.0)
      nSub = std::clamp(int(std::ceil(moveLen / collisionStep)), 1, 256);
    const Vec3 subDelta = disp * (1.0 / real(nSub));
    for (int i = 0; i < nSub; ++i) {
      const real dUp = subDelta.dot(globe.up);
      globe.walk(subDelta - globe.up * dUp, dUp);
      Vec3 pos = globe.pos;
      scene.resolvePlayerCollision(pos, velocity, body);
      globe.setPosition(pos);
    }
    // Walk mode: only *support* from below — never pull downward while airborne
    // (jump / fall). Corrects penetration when feet sink under groundHeightAt.
    if (!freefall && !zeroGravity) {
      const real gz = scene.groundHeightAt(globe.pos.x, globe.pos.y);
      const real target_eye =
          nqg::physics::geodesy::eye_from_ground(gz, body.eyeHeight);
      const real dz = target_eye - globe.pos.z;
      const real vUp = velocity.dot(globe.up);
      // Lift only if penetrating (eye below support); ignore if airborne above.
      if (std::isfinite(dz) && dz > 1e-4 && dz < 5.0) {
        globe.walk(engine::Vec3(0, 0, 0), dz);
        if (vUp < 0)
          velocity = velocity - globe.up * vUp;
      }
    }
    return globe.pos - oldPos;
  }

  void publishObservation(const CleanRoomScene &scene, double fpsHint = 0) {
    observation.id = id;
    observation.freefall = freefall;
    observation.zeroGravity = zeroGravity;
    observation.modeLabel = freefall ? "FREE-FALL" : "WALK";
    observation.position = globe.pos;
    observation.velocity = velocity;
    observation.speed = velocity.norm();
    observation.pitch = pitch;
    observation.altitude = globe.altitude();
    observation.revolutionFraction = globe.revolutionFraction();
    observation.turns = globe.turns;
    observation.footZ = body.footZ(globe.altitude());
    observation.headZ = body.headZ(globe.altitude());
    observation.gravity = zeroGravity ? 0.0 : scene.currentGravity;
    observation.density = scene.currentDensity;
    observation.windMag = scene.currentWind.norm();
    observation.insideRoom = scene.room.insideXY(globe.pos);
    observation.insideSpillway =
        scene.water.isInsideSpillway(globe.pos.x, globe.pos.y);
    observation.renderW = viewW;
    observation.renderH = viewH;
    observation.fps = fpsHint;

    {
      nqg::hud::TelemetryView tv;
      tv.x = observation.position.x;
      tv.y = observation.position.y;
      tv.z = observation.position.z;
      tv.foot_z = observation.footZ;
      tv.head_z = observation.headZ;
      tv.inside_structure = observation.insideRoom;
      tv.spillway = observation.insideSpillway;
      tv.altitude = observation.altitude;
      tv.revolution = observation.revolutionFraction;
      tv.turns = observation.turns;
      tv.gravity = observation.gravity;
      tv.speed = observation.speed;
      tv.fps = observation.fps;
      tv.density = observation.density;
      tv.wind = observation.windMag;
      tv.mode = observation.modeLabel.c_str();
      tv.view_w = viewW;
      tv.view_h = viewH;
      tv.capsule_name = name;
      nqg::hud::build_lines(tv, observation.lines);
    }
  }

  // Unique path: client writes commands; engine fills framebuffer/audio/obs
  void tick(real dt, CleanRoomScene &scene, double fpsHint = 0) {
    NQG_PERF_SCOPE("capsule.tick");
    beginFrame();

    Vec3 oldPos = eyePosition();
    {
      NQG_PERF_SCOPE("capsule.integrate");
      integrateMotion(dt, scene);
    }

    Vec3 camPos = eyePosition();
    Vec3 camVel = velocity;
    const Vec3 lookDir = lookDirection();

    {
      NQG_PERF_SCOPE("capsule.couplePlayer");
      Vec3 moveVel = (camPos - oldPos) * (1.0 / std::max(1e-3, dt));
      moveVel.z = 0.0;
      if (!(moveVel.norm() < 50.0))
        moveVel = Vec3(0, 0, 0);
      scene.couplePlayer(camPos, camVel, moveVel, body, dt);
    }

    const Vec3 spawnTarget = camPos + lookDir * 2.2;
    if (commands.action1) {
      scene.water.addVolumeSpread(spawnTarget.x, spawnTarget.y, 0.004 * dt);
      hear(320.f, 0.45f, 22.f);
    }
    if (commands.action2) {
      scene.sand.pourSand(spawnTarget.x, spawnTarget.y, 0.05);
      scene.sand.relaxAvalanche(5);
      hear(880.f, 0.22f, 35.f);
    }
    if (commands.action3) {
      continuum::RigidSolidElement box;
      box.pos = spawnTarget;
      box.size = Vec3(0.4, 0.4, 0.4);
      box.albedo = {0.88f, 0.88f, 0.82f};
      box.metallic = 0.2;
      box.roughness = 0.3;
      scene.addSolid(box);
    }
    if (commands.toggleGravity) {
      zeroGravity = !zeroGravity;
      scene.setZeroGravity(zeroGravity);
    }
    if (commands.toggleWind) {
      scene.air.windVelocity = Vec3(lookDir.x * 6.5, lookDir.y * 6.5, 0.8);
    }
    if (commands.toggleLight) {
      static int lightMode = 0;
      lightMode = (lightMode + 1) % 3;
      if (lightMode == 0) {
        scene.lighting = cleanroom::LightingSystem::createApartmentPreset();
      } else if (lightMode == 1) {
        for (auto &l : scene.lighting.lights) {
          l.temperatureK = 2700.0;
          l.intensity = 2.0;
        }
      } else {
        for (std::size_t i = 0; i < scene.lighting.lights.size(); ++i) {
          scene.lighting.lights[i].active = (i == 0);
          scene.lighting.lights[i].intensity = 0.9;
        }
      }
    }
    if (commands.clearDynamics) {
      scene.matterSim.clear();
      scene.removeDynamicSpheres();
    }

    scene.setPhysicsObserver(globe.pos, lookDir, viewW, viewH);
    scene.stepPhysics(dt);

    {
      Vec3 dPos, dVel;
      if (!freefall && scene.consumePlayerFeedback(dPos, dVel)) {
        globe.setPosition(camPos + dPos);
        velocity = velocity + dVel;
        camPos = eyePosition();
        camVel = velocity;
      }
    }

    for (const auto &ev : scene.matterSim.frameAudioEvents) {
      hear(float(ev.frequency), float(ev.intensity), 25.f);
    }

    renderFrame(scene);
    publishObservation(scene, fpsHint);
    commands.clearEdges();
  }

  // Present path only (raytrace). Physics stays in tick; clients may parallelize this later.
  void renderFrame(CleanRoomScene &scene) {
    NQG_PERF_SCOPE("capsule.renderFrame");
    const Vec3 camPos = eyePosition();
    const Vec3 lookDir = lookDirection();
    const Vec3 viewUpV = viewUp();
    if (renderEnabled)
      framebuffer = scene.renderView(viewW, viewH, camPos, lookDir, viewUpV);
    else if (framebuffer.w != viewW || framebuffer.h != viewH)
      framebuffer = Image(viewW, viewH);
  }
};

using CapsuleObserver = Capsule;

} // namespace observer
} // namespace nqg
