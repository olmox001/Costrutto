// ============================================================================
//  nqg_cleanroom_game.cpp  -  Appartamento su Globo Terrestre (SDL3)
//  FIX 2025e:
//   - renderW/H: 640x480 -> 320x240 (4x meno pixel, ~4x FPS).
//     Modifica questi due valori se vuoi piu' qualita' o piu' fps.
//   - Campo EM di esempio impostato su due solidi (charge ±1e-3 C)
//     per verificare visivamente l'effetto della forza di Coulomb.
// ============================================================================
#include "nqg_cleanroom_engine.hpp"
#include "nqg_matter_physics.hpp"
#include "nqg_window_sdl3.hpp"
#include <algorithm>
#include <iostream>
#include <memory>

using namespace nqg;
using namespace nqg::cleanroom;
using namespace nqg::matter;
using namespace nqg::window;

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  std::cout << "Avvio NQG Apartment su Globo Terrestre...\n";

  WindowConfig cfg;
  cfg.title = "NQG Apartment on Earth - Unified Collision";
  cfg.windowWidth = 1280;
  cfg.windowHeight = 720;
  cfg.resizable = true;
  cfg.vsync = true;
  cfg.headless = false;
  cfg.enableAudio = true;
  cfg.showHud = true;

  WindowComponent app;
  if (!app.init(cfg)) {
    std::cerr << "Errore apertura finestra SDL3!\n";
    return 1;
  }

  CleanRoomScene scene;

  // --- FPS: 320x240 e' 4x piu' veloce di 640x480 ---
  // Se hai una GPU veloce e vuoi piu' dettaglio, alza a 480x360 o 640x480.
  int renderW = 320;
  int renderH = 240;

  // --- Campo EM di esempio: due solidi carichi (attrazione Coulomb) ---
  // Attivalo/commentalo per testare.
  if (scene.solids.size() >= 4) {
    scene.solids[2].charge = +1e-3; // cassa grande
    scene.solids[3].charge = -1e-3; // cassa piccola
  }

  apartment::CapsuleCollider playerBody;
  playerBody.radius = 0.30;
  playerBody.height = 1.80;
  playerBody.eyeHeight = 1.70;

  Vec3 camPos(0, -3.0, playerBody.eyeHeight);
  real camYaw = 0.0;
  real camPitch = -0.05;
  bool zeroGravity = false;

  Vec3 camVel(0, 0, 0);
  bool freefallMode = false;
  bool spaceWasDown = false;
  double spacePressTime = 0.0;
  double lastTapTime = -100.0;
  double controlTime = 0.0;
  double lastRippleTime = -1.0;

  constexpr double TAP_MAX_DURATION = 0.25;
  constexpr double DOUBLE_TAP_WINDOW = 0.40;
  constexpr real BASE_MOVE_SPEED = 10.0;
  constexpr real BASE_FLY_SPEED = 15.0;
  constexpr real JUMP_IMPULSE = 5.5;
  constexpr real FREEFALL_THRUST = 20.0;
  constexpr real SHIFT_BOOST = 10.0;
  constexpr real COLLISION_STEP = 0.05;
  constexpr double RIPPLE_INTERVAL = 0.10;

  std::cout
      << "Appartamento: 12m x 10m x 3.2m con porta sud.\n"
         "SPACE: tap=salto | hold=volo | 2xTap=FREE-FALL\n"
         "SHIFT: boost 10x\n"
         "[1] Acqua | [2] Sabbia | [3] Solido | [T] Vento | [G] Gravita'\n"
         "[L] Luce | [X] Pulisci | [ESC] Esci\n";

  while (app.pollEvents()) {
    double dt = app.computeDeltaTime();
    controlTime += dt;
    auto &im = app.input();

    if (im.isMouseLeftDown()) {
      camYaw += im.mouseDeltaX() * 0.0035;
      camPitch -= im.mouseDeltaY() * 0.0035;
      camPitch = std::clamp(camPitch, -1.4, 1.4);
    }
    if (im.isKeyDown(SDL_SCANCODE_LEFT))
      camYaw -= 1.6 * dt;
    if (im.isKeyDown(SDL_SCANCODE_RIGHT))
      camYaw += 1.6 * dt;
    if (im.isKeyDown(SDL_SCANCODE_UP))
      camPitch = std::clamp(camPitch + 1.2 * dt, -1.4, 1.4);
    if (im.isKeyDown(SDL_SCANCODE_DOWN))
      camPitch = std::clamp(camPitch - 1.2 * dt, -1.4, 1.4);

    Vec3 forward(std::sin(camYaw), std::cos(camYaw), 0);
    Vec3 right(std::cos(camYaw), -std::sin(camYaw), 0);
    Vec3 lookDir(std::sin(camYaw) * std::cos(camPitch),
                 std::cos(camYaw) * std::cos(camPitch), std::sin(camPitch));

    real altitudeFactor = std::sqrt(1.0 + std::max(0.0, camPos.z) / 10.0);
    real shiftMult =
        (im.isKeyDown(SDL_SCANCODE_LSHIFT) || im.isKeyDown(SDL_SCANCODE_RSHIFT))
            ? SHIFT_BOOST
            : 1.0;
    real moveSpeed = BASE_MOVE_SPEED * altitudeFactor * shiftMult;
    real flySpeed = BASE_FLY_SPEED * altitudeFactor * shiftMult;
    real thrustScaled = FREEFALL_THRUST * altitudeFactor;

    const bool spaceDown = im.isKeyDown(SDL_SCANCODE_SPACE);
    const bool spaceJustPressed = spaceDown && !spaceWasDown;
    const bool spaceJustReleased = !spaceDown && spaceWasDown;
    const bool spaceHeldLongEnough =
        spaceDown && (controlTime - spacePressTime) > TAP_MAX_DURATION;

    if (spaceJustPressed)
      spacePressTime = controlTime;
    if (spaceJustReleased) {
      const double heldDuration = controlTime - spacePressTime;
      if (heldDuration < TAP_MAX_DURATION) {
        if (controlTime - lastTapTime < DOUBLE_TAP_WINDOW) {
          freefallMode = !freefallMode;
          lastTapTime = -100.0;
          if (app.audio())
            app.audio()->triggerTransient(freefallMode ? 220.0f : 660.0f, 0.5f,
                                          45.0f);
        } else {
          camVel.z = JUMP_IMPULSE;
          lastTapTime = controlTime;
          if (app.audio())
            app.audio()->triggerTransient(440.0f, 0.35f, 22.0f);
        }
      }
    }
    spaceWasDown = spaceDown;

    Vec3 oldPos = camPos;
    if (freefallMode) {
      Vec3 acc(0, 0, 0);
      if (!zeroGravity)
        acc.z -= scene.currentGravity;

      const real rho = scene.currentDensity;
      const Vec3 relVel = camVel - scene.currentWind;
      acc = acc - relVel * (rho * 0.4);
      if (spaceHeldLongEnough)
        acc.z += thrustScaled;

      const real airCtrl = 3.0 * altitudeFactor;
      if (im.isKeyDown(SDL_SCANCODE_W))
        acc = acc + forward * airCtrl;
      if (im.isKeyDown(SDL_SCANCODE_S))
        acc = acc - forward * airCtrl;
      if (im.isKeyDown(SDL_SCANCODE_A))
        acc = acc - right * airCtrl;
      if (im.isKeyDown(SDL_SCANCODE_D))
        acc = acc + right * airCtrl;

      camVel = camVel + acc * dt;
      camPos = camPos + camVel * dt;
    } else {
      if (im.isKeyDown(SDL_SCANCODE_W))
        camPos = camPos + forward * (moveSpeed * dt);
      if (im.isKeyDown(SDL_SCANCODE_S))
        camPos = camPos - forward * (moveSpeed * dt);
      if (im.isKeyDown(SDL_SCANCODE_A))
        camPos = camPos - right * (moveSpeed * dt);
      if (im.isKeyDown(SDL_SCANCODE_D))
        camPos = camPos + right * (moveSpeed * dt);

      if (spaceHeldLongEnough)
        camVel.z = std::max(camVel.z, flySpeed);
      else if (!zeroGravity && camPos.z > playerBody.eyeHeight)
        camVel.z -= scene.currentGravity * dt;

      if (im.isKeyDown(SDL_SCANCODE_C)) {
        camPos.z = std::max(playerBody.eyeHeight, camPos.z - moveSpeed * dt);
        camVel.z = 0;
      } else {
        camPos.z += camVel.z * dt;
      }
    }

    Vec3 delta = camPos - oldPos;
    real moveLen = delta.norm();
    int nSub = 1;
    if (std::isfinite(moveLen) && moveLen > COLLISION_STEP) {
      nSub = std::clamp(int(std::ceil(moveLen / COLLISION_STEP)), 1, 256);
    }

    camPos = oldPos;
    Vec3 subDelta = delta * (1.0 / real(nSub));
    for (int i = 0; i < nSub; ++i) {
      camPos = camPos + subDelta;
      scene.resolvePlayerCollision(camPos, camVel, playerBody);
    }

    real actualMove = (camPos - oldPos).norm();
    real actualSpeed = actualMove / std::max(0.001, dt);
    if (actualSpeed > 0.3 && controlTime - lastRippleTime > RIPPLE_INTERVAL) {
      scene.notifyPlayerInWater(camPos, playerBody, actualSpeed);
      lastRippleTime = controlTime;
    }

    Vec3 spawnTarget = camPos + lookDir * 2.2;

    if (im.wasKeyPressed(SDL_SCANCODE_1) || im.isKeyDown(SDL_SCANCODE_1)) {
      scene.water.addImpulse(spawnTarget, scene.simTime, 0.08);
      if (app.audio())
        app.audio()->triggerTransient(320.0f, 0.45f, 22.0f);
    }
    if (im.wasKeyPressed(SDL_SCANCODE_2) || im.isKeyDown(SDL_SCANCODE_2)) {
      scene.sand.pourSand(spawnTarget.x, spawnTarget.y, 0.05);
      scene.sand.relaxAvalanche(5);
      if (app.audio())
        app.audio()->triggerTransient(880.0f, 0.22f, 35.0f);
    }
    if (im.wasKeyPressed(SDL_SCANCODE_3)) {
      continuum::RigidSolidElement box;
      box.pos = spawnTarget;
      box.size = Vec3(0.4, 0.4, 0.4);
      box.albedo = {0.88f, 0.88f, 0.82f};
      box.metallic = 0.2;
      box.roughness = 0.3;
      scene.solids.push_back(box);
      scene.solids.back().refreshWorldHalf();
    }
    if (im.wasKeyPressed(SDL_SCANCODE_G)) {
      zeroGravity = !zeroGravity;
      scene.setZeroGravity(zeroGravity);
    }
    if (im.isKeyDown(SDL_SCANCODE_T)) {
      scene.air.windVelocity = Vec3(lookDir.x * 6.5, lookDir.y * 6.5, 0.8);
    }
    if (im.wasKeyPressed(SDL_SCANCODE_L)) {
      static int lightMode = 0;
      lightMode = (lightMode + 1) % 3;
      if (lightMode == 0) {
        scene.lighting = LightingSystem::createApartmentPreset();
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
    if (im.wasKeyPressed(SDL_SCANCODE_X)) {
      scene.matterSim.clear();
      scene.spheres.clear();
    }

    scene.stepPhysics(dt);

    for (const auto &ev : scene.matterSim.frameAudioEvents) {
      if (app.audio())
        app.audio()->triggerTransient(static_cast<float>(ev.frequency),
                                      static_cast<float>(ev.intensity), 28.0f);
    }
    real windMag = scene.currentWind.norm();
    app.updateAudio(dt, 1.0, 0.0, windMag > 0.5 ? windMag * 0.15 : 0.0, 0.0);

    Image frame = scene.render(renderW, renderH, camPos, camYaw, camPitch);
    app.renderFrame(frame);

    SDL_Renderer *ren = app.renderer();
    if (ren && app.hud().mode() != HudRenderer::HudMode::Off) {
      int winW = 0, winH = 0;
      SDL_GetWindowSize(app.window(), &winW, &winH);
      SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

      SDL_FRect rMat{20, 20, 460, 362};
      SDL_SetRenderDrawColor(ren, 15, 25, 38, 230);
      SDL_RenderFillRect(ren, &rMat);
      SDL_SetRenderDrawColor(ren, 50, 130, 210, 255);
      SDL_RenderRect(ren, &rMat);

      SDL_SetRenderDrawColor(ren, 100, 220, 255, 255);
      SDL_RenderDebugText(ren, 30, 28, "=== APPARTAMENTO SU TERRA ===");
      SDL_SetRenderDrawColor(ren, 230, 240, 255, 255);
      SDL_RenderDebugTextFormat(ren, 30, 48, "Pos: %.2f, %.2f, %.2f", camPos.x,
                                camPos.y, camPos.z);
      SDL_RenderDebugTextFormat(ren, 30, 66, "Piedi/Testa: %.2f / %.2f m",
                                playerBody.footZ(camPos.z),
                                playerBody.headZ(camPos.z));
      SDL_RenderDebugTextFormat(
          ren, 30, 84, "In stanza: %s | In spillway: %s",
          scene.room.insideXY(camPos) ? "SI" : "NO",
          scene.water.isInsideSpillway(camPos.x, camPos.y) ? "SI" : "NO");
      SDL_RenderDebugTextFormat(ren, 30, 102, "Altitudine: %.1f m",
                                scene.currentAltitude);
      SDL_RenderDebugTextFormat(ren, 30, 120, "Gravita': %.4f m/s^2",
                                zeroGravity ? 0.0 : scene.currentGravity);
      SDL_RenderDebugTextFormat(ren, 30, 138, "Pressione: %.0f Pa",
                                scene.currentPressure);
      SDL_RenderDebugTextFormat(ren, 30, 156, "Densita': %.5f kg/m^3",
                                scene.currentDensity);
      SDL_RenderDebugTextFormat(ren, 30, 174, "Vel. camera: %.2f m/s",
                                camVel.norm());
      SDL_RenderDebugTextFormat(ren, 30, 192, "Substep collisione: %d", nSub);
      SDL_RenderDebugTextFormat(
          ren, 30, 210, "Acqua livello: %.3f m (vol %.2f m^3)",
          scene.water.currentLevel(), scene.water.waterVolume);
      SDL_RenderDebugTextFormat(ren, 30, 228, "Ripples attivi: %zu",
                                scene.water.ripples.size());

      // Stato sleeping/EM dei solidi
      int nAsleep = 0, nAwake = 0;
      real maxEM = 0.0;
      for (const auto &s : scene.solids) {
        if (s.isStatic)
          continue;
        if (s.asleep)
          ++nAsleep;
        else
          ++nAwake;
        maxEM = std::max(maxEM, s.lastEMForce.norm());
      }
      SDL_RenderDebugTextFormat(ren, 30, 246,
                                "Solidi: awake=%d asleep=%d | |F_EM|max=%.3e",
                                nAwake, nAsleep, maxEM);

      SDL_SetRenderDrawColor(ren, freefallMode ? 255 : 100,
                             freefallMode ? 180 : 220, 255, 255);
      SDL_RenderDebugTextFormat(ren, 30, 264,
                                "SPACE: tap=salt | hold=vola | 2x=%s",
                                freefallMode ? "FREE-FALL ON" : "Direct");
      SDL_SetRenderDrawColor(ren, 230, 240, 255, 255);
      SDL_RenderDebugTextFormat(ren, 30, 282,
                                "Stanza 12x10x3.2m + spillway 2x3m");
      SDL_RenderDebugTextFormat(ren, 30, 300, "Solidi totali: %zu | FPS: %.1f",
                                scene.solids.size(), app.fps());
      SDL_RenderDebugTextFormat(ren, 30, 318,
                                "Render: %dx%d (alza in cpp se vuoi)", renderW,
                                renderH);
      SDL_RenderDebugTextFormat(ren, 30, 336, "Pos. spawn: %.2f, %.2f, %.2f",
                                spawnTarget.x, spawnTarget.y, spawnTarget.z);

      SDL_FRect rAir{static_cast<float>(winW - 360), 20, 340, 120};
      SDL_SetRenderDrawColor(ren, 15, 25, 38, 230);
      SDL_RenderFillRect(ren, &rAir);
      SDL_SetRenderDrawColor(ren, 210, 150, 40, 255);
      SDL_RenderRect(ren, &rAir);

      SDL_SetRenderDrawColor(ren, 255, 210, 100, 255);
      SDL_RenderDebugText(ren, winW - 350, 28, "=== AMBIENTE ===");
      SDL_SetRenderDrawColor(ren, 230, 240, 255, 255);
      SDL_RenderDebugTextFormat(ren, winW - 350, 48, "Scattering: %.2e /m",
                                scene.effectiveScattering());
      SDL_RenderDebugTextFormat(ren, winW - 350, 64, "US Std 1976 (7 strati)");
      SDL_RenderDebugTextFormat(ren, winW - 350, 80, "Somigliana + free-air");
      SDL_RenderDebugTextFormat(ren, winW - 350, 96, "Lights: %zu",
                                scene.lighting.lights.size());

      SDL_SetRenderDrawColor(ren, 80, 210, 255, 200);
      SDL_RenderLine(ren, winW / 2 - 8, winH / 2, winW / 2 + 8, winH / 2);
      SDL_RenderLine(ren, winW / 2, winH / 2 - 8, winW / 2, winH / 2 + 8);

      SDL_SetRenderDrawColor(ren, 220, 230, 240, 240);
      SDL_RenderDebugText(
          ren, winW / 2 - 540, winH - 24,
          "[SPACE] tap=Salt | hold=Volo | 2x=FreeFall | [SHIFT] Boost 10x | "
          "[1] Acqua | [2] Sabbia | [3] Solido | [T] Vento | [G] Grav | "
          "[L] Luce | [ESC] Esci");
    }

    app.present();
  }

  app.shutdown();
  std::cout << "Appartamento chiuso.\n";
  return 0;
}