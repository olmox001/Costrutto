// ============================================================================
//  nqg_sample_game.cpp  -  Gioco / Simulatore Relativistico Interattivo NQG
//  Utilizza i tre componenti dell'engine:
//    1. nqg_physics_core.hpp  (Fisica gravitazionale di Schwarzschild, LQG, V11)
//    2. nqg_engine3d.hpp       (Ray tracing geodetico relativistico, osservatore)
//    3. nqg_window_sdl3.hpp    (Finestra interattiva SDL3, texture streaming, HUD, audio)
// ============================================================================
#include "nqg_window_sdl3.hpp"
#include <iostream>
#include <iomanip>

using namespace nqg;
using namespace nqg::engine;
using namespace nqg::window;

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  std::cout << "========================================================\n";
  std::cout << "  NQG ENGINE 3D - RELATIVISTIC OBSERVER SIMULATOR\n";
  std::cout << "  Premere ESC per uscire, H per HUD, W/S per spinta radiale\n";
  std::cout << "========================================================\n";

  // 1. Configurazione Finestra SDL3
  WindowConfig cfg;
  cfg.title = "NQG Event Horizon - Relativistic Observer (SDL3)";
  cfg.windowWidth = 1280;
  cfg.windowHeight = 720;
  cfg.resizable = true;
  cfg.vsync = true;
  cfg.headless = false;
  cfg.enableAudio = true;
  cfg.showHud = true;

  WindowComponent app;
  if (!app.init(cfg)) {
    std::cerr << "Errore nell'inizializzazione della finestra SDL3!\n";
    return 1;
  }

  // Porta la finestra in primo piano su macOS
  if (app.window()) {
    SDL_ShowWindow(app.window());
    SDL_RaiseWindow(app.window());
  }

  // 2. Inizializzazione della simulazione di gioco
  Scene scene;
  scene.M = 1.0;
  scene.rIn = 6.0;   // ISCO
  scene.rOut = 22.0; // Bordo esterno disco di accrescimento
  scene.beaconR = 9.0;
  scene.beaconNu = 30.0;

  ObserverCapacity cap;
  cap.width = 320;
  cap.height = 180;
  cap.fs = 60.0;
  cap.Cops = 5e9;

  std::unique_ptr<Game> game = std::make_unique<Game>(scene, cap);
  game->cam.r = 28.0;     // Distanza orbitale iniziale di sicurezza
  game->cam.theta = 1.35; // Inclinazione della vista rispetto al disco
  game->cam.phi = 0.0;

  // 3. Ciclo Principale (Game Loop a 60 FPS con rendering geodetico interattivo)
  while (app.pollEvents()) {
    double dt = app.computeDeltaTime();
    auto &im = app.input();

    // Hotkey [R] per riavviare la simulazione
    if (im.wasKeyPressed(SDL_SCANCODE_R)) {
      game = std::make_unique<Game>(scene, cap);
      game->cam.r = 28.0;
      std::cout << "[Simulator] Reset eseguito.\n";
    }

    // Hotkeys [1-4] per i preset di risoluzione e computazione dell'osservatore
    if (im.wasKeyPressed(SDL_SCANCODE_1)) {
      game->cap.width = 160; game->cap.height = 90;
      game->plan = negotiate(game->cap);
    }
    if (im.wasKeyPressed(SDL_SCANCODE_2)) {
      game->cap.width = 320; game->cap.height = 180;
      game->plan = negotiate(game->cap);
    }
    if (im.wasKeyPressed(SDL_SCANCODE_3)) {
      game->cap.width = 480; game->cap.height = 270;
      game->plan = negotiate(game->cap);
    }
    if (im.wasKeyPressed(SDL_SCANCODE_4)) {
      game->cap.width = 640; game->cap.height = 360;
      game->plan = negotiate(game->cap);
    }

    // Gestione zoom/distanza col mouse wheel
    if (std::abs(im.mouseWheelY()) > 0.01f) {
      game->cam.r = std::clamp(game->cam.r - im.mouseWheelY() * 1.5, 2.2, 80.0);
    }

    // Spinta di emergenza con la Barra Spaziatrice (hovering anti-caduta)
    engine::Input in = im.toEngineInput(1.0);
    if (im.isKeyDown(SDL_SCANCODE_SPACE)) {
      in.thrustR += 2.0; // Spinta outward vigorosa
    }

    // Calcolo modulo della spinta per l'effetto sonoro
    real thrustMag = std::abs(in.thrustR) + std::abs(in.orbitRate);

    // Passo fisico nel tempo PROPRIO della nave
    real dtau = dt; // dtau in secondi propri dell'osservatore
    game->step(dtau, in);

    // Aggiornamento audio relativistico continuo
    app.updateAudio(
        dt,
        game->alpha(),
        game->scene.beaconNu,
        thrustMag,
        game->tidal()
    );

    // Rendering fotonico geodetico del frame
    Image frame = game->frame();

    // Visualizzazione con HUD telemetrico su finestra SDL3
    app.renderGameWithHud(frame, *game);
  }

  app.shutdown();
  std::cout << "[Simulator] Uscita completata.\n";
  return 0;
}
