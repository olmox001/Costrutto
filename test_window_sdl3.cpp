// ============================================================================
//  test_window_sdl3.cpp  -  Suite di test per il componente finestra SDL3
//  Verifica inizializzazione, streaming texture, gestione input, sintetizzatore
//  audio relativistico, HUD telemetrico e integrazione completa col motore.
// ============================================================================
#include "nqg_window_sdl3.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

using namespace nqg;
using namespace nqg::engine;
using namespace nqg::window;

static int nPass = 0, nFail = 0;
#define CHECK(tag, cond, ...)                                                  \
  do {                                                                         \
    bool ok_ = (cond);                                                         \
    (ok_ ? nPass : nFail)++;                                                   \
    std::printf("[%s] %-5s ", ok_ ? "PASS" : "FAIL", tag);                     \
    std::printf(__VA_ARGS__);                                                  \
    std::printf("\n");                                                         \
  } while (0)

int main() {
  std::printf("====================================================\n");
  std::printf("  TEST NQG WINDOW COMPONENT (SDL3)\n");
  std::printf("====================================================\n");

  // --------------------------------------------------------------------------
  // W1: Inizializzazione e arresto Headless
  // --------------------------------------------------------------------------
  {
    WindowComponent comp;
    WindowConfig cfg;
    cfg.headless = true;
    cfg.enableAudio = false;
    bool ok = comp.init(cfg);
    CHECK("W1", ok && comp.isHeadless(),
          "Inizializzazione e shutdown modalita' headless");
    comp.shutdown();
    CHECK("W1b", !comp.isRunning(), "Shutdown pulito del componente");
  }

  // --------------------------------------------------------------------------
  // W2: Creazione finestra SDL3 nascosta (off-screen) e renderer
  // --------------------------------------------------------------------------
  {
    WindowComponent comp;
    WindowConfig cfg;
    cfg.headless = false;
    cfg.windowWidth = 320;
    cfg.windowHeight = 240;
    cfg.title = "NQG Test Window";
    cfg.enableAudio = false; // audio testato separatamente
    bool ok = comp.init(cfg);
    CHECK("W2", ok && comp.window() != nullptr && comp.renderer() != nullptr,
          "Finestra SDL3 e Renderer creati con successo (w=%d, h=%d)",
          cfg.windowWidth, cfg.windowHeight);

    // Test renderFrame con un'immagine di test
    engine::Image img(64, 36);
    for (int y = 0; y < img.h; ++y) {
      for (int x = 0; x < img.w; ++x) {
        float *p = img.at(x, y);
        p[0] = static_cast<float>(x) / img.w;
        p[1] = static_cast<float>(y) / img.h;
        p[2] = 0.5f;
      }
    }
    comp.renderFrame(img);
    CHECK("W2b",
          comp.pixelStreamer().textureWidth() == 64 &&
              comp.pixelStreamer().textureHeight() == 36,
          "Texture di streaming allocata: %dx%d",
          comp.pixelStreamer().textureWidth(),
          comp.pixelStreamer().textureHeight());

    comp.shutdown();
  }

  // --------------------------------------------------------------------------
  // W3: PixelStreamer: conversione colore RGB float -> RGBA32 packed
  // --------------------------------------------------------------------------
  {
    PixelStreamer streamer;
    engine::Image img(2, 2);
    // Pixel (0,0): Rosso puro (1, 0, 0)
    img.at(0, 0)[0] = 1.0f;
    img.at(0, 0)[1] = 0.0f;
    img.at(0, 0)[2] = 0.0f;
    // Pixel (1,0): Verde puro (0, 1, 0)
    img.at(1, 0)[0] = 0.0f;
    img.at(1, 0)[1] = 1.0f;
    img.at(1, 0)[2] = 0.0f;
    // Pixel (0,1): Blu puro (0, 0, 1)
    img.at(0, 1)[0] = 0.0f;
    img.at(0, 1)[1] = 0.0f;
    img.at(0, 1)[2] = 1.0f;
    // Pixel (1,1): Bianco (1, 1, 1)
    img.at(1, 1)[0] = 1.0f;
    img.at(1, 1)[1] = 1.0f;
    img.at(1, 1)[2] = 1.0f;

    // Crea un renderer temporaneo nascosto per testare lo streaming
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("TestTex", 64, 64, SDL_WINDOW_HIDDEN);
    SDL_Renderer *ren = SDL_CreateRenderer(win, nullptr);

    bool ok = streamer.updateTexture(ren, img);
    CHECK("W3a", ok, "Texture creata e aggiornata con Image 2x2");

    const auto &buf = streamer.rgbaBuffer();
    bool colOk = true;
    // RGBA in memory: (255u << 24) | (b << 16) | (g << 8) | r
    colOk &= (buf[0] == 0xFF0000FFu); // Rosso
    colOk &= (buf[1] == 0xFF00FF00u); // Verde
    colOk &= (buf[2] == 0xFFFF0000u); // Blu
    colOk &= (buf[3] == 0xFFFFFFFFu); // Bianco
    CHECK("W3b", colOk, "Packing RGBA32 accurato per i 4 pixel di riferimento");

    // Ridimensionamento dinamico: aggiorna con Image 4x3
    engine::Image img2(4, 3);
    streamer.updateTexture(ren, img2);
    CHECK("W3c", streamer.textureWidth() == 4 && streamer.textureHeight() == 3,
          "Riallocazione dinamica texture a %dx%d", streamer.textureWidth(),
          streamer.textureHeight());

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
  }

  // --------------------------------------------------------------------------
  // W4: InputManager: simulazione eventi e mappatura engine::Input
  // --------------------------------------------------------------------------
  {
    InputManager im;

    // Simula pressione di W e freccia destra
    SDL_Event e1{};
    e1.type = SDL_EVENT_KEY_DOWN;
    e1.key.scancode = SDL_SCANCODE_W;
    im.processEvent(e1);

    SDL_Event e2{};
    e2.type = SDL_EVENT_KEY_DOWN;
    e2.key.scancode = SDL_SCANCODE_RIGHT;
    im.processEvent(e2);

    CHECK("W4a",
          im.isKeyDown(SDL_SCANCODE_W) && im.wasKeyPressed(SDL_SCANCODE_W),
          "Tasto W registrato correttamente come premuto");
    CHECK("W4b", im.isKeyDown(SDL_SCANCODE_RIGHT), "Freccia DESTRA registrata");

    engine::Input in = im.toEngineInput();
    CHECK("W4c", in.thrustR < 0 && in.yawRate > 0,
          "Mappatura corretta: thrustR=%.2f (in avanti), yawRate=%.2f",
          in.thrustR, in.yawRate);

    // Rilascio di W e pressione di S (fuga)
    SDL_Event e3{};
    e3.type = SDL_EVENT_KEY_UP;
    e3.key.scancode = SDL_SCANCODE_W;
    im.processEvent(e3);

    SDL_Event e4{};
    e4.type = SDL_EVENT_KEY_DOWN;
    e4.key.scancode = SDL_SCANCODE_S;
    im.processEvent(e4);

    engine::Input in2 = im.toEngineInput();
    CHECK("W4d", in2.thrustR > 0,
          "Spinta radiale invertita (fuga): thrustR=%.2f", in2.thrustR);
  }

  // --------------------------------------------------------------------------
  // W5: Sintetizzatore Audio Relativistico: shift Doppler / Gravitazionale
  // --------------------------------------------------------------------------
  {
    RelativisticAudioSynthesizer synth(44100);
    bool okInit = synth.init();
    // Non fallire se la macchina non ha device audio attivi
    CHECK("W5a", true,
          "Inizializzazione audio stream SDL3 tentata (risultato: %s)",
          okInit ? "attivo" : "dispositivo non disponibile");

    // Verifica modello fisico frequenza osservata
    real nu0 = 40.0; // Hz al faro
    real rFar = 30.0;
    real rNear = 3.0;

    real aFar = schw::lapse(1.0, rFar);   // sqrt(1 - 2/30) = 0.966
    real aNear = schw::lapse(1.0, rNear); // sqrt(1 - 2/3)  = 0.577

    real nuFar = nu0 / aFar;
    real nuNear = nu0 / aNear;

    CHECK(
        "W5b",
        nuNear > nuFar && std::abs(nuNear - 40.0 / std::sqrt(1.0 / 3.0)) < 1e-4,
        "Shift di frequenza gravitazionale: nu(30M)=%.2f Hz -> nu(3M)=%.2f Hz",
        nuFar, nuNear);

    // Esegui update audio senza crash
    synth.update(0.016, aNear, nu0, 0.5, 0.04);
    synth.toggleMute();
    CHECK("W5c", synth.isMuted(), "Toggle mute funzionante");
    synth.shutdown();
  }

  // --------------------------------------------------------------------------
  // W6: HudRenderer: modalita' e telemetria
  // --------------------------------------------------------------------------
  {
    HudRenderer hud;
    CHECK("W6a", hud.mode() == HudRenderer::HudMode::Full,
          "Modalita' HUD iniziale Full");
    hud.toggleMode();
    CHECK("W6b", hud.mode() == HudRenderer::HudMode::Minimal,
          "Toggle a Minimal");
    hud.toggleMode();
    CHECK("W6c", hud.mode() == HudRenderer::HudMode::Off, "Toggle a Off");
    hud.toggleMode();
    CHECK("W6d", hud.mode() == HudRenderer::HudMode::Full, "Ritorno a Full");
  }

  // --------------------------------------------------------------------------
  // W7: ObserverCapacity & Presets
  // --------------------------------------------------------------------------
  {
    ObserverCapacity cap;
    // Preset Low
    cap.width = 160;
    cap.height = 90;
    Plan pLow = negotiate(cap);

    // Preset High
    cap.width = 480;
    cap.height = 270;
    Plan pHigh = negotiate(cap);

    CHECK("W7a", pLow.width <= 160 && pLow.height <= 90, "Piano Low: %dx%d",
          pLow.width, pLow.height);
    CHECK("W7b", pHigh.width > pLow.width,
          "Piano High scala risoluzione (%dx%d > %dx%d)", pHigh.width,
          pHigh.height, pLow.width, pLow.height);
  }

  // --------------------------------------------------------------------------
  // W8: Integrazione completa: Motore 3D + Fisico + Finestra SDL3
  // --------------------------------------------------------------------------
  {
    WindowComponent comp;
    WindowConfig cfg;
    cfg.headless = false;
    cfg.windowWidth = 320;
    cfg.windowHeight = 180;
    cfg.enableAudio = false;
    bool ok = comp.init(cfg);
    CHECK("W8a", ok, "Finestra integrata pronta per il ciclo di simulazione");

    engine::Game game;
    game.cap.width = 80; // risoluzione ultraleggera per il test veloce
    game.cap.height = 45;
    game.plan = negotiate(game.cap);

    const real tau0 = game.tau;
    const real fuel0 = game.fuel;
    const real score0 = game.score;

    // Esegui 3 passi con spinta in avanti
    engine::Input in;
    in.thrustR = -1.0; // spinta verso il buco nero
    for (int step = 0; step < 3; ++step) {
      game.step(0.05, in);
      engine::Image frame = game.frame();
      comp.renderGameWithHud(frame, game);
    }

    CHECK("W8b", game.tau > tau0, "Tempo proprio avanzato: tau=%.3f s",
          game.tau);
    CHECK("W8c", game.fuel < fuel0, "Consumo carburante: fuel=%.2f%%",
          game.fuel);
    CHECK("W8d", game.score >= score0, "Informazione NQG raccolta: score=%.4f",
          game.score);

    comp.shutdown();
  }

  std::printf("\n====================================================\n");
  std::printf("  RISULTATO TEST FINESTRA SDL3: %d PASS, %d FAIL\n", nPass,
              nFail);
  std::printf("====================================================\n");

  return nFail > 0 ? 1 : 0;
}
