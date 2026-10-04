// ============================================================================
//  nqg_window_sdl3.hpp  -  Componente finestra interattiva SDL3 per NQG Engine
//  Collega il core fisico (nqg_physics_core.hpp) e il motore 3D relativistico
//  (nqg_engine3d.hpp) con una finestra grafica SDL3, gestione input da tastiera,
//  mouse e controller, rendering streaming e sintetizzatore audio relativistico.
// ============================================================================
#ifndef NQG_WINDOW_SDL3_HPP
#define NQG_WINDOW_SDL3_HPP

#include "nqg_physics_core.hpp"
#include "nqg_engine3d.hpp"

#include <SDL3/SDL.h>
#ifdef __APPLE__
#include <objc/objc-runtime.h>
inline void macosBringToFront() {
  id nsAppClass = (id)objc_getClass("NSApplication");
  if (!nsAppClass) return;
  SEL sharedAppSel = sel_registerName("sharedApplication");
  id app = ((id(*)(id, SEL))objc_msgSend)(nsAppClass, sharedAppSel);
  if (!app) return;

  SEL setPolicySel = sel_registerName("setActivationPolicy:");
  ((void(*)(id, SEL, long))objc_msgSend)(app, setPolicySel, 0); // NSApplicationActivationPolicyRegular

  SEL activateSel = sel_registerName("activateIgnoringOtherApps:");
  ((void(*)(id, SEL, BOOL))objc_msgSend)(app, activateSel, YES);
}
#endif
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

namespace nqg {
namespace window {

// ----------------------------------------------------------------------------
// Configurazione della Finestra
// ----------------------------------------------------------------------------
struct WindowConfig {
  std::string title = "NQG Engine 3D - Relativistic Schwarzschild Observer";
  int windowWidth = 1280;
  int windowHeight = 720;
  bool resizable = true;
  bool fullscreen = false;
  bool vsync = true;
  bool headless = false;       // Per test automatici e modalita' off-screen
  bool enableAudio = true;      // Sintesi audio relativistica
  bool showHud = true;          // Overlay telemetria e NQG
  int audioSampleRate = 44100;
};

// ----------------------------------------------------------------------------
// Gestione degli Input (Tastiera + Mouse + Mappatura Comandi Motore)
// ----------------------------------------------------------------------------
class InputManager {
public:
  void resetPerFrame() {
    mouseDeltaX_ = 0.0f;
    mouseDeltaY_ = 0.0f;
    mouseWheelY_ = 0.0f;
    justPressedKeys_.clear();
  }

  void processEvent(const SDL_Event &event) {
    switch (event.type) {
      case SDL_EVENT_KEY_DOWN: {
        SDL_Scancode sc = event.key.scancode;
        if (!isKeyDown(sc)) {
          justPressedKeys_.push_back(sc);
        }
        keys_[sc] = true;
        break;
      }
      case SDL_EVENT_KEY_UP: {
        keys_[event.key.scancode] = false;
        break;
      }
      case SDL_EVENT_MOUSE_MOTION: {
        mouseDeltaX_ += event.motion.xrel;
        mouseDeltaY_ += event.motion.yrel;
        mouseX_ = event.motion.x;
        mouseY_ = event.motion.y;
        break;
      }
      case SDL_EVENT_MOUSE_BUTTON_DOWN: {
        if (event.button.button == SDL_BUTTON_LEFT) mouseLeft_ = true;
        if (event.button.button == SDL_BUTTON_RIGHT) mouseRight_ = true;
        break;
      }
      case SDL_EVENT_MOUSE_BUTTON_UP: {
        if (event.button.button == SDL_BUTTON_LEFT) mouseLeft_ = false;
        if (event.button.button == SDL_BUTTON_RIGHT) mouseRight_ = false;
        break;
      }
      case SDL_EVENT_MOUSE_WHEEL: {
        mouseWheelY_ += event.wheel.y;
        break;
      }
      default:
        break;
    }
  }

  bool isKeyDown(SDL_Scancode sc) const {
    if (sc >= 0 && static_cast<std::size_t>(sc) < keys_.size()) {
      return keys_[sc];
    }
    return false;
  }

  bool wasKeyPressed(SDL_Scancode sc) const {
    return std::find(justPressedKeys_.begin(), justPressedKeys_.end(), sc) != justPressedKeys_.end();
  }

  float mouseDeltaX() const { return mouseDeltaX_; }
  float mouseDeltaY() const { return mouseDeltaY_; }
  float mouseWheelY() const { return mouseWheelY_; }
  float mouseX() const { return mouseX_; }
  float mouseY() const { return mouseY_; }
  bool isMouseLeftDown() const { return mouseLeft_; }
  bool isMouseRightDown() const { return mouseRight_; }

  // Mappatura automatica sui controlli del motore (thrust radiale, yaw, pitch, orbita)
  engine::Input toEngineInput(real sensitivity = 1.0) const {
    engine::Input in;

    // Spinta radiale: W (in avanti / verso il buco nero) vs S (indietro / fuga)
    if (isKeyDown(SDL_SCANCODE_W)) in.thrustR -= 1.0 * sensitivity;
    if (isKeyDown(SDL_SCANCODE_S)) in.thrustR += 1.0 * sensitivity;

    // Orbita circolare: A (progrado / rotazione attorno) vs D (retrogrado)
    if (isKeyDown(SDL_SCANCODE_A)) in.orbitRate -= 0.6 * sensitivity;
    if (isKeyDown(SDL_SCANCODE_D)) in.orbitRate += 0.6 * sensitivity;

    // Pitch & Yaw da tastiera (Frecce direzionali)
    if (isKeyDown(SDL_SCANCODE_LEFT))  in.yawRate   -= 1.2 * sensitivity;
    if (isKeyDown(SDL_SCANCODE_RIGHT)) in.yawRate   += 1.2 * sensitivity;
    if (isKeyDown(SDL_SCANCODE_UP))    in.pitchRate += 1.0 * sensitivity;
    if (isKeyDown(SDL_SCANCODE_DOWN))  in.pitchRate -= 1.0 * sensitivity;

    // Controllo vista col mouse (tasto sinistro premuto)
    if (mouseLeft_) {
      in.yawRate   += mouseDeltaX_ * 0.005 * sensitivity;
      in.pitchRate -= mouseDeltaY_ * 0.005 * sensitivity;
    }

    return in;
  }

private:
  std::vector<bool> keys_ = std::vector<bool>(SDL_SCANCODE_COUNT, false);
  std::vector<SDL_Scancode> justPressedKeys_;
  float mouseX_ = 0.0f;
  float mouseY_ = 0.0f;
  float mouseDeltaX_ = 0.0f;
  float mouseDeltaY_ = 0.0f;
  float mouseWheelY_ = 0.0f;
  bool mouseLeft_ = false;
  bool mouseRight_ = false;
};

// ----------------------------------------------------------------------------
// Sintetizzatore Audio Relativistico (SDL3 AudioStream)
// ----------------------------------------------------------------------------
class RelativisticAudioSynthesizer {
public:
  RelativisticAudioSynthesizer(int sampleRate = 44100) : sampleRate_(sampleRate) {}

  ~RelativisticAudioSynthesizer() {
    shutdown();
  }

  bool init() {
    SDL_AudioSpec spec;
    spec.format = SDL_AUDIO_F32;
    spec.channels = 1;
    spec.freq = sampleRate_;

    stream_ = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, nullptr, nullptr);
    if (!stream_) {
      std::fprintf(stderr, "[Audio] Warning: Impossible to open SDL3 audio device: %s\n", SDL_GetError());
      return false;
    }
    SDL_ResumeAudioStreamDevice(stream_);
    initialized_ = true;
    return true;
  }

  void shutdown() {
    if (stream_) {
      SDL_DestroyAudioStream(stream_);
      stream_ = nullptr;
    }
    initialized_ = false;
  }

  bool isMuted() const { return muted_; }
  void toggleMute() { muted_ = !muted_; }
  void setMuted(bool m) { muted_ = m; }

  struct Transient {
    float freq = 440.0f;
    float amp = 0.0f;
    float decay = 15.0f;
    float phase = 0.0f;
  };

  void triggerTransient(float freq, float intensity, float decay = 25.0f) {
    if (muted_ || !initialized_) return;
    Transient t;
    t.freq = std::clamp(freq, 60.0f, 6000.0f);
    t.amp = std::clamp(intensity, 0.0f, 0.6f);
    t.decay = decay;
    t.phase = 0.0f;
    transients_.push_back(t);
  }

  // Genera audio solo se ci sono eventi fisici o flusso d'aria effettivo (zero ronzio di fondo)
  void update(real dt, real lapseAlpha, real beaconNuCoord, real thrustMag, real tidalStress) {
    (void)lapseAlpha;
    (void)beaconNuCoord;
    (void)tidalStress;
    if (!initialized_ || !stream_ || muted_) return;

    int numSamples = static_cast<int>(dt * sampleRate_);
    numSamples = std::clamp(numSamples, 64, 4096);

    std::vector<float> buffer(numSamples, 0.0f);
    float dtSec = 1.0f / sampleRate_;

    // Se non ci sono transienti attivi e la velocita' e' minima, silenzio assoluto pulito
    if (transients_.empty() && thrustMag < 0.05) {
      // Invia silenzio pulito per mantenere lo stream sincronizzato senza glitch
      SDL_PutAudioStreamData(stream_, buffer.data(), buffer.size() * sizeof(float));
      return;
    }

    for (int i = 0; i < numSamples; ++i) {
      float out = 0.0f;

      // 1. Rumore di flusso d'aria dolce (whoosh aerodinamico sopra soglia, zero ronzio)
      if (thrustMag > 0.08) {
        float white = (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)) * 2.0f - 1.0f;
        // Filtro passa-basso dolce (rumore d'aria naturale, non ronzio elettrico)
        thrusterRumbleFilter_ = 0.95f * thrusterRumbleFilter_ + 0.05f * white;
        float airVolume = static_cast<float>(std::clamp((thrustMag - 0.08) * 0.12, 0.0, 0.25));
        out += thrusterRumbleFilter_ * airVolume;
      }

      // 2. Transienti acustici fisici (gocce d'acqua, urti granelli di sabbia, impatti solidi)
      for (auto &tr : transients_) {
        tr.phase += 2.0f * static_cast<float>(nqg::PI) * tr.freq * dtSec;
        if (tr.phase > 2.0f * nqg::PI) tr.phase -= 2.0f * static_cast<float>(nqg::PI);
        out += tr.amp * std::sin(tr.phase);
        tr.amp *= (1.0f - tr.decay * dtSec); // Decadimento esponenziale
      }

      buffer[i] = std::clamp(out, -0.8f, 0.8f);
    }

    // Rimuovi transienti esauriti
    transients_.erase(
        std::remove_if(transients_.begin(), transients_.end(),
                       [](const Transient &t) { return t.amp < 0.001f; }),
        transients_.end()
    );

    SDL_PutAudioStreamData(stream_, buffer.data(), buffer.size() * sizeof(float));
  }

private:
  int sampleRate_ = 44100;
  SDL_AudioStream *stream_ = nullptr;
  bool initialized_ = false;
  bool muted_ = false;
  float thrusterRumbleFilter_ = 0.0f;
  std::vector<Transient> transients_;
};

// ----------------------------------------------------------------------------
// Pixel Streamer e Texture Renderer
// ----------------------------------------------------------------------------
class PixelStreamer {
public:
  ~PixelStreamer() {
    destroyTexture();
  }

  void destroyTexture() {
    if (texture_) {
      SDL_DestroyTexture(texture_);
      texture_ = nullptr;
    }
    texW_ = 0;
    texH_ = 0;
  }

  bool updateTexture(SDL_Renderer *renderer, const engine::Image &img) {
    if (!renderer || img.w <= 0 || img.h <= 0) return false;

    // Crea o rialloca la texture se le dimensioni sono cambiate
    if (!texture_ || texW_ != img.w || texH_ != img.h) {
      destroyTexture();
      texture_ = SDL_CreateTexture(
          renderer,
          SDL_PIXELFORMAT_RGBA32,
          SDL_TEXTUREACCESS_STREAMING,
          img.w,
          img.h
      );
      if (!texture_) {
        std::fprintf(stderr, "[PixelStreamer] SDL_CreateTexture failed: %s\n", SDL_GetError());
        return false;
      }
      SDL_SetTextureScaleMode(texture_, SDL_SCALEMODE_LINEAR);
      texW_ = img.w;
      texH_ = img.h;
      rgbaBuffer_.resize(std::size_t(texW_) * texH_);
    }

    // Conversione da float RGB [0, 1] a RGBA32 (0xAABBGGRR / 32-bit packed)
    const std::size_t numPixels = std::size_t(img.w) * img.h;
    for (std::size_t i = 0; i < numPixels; ++i) {
      const float r = std::clamp(img.px[i * 3 + 0], 0.0f, 1.0f);
      const float g = std::clamp(img.px[i * 3 + 1], 0.0f, 1.0f);
      const float b = std::clamp(img.px[i * 3 + 2], 0.0f, 1.0f);

      const uint32_t ir = static_cast<uint32_t>(r * 255.0f);
      const uint32_t ig = static_cast<uint32_t>(g * 255.0f);
      const uint32_t ib = static_cast<uint32_t>(b * 255.0f);

      rgbaBuffer_[i] = (255u << 24) | (ib << 16) | (ig << 8) | ir;
    }

    // Aggiorna lo streaming texture
    return SDL_UpdateTexture(
        texture_,
        nullptr,
        rgbaBuffer_.data(),
        static_cast<int>(texW_ * sizeof(uint32_t))
    );
  }

  void render(SDL_Renderer *renderer, int winW, int winH) {
    if (!renderer || !texture_) return;

    // Calcola il rettangolo di destinazione mantenendo il rapporto d'aspetto (letterbox/pillarbox)
    float targetAspect = static_cast<float>(texW_) / static_cast<float>(texH_);
    float winAspect = static_cast<float>(winW) / static_cast<float>(winH);

    SDL_FRect dst;
    if (winAspect > targetAspect) {
      dst.h = static_cast<float>(winH);
      dst.w = dst.h * targetAspect;
      dst.x = (winW - dst.w) * 0.5f;
      dst.y = 0.0f;
    } else {
      dst.w = static_cast<float>(winW);
      dst.h = dst.w / targetAspect;
      dst.x = 0.0f;
      dst.y = (winH - dst.h) * 0.5f;
    }

    SDL_RenderTexture(renderer, texture_, nullptr, &dst);
  }

  int textureWidth() const { return texW_; }
  int textureHeight() const { return texH_; }
  const std::vector<uint32_t>& rgbaBuffer() const { return rgbaBuffer_; }

private:
  SDL_Texture *texture_ = nullptr;
  int texW_ = 0;
  int texH_ = 0;
  std::vector<uint32_t> rgbaBuffer_;
};

// ----------------------------------------------------------------------------
// Renderer HUD Grafico e Telemetrico NQG (SDL3)
// ----------------------------------------------------------------------------
class HudRenderer {
public:
  enum class HudMode {
    Full,
    Minimal,
    Off
  };

  void toggleMode() {
    if (mode_ == HudMode::Full) mode_ = HudMode::Minimal;
    else if (mode_ == HudMode::Minimal) mode_ = HudMode::Off;
    else mode_ = HudMode::Full;
  }

  void setMode(HudMode m) { mode_ = m; }
  HudMode mode() const { return mode_; }

  void draw(SDL_Renderer *ren, int winW, int winH, const engine::Game &game, double fps) {
    if (mode_ == HudMode::Off || !ren) return;

    // Abilita alpha blending per il testo e i pannelli HUD
    SDL_SetRenderDrawBlendMode(ren, SDL_BLENDMODE_BLEND);

    // 1. Mirino centrale e vettore orizzonte
    drawCenterReticle(ren, winW, winH, game);

    // 2. Pannelli minimi: sempre disegnati in Minimal e Full
    drawVitals(ren, 20.0f, winH - 90.0f, game);

    if (mode_ == HudMode::Minimal) {
      SDL_SetRenderDrawColor(ren, 180, 220, 255, 220);
      SDL_RenderDebugTextFormat(ren, winW - 220.0f, 20.0f, "FPS: %.1f | r: %.2f M", fps, game.cam.r);
      return;
    }

    // 3. Pannello Telemetria Relativistica (in alto a sinistra)
    drawTelemetryPanel(ren, 20.0f, 20.0f, game, fps);

    // 4. Pannello Capacita' Osservatore (in alto a destra)
    drawObserverPanel(ren, winW - 320.0f, 20.0f, game);

    // 5. Pannello Ipotesi NQG - Livelli Annidati Fisher-Rao (in basso a destra)
    drawNestedLevelsPanel(ren, winW - 340.0f, winH - 120.0f, game);

    // 6. Guida ai comandi (in basso al centro)
    drawHelpHint(ren, winW, winH);
  }

private:
  HudMode mode_ = HudMode::Full;

  void drawBox(SDL_Renderer *ren, float x, float y, float w, float h, SDL_Color bg, SDL_Color border) {
    SDL_FRect r{x, y, w, h};
    SDL_SetRenderDrawColor(ren, bg.r, bg.g, bg.b, bg.a);
    SDL_RenderFillRect(ren, &r);
    SDL_SetRenderDrawColor(ren, border.r, border.g, border.b, border.a);
    SDL_RenderRect(ren, &r);
  }

  void drawProgressBar(SDL_Renderer *ren, float x, float y, float w, float h, float frac,
                       SDL_Color fill, SDL_Color bg) {
    frac = std::clamp(frac, 0.0f, 1.0f);
    SDL_FRect bgRect{x, y, w, h};
    SDL_SetRenderDrawColor(ren, bg.r, bg.g, bg.b, bg.a);
    SDL_RenderFillRect(ren, &bgRect);

    SDL_FRect fillRect{x, y, w * frac, h};
    SDL_SetRenderDrawColor(ren, fill.r, fill.g, fill.b, fill.a);
    SDL_RenderFillRect(ren, &fillRect);

    SDL_SetRenderDrawColor(ren, 120, 140, 160, 200);
    SDL_RenderRect(ren, &bgRect);
  }

  void drawCenterReticle(SDL_Renderer *ren, int winW, int winH, const engine::Game &game) {
    float cx = winW * 0.5f;
    float cy = winH * 0.5f;

    // Cerchio/mirino centrale
    SDL_SetRenderDrawColor(ren, 80, 200, 255, 120);
    SDL_RenderLine(ren, cx - 14.0f, cy, cx - 4.0f, cy);
    SDL_RenderLine(ren, cx + 4.0f, cy, cx + 14.0f, cy);
    SDL_RenderLine(ren, cx, cy - 14.0f, cx, cy - 4.0f);
    SDL_RenderLine(ren, cx, cy + 4.0f, cx, cy + 14.0f);

    // Indicatore di pericolo orizzonte (se r < 4M)
    if (game.cam.r < 4.0) {
      SDL_SetRenderDrawColor(ren, 255, 60, 60, 200);
      SDL_RenderDebugText(ren, cx - 64.0f, cy + 24.0f, "! HORIZON PROXIMITY !");
    }
  }

  void drawTelemetryPanel(SDL_Renderer *ren, float x, float y, const engine::Game &game, double fps) {
    drawBox(ren, x, y, 290.0f, 150.0f, {15, 20, 30, 200}, {40, 70, 100, 240});

    SDL_SetRenderDrawColor(ren, 100, 220, 255, 255);
    SDL_RenderDebugText(ren, x + 10.0f, y + 10.0f, "--- RELATIVISTIC TELEMETRY ---");

    real a = game.alpha();
    real tidalStress = game.tidal();
    real aHover = game.hoverAcceleration();

    SDL_SetRenderDrawColor(ren, 220, 230, 245, 240);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 30.0f, "Radius (r):    %.2f M  (Rs=2.0M)", game.cam.r);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 46.0f, "Lapse alpha:   %.4f  (dt/dtau=%.2f)", a, 1.0 / a);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 62.0f, "Proper Tau:    %.2f s", game.tau);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 78.0f, "Coord Time t:  %.2f s", game.tCoord);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 94.0f, "Tidal Force:   %.4f", tidalStress);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 110.0f, "Hover Accel:   %.3f M/s^2", aHover);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 126.0f, "Engine FPS:    %.1f", fps);
  }

  void drawObserverPanel(SDL_Renderer *ren, float x, float y, const engine::Game &game) {
    drawBox(ren, x, y, 300.0f, 150.0f, {15, 20, 30, 200}, {40, 70, 100, 240});

    SDL_SetRenderDrawColor(ren, 120, 255, 160, 255);
    SDL_RenderDebugText(ren, x + 10.0f, y + 10.0f, "--- OBSERVER CAPACITY (C) ---");

    auto rep = engine::observeBeacon(game.scene.M, game.cam.r, game.scene.beaconNu, game.cap.fs);

    SDL_SetRenderDrawColor(ren, 220, 230, 245, 240);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 30.0f, "Sensor Res:    %dx%d @ %.1f Hz", game.plan.width, game.plan.height, game.plan.fsEff);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 46.0f, "Budget Limit:  %s", game.plan.limitedBy.c_str());
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 62.0f, "C_ops Usage:   %.1f%%", game.plan.usage * 100.0);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 78.0f, "Beacon Coord:  %.1f Hz", game.scene.beaconNu);
    SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 94.0f, "Observed Nu:   %.1f Hz", rep.nuObs);

    if (rep.aliased) {
      SDL_SetRenderDrawColor(ren, 255, 100, 100, 255);
      SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 114.0f, "Nyquist: ALIASED (alias: %.1f Hz)", rep.aliasFreq);
    } else {
      SDL_SetRenderDrawColor(ren, 100, 255, 120, 255);
      SDL_RenderDebugText(ren, x + 10.0f, y + 114.0f, "Nyquist: RESOLVED (signal intact)");
    }
  }

  void drawVitals(SDL_Renderer *ren, float x, float y, const engine::Game &game) {
    drawBox(ren, x, y, 280.0f, 75.0f, {15, 20, 30, 200}, {40, 70, 100, 240});

    // Fuel Bar
    SDL_SetRenderDrawColor(ren, 200, 220, 240, 240);
    SDL_RenderDebugTextFormat(ren, x + 8.0f, y + 8.0f, "FUEL:  %3.0f%%", game.fuel);
    drawProgressBar(ren, x + 90.0f, y + 8.0f, 175.0f, 10.0f, static_cast<float>(game.fuel / 100.0),
                    {50, 220, 100, 255}, {40, 50, 60, 200});

    // Hull Integrity Bar
    SDL_SetRenderDrawColor(ren, 200, 220, 240, 240);
    SDL_RenderDebugTextFormat(ren, x + 8.0f, y + 26.0f, "HULL:  %3.0f%%", game.hull);
    SDL_Color hullCol = (game.hull < 30.0) ? SDL_Color{255, 60, 60, 255} : SDL_Color{240, 140, 40, 255};
    drawProgressBar(ren, x + 90.0f, y + 26.0f, 175.0f, 10.0f, static_cast<float>(game.hull / 100.0),
                    hullCol, {40, 50, 60, 200});

    // Quantum Knowledge Score Bar
    SDL_SetRenderDrawColor(ren, 200, 220, 240, 240);
    SDL_RenderDebugTextFormat(ren, x + 8.0f, y + 44.0f, "SCORE: %.2f", game.score);
    drawProgressBar(ren, x + 90.0f, y + 44.0f, 175.0f, 10.0f, static_cast<float>(std::min(1.0, game.score / 2.5)),
                    {80, 150, 255, 255}, {40, 50, 60, 200});

    if (game.over) {
      SDL_SetRenderDrawColor(ren, 255, 50, 50, 255);
      SDL_RenderDebugTextFormat(ren, x + 8.0f, y + 58.0f, "GAME OVER: %s", game.status.c_str());
    }
  }

  void drawNestedLevelsPanel(SDL_Renderer *ren, float x, float y, const engine::Game &game) {
    drawBox(ren, x, y, 320.0f, 105.0f, {15, 20, 30, 200}, {40, 70, 100, 240});

    SDL_SetRenderDrawColor(ren, 220, 180, 255, 255);
    SDL_RenderDebugText(ren, x + 10.0f, y + 8.0f, "--- NQG NESTED LEVELS (V11) ---");

    const auto &lvls = game.levels;
    for (std::size_t i = 0; i < lvls.size() && i < 3; ++i) {
      real sCur = lvls.entropy(i);
      real s0 = (game.S0.size() > i) ? game.S0[i] : sCur;
      real deltaS = s0 - sCur;
      SDL_SetRenderDrawColor(ren, 200, 220, 255, 240);
      SDL_RenderDebugTextFormat(ren, x + 10.0f, y + 26.0f + i * 16.0f,
                                "L%zu (N=%zu): S=%.3f (dS=%.3f, lam=%.2f)",
                                i, lvls.N(i), sCur, deltaS, game.lambdaOf(i));
    }
    SDL_SetRenderDrawColor(ren, 160, 200, 240, 200);
    SDL_RenderDebugText(ren, x + 10.0f, y + 78.0f, "Status: Fisher-Rao Information Collapse");
  }

  void drawHelpHint(SDL_Renderer *ren, int winW, int winH) {
    SDL_SetRenderDrawColor(ren, 160, 180, 200, 180);
    SDL_RenderDebugText(
        ren,
        winW * 0.5f - 240.0f,
        winH - 24.0f,
        "W/S: Thrust | A/D: Orbit | Mouse Drag / Arrows: Look | [1-4]: Res | H: HUD | M: Mute"
    );
  }
};

// ----------------------------------------------------------------------------
// Componente Finestra SDL3 Completo (NQG Window Component)
// ----------------------------------------------------------------------------
class WindowComponent {
public:
  WindowComponent() = default;
  ~WindowComponent() {
    shutdown();
  }

  bool init(const WindowConfig &config) {
    config_ = config;

    uint32_t sdlFlags = SDL_INIT_VIDEO | SDL_INIT_EVENTS;
    if (config_.enableAudio) {
      sdlFlags |= SDL_INIT_AUDIO;
    }

    if (!SDL_Init(sdlFlags)) {
      std::fprintf(stderr, "[WindowComponent] SDL_Init failed: %s\n", SDL_GetError());
      return false;
    }

    // Modalita' headless per test automatici e offscreen
    if (config_.headless) {
      std::cout << "[WindowComponent] Initialized in HEADLESS mode.\n";
      isHeadless_ = true;
      return true;
    }

    uint32_t winFlags = 0;
    if (config_.resizable)  winFlags |= SDL_WINDOW_RESIZABLE;
    if (config_.fullscreen) winFlags |= SDL_WINDOW_FULLSCREEN;

    window_ = SDL_CreateWindow(
        config_.title.c_str(),
        config_.windowWidth,
        config_.windowHeight,
        winFlags
    );
    if (!window_) {
      std::fprintf(stderr, "[WindowComponent] SDL_CreateWindow failed: %s\n", SDL_GetError());
      SDL_Quit();
      return false;
    }

#ifdef __APPLE__
    macosBringToFront();
#endif
    SDL_ShowWindow(window_);
    SDL_RaiseWindow(window_);

    renderer_ = SDL_CreateRenderer(window_, nullptr);
    if (!renderer_) {
      std::fprintf(stderr, "[WindowComponent] SDL_CreateRenderer failed: %s\n", SDL_GetError());
      SDL_DestroyWindow(window_);
      window_ = nullptr;
      SDL_Quit();
      return false;
    }

    if (config_.vsync) {
      SDL_SetRenderVSync(renderer_, 1);
    }

    if (config_.enableAudio) {
      audioSynth_ = std::make_unique<RelativisticAudioSynthesizer>(config_.audioSampleRate);
      audioSynth_->init();
    }

    hud_.setMode(config_.showHud ? HudRenderer::HudMode::Full : HudRenderer::HudMode::Off);

    lastFrameTime_ = std::chrono::steady_clock::now();
    running_ = true;
    return true;
  }

  void shutdown() {
    if (audioSynth_) {
      audioSynth_->shutdown();
      audioSynth_.reset();
    }
    pixelStreamer_.destroyTexture();
    if (renderer_) {
      SDL_DestroyRenderer(renderer_);
      renderer_ = nullptr;
    }
    if (window_) {
      SDL_DestroyWindow(window_);
      window_ = nullptr;
    }
    if (SDL_WasInit(SDL_INIT_VIDEO)) {
      SDL_Quit();
    }
    running_ = false;
  }

  bool isRunning() const { return running_; }
  void stop() { running_ = false; }

  // Gestione messaggi ed eventi SDL3
  bool pollEvents() {
    input_.resetPerFrame();
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
      if (ev.type == SDL_EVENT_QUIT) {
        running_ = false;
        return false;
      }
      if (ev.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) {
        running_ = false;
        return false;
      }
      if (ev.type == SDL_EVENT_KEY_DOWN) {
        if (ev.key.scancode == SDL_SCANCODE_ESCAPE) {
          running_ = false;
          return false;
        }
        if (ev.key.scancode == SDL_SCANCODE_H) {
          hud_.toggleMode();
        }
        if (ev.key.scancode == SDL_SCANCODE_M && audioSynth_) {
          audioSynth_->toggleMute();
        }
      }
      input_.processEvent(ev);
    }
    return running_;
  }

  // Calcola il delta-time di frame
  double computeDeltaTime() {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - lastFrameTime_).count();
    lastFrameTime_ = now;

    // Aggiorna media FPS
    if (dt > 0.0) {
      double instantFps = 1.0 / dt;
      fps_ = 0.9 * fps_ + 0.1 * instantFps;
    }
    return std::clamp(dt, 0.0001, 0.1);
  }

  // Rendering del frame generato da nqg::engine
  void renderFrame(const engine::Image &img) {
    if (isHeadless_ || !renderer_) return;

    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);

    int w = 0, h = 0;
    SDL_GetWindowSize(window_, &w, &h);

    pixelStreamer_.updateTexture(renderer_, img);
    pixelStreamer_.render(renderer_, w, h);
  }

  // Rendering completo del gioco con HUD sovrapposto
  void renderGameWithHud(const engine::Image &img, const engine::Game &game) {
    if (isHeadless_ || !renderer_) return;

    renderFrame(img);

    int w = 0, h = 0;
    SDL_GetWindowSize(window_, &w, &h);
    hud_.draw(renderer_, w, h, game, fps_);
    SDL_RenderPresent(renderer_);
  }

  // Presentazione a schermo
  void present() {
    if (isHeadless_ || !renderer_) return;
    SDL_RenderPresent(renderer_);
  }

  // Aggiornamento audio
  void updateAudio(real dt, real lapseAlpha, real beaconNuCoord, real thrustMag, real tidalStress) {
    if (audioSynth_) {
      audioSynth_->update(dt, lapseAlpha, beaconNuCoord, thrustMag, tidalStress);
    }
  }

  // Accessori
  InputManager& input() { return input_; }
  const InputManager& input() const { return input_; }
  HudRenderer& hud() { return hud_; }
  PixelStreamer& pixelStreamer() { return pixelStreamer_; }
  RelativisticAudioSynthesizer* audio() { return audioSynth_.get(); }
  SDL_Window* window() { return window_; }
  SDL_Renderer* renderer() { return renderer_; }
  double fps() const { return fps_; }
  bool isHeadless() const { return isHeadless_; }

private:
  WindowConfig config_;
  SDL_Window *window_ = nullptr;
  SDL_Renderer *renderer_ = nullptr;
  InputManager input_;
  PixelStreamer pixelStreamer_;
  HudRenderer hud_;
  std::unique_ptr<RelativisticAudioSynthesizer> audioSynth_;
  std::chrono::steady_clock::time_point lastFrameTime_;
  double fps_ = 60.0;
  bool running_ = false;
  bool isHeadless_ = false;
};

} // namespace window
} // namespace nqg

#endif // NQG_WINDOW_SDL3_HPP
