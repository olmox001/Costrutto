/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2026 olmox001
 *
 * nqg_sdl_platform.hpp — SDL3 backend for IPlatform.
 * ONLY file (with tests) that may include <SDL3/SDL.h>.
 * Engine / observer / commands never see SDL.
 */
#pragma once

#include "nqg_platform.hpp"

#include <SDL3/SDL.h>

#if defined(__APPLE__) && defined(TARGET_OS_OSX) && TARGET_OS_OSX
#include <objc/objc-runtime.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace nqg {
namespace platform {

// ============================================================================
// Internal: float RGB Image → streaming RGBA texture
// ============================================================================
class PixelStreamer {
public:
  ~PixelStreamer() { destroy(); }
  void destroy() {
    if (texture_) {
      SDL_DestroyTexture(texture_);
      texture_ = nullptr;
    }
    texW_ = texH_ = 0;
  }
  bool upload(SDL_Renderer *ren, const Image &img) {
    if (!ren || img.w <= 0 || img.h <= 0)
      return false;
    if (!texture_ || texW_ != img.w || texH_ != img.h) {
      destroy();
      texture_ = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGBA32,
                                   SDL_TEXTUREACCESS_STREAMING, img.w, img.h);
      if (!texture_)
        return false;
      SDL_SetTextureScaleMode(texture_, SDL_SCALEMODE_LINEAR);
      texW_ = img.w;
      texH_ = img.h;
      buf_.resize(std::size_t(texW_) * texH_);
    }
    const std::size_t n = std::size_t(img.w) * img.h;
    for (std::size_t i = 0; i < n; ++i) {
      const uint32_t ir = uint32_t(std::clamp(img.px[i * 3 + 0], 0.f, 1.f) * 255.f);
      const uint32_t ig = uint32_t(std::clamp(img.px[i * 3 + 1], 0.f, 1.f) * 255.f);
      const uint32_t ib = uint32_t(std::clamp(img.px[i * 3 + 2], 0.f, 1.f) * 255.f);
      buf_[i] = (255u << 24) | (ib << 16) | (ig << 8) | ir;
    }
    return SDL_UpdateTexture(texture_, nullptr, buf_.data(),
                             int(texW_ * sizeof(uint32_t)));
  }
  void draw(SDL_Renderer *ren, int winW, int winH) {
    if (!ren || !texture_)
      return;
    const float ta = float(texW_) / float(texH_);
    const float wa = float(winW) / float(winH);
    SDL_FRect dst;
    if (wa > ta) {
      dst.h = float(winH);
      dst.w = dst.h * ta;
      dst.x = (winW - dst.w) * 0.5f;
      dst.y = 0;
    } else {
      dst.w = float(winW);
      dst.h = dst.w / ta;
      dst.x = 0;
      dst.y = (winH - dst.h) * 0.5f;
    }
    SDL_RenderTexture(ren, texture_, nullptr, &dst);
  }

private:
  SDL_Texture *texture_ = nullptr;
  int texW_ = 0, texH_ = 0;
  std::vector<uint32_t> buf_;
};

// ============================================================================
// Internal: simple transient synthesizer (SDL audio stream)
// ============================================================================
class TransientSynth {
public:
  explicit TransientSynth(int sr = 44100) : sr_(sr) {}
  ~TransientSynth() { shutdown(); }
  bool init() {
    SDL_AudioSpec spec{SDL_AUDIO_F32, 1, sr_};
    stream_ = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK,
                                        &spec, nullptr, nullptr);
    if (!stream_)
      return false;
    SDL_ResumeAudioStreamDevice(stream_);
    ok_ = true;
    return true;
  }
  void shutdown() {
    if (stream_) {
      SDL_DestroyAudioStream(stream_);
      stream_ = nullptr;
    }
    ok_ = false;
  }
  void setMuted(bool m) { muted_ = m; }
  bool isMuted() const { return muted_; }
  void trigger(float freq, float intensity, float decay) {
    if (muted_ || !ok_)
      return;
    t_.freq = std::clamp(freq, 60.f, 6000.f);
    t_.amp = std::clamp(intensity, 0.f, 1.f) * 0.35f;
    t_.decay = std::max(1.f, decay);
    t_.phase = 0.f;
  }
  void pump(double dt) {
    if (!ok_ || muted_ || t_.amp < 1e-5f)
      return;
    const int n = std::max(1, int(sr_ * dt));
    std::vector<float> buf((std::size_t(n)));
    const float invSr = 1.f / float(sr_);
    for (int i = 0; i < n; ++i) {
      buf[std::size_t(i)] = t_.amp * std::sin(6.28318530718f * t_.freq * t_.phase);
      t_.phase += invSr;
      t_.amp *= std::exp(-t_.decay * invSr);
    }
    SDL_PutAudioStreamData(stream_, buf.data(), int(n * sizeof(float)));
  }

private:
  int sr_;
  SDL_AudioStream *stream_ = nullptr;
  bool ok_ = false, muted_ = false;
  struct {
    float freq = 440, amp = 0, decay = 15, phase = 0;
  } t_;
};

// ============================================================================
// Input collector (SDL → Commands)
// ============================================================================
class SdlInput {
public:
  void resetEdges() {
    mouseDX_ = mouseDY_ = 0;
    lookDX_ = lookDY_ = 0;
    justPressed_.clear();
  }
  void onEvent(const SDL_Event &e) {
    switch (e.type) {
    case SDL_EVENT_KEY_DOWN:
      if (!keyDown(e.key.scancode))
        justPressed_.push_back(e.key.scancode);
      keys_[e.key.scancode] = true;
      break;
    case SDL_EVENT_KEY_UP:
      keys_[e.key.scancode] = false;
      break;
    case SDL_EVENT_MOUSE_MOTION:
      mouseDX_ += e.motion.xrel;
      mouseDY_ += e.motion.yrel;
      break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
      if (e.button.button == SDL_BUTTON_LEFT)
        mouseLeft_ = true;
      break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
      if (e.button.button == SDL_BUTTON_LEFT)
        mouseLeft_ = false;
      break;
    case SDL_EVENT_FINGER_DOWN: {
      const SDL_FingerID id = e.tfinger.fingerID;
      Finger f;
      f.id = id;
      f.x = e.tfinger.x;
      f.y = e.tfinger.y;
      if (e.tfinger.x < 0.45f && moveId_ == 0) {
        f.role = Role::Move;
        moveId_ = id;
        moveOx_ = f.x;
        moveOy_ = f.y;
      } else if (e.tfinger.x >= 0.45f && lookId_ == 0) {
        f.role = Role::Look;
        lookId_ = id;
      }
      fingers_[id] = f;
      break;
    }
    case SDL_EVENT_FINGER_MOTION: {
      auto it = fingers_.find(e.tfinger.fingerID);
      if (it == fingers_.end())
        break;
      Finger &f = it->second;
      f.x = e.tfinger.x;
      f.y = e.tfinger.y;
      if (f.role == Role::Move && f.id == moveId_) {
        const float dx = f.x - moveOx_, dy = f.y - moveOy_;
        const float len = std::sqrt(dx * dx + dy * dy);
        const float maxR = 0.12f, dead = 0.015f;
        if (len > dead) {
          const float s = std::min(1.f, (len - dead) / (maxR - dead));
          moveAX_ = (dx / len) * s;
          moveAY_ = (dy / len) * s;
        } else {
          moveAX_ = moveAY_ = 0;
        }
      } else if (f.role == Role::Look && f.id == lookId_) {
        lookDX_ += e.tfinger.dx * 400.f;
        lookDY_ += e.tfinger.dy * 400.f;
      }
      break;
    }
    case SDL_EVENT_FINGER_UP: {
      const SDL_FingerID id = e.tfinger.fingerID;
      if (id == moveId_) {
        moveId_ = 0;
        moveAX_ = moveAY_ = 0;
      }
      if (id == lookId_)
        lookId_ = 0;
      fingers_.erase(id);
      break;
    }
    default:
      break;
    }
  }

  Commands toCommands(double dt, float lookSens = 0.0035f) const {
    Commands c;
    if (moveId_ != 0) {
      c.moveX = moveAX_;
      c.moveY = -moveAY_;
    } else {
      if (keyDown(SDL_SCANCODE_W))
        c.moveY += 1.f;
      if (keyDown(SDL_SCANCODE_S))
        c.moveY -= 1.f;
      if (keyDown(SDL_SCANCODE_A))
        c.moveX -= 1.f;
      if (keyDown(SDL_SCANCODE_D))
        c.moveX += 1.f;
      const float l2 = c.moveX * c.moveX + c.moveY * c.moveY;
      if (l2 > 1.f) {
        const float inv = 1.f / std::sqrt(l2);
        c.moveX *= inv;
        c.moveY *= inv;
      }
    }
    if (mouseLeft_) {
      c.lookYaw += mouseDX_ * lookSens;
      c.lookPitch += mouseDY_ * lookSens;
    }
    if (lookId_ != 0) {
      c.lookYaw += lookDX_ * lookSens;
      c.lookPitch += lookDY_ * lookSens;
    }
    if (keyDown(SDL_SCANCODE_LEFT))
      c.lookYaw -= 1.6 * dt;
    if (keyDown(SDL_SCANCODE_RIGHT))
      c.lookYaw += 1.6 * dt;
    if (keyDown(SDL_SCANCODE_UP))
      c.lookPitch -= 1.2 * dt;
    if (keyDown(SDL_SCANCODE_DOWN))
      c.lookPitch += 1.2 * dt;

    c.jumpHeld = keyDown(SDL_SCANCODE_SPACE);
    c.jumpPressed = wasPressed(SDL_SCANCODE_SPACE) || (c.jumpHeld && !prevJumpHeld_);
    c.jumpReleased = (!c.jumpHeld && prevJumpHeld_);
    prevJumpHeld_ = c.jumpHeld;
    c.crouch = keyDown(SDL_SCANCODE_C);
    c.boost = keyDown(SDL_SCANCODE_LSHIFT) || keyDown(SDL_SCANCODE_RSHIFT);
    c.action1 = wasPressed(SDL_SCANCODE_1) || keyDown(SDL_SCANCODE_1);
    c.action2 = wasPressed(SDL_SCANCODE_2) || keyDown(SDL_SCANCODE_2);
    c.action3 = wasPressed(SDL_SCANCODE_3);
    c.toggleWind = wasPressed(SDL_SCANCODE_T) || keyDown(SDL_SCANCODE_T);
    c.toggleGravity = wasPressed(SDL_SCANCODE_G);
    c.toggleLight = wasPressed(SDL_SCANCODE_L);
    c.clearDynamics = wasPressed(SDL_SCANCODE_X);
    c.quit = wasPressed(SDL_SCANCODE_ESCAPE);
    c.toggleHud = wasPressed(SDL_SCANCODE_H);
    c.toggleMute = wasPressed(SDL_SCANCODE_M);
    return c;
  }

private:
  enum class Role { None, Move, Look };
  struct Finger {
    SDL_FingerID id = 0;
    Role role = Role::None;
    float x = 0, y = 0;
  };
  bool keyDown(SDL_Scancode s) const {
    auto it = keys_.find(s);
    return it != keys_.end() && it->second;
  }
  bool wasPressed(SDL_Scancode s) const {
    return std::find(justPressed_.begin(), justPressed_.end(), s) !=
           justPressed_.end();
  }
  std::unordered_map<SDL_Scancode, bool> keys_;
  std::vector<SDL_Scancode> justPressed_;
  float mouseDX_ = 0, mouseDY_ = 0;
  bool mouseLeft_ = false;
  std::unordered_map<SDL_FingerID, Finger> fingers_;
  SDL_FingerID moveId_ = 0, lookId_ = 0;
  float moveOx_ = 0, moveOy_ = 0, moveAX_ = 0, moveAY_ = 0;
  float lookDX_ = 0, lookDY_ = 0;
  mutable bool prevJumpHeld_ = false;
};

// ============================================================================
// SdlPlatform — concrete IPlatform
// ============================================================================
class SdlPlatform final : public IPlatform {
public:
  bool init(const PlatformConfig &cfg) override {
    cfg_ = cfg;
    uint32_t flags = SDL_INIT_VIDEO | SDL_INIT_EVENTS;
    if (cfg.enableAudio)
      flags |= SDL_INIT_AUDIO;
    if (!SDL_Init(flags)) {
      std::fprintf(stderr, "[SdlPlatform] SDL_Init: %s\n", SDL_GetError());
      return false;
    }
    if (cfg.headless) {
      headless_ = true;
      last_ = std::chrono::steady_clock::now();
      return true;
    }
    uint32_t wf = 0;
    if (cfg.resizable)
      wf |= SDL_WINDOW_RESIZABLE;
    if (cfg.fullscreen)
      wf |= SDL_WINDOW_FULLSCREEN;
    window_ = SDL_CreateWindow(cfg.title.c_str(), cfg.width, cfg.height, wf);
    if (!window_) {
      std::fprintf(stderr, "[SdlPlatform] CreateWindow: %s\n", SDL_GetError());
      SDL_Quit();
      return false;
    }
#if defined(__APPLE__) && defined(TARGET_OS_OSX) && TARGET_OS_OSX
    {
      id cls = (id)objc_getClass("NSApplication");
      if (cls) {
        id app = ((id(*)(id, SEL))objc_msgSend)(cls, sel_registerName("sharedApplication"));
        if (app) {
          ((void (*)(id, SEL, long))objc_msgSend)(app, sel_registerName("setActivationPolicy:"), 0);
          ((void (*)(id, SEL, BOOL))objc_msgSend)(app, sel_registerName("activateIgnoringOtherApps:"), YES);
        }
      }
    }
#endif
    SDL_ShowWindow(window_);
    SDL_RaiseWindow(window_);
    renderer_ = SDL_CreateRenderer(window_, nullptr);
    if (!renderer_) {
      std::fprintf(stderr, "[SdlPlatform] CreateRenderer: %s\n", SDL_GetError());
      SDL_DestroyWindow(window_);
      window_ = nullptr;
      SDL_Quit();
      return false;
    }
    if (cfg.vsync)
      SDL_SetRenderVSync(renderer_, 1);
    if (cfg.enableAudio) {
      synth_ = std::make_unique<TransientSynth>(cfg.audioSampleRate);
      synth_->init();
    }
    last_ = std::chrono::steady_clock::now();
    return true;
  }

  void shutdown() override {
    synth_.reset();
    streamer_.destroy();
    if (renderer_) {
      SDL_DestroyRenderer(renderer_);
      renderer_ = nullptr;
    }
    if (window_) {
      SDL_DestroyWindow(window_);
      window_ = nullptr;
    }
    if (!headless_)
      SDL_Quit();
  }

  bool poll() override {
    input_.resetEdges();
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
      if (ev.type == SDL_EVENT_QUIT ||
          ev.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED)
        return false;
      if (ev.type == SDL_EVENT_KEY_DOWN &&
          ev.key.scancode == SDL_SCANCODE_ESCAPE)
        return false;
      if (ev.type == SDL_EVENT_KEY_DOWN &&
          ev.key.scancode == SDL_SCANCODE_H)
        hudVisible_ = !hudVisible_;
      if (ev.type == SDL_EVENT_KEY_DOWN &&
          ev.key.scancode == SDL_SCANCODE_M && synth_)
        synth_->setMuted(!synth_->isMuted());
      input_.onEvent(ev);
    }
    // provisional dt for rate-based keys inside toCommands
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_).count();
    dt = std::clamp(dt, 0.0001, 0.1);
    commands_ = input_.toCommands(dt);
    if (commands_.toggleHud)
      hudVisible_ = !hudVisible_;
    if (commands_.toggleMute && synth_)
      synth_->setMuted(!synth_->isMuted());
    if (commands_.quit)
      return false;
    return true;
  }

  const Commands &commands() const override { return commands_; }

  double deltaTime() override {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_).count();
    last_ = now;
    dt = std::clamp(dt, 0.0001, 0.1);
    if (dt > 0)
      fps_ = 0.9 * fps_ + 0.1 * (1.0 / dt);
    if (synth_)
      synth_->pump(dt);
    return dt;
  }

  double fps() const override { return fps_; }

  // Present only capsule data ports (framebuffer + observation + audio)
  void presentCapsule(const Capsule &cap) override {
    if (headless_ || !renderer_)
      return;
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);
    int w = 0, h = 0;
    SDL_GetWindowSize(window_, &w, &h);
    streamer_.upload(renderer_, cap.framebuffer);
    streamer_.draw(renderer_, w, h);

    if (hudVisible_) {
      SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
      int y = 28;
      for (const auto &line : cap.observation.lines) {
        SDL_SetRenderDrawColor(renderer_, 230, 240, 255, 255);
        SDL_RenderDebugText(renderer_, 30, float(y), line.c_str());
        y += 16;
      }
      // crosshair
      SDL_SetRenderDrawColor(renderer_, 80, 210, 255, 200);
      SDL_RenderLine(renderer_, w / 2 - 8, h / 2, w / 2 + 8, h / 2);
      SDL_RenderLine(renderer_, w / 2, h / 2 - 8, w / 2, h / 2 + 8);
    }

    // Play audio samples owned by the capsule
    if (synth_) {
      for (const auto &a : cap.audioOut)
        synth_->trigger(a.freqHz, a.intensity, a.decay);
    }
    SDL_RenderPresent(renderer_);
  }

  void setMuted(bool m) override {
    if (synth_)
      synth_->setMuted(m);
  }
  bool isMuted() const override {
    return synth_ ? synth_->isMuted() : true;
  }

private:
  PlatformConfig cfg_;
  bool headless_ = false;
  bool hudVisible_ = true;
  SDL_Window *window_ = nullptr;
  SDL_Renderer *renderer_ = nullptr;
  PixelStreamer streamer_;
  SdlInput input_;
  Commands commands_;
  std::unique_ptr<TransientSynth> synth_;
  std::chrono::steady_clock::time_point last_;
  double fps_ = 60.0;
};

// ============================================================================
// Headless platform (tests / CI — no window, no SDL video required if
// linked carefully; still needs SDL if other code pulls it)
// ============================================================================
class HeadlessPlatform final : public IPlatform {
public:
  bool init(const PlatformConfig &) override {
    last_ = std::chrono::steady_clock::now();
    return true;
  }
  void shutdown() override {}
  bool poll() override { return running_; }
  const Commands &commands() const override { return commands_; }
  double deltaTime() override {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_).count();
    last_ = now;
    return std::clamp(dt, 0.0001, 0.1);
  }
  double fps() const override { return 0; }
  void presentCapsule(const Capsule &) override {}
  void setMuted(bool) override {}
  bool isMuted() const override { return true; }

  // test helpers
  void inject(const Commands &c) { commands_ = c; }
  void requestQuit() { running_ = false; }

private:
  Commands commands_;
  bool running_ = true;
  std::chrono::steady_clock::time_point last_;
};

} // namespace platform
} // namespace nqg
