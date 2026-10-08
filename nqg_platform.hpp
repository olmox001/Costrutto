/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2026 olmox001
 *
 * nqg_platform.hpp — Client interface abstraction.
 *
 * The client ONLY exchanges raw data with Capsule objects:
 *   write: Capsule::commands
 *   read:  Capsule::framebuffer, Capsule::audioOut, Capsule::observation
 *
 * No bypass into the engine kernel.
 */
#pragma once

#include "nqg_commands.hpp"
#include "nqg_engine3d.hpp"
#include "nqg_observer.hpp"

#include <string>

namespace nqg {
namespace platform {

using cmd::Commands;
using engine::Image;
using observer::AudioSample;
using observer::Capsule;
using observer::Observation;

struct PlatformConfig {
  std::string title = "NQG";
  int width = 1280;
  int height = 720;
  bool resizable = true;
  bool fullscreen = false;
  bool vsync = true;
  bool headless = false;
  bool enableAudio = true;
  int audioSampleRate = 44100;
};

// Client interface: only talks to Capsule data ports
class IPlatform {
public:
  virtual ~IPlatform() = default;

  virtual bool init(const PlatformConfig &cfg) = 0;
  virtual void shutdown() = 0;

  // Poll OS events; returns false → quit
  virtual bool poll() = 0;

  // Latest commands collected this frame (to be written into capsule.commands)
  virtual const Commands &commands() const = 0;

  virtual double deltaTime() = 0;
  virtual double fps() const = 0;

  // Present capsule sensory streams — the ONLY output path
  // framebuffer + observation (HUD) + audioOut
  virtual void presentCapsule(const Capsule &capsule) = 0;

  virtual void setMuted(bool muted) = 0;
  virtual bool isMuted() const = 0;
};

} // namespace platform
} // namespace nqg
