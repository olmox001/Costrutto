/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2026 olmox001
 *
 * nqg_host.hpp — Engine HOST interface (zero SDL).
 *
 * The host owns the kernel (CleanRoomScene) and one or more Capsules.
 * Clients (SDL, CLI/LLM, network…) only:
 *   - write Capsule::commands
 *   - call Host::step(dt)
 *   - read Capsule::{framebuffer, audioOut, observation}
 *   - read Host::logBuffer() (structured lines for AI)
 *
 * Observation capacity (resolution, HUD lines) is enforced by the Capsule.
 */
#pragma once

#include "nqg_commands.hpp"
#include "nqg_observer.hpp"
#include "engine/perf_profiler.hpp"
#include <thread>
#include <atomic>
#include "nqg_cleanroom_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <sstream>
#include <string>
#include <vector>

namespace nqg {
namespace host {

using cmd::Commands;
using cleanroom::CleanRoomScene;
using observer::Capsule;
using observer::Observation;

// Structured log entry — easy for an AI / parser to consume
struct LogEntry {
  double t = 0;           // simulation time
  int frame = 0;
  int capsuleId = 0;
  std::string kind;       // "obs" | "cmd" | "audio" | "event" | "info"
  std::string key;        // short key
  std::string value;      // free text / numbers
};

inline std::string formatLogLine(const LogEntry &e) {
  // Pipe-separated, stable schema for LLM recovery
  // t|frame|cap|kind|key|value
  char head[128];
  std::snprintf(head, sizeof(head), "%.4f|%d|%d|%s|%s|", e.t, e.frame,
                e.capsuleId, e.kind.c_str(), e.key.c_str());
  return std::string(head) + e.value;
}

class Host {
public:
  static constexpr std::size_t kMaxLog = 4096;

  Host() {
    // Default capsule "appartamento"
    Capsule c;
    c.id = 0;
    c.name = "capsule0";
    // The observer resolution is the actual rendered image resolution.
    // The SDL window is presentation-only and may be larger/Retina-scaled.
    c.viewW = 480;
    c.viewH = 360;
    c.renderEnabled = true;
    c.spawnAt(engine::Vec3(0, -3.0, c.body.eyeHeight + c.body.spawnClearance));
    c.pitch = -0.05;
    capsules_.push_back(c);
    // Publish initial observation so CLI/obs works before first step
    capsules_.back().publishObservation(scene_, 60.0);
    log("info", "host", "started capsule0 view=480x360");
  }

  CleanRoomScene &scene() { return scene_; }
  const CleanRoomScene &scene() const { return scene_; }

  std::size_t capsuleCount() const { return capsules_.size(); }
  Capsule &capsule(std::size_t i = 0) { return capsules_.at(i); }
  const Capsule &capsule(std::size_t i = 0) const { return capsules_.at(i); }

  // Add another observer (multi-capsule)
  int addCapsule(const std::string &name, int viewW, int viewH) {
    Capsule c;
    c.id = int(capsules_.size());
    c.name = name;
    c.viewW = viewW;
    c.viewH = viewH;
    c.spawnAt(engine::Vec3(0, -3.0, c.body.eyeHeight + c.body.spawnClearance));
    capsules_.push_back(c);
    log("info", "capsule_add", name + " id=" + std::to_string(c.id));
    return c.id;
  }

  // Set view resolution (observation capacity) — HUD auto-follows
  void setRenderEnabled(bool on, std::size_t cap = 0) {
    capsules_.at(cap).renderEnabled = on;
  }

  void setViewResolution(int w, int h, std::size_t cap = 0) {
    w = std::max(32, std::min(w, 1920));
    h = std::max(32, std::min(h, 1080));
    capsules_.at(cap).viewW = w;
    capsules_.at(cap).viewH = h;
    log("event", "resolution", std::to_string(w) + "x" + std::to_string(h));
  }

  // Inject commands into a capsule (client → host)
  void setCommands(const Commands &cmd, std::size_t cap = 0) {
    capsules_.at(cap).commands = cmd;
  }

  // Advance simulation for all capsules
  void setParallelRender(bool on) { parallel_render_ = on; }

  void step(double dt) {
    NQG_PERF_SCOPE("host.step");
    dt = std::clamp(dt, 1e-4, 0.1);
    simTime_ += dt;
    ++frame_;
    // Physics+logic serial (shared scene). Render may fan out after.
    const bool do_par = parallel_render_ && capsules_.size() > 1;
    for (auto &c : capsules_) {
      const bool want = c.renderEnabled;
      if (do_par) c.renderEnabled = false; // defer present
      c.tick(dt, scene_, lastFps_);
      if (do_par) c.renderEnabled = want;

      // Structured observation log
      const Observation &o = c.observation;
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "pos=%.3f,%.3f,%.3f vel=%.3f mode=%s alt=%.2f g=%.2f room=%d",
                    o.position.x, o.position.y, o.position.z, o.speed,
                    o.modeLabel.c_str(), o.altitude, o.gravity,
                    o.insideRoom ? 1 : 0);
      logCap(c.id, "obs", "state", buf);
      if (!c.audioOut.empty()) {
        std::snprintf(buf, sizeof(buf), "n=%zu", c.audioOut.size());
        logCap(c.id, "audio", "samples", buf);
      }
    }
    if (do_par) {
      NQG_PERF_SCOPE("host.parallel_render");
      std::vector<std::thread> pool;
      pool.reserve(capsules_.size());
      for (auto &c : capsules_) {
        if (!c.renderEnabled) continue;
        pool.emplace_back([&c, this]() { c.renderFrame(scene_); });
      }
      for (auto &t : pool) t.join();
    }
    // crude fps estimate from dt
    if (dt > 0)
      lastFps_ = 0.9 * lastFps_ + 0.1 * (1.0 / dt);
  }

  // Run N steps with fixed dt (batch / headless)
  void stepN(int n, double dt = 1.0 / 60.0) {
    for (int i = 0; i < n; ++i)
      step(dt);
  }

  double simTime() const { return simTime_; }
  int frame() const { return frame_; }
  double fps() const { return lastFps_; }

  // Parseable log buffer (ring)
  const std::deque<std::string> &logBuffer() const { return logLines_; }
  std::string logDump(std::size_t lastN = 0) const {
    std::ostringstream oss;
    oss << "# schema: t|frame|cap|kind|key|value\n";
    if (lastN == 0 || lastN >= logLines_.size()) {
      for (const auto &l : logLines_)
        oss << l << "\n";
    } else {
      for (std::size_t i = logLines_.size() - lastN; i < logLines_.size(); ++i)
        oss << logLines_[i] << "\n";
    }
    return oss.str();
  }

  void clearLog() { logLines_.clear(); }

  // Save capsule framebuffer as PPM (portable, no deps)
  bool saveFramebufferPPM(const std::string &path, std::size_t cap = 0) {
    auto &c = capsules_.at(cap);
    const bool prev = c.renderEnabled;
    c.renderEnabled = true;
    // one physics-less present: re-render current view
    {
      const engine::Vec3 camPos = c.eyePosition();
      const engine::Vec3 lookDir = c.lookDirection();
      const engine::Vec3 viewUpV = c.viewUp();
      c.framebuffer = scene_.renderView(c.viewW, c.viewH, camPos, lookDir, viewUpV);
    }
    c.renderEnabled = prev;
    return c.framebuffer.writePPM(path);
  }

  // Apply a textual command (CLI / LLM friendly) onto capsule 0 by default
  // Returns true if recognized.
  bool applyTextCommand(const std::string &line, std::size_t cap = 0) {
    Commands c = capsules_.at(cap).commands;
    // reset continuous axes each text cmd unless sticky
    auto trim = [](std::string s) {
      while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' '))
        s.pop_back();
      return s;
    };
    std::string s = trim(line);
    if (s.empty())
      return false;

    // touch-like virtual stick: move dx dy  (dx,dy in [-1,1])
    // look yaw pitch (radians delta)
    // action names
    if (s == "help") {
      log("info", "help",
          "move x y | look yaw pitch | jump|jump_hold|jump_release | "
          "crouch|boost | water|sand|solid | wind|gravity|light|clear | "
          "res W H | step [n] | tp x y z | freefall | quit");
      return true;
    }
    if (s.rfind("move ", 0) == 0) {
      float x = 0, y = 0;
      if (std::sscanf(s.c_str() + 5, "%f %f", &x, &y) >= 1) {
        c.moveX = std::clamp(x, -1.f, 1.f);
        c.moveY = std::clamp(y, -1.f, 1.f);
        setCommands(c, cap);
        logCap(int(cap), "cmd", "move", s.substr(5));
        return true;
      }
    }
    if (s.rfind("look ", 0) == 0) {
      float yaw = 0, pitch = 0;
      if (std::sscanf(s.c_str() + 5, "%f %f", &yaw, &pitch) >= 1) {
        c.lookYaw = yaw;
        c.lookPitch = pitch;
        setCommands(c, cap);
        logCap(int(cap), "cmd", "look", s.substr(5));
        return true;
      }
    }
    if (s == "stop") {
      c.moveX = c.moveY = 0;
      c.lookYaw = c.lookPitch = 0;
      c.jumpHeld = false;
      setCommands(c, cap);
      logCap(int(cap), "cmd", "stop", "");
      return true;
    }
    if (s == "jump") {
      c.jumpPressed = true;
      c.jumpHeld = true;
      setCommands(c, cap);
      logCap(int(cap), "cmd", "jump", "press");
      return true;
    }
    if (s == "jump_hold") {
      c.jumpHeld = true;
      setCommands(c, cap);
      return true;
    }
    if (s == "jump_release") {
      c.jumpReleased = true;
      c.jumpHeld = false;
      setCommands(c, cap);
      return true;
    }
    if (s == "crouch") {
      c.crouch = !c.crouch;
      setCommands(c, cap);
      return true;
    }
    if (s == "boost") {
      c.boost = !c.boost;
      setCommands(c, cap);
      return true;
    }
    if (s == "water" || s == "1") {
      c.action1 = true;
      setCommands(c, cap);
      return true;
    }
    if (s == "sand" || s == "2") {
      c.action2 = true;
      setCommands(c, cap);
      return true;
    }
    if (s == "solid" || s == "3") {
      c.action3 = true;
      setCommands(c, cap);
      return true;
    }
    if (s == "wind" || s == "t") {
      c.toggleWind = true;
      setCommands(c, cap);
      return true;
    }
    if (s == "gravity" || s == "g") {
      c.toggleGravity = true;
      setCommands(c, cap);
      return true;
    }
    if (s == "light" || s == "l") {
      c.toggleLight = true;
      setCommands(c, cap);
      return true;
    }
    if (s == "clear" || s == "x") {
      c.clearDynamics = true;
      setCommands(c, cap);
      return true;
    }
    if (s.rfind("res ", 0) == 0) {
      int w = 320, h = 240;
      if (std::sscanf(s.c_str() + 4, "%d %d", &w, &h) == 2) {
        setViewResolution(w, h, cap);
        return true;
      }
    }
    if (s.rfind("step", 0) == 0) {
      int n = 1;
      double dt = 1.0 / 60.0;
      std::sscanf(s.c_str() + 4, "%d", &n);
      stepN(std::max(1, n), dt);
      // clear one-shot edges after batch
      capsules_.at(cap).commands.clearEdges();
      
      
      capsules_.at(cap).commands.lookYaw = 0;
      capsules_.at(cap).commands.lookPitch = 0;
      return true;
    }
    if (s.rfind("tp ", 0) == 0) {
      double x = 0, y = 0, z = 1.75;
      if (std::sscanf(s.c_str() + 3, "%lf %lf %lf", &x, &y, &z) >= 2) {
        auto &capRef = capsules_.at(cap);
        capRef.spawnAt(engine::Vec3(x, y, z));
        // publish without full physics step
        capRef.publishObservation(scene_, 60.0);
        logCap(int(cap), "cmd", "tp", s.substr(3));
        return true;
      }
    }
    if (s == "perf on") {
      nqg::perf::Profiler::instance().enabled(true);
      nqg::perf::Profiler::instance().reset();
      log("info", "perf", "on");
      return true;
    }
    if (s == "perf off") {
      nqg::perf::Profiler::instance().enabled(false);
      log("info", "perf", "off");
      return true;
    }
    if (s == "perf") {
      log("info", "perf_report", nqg::perf::Profiler::instance().report());
      return true;
    }
    if (s == "freefall") {
      auto &capRef = capsules_.at(cap);
      capRef.freefall = !capRef.freefall;
      logCap(int(cap), "cmd", "freefall", capRef.freefall ? "on" : "off");
      return true;
    }
    if (s == "quit" || s == "exit") {
      quit_ = true;
      return true;
    }
    log("info", "unknown_cmd", s);
    return false;
  }

  bool shouldQuit() const { return quit_; }

private:
  void log(const char *kind, const char *key, const std::string &value) {
    logCap(0, kind, key, value);
  }
  void logCap(int id, const char *kind, const char *key,
              const std::string &value) {
    LogEntry e;
    e.t = simTime_;
    e.frame = frame_;
    e.capsuleId = id;
    e.kind = kind;
    e.key = key;
    e.value = value;
    logLines_.push_back(formatLogLine(e));
    while (logLines_.size() > kMaxLog)
      logLines_.pop_front();
  }

  CleanRoomScene scene_;
  std::vector<Capsule> capsules_;
  std::deque<std::string> logLines_;
  double simTime_ = 0;
  int frame_ = 0;
  double lastFps_ = 60.0;
  bool parallel_render_ = true;
  bool quit_ = false;
};

} // namespace host
} // namespace nqg
