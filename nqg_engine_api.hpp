/* SPDX-License-Identifier: GPL-2.0-or-later
 * nqg_engine_api.hpp — Common public engine surface (NASA-style module boundary).
 *
 * Every client / test must go through this facade or Host/Capsule ports.
 * Compile-time checks below enforce presence of core types.
 */
#pragma once

#include "nqg_physics_core.hpp"
#include "nqg_engine3d.hpp"
#include "nqg_sdf.hpp"
#include "nqg_commands.hpp"
#include "nqg_observer.hpp"
#include "nqg_host.hpp"
#include "nqg_cleanroom_engine.hpp"

#include <cstddef>
#include <type_traits>

namespace nqg {
namespace api {

// Re-exports (single include for clients)
using engine::Image;
using engine::Vec3;
using engine::Rgb;
using cmd::Commands;
using observer::Capsule;
using observer::Observation;
using observer::AudioSample;
using host::Host;
using host::LogEntry;
using cleanroom::CleanRoomScene;

// Compile-time contract checks (two assertions per critical type)
static_assert(sizeof(Vec3) >= 3 * sizeof(double) || sizeof(Vec3) >= 3 * sizeof(float),
              "Vec3 must hold 3 scalars");
static_assert(std::is_default_constructible<Vec3>::value, "Vec3 default constructible");

static_assert(std::is_default_constructible<Image>::value, "Image default constructible");
static_assert(std::is_default_constructible<Commands>::value, "Commands default constructible");

static_assert(std::is_default_constructible<Capsule>::value, "Capsule default constructible");
static_assert(std::is_default_constructible<Host>::value, "Host default constructible");

static_assert(std::is_default_constructible<CleanRoomScene>::value,
              "CleanRoomScene default constructible");
static_assert(std::is_default_constructible<Observation>::value,
              "Observation default constructible");

// Host must expose capsule ports
inline void compile_time_host_surface_check() {
  Host h;
  Capsule &c = h.capsule();
  (void)c.commands;
  (void)c.framebuffer;
  (void)c.audioOut;
  (void)c.observation;
  h.step(1.0 / 60.0);
  (void)h.logDump(1);
}

} // namespace api
} // namespace nqg
