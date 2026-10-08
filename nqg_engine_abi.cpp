/* SPDX-License-Identifier: GPL-2.0-or-later
 * Minimal C ABI for optional shared library builds (dylib/so).
 * Full engine remains header-driven; this exports version/features only.
 */
#include <cstring>

extern "C" {

int nqg_abi_version(void) { return 3; }

const char *nqg_abi_features(void) {
  return "host,capsule,cli,geodesy,hud,perf,parallel_render,sh_terrain";
}

const char *nqg_abi_sdl_target(void) {
  return "SDL3-3.5.0"; /* verified against provided SDL-main headers */
}

} // extern "C"
