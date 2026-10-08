/* SPDX-License-Identifier: GPL-2.0-or-later
 * NASA / JPL Power of 10 — project conventions for physics & world modules.
 *
 *  1. Simple control flow (no goto / longjmp).
 *  2. All loops have a fixed upper bound (NQG_MAX_*).
 *  3. No heap allocation inside hot step functions (callers preallocate).
 *  4. Functions stay short (target ≤ 60 lines; flag LONG in catalog).
 *  5. ≥ 2 assertions (NQG_REQUIRE) per public function where meaningful.
 *  6. Minimal data scope.
 *  7. Check return values / validate inputs at boundaries.
 *  8. Limited preprocessor (include guards + NQG_REQUIRE / NQG_STATIC_ASSERT).
 *  9. Prefer value types / single-level pointers.
 * 10. Build with -Wall -Wextra; unit tests must pass at runtime.
 */
#pragma once

#include <cassert>
#include <cstddef>
#include <cmath>

#ifndef NQG_NDEBUG
#define NQG_REQUIRE(cond) assert(cond)
#else
#define NQG_REQUIRE(cond) ((void)0)
#endif

#define NQG_STATIC_ASSERT(cond, msg) static_assert(cond, msg)

constexpr std::size_t NQG_MAX_GEODESIC_STEPS = 100000;
constexpr std::size_t NQG_MAX_PARTICLES = 65536;
constexpr std::size_t NQG_MAX_SOLIDS = 4096;
constexpr std::size_t NQG_MAX_GRID = 512;
constexpr std::size_t NQG_MAX_SH_DEGREE = 64;
constexpr std::size_t NQG_MAX_FRAGMENTS = 256;
constexpr std::size_t NQG_MAX_MIX_COMPONENTS = 8;
constexpr std::size_t NQG_MAX_FUNCTION_LINES = 60;

inline bool nqg_finite(double x) { return std::isfinite(x); }
