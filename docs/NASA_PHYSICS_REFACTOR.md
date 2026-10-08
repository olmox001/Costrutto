# NASA Power of 10 — Physics modularization

## Catalog summary (legacy headers)

| Bucket | Count |
|--------|------:|
| <20 lines | 283 |
| 20–40 | 28 |
| 40–60 | 14 |
| 60–100 | 6 |
| >100 | 2 |

**Must split (rule 4):** `water_solver::substep` (266), `matter::computeForces` (257).

Full table: `docs/PHYSICS_FUNCTION_CATALOG.md`.

## New modular English API (`physics/`)

| Module | Responsibility |
|--------|----------------|
| `nasa_rules.hpp` | NQG_REQUIRE, fixed bounds |
| `physics_constants.hpp` | G, c, ħ, Planck scales |
| `physics_schwarzschild.hpp` | lapse, redshift, circular L |
| `physics_info.hpp` | BH/Bekenstein entropy, Kretschmann |
| `physics_drag.hpp` | quadratic drag force |
| `physics.hpp` | module index |

Each public function has **≥2 runtime checks** in `test_physics_modules.cpp` (32 PASS).

## Compatibility

- `nqg_physics_core.hpp` — still the full legacy core
- `nqg_drag_physics.hpp` — thin alias to `physics::drag`

## Next splits (ordered by length)

1. `nqg_water_solver.hpp` — extract `substep` into bounded stages
2. `nqg_matter_physics.hpp` — extract `computeForces`
3. `nqg_water_spray.hpp` — advect / raycast / limitParticles

## CLI exploration (verified)

Apartment world via Host CLI; screenshots in `screenshots/explore*.png`.
