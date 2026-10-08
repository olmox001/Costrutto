<!-- SPDX-License-Identifier: GPL-2.0-or-later -->

## COSTRUTTO — verified release

Engine **host/kernel** (no SDL). Capsule ports only. Clients: **CLI/LLM** + optional **SDL**.

### `make` (automatic)

Builds all available targets and runs **all** tests including:

- 8 engine unit suites (234 asserts)
- `nqg_cli_client --test` (20 Host/CLI checks: move, look, jump, water/sand/solid, multi-capsule, log, PPM)

### CLI / LLM

```bash
./build/*/nqg_cli_client --test
./build/*/nqg_cli_client -e "move 0 1" -e "step 30" -e shot -e obs -e log 10
./build/*/nqg_cli_client   # interactive
```

Multi-capsule: `addcap player2 120 90` then `cap 1` …

Log schema: `t|frame|cap|kind|key|value`

### Screenshots (CLI verification)

See `screenshots/`:

- `nqg_suite_main.png` — apartment view after interaction
- `nqg_suite_cap0.png` / `nqg_suite_cap1.png` — dual capsule multiplayer frames

### Systems maintained

| Path | Status |
|------|--------|
| Host + Capsule + CLI (no SDL) | primary, tested |
| SDL client (`NQG_WITH_SDL`) | optional when SDL3 present |
| `make app` / `make ios` | macOS packages |
| Universal macOS | `BUILD_TYPE=macos-universal` |

### Public API

`#include "nqg_engine_api.hpp"` — facade + compile-time contracts.
