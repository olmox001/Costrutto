# Function diff analysis (new vs original) — Round 1 skeleton

## Method
For each area: **original role** → **current module** → **behavioral delta** → **risk**.

## Capsule / input / HUD

| Original | Current | Delta | Risk |
|----------|---------|-------|------|
| Keyboard WASD in game loop | `Commands` on Capsule + CLI/SDL | Same axes semantics; sticky move until stop | Low |
| Italian HUD "In stanza" | `nqg_hud.hpp` English keys | Text only; `insideRoom` field kept | Low |
| Mouse look | `look yaw pitch` | Delta radians per command | Low |

## Ground / house

| Original | Current | Delta | Risk |
|----------|---------|-------|------|
| Flat z=0 ground | `groundHeightAt` → geodesy combine | Sand+pad+terrain | Low |
| Script static walls | RigidSolid + mass ρV + MaterialMix | Physics contacts | Medium if isStatic removed |
| sampleBed max tops | Up-face below mid-height | Ceiling no longer bed | **Fixed regression** |

## Water

| Original | Current | Delta | Risk |
|----------|---------|-------|------|
| Sessile on horizontal | + surfaceN + wall sessile drain | Multi-face films partial | Medium |
| Shallow 2D grid | Unchanged core solver | Still planar bed | Known limit |

## Render

| Original | Current | Delta | Risk |
|----------|---------|-------|------|
| Always render | `renderEnabled` flag | Physics tests skip RT | Low |
| Thread pool rays | Unchanged in engine3d | Physics still single-core | Plan Round 3 |

## Open formal renames (Round 2)

- `insideRoom` → prefer `inside_structure` in API (field alias)
- `RoomGeometry` → `StructureShell` (name only, gradual)
- `appartamento` default capsule name → configurable


## Host / CLI (created vs original main loop)

| Original (`nqg_cleanroom_game` loop) | Current | Kept behavior | Notes |
|--------------------------------------|---------|---------------|-------|
| `main` + SDL window pump | `Host::step` + optional SDL/CLI | dt physics, render optional | Headless without SDL |
| Poll keyboard | `Commands` ports | move/look/jump/actions | Transport-agnostic |
| Draw HUD text on SDL | `Observation.lines` via `nqg_hud` | Same metrics | English keys |
| Single player | `Host` multi-capsule | N capsules | `addcap` / `cap N` |

## Physics modules (new, no original equivalent file)

| Module | Role | Original source of math | Tests |
|--------|------|-------------------------|-------|
| `physics_atmosphere` | density, terminal, authority | earth atmosphere + drag ideas | physics_all |
| `physics_geodesy` | combine_ground, eye, impact | groundHeightAt / resolvePlayer | physics_all |
| `physics_fluid_surface` | face film drainage | new formalization of wall runoff | physics_all |
| `spherical_harmonics` | h(θ,φ) band-limited | TerrainGenerator harmonics + paper L | world_sh |
| `volume_ledger` | mass conservation | new | world_sh |
| `material_mix` | % → ρ,V,albedo | materials table | physics_all |

## Intentionally not renamed yet (avoid mass breakage)

- `RoomGeometry` — still used widely; treat as structure shell in docs
- `insideRoom` field — alias `inside_structure()` added
- `CleanRoomScene` — name historical; is the world+house scene

## Ambiguous names formalized this round

| Old usage | Formal |
|-----------|--------|
| "In stanza" HUD | `inside_structure: yes/no` |
| `appartamento` default | `capsule0` |
| "room" in debug prose | structure / house footprint |

## Risk register

| Item | Risk | Mitigation |
|------|------|------------|
| Removing isStatic on house | High (solver explode) | Keep kinematic until friction verified |
| Split substep 266 LOC | High | Entry asserts only until isolated tests |
| Multicore physics | Medium (races on solids) | After profiler baseline |
