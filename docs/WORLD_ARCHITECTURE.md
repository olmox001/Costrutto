# World architecture (English modules)

## Premise change
The "room" is not a special sandbox. It is a **house**: a set of **concrete solid slabs**
on a **planet surface**. Coordinates bind to the world/planet frame.

```
Solar system (future: sun + real illumination scale)
    └── Planet body (COM, ellipsoid, gravity, atmosphere)
            └── Local ENU frame (latitude/longitude origin)
                    ├── TerrainCache (seeded height grid)
                    ├── House (concrete solids, real mass/volume)
                    ├── Other solids / fluids / capsule
                    └── Physics depends on planet environment
```

## Modules
| Path | Role |
|------|------|
| `world/planet_frame.hpp` | Ellipsoid, polar radius, COM, ENU↔ECEF |
| `world/terrain_cache.hpp` | Cached grid, seed, raw heights, bilinear sample |
| `world/house.hpp` | HouseBlueprint → SolidDesc with concrete density |
| `physics/physics_atmosphere.hpp` | Pressure/density, terminal speed, control authority |
| `physics/physics_fluid_surface.hpp` | Multi-face films, drainage direction, retention |

## Water
House slabs participate in the fluid **bed** (`contributesToBed` includes room structure).
Films can attach to oriented faces; excess volume runs off by tilt.

## Stability
House mass = ρ_concrete × volume. No scripted "lock to ground" — static flag is temporary
kinematic constraint while contact solver gains full static friction stacking.

## Spherical harmonics (paper)

\[
h(\theta,\phi)=\sum_{\ell=0}^{L_{\mathrm{paper}}}\sum_{m=-\ell}^{\ell} c_{\ell m} Y_{\ell m}(\theta,\phi)
\]

- `L_PAPER_MAX = 16384` (design / coefficient space)
- Runtime band-limited evaluation `L_RUNTIME_MAX` (default synthesize ≤32)
- Higher degrees live in **local interaction grids** (SDF caches), not dense global storage

## Volume conservation

`VolumeLedger`: solids ↔ air dust ↔ surface film. Fragmentation power-law; fines → dust fraction.
House program: mean pad height + `flatten_pad` with spoil accounted in ledger.
