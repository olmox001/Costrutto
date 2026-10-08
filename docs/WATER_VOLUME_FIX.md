# Water volume loss — analysis and fix

## Symptom
Starting volume 6.000 m³ → 3.618 m³ after ~1 s, `absorbedVolume ≈ 2.38 m³`,
`lost = 0`, spray only ~0.0003 m³. The water should spread, not vanish.

## Measurement (tools/diag_water_budget.cpp, per-channel counters `absorbedBy`)
Of the 2.3814 m³ "absorbed", **100 % went through the `noNeighbour` channel**
(`ShallowFlow::redistributeLost` fallback). Infiltration, retention, tile
compaction and bed-rise were ~0 (< 0.001 m³). One event at t = 0.25 s removed
5996 columns (≈ 60 m², 2.31 m³).

## Root cause (two defects that compound)
1. **Initialisation order** (`CleanRoomScene` constructor): the initial 6 m³
   were poured (`initialFillVolume`) *before* `terrain_cache.configure/rebuild`.
   Cells therefore stored a flat bed (`c.b = 0`). Once the terrain existed,
   `sampleBed` returned 0.01–0.31 m under the 5 cm of water. At the first
   periodic `refreshBed()` (every 0.25 s) the bed "rose" above the water and
   every column was treated as displaced by a body.
2. **Non-conservative fallback**: displaced volume was redistributed only to
   *wet* cells that were not `body`. The room floor is flagged `body`, so no
   candidate existed within 64 rings and the volume was added to
   `absorbedVolume` — i.e. deleted.

(In the original `Costrutto-main` the terrain was flat, so (1) could not
manifest; it was introduced with the planetary terrain cache.)

## Fix
* Water is now created at the end of the constructor, after terrain, house pad
  and all solids exist → stored bed == current bed (test-enforced).
* `redistributeLost`: pass 1 unchanged (wet, free cells); pass 2 accepts any
  non-solid cell (dry or on a bed top); if still nothing, the volume stays in a
  bounded pending queue (`pendingQ`, 4096) that is counted in `totalVolume()`
  and retried on every refresh. Only queue overflow is a loss, and it is
  counted in `absorbedBy.noNeighbour`.
* `absorbedBy` {infiltration, retention, compaction, bedRise, noNeighbour}
  always sums to `absorbedVolume`.

## Result (same scenario)
6.0000 → 5.9994 m³ after 1 s (only genuine thin-film retention/infiltration),
wet cells 8231 → 10385 (front expands). Tests: `test_water_solver` W14–W17,
`test_water_budget` (8 checks, scene level).

## Known, not changed here
Inside the house footprint the water bed follows raw terrain relief
(up to +0.31 m above the 0.076 m pad) because `combine_ground` only ever
raises ground (`max`). The floor is therefore not flat; this is a world-design
decision (cut vs fill) and is left for a separate change.
