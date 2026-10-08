# Costrutto / NQG — status

## Completato

- Engine kernel + Capsule ports (commands, framebuffer, audio, observation)
- Host multi-capsula + CLI LLM (`tp`, `freefall`, `probe`, sticky move)
- Separazione SDL (IPlatform / SDL opzionale)
- Makefile unificato (Linux / macOS universal / iOS target / Windows flags)
- NASA modules: constants, atmosphere, drag, SH, geodesy, fluid_surface, math
- Terrain SH + cache 512, house program + volume ledger, MaterialMix su muri
- sampleBed senza soffitto, massa calcestruzzo reale, geodesy feet
- Acqua: sessile multi-faccia (`surfaceN` + drainage)
- Test: physics_all, physics_modules, world_sh, water_solver, CLI probe
- Screenshot debug_shots/ (ground, far, high altitude)

## Volutamente parziale / fuori scope runtime qui

| Voce | Motivo |
|------|--------|
| Split `substep` (266 LOC) / `computeForces` (257) | Refactor rischioso; assert in ingresso aggiunti |
| Spray LONG helpers | Assert + multi-face; split meccanico non fatto |
| Sole / sistema solare scala reale | Solo `PlanetBody` + LocalFrame |
| Build IPA su device | Richiede Xcode + provisioning |
| Cross-compile Windows binario | Flag pronti; host non Windows |
| SDL interactive test | SDL assente su questo runner |

## Comandi utili

```
make              # core + test
make core
./nqg_cli_client -e probe -e "tp 0 0 80" -e freefall -e "step 30" -e obs
```
