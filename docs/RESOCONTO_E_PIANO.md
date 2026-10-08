# Resoconto completo — Round 5 (release-ready core)

**Data:** 2026-10-08  
**Ambito:** stabilizzazione build + dinamica Capsule + test multi-observer  
**Input esterno:** revisione agente (14 punti) — integrata e verificata sotto.

---

## 1. Verifica punti agente revisione

| # | Priorità agente | Verificato? | Azione Round 5 |
|---|-----------------|-------------|----------------|
| 1 | Makefile rotto (`test_physics_all` multi-target) | **Sì, reale** | Regole separate; package `cp` corretto; `test_cleanroom` → stress |
| 2 | Dinamica verticale / feet sotto terreno | **Sì, bug reale** | Feet-snap non tira più verso il basso in aria → jump OK (280 PASS) |
| 3 | Profile `capsule.tick` | Parziale | Scope `integrate` + `couplePlayer` + `renderFrame` |
| 4 | Timestep robustezza | Parziale | Ciclo jump a dt=1/60 e 1/30 nello stesso test |
| 5 | Simulazione ≠ client | OK già | Headless/CLI senza SDL; render separato |
| 6 | Multi-observer | **Test nuovo** | `test_multi_observer` 14 PASS |
| 7 | Pipeline SDL3 | Header 3.5.0 OK | Runtime SDL non installato su runner |
| 8 | test_cleanroom lento | **Sì** | Fuori da `CORE_TARGETS`; target `stress` |
| 9–14 | Collisioni aggressive, ECEF, renderer, contratto, CLI, docs | Documentati | Non tutti implementati in R5 |

**Priorità pratica agente rispettata:** ① Makefile → ② collisioni/jump → ③ profiling scopes → ⑤ multi-observer.

---

## 2. Build chain (macchina pulita, senza SDL)

```text
make dirs
make core          # CORE_TARGETS (no SDL, no test_cleanroom)
make test          # run-tests su binari in BUILD_DIR
make libnqg        # libnqg_engine.so ABI
```

**Fix Makefile:**
- Prima: `$(BUILD_DIR)/nqg_cli_client test_physics_modules test_physics_all` (target malformati)
- Ora: una regola per target, path completi in `package`

---

## 3. Test Round 5 (eseguiti)

| Suite | Esito |
|-------|--------|
| test_physics_all | 27 PASS |
| test_physics_modules | 38 PASS |
| test_world_sh | 20 PASS |
| test_water_solver | 13 PASS |
| test_capsule_dynamics | **280 PASS** (dt 60/30, no tunnel, no deep pen, jump height) |
| test_multi_observer | **14 PASS** |
| test_world_cli | **8 PASS** |

---

## 4. Bug chiusi R5

1. **Makefile** multi-target / package  
2. **Feet snap** annullava il salto (forzava eye a ground ogni frame) → solo *lift* se penetrazione  

---

## 5. Ancora fuori release piena (onesto)

- Parallel physics solidi  
- Split LONG substep/computeForces  
- Full engine in dylib  
- IPA / Windows / link SDL3.runtime  
- ECEF round-trip suite dedicata  
- Renderer visuale  
- test_cleanroom come integration CI (ora solo stress manuale)  

Il **core headless** (Host + Capsule + world + physics modules + CLI + make core) è **stabile e verificato** per release engineering su Linux senza SDL.

---

## 6. Scopo prodotto

| Layer | Stato |
|-------|--------|
| Engine kernel + Capsule ports | Release-ready headless |
| CLI LLM client | Release-ready |
| SDL client | Code + header verify; needs libSDL3 on host |
| iOS/macOS packaging | Scripts presenti; non eseguiti qui |
