# SPDX-License-Identifier: GPL-2.0-or-later
# make → build all possible + run all tests (automatic)
include mk/platform.mk
include mk/test.mk

.DEFAULT_GOAL := all

# Fast/default core — includes the complete CleanRoom regression suite.
CORE_TARGETS = test_nqg_core test_engine3d test_cleanroom test_matter_physics \
               test_continuum_physics test_water_solver test_sdf test_water_spray \
               nqg_cli_client test_physics_modules test_physics_all test_world_cli \
               test_world_modules test_world_sh test_capsule_dynamics test_multi_observer \
               test_water_budget
STRESS_TARGETS =

SDL_TARGETS  = test_window_sdl3 nqg_sample_game nqg_cleanroom_game

ENGINE_HDRS = nqg_physics_core.hpp nqg_engine3d.hpp nqg_sdf.hpp \
              nqg_earth_environment.hpp nqg_apartment.hpp nqg_drag_physics.hpp \
              nqg_air_physics.hpp nqg_matter_physics.hpp nqg_continuum_physics.hpp \
              nqg_water_solver.hpp nqg_water_spray.hpp nqg_cleanroom_engine.hpp \
              nqg_materials.hpp

GAME_HDRS = nqg_commands.hpp nqg_observer.hpp nqg_platform.hpp nqg_host.hpp nqg_engine_api.hpp

SDL_HDRS  = nqg_sdl_platform.hpp nqg_window_sdl3.hpp

ifeq ($(SDL_FOUND),1)
  TARGETS  := $(CORE_TARGETS) $(SDL_TARGETS)
  BINARIES := $(addprefix $(BUILD_DIR)/,$(addsuffix $(EXE_EXT),$(TARGETS)))
  CXXFLAGS_SDL := $(CXXFLAGS) -DNQG_WITH_SDL
else
  TARGETS  := $(CORE_TARGETS)
  BINARIES := $(addprefix $(BUILD_DIR)/,$(addsuffix $(EXE_EXT),$(TARGETS)))
  CXXFLAGS_SDL := $(CXXFLAGS)
  $(info → SDL3 not found: core + CLI client only)
endif

CORE_BINS = $(addprefix $(BUILD_DIR)/,$(addsuffix $(EXE_EXT),$(CORE_TARGETS)))
SDL_BINS  = $(addprefix $(BUILD_DIR)/,$(addsuffix $(EXE_EXT),$(SDL_TARGETS)))

.PHONY: all core apps package app ios dist verify test run run_cleanroom clean info help dirs prepare-test

all: dirs $(BINARIES) package prepare-test verify run-tests report
	@echo ""
	@echo "================================================================"
	@echo "  ALL DONE — builds + tests complete"
	@echo "================================================================"

package:
ifeq ($(SDL_FOUND),1)
	@$(MAKE) --no-print-directory apps || true
	@$(MAKE) --no-print-directory app || true
	@$(MAKE) --no-print-directory ios || true
	@$(MAKE) --no-print-directory dist || true
else
	@mkdir -p $(DIST_DIR)/bin
	@cp -f $(BUILD_DIR)/nqg_cli_client$(EXE_EXT) $(BUILD_DIR)/test_physics_all$(EXE_EXT) $(BUILD_DIR)/test_physics_modules$(EXE_EXT) $(DIST_DIR)/bin/ 2>/dev/null || true
	@echo "→ dist/bin (CLI host client)"
endif

core: dirs $(CORE_BINS)
	@echo "✓ core + CLI ready"

apps: dirs $(SDL_BINS)
	@echo "✓ SDL apps ready"

# ---- core / host (no SDL) ----
$(BUILD_DIR)/test_nqg_core$(EXE_EXT): test_nqg_core.cpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_engine3d$(EXE_EXT): test_engine3d.cpp nqg_engine3d.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_cleanroom$(EXE_EXT): test_cleanroom.cpp $(ENGINE_HDRS) $(GAME_HDRS) | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_matter_physics$(EXE_EXT): test_matter_physics.cpp \
        nqg_matter_physics.hpp nqg_sdf.hpp nqg_air_physics.hpp nqg_drag_physics.hpp \
        nqg_engine3d.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_continuum_physics$(EXE_EXT): test_continuum_physics.cpp \
        nqg_continuum_physics.hpp nqg_water_solver.hpp nqg_sdf.hpp nqg_air_physics.hpp \
        nqg_drag_physics.hpp nqg_engine3d.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_water_solver$(EXE_EXT): test_water_solver.cpp nqg_water_solver.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_sdf$(EXE_EXT): test_sdf.cpp $(ENGINE_HDRS) | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_water_spray$(EXE_EXT): test_water_spray.cpp $(ENGINE_HDRS) | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/nqg_cli_client$(EXE_EXT): nqg_cli_client.cpp nqg_host.hpp nqg_engine_api.hpp $(ENGINE_HDRS) $(GAME_HDRS) | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_physics_modules$(EXE_EXT): test_physics_modules.cpp physics/physics.hpp physics/nasa_rules.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_physics_all$(EXE_EXT): test_physics_all.cpp physics/physics.hpp world/world.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_world_cli$(EXE_EXT): test_world_cli.cpp nqg_host.hpp nqg_engine_api.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_world_modules$(EXE_EXT): test_world_modules.cpp world/world.hpp world/terrain_cache.hpp world/spherical_harmonics.hpp physics/physics.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_world_sh$(EXE_EXT): test_world_sh.cpp world/world.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_capsule_dynamics$(EXE_EXT): test_capsule_dynamics.cpp nqg_host.hpp nqg_engine_api.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_multi_observer$(EXE_EXT): test_multi_observer.cpp nqg_host.hpp nqg_engine_api.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

$(BUILD_DIR)/test_water_budget$(EXE_EXT): test_water_budget.cpp nqg_host.hpp nqg_engine_api.hpp $(ENGINE_HDRS) | dirs
	$(CXX) $(CXXFLAGS) -I. $< -o $@ && chmod +x $@

.PHONY: stress
stress: dirs $(addprefix $(BUILD_DIR)/,$(addsuffix $(EXE_EXT),$(STRESS_TARGETS)))
	@echo "✓ stress binaries ready (run manually; long)"

# ---- SDL optional ----
$(BUILD_DIR)/test_window_sdl3$(EXE_EXT): test_window_sdl3.cpp $(SDL_HDRS) nqg_engine3d.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $(LDFLAGS) $< -o $@ && chmod +x $@

$(BUILD_DIR)/nqg_sample_game$(EXE_EXT): nqg_sample_game.cpp nqg_window_sdl3.hpp nqg_engine3d.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) -I. $(LDFLAGS) $< -o $@ && chmod +x $@

$(BUILD_DIR)/nqg_cleanroom_game$(EXE_EXT): nqg_cleanroom_game.cpp nqg_host.hpp nqg_engine_api.hpp $(ENGINE_HDRS) $(GAME_HDRS) $(SDL_HDRS) | dirs
	$(CXX) $(CXXFLAGS_SDL) -I. $(LDFLAGS) $< -o $@ && chmod +x $@

include mk/apps.mk

# Optional shared library (does not replace existing static header-only host flow)
.PHONY: libnqg
libnqg: dirs
	@echo "→ libnqg_engine shared (ABI + optional; apps still header-link engine)"
	$(CXX) $(CXXFLAGS) -shared -fPIC -I. -o $(BUILD_DIR)/libnqg_engine$(if $(filter macos,$(DETECTED_OS)),.dylib,$(if $(filter windows,$(DETECTED_OS)),.dll,.so)) nqg_engine_abi.cpp
	@echo "✓ $(BUILD_DIR)/libnqg_engine.*  symbols: nqg_abi_version, nqg_abi_features"
