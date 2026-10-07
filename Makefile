# ============================================================================
# NQG Project Makefile
# Project sources, targets and compilation rules only.
# Platform logic  → mk/platform.mk
# Test targets    → mk/test.mk
# ============================================================================

include mk/platform.mk
include mk/test.mk

.DEFAULT_GOAL := all

# ---------------------------------------------------------------------------
# Project targets
# ---------------------------------------------------------------------------
TARGETS = test_nqg_core test_engine3d test_window_sdl3 test_cleanroom \
          test_matter_physics test_continuum_physics test_sdf test_water_spray \
          nqg_sample_game nqg_cleanroom_game

COMMON_HDRS = nqg_physics_core.hpp nqg_engine3d.hpp nqg_sdf.hpp \
              nqg_earth_environment.hpp nqg_apartment.hpp nqg_air_physics.hpp \
              nqg_matter_physics.hpp nqg_continuum_physics.hpp \
              nqg_water_spray.hpp nqg_cleanroom_engine.hpp

BINARIES = $(addprefix $(BUILD_DIR)/,$(addsuffix $(EXE_EXT),$(TARGETS)))

# ---------------------------------------------------------------------------
# Default goal – everything runs automatically
# ---------------------------------------------------------------------------
.PHONY: all
all: dirs $(BINARIES) app prepare-test verify run-tests report

# ---------------------------------------------------------------------------
# Compilation rules
# ---------------------------------------------------------------------------
$(BUILD_DIR)/test_nqg_core$(EXE_EXT): test_nqg_core.cpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/test_engine3d$(EXE_EXT): test_engine3d.cpp nqg_engine3d.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/test_window_sdl3$(EXE_EXT): test_window_sdl3.cpp nqg_window_sdl3.hpp \
        nqg_engine3d.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $< -o $@

$(BUILD_DIR)/test_cleanroom$(EXE_EXT): test_cleanroom.cpp $(COMMON_HDRS) | dirs
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/test_matter_physics$(EXE_EXT): test_matter_physics.cpp \
        nqg_matter_physics.hpp nqg_sdf.hpp nqg_air_physics.hpp \
        nqg_engine3d.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/test_continuum_physics$(EXE_EXT): test_continuum_physics.cpp \
        nqg_continuum_physics.hpp nqg_sdf.hpp nqg_air_physics.hpp \
        nqg_engine3d.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/test_sdf$(EXE_EXT): test_sdf.cpp $(COMMON_HDRS) | dirs
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/test_water_spray$(EXE_EXT): test_water_spray.cpp $(COMMON_HDRS) | dirs
	$(CXX) $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/nqg_sample_game$(EXE_EXT): nqg_sample_game.cpp nqg_window_sdl3.hpp \
        nqg_engine3d.hpp nqg_physics_core.hpp | dirs
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $< -o $@

$(BUILD_DIR)/nqg_cleanroom_game$(EXE_EXT): nqg_cleanroom_game.cpp $(COMMON_HDRS) \
        nqg_window_sdl3.hpp | dirs
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $< -o $@