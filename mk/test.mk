# ============================================================================
# NQG Test targets
# Independent from platform and project sources.
# ============================================================================

.PHONY: run-tests test run run_cleanroom

run-tests: prepare-test
	@echo "→ Running unit tests..."
	$(TEST_DIR)/test_nqg_core$(EXE_EXT)
	$(TEST_DIR)/test_engine3d$(EXE_EXT)
	$(TEST_DIR)/test_window_sdl3$(EXE_EXT)
	$(TEST_DIR)/test_cleanroom$(EXE_EXT)
	$(TEST_DIR)/test_matter_physics$(EXE_EXT)
	$(TEST_DIR)/test_continuum_physics$(EXE_EXT)
	$(TEST_DIR)/test_sdf$(EXE_EXT)
	$(TEST_DIR)/test_water_spray$(EXE_EXT)
	@echo "✓ All unit tests finished"

test: run-tests

run: prepare-test
	$(TEST_DIR)/nqg_sample_game$(EXE_EXT)

run_cleanroom: prepare-test
	$(TEST_DIR)/nqg_cleanroom_game$(EXE_EXT)