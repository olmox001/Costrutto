# SPDX-License-Identifier: GPL-2.0-or-later
.PHONY: run-tests test run run_cleanroom prepare-test

prepare-test: | dirs
	@mkdir -p $(TEST_DIR)
	@n=0; for f in $(BINARIES); do \
		if [ -f "$$f" ]; then cp "$$f" $(TEST_DIR)/; chmod +x "$(TEST_DIR)/$$(basename $$f)" 2>/dev/null || true; n=$$((n+1)); fi; \
	done; echo "✓ $(TEST_DIR) ($$n binaries)"

run-tests: prepare-test
	@echo ""; echo "→ Running unit tests..."; echo ""
	@failed=0; passed=0; skipped=0; \
	for name in test_physics_modules test_physics_all test_world_cli test_world_modules test_world_sh test_nqg_core test_engine3d test_cleanroom \
	            test_capsule_dynamics test_multi_observer test_water_budget \
	            test_matter_physics test_continuum_physics test_water_solver test_sdf \
	            test_water_spray test_window_sdl3; do \
	  bin="$(BUILD_DIR)/$${name}$(EXE_EXT)"; \
	  if [ ! -f "$$bin" ]; then echo "  SKIP  $$name"; skipped=$$((skipped+1)); continue; fi; \
	  chmod +x "$$bin" 2>/dev/null || true; \
	  echo "  RUN   $$name"; \
	  if "$$bin" >/tmp/nqg_test_out.txt 2>&1; then \
	    tail -3 /tmp/nqg_test_out.txt 2>/dev/null; passed=$$((passed+1)); \
	  elif cp "$$bin" /tmp/nqg_test_run$(EXE_EXT) && chmod +x /tmp/nqg_test_run$(EXE_EXT) && /tmp/nqg_test_run$(EXE_EXT) >/tmp/nqg_test_out.txt 2>&1; then \
	    tail -3 /tmp/nqg_test_out.txt 2>/dev/null; passed=$$((passed+1)); \
	  else echo "  FAIL  $$name"; tail -15 /tmp/nqg_test_out.txt 2>/dev/null; failed=$$((failed+1)); fi; \
	  echo ""; \
	done; \
	cli="$(BUILD_DIR)/nqg_cli_client$(EXE_EXT)"; \
	if [ -f "$$cli" ]; then \
	  echo "  RUN   nqg_cli_client --test"; \
	  chmod +x "$$cli" 2>/dev/null || true; \
	  if "$$cli" --test >/tmp/nqg_cli_out.txt 2>&1; then \
	    tail -5 /tmp/nqg_cli_out.txt; passed=$$((passed+1)); \
	  elif cp "$$cli" /tmp/nqg_cli_run$(EXE_EXT) && chmod +x /tmp/nqg_cli_run$(EXE_EXT) && /tmp/nqg_cli_run$(EXE_EXT) --test >/tmp/nqg_cli_out.txt 2>&1; then \
	    tail -5 /tmp/nqg_cli_out.txt; passed=$$((passed+1)); \
	  else echo "  FAIL  nqg_cli_client"; tail -15 /tmp/nqg_cli_out.txt; failed=$$((failed+1)); fi; \
	  echo ""; \
	else echo "  SKIP  nqg_cli_client"; skipped=$$((skipped+1)); fi; \
	echo "================================================================"; \
	echo "  TESTS: $$passed passed, $$failed failed, $$skipped skipped"; \
	echo "================================================================"; \
	if [ "$$failed" -eq 0 ]; then echo "✓ All available unit tests passed"; else exit 1; fi

test: run-tests

run: prepare-test
	@bin="$(BUILD_DIR)/nqg_sample_game$(EXE_EXT)"; if [ -f "$$bin" ]; then "$$bin"; else echo "need SDL3"; exit 1; fi

run_cleanroom: prepare-test
	@bin="$(BUILD_DIR)/nqg_cleanroom_game$(EXE_EXT)"; if [ -f "$$bin" ]; then "$$bin"; else echo "use nqg_cli_client"; exit 1; fi
