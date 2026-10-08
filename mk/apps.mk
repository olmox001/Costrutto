# SPDX-License-Identifier: GPL-2.0-or-later
# ============================================================================
# Dual-app packaging: macOS .app + iOS .ipa
# Included from Makefile after platform.mk
# ============================================================================

# Override single-app defaults with dual products
APP_CLEANROOM := $(DIST_DIR)/Costrutto.app
APP_SAMPLE    := $(DIST_DIR)/NQG_Sample.app
IPA_CLEANROOM := $(DIST_DIR)/Costrutto.ipa
IPA_SAMPLE    := $(DIST_DIR)/NQG_Sample.ipa

APP_EXEC_CR   := NQG_CleanRoom
APP_EXEC_SM   := NQG_Sample

# ---------------------------------------------------------------------------
# Helper: embed SDL3 dylib into a macOS .app with @rpath (dynamic link)
# Usage: $(call EMBED_SDL3,bundle_path,exec_name)
# ---------------------------------------------------------------------------
define EMBED_SDL3
	@if [ -n "$(SDL_DYLIB)" ] && [ -f "$(SDL_DYLIB)" ]; then \
		FW="$(1)/Contents/Frameworks"; \
		EXEC="$(1)/Contents/MacOS/$(2)"; \
		mkdir -p "$$FW"; \
		REAL=$$(python3 -c "import os; print(os.path.realpath('$(SDL_DYLIB)'))" 2>/dev/null \
			|| readlink -f "$(SDL_DYLIB)" 2>/dev/null || echo "$(SDL_DYLIB)"); \
		BASE=$$(basename "$$REAL"); \
		echo "  → Embedding $$BASE (dynamic @rpath)"; \
		cp "$$REAL" "$$FW/$$BASE"; \
		cp "$$REAL" "$$FW/libSDL3.dylib"; \
		cp "$$REAL" "$$FW/libSDL3.0.dylib"; \
		install_name_tool -id "@rpath/libSDL3.0.dylib" "$$FW/libSDL3.0.dylib" 2>/dev/null || true; \
		install_name_tool -id "@rpath/libSDL3.dylib" "$$FW/libSDL3.dylib" 2>/dev/null || true; \
		install_name_tool -id "@rpath/$$BASE" "$$FW/$$BASE" 2>/dev/null || true; \
		for src in "$(SDL_DYLIB)" "$$REAL" \
			"/usr/local/lib/libSDL3.dylib" "/usr/local/lib/libSDL3.0.dylib" \
			"/opt/homebrew/lib/libSDL3.dylib" "/opt/homebrew/lib/libSDL3.0.dylib" \
			"@rpath/libSDL3.dylib"; do \
			install_name_tool -change "$$src" "@rpath/libSDL3.0.dylib" "$$EXEC" 2>/dev/null || true; \
		done; \
		install_name_tool -add_rpath "@executable_path/../Frameworks" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -delete_rpath "/usr/local/lib" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -delete_rpath "/opt/homebrew/lib" "$$EXEC" 2>/dev/null || true; \
	else \
		echo "  ⚠ SDL_DYLIB not set – binary will use system/dynamic search paths"; \
	fi
endef

define WRITE_PLIST
	@printf '%s\n' \
		'<?xml version="1.0" encoding="UTF-8"?>' \
		'<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">' \
		'<plist version="1.0"><dict>' \
		'	<key>CFBundleDisplayName</key><string>$(3)</string>' \
		'	<key>CFBundleExecutable</key><string>$(2)</string>' \
		'	<key>CFBundleIdentifier</key><string>$(4)</string>' \
		'	<key>CFBundleIconFile</key><string>icon.icns</string>' \
		'	<key>CFBundleName</key><string>$(3)</string>' \
		'	<key>CFBundlePackageType</key><string>APPL</string>' \
		'	<key>CFBundleVersion</key><string>1.0</string>' \
		'	<key>CFBundleShortVersionString</key><string>1.0</string>' \
		'	<key>LSMinimumSystemVersion</key><string>11.0</string>' \
		'</dict></plist>' > $(1)/Contents/Info.plist
endef

# ---------------------------------------------------------------------------
# macOS .app bundles (both products)
# ---------------------------------------------------------------------------
.PHONY: app app-cleanroom app-sample

ifeq ($(DETECTED_OS),macos)

app: app-cleanroom app-sample
	@echo "✓ macOS apps ready in $(DIST_DIR)/"

app-cleanroom: $(BUILD_DIR)/nqg_cleanroom_game$(EXE_EXT) icon.icns | dirs
	@echo "→ $(APP_CLEANROOM) [$(BUILD_TYPE)]"
	@rm -rf $(APP_CLEANROOM)
	@mkdir -p $(APP_CLEANROOM)/Contents/MacOS
	@mkdir -p $(APP_CLEANROOM)/Contents/Resources
	@mkdir -p $(APP_CLEANROOM)/Contents/Frameworks
	cp $(BUILD_DIR)/nqg_cleanroom_game$(EXE_EXT) $(APP_CLEANROOM)/Contents/MacOS/$(APP_EXEC_CR)
	chmod +x $(APP_CLEANROOM)/Contents/MacOS/$(APP_EXEC_CR)
	cp icon.icns $(APP_CLEANROOM)/Contents/Resources/icon.icns
	$(call EMBED_SDL3,$(APP_CLEANROOM),$(APP_EXEC_CR))
	$(call WRITE_PLIST,$(APP_CLEANROOM),$(APP_EXEC_CR),NQG CleanRoom,com.nqg.cleanroom)
	@echo "✓ $(APP_CLEANROOM)"

app-sample: $(BUILD_DIR)/nqg_sample_game$(EXE_EXT) icon.icns | dirs
	@echo "→ $(APP_SAMPLE) [$(BUILD_TYPE)]"
	@rm -rf $(APP_SAMPLE)
	@mkdir -p $(APP_SAMPLE)/Contents/MacOS
	@mkdir -p $(APP_SAMPLE)/Contents/Resources
	@mkdir -p $(APP_SAMPLE)/Contents/Frameworks
	cp $(BUILD_DIR)/nqg_sample_game$(EXE_EXT) $(APP_SAMPLE)/Contents/MacOS/$(APP_EXEC_SM)
	chmod +x $(APP_SAMPLE)/Contents/MacOS/$(APP_EXEC_SM)
	cp icon.icns $(APP_SAMPLE)/Contents/Resources/icon.icns
	$(call EMBED_SDL3,$(APP_SAMPLE),$(APP_EXEC_SM))
	$(call WRITE_PLIST,$(APP_SAMPLE),$(APP_EXEC_SM),NQG Sample,com.nqg.sample)
	@echo "✓ $(APP_SAMPLE)"

else
app app-cleanroom app-sample:
	@echo "Note: .app bundles are macOS-only (OS=$(DETECTED_OS)). Binaries are in $(BUILD_DIR)/"
endif

# ---------------------------------------------------------------------------
# iOS IPA (unsigned Payload) — requires macOS + iOS SDK + SDL3.framework iOS
# ---------------------------------------------------------------------------
.PHONY: ios ios-cleanroom ios-sample

IOS_MIN_VERSION ?= 15.0
IOS_SDK ?= $(shell xcrun --sdk iphoneos --show-sdk-path 2>/dev/null)
IOS_CXX ?= $(shell xcrun --find clang++ 2>/dev/null)
# Path to iOS SDL3.framework (user must provide)
SDL3_IOS_FRAMEWORK ?= $(HOME)/SDL3-iOS/SDL3.framework

IOS_BUILD_DIR := build/ios-arm64

define IOS_CXXFLAGS
-O2 -std=c++17 -Wall -arch arm64 \
-isysroot $(IOS_SDK) -miphoneos-version-min=$(IOS_MIN_VERSION) \
-fobjc-arc -I. -F$(dir $(SDL3_IOS_FRAMEWORK))
endef

define IOS_LDFLAGS
-isysroot $(IOS_SDK) -arch arm64 \
-F$(dir $(SDL3_IOS_FRAMEWORK)) -framework SDL3 \
-framework UIKit -framework Foundation -framework CoreFoundation \
-framework Metal -framework QuartzCore -framework AudioToolbox \
-framework AVFoundation -framework GameController -framework CoreMotion \
-framework CoreHaptics -framework OpenGLES
endef

ifeq ($(DETECTED_OS),macos)

ios: ios-cleanroom ios-sample
	@echo "✓ iOS IPAs in $(DIST_DIR)/ (unsigned — sign with Xcode/codesign before device)"

ios-check:
	@if [ -z "$(IOS_SDK)" ]; then \
		echo "✗ iOS SDK not found (install Xcode Command Line Tools / Xcode)"; exit 1; \
	fi
	@if [ ! -d "$(SDL3_IOS_FRAMEWORK)" ]; then \
		echo "✗ SDL3.framework for iOS not at $(SDL3_IOS_FRAMEWORK)"; \
		echo "  Build SDL3 for iOS or set SDL3_IOS_FRAMEWORK=/path/to/SDL3.framework"; exit 1; \
	fi
	@echo "✓ iOS SDK: $(IOS_SDK)"
	@echo "✓ SDL3 iOS: $(SDL3_IOS_FRAMEWORK)"

$(IOS_BUILD_DIR)/nqg_cleanroom_game: nqg_cleanroom_game.cpp $(ENGINE_HDRS) $(GAME_HDRS) | ios-check
	@mkdir -p $(IOS_BUILD_DIR)
	$(IOS_CXX) $(IOS_CXXFLAGS) $(IOS_LDFLAGS) $< -o $@

$(IOS_BUILD_DIR)/nqg_sample_game: nqg_sample_game.cpp nqg_window_sdl3.hpp nqg_engine3d.hpp nqg_physics_core.hpp | ios-check
	@mkdir -p $(IOS_BUILD_DIR)
	$(IOS_CXX) $(IOS_CXXFLAGS) $(IOS_LDFLAGS) $< -o $@

define PACK_IPA
	@echo "→ Packaging $(2)"
	@rm -rf $(DIST_DIR)/_ipa_$(1)
	@mkdir -p $(DIST_DIR)/_ipa_$(1)/Payload/$(1).app
	@mkdir -p $(DIST_DIR)/_ipa_$(1)/Payload/$(1).app/Frameworks
	cp $(IOS_BUILD_DIR)/$(3) $(DIST_DIR)/_ipa_$(1)/Payload/$(1).app/$(1)
	chmod +x $(DIST_DIR)/_ipa_$(1)/Payload/$(1).app/$(1)
	cp -R $(SDL3_IOS_FRAMEWORK) $(DIST_DIR)/_ipa_$(1)/Payload/$(1).app/Frameworks/
	@printf '%s\n' \
		'<?xml version="1.0" encoding="UTF-8"?>' \
		'<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">' \
		'<plist version="1.0"><dict>' \
		'	<key>CFBundleExecutable</key><string>$(1)</string>' \
		'	<key>CFBundleIdentifier</key><string>$(4)</string>' \
		'	<key>CFBundleName</key><string>$(1)</string>' \
		'	<key>CFBundlePackageType</key><string>APPL</string>' \
		'	<key>CFBundleVersion</key><string>1</string>' \
		'	<key>CFBundleShortVersionString</key><string>1.0</string>' \
		'	<key>MinimumOSVersion</key><string>$(IOS_MIN_VERSION)</string>' \
		'	<key>UILaunchScreen</key><dict/>' \
		'	<key>UIRequiredDeviceCapabilities</key><array><string>arm64</string></array>' \
		'	<key>UISupportedInterfaceOrientations</key>' \
		'	<array><string>UIInterfaceOrientationLandscapeLeft</string>' \
		'	<string>UIInterfaceOrientationLandscapeRight</string></array>' \
		'</dict></plist>' > $(DIST_DIR)/_ipa_$(1)/Payload/$(1).app/Info.plist
	@cd $(DIST_DIR)/_ipa_$(1) && zip -qry ../$(1).ipa Payload
	@rm -rf $(DIST_DIR)/_ipa_$(1)
	@echo "✓ $(2) (unsigned IPA — codesign before install)"
endef

ios-cleanroom: $(IOS_BUILD_DIR)/nqg_cleanroom_game | dirs
	$(call PACK_IPA,NQG_CleanRoom,$(IPA_CLEANROOM),nqg_cleanroom_game,com.nqg.cleanroom)

ios-sample: $(IOS_BUILD_DIR)/nqg_sample_game | dirs
	$(call PACK_IPA,NQG_Sample,$(IPA_SAMPLE),nqg_sample_game,com.nqg.sample)

else
ios ios-cleanroom ios-sample ios-check:
	@echo "Note: iOS IPA targets require macOS + Xcode (OS=$(DETECTED_OS))"
endif

# ---------------------------------------------------------------------------
# dist: collect binaries + apps
# ---------------------------------------------------------------------------
.PHONY: dist
dist: dirs $(SDL_BINS)
	@mkdir -p $(DIST_DIR)/bin
	@cp -f $(BUILD_DIR)/nqg_cleanroom_game$(EXE_EXT) $(DIST_DIR)/bin/ 2>/dev/null || true
	@cp -f $(BUILD_DIR)/nqg_sample_game$(EXE_EXT) $(DIST_DIR)/bin/ 2>/dev/null || true
	@echo "✓ dist/bin updated"
ifeq ($(DETECTED_OS),macos)
	@$(MAKE) app
endif

# ---------------------------------------------------------------------------
# verify — dynamic linkage + architectures
# ---------------------------------------------------------------------------
.PHONY: verify-links
verify-links:
	@echo "→ Dynamic link verification"
ifeq ($(DETECTED_OS),macos)
	@for b in $(APP_CLEANROOM)/Contents/MacOS/$(APP_EXEC_CR) \
	          $(APP_SAMPLE)/Contents/MacOS/$(APP_EXEC_SM) \
	          $(BUILD_DIR)/nqg_cleanroom_game $(BUILD_DIR)/nqg_sample_game; do \
		if [ -f "$$b" ]; then \
			echo "  $$b"; \
			echo "    arch: $$(lipo -info $$b 2>/dev/null || file $$b)"; \
			echo "    SDL : $$(otool -L $$b 2>/dev/null | grep -i sdl | head -3 | tr '\n' ';')"; \
			echo "    rpath: $$(otool -l $$b 2>/dev/null | grep -A2 LC_RPATH | grep path | tr '\n' ';')"; \
		fi; \
	done
else ifeq ($(DETECTED_OS),linux)
	@for b in $(BUILD_DIR)/nqg_cleanroom_game $(BUILD_DIR)/nqg_sample_game; do \
		if [ -f "$$b" ]; then \
			echo "  $$b"; \
			file "$$b"; \
			ldd "$$b" 2>/dev/null | grep -i sdl || echo "    (SDL not linked or static)"; \
		fi; \
	done
endif
	@echo "✓ verify-links done"

# Extend existing verify
verify: verify-links
