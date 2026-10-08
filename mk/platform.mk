# SPDX-License-Identifier: GPL-2.0-or-later
# ============================================================================
# NQG Platform Makefile
# OS / architecture detection, SDL3, universal binaries, .app bundle,
# path/rpath verification, dist/test folder, clean, info, help, report.
# Independent from project sources – safe to share or replace alone.
# ============================================================================


# Serialize critical package steps across parallel make / multi-arch
NQG_BUILD_LOCK ?= $(abspath .)/.nqg_build.lock
define nqg_with_lock
	@mkdir -p "$(dir $(NQG_BUILD_LOCK))"
	@exec 9>"$(NQG_BUILD_LOCK)"; flock -w 120 9; $(1)
endef

# ---------------------------------------------------------------------------
# Basic settings
# ---------------------------------------------------------------------------
CXX       ?= clang++
CXXFLAGS   = -O2 -std=c++17 -Wall -Wextra -pthread
LDFLAGS    =

# ---------------------------------------------------------------------------
# Host detection
# ---------------------------------------------------------------------------
UNAME_S := $(shell uname -s 2>/dev/null || echo Unknown)
UNAME_M := $(shell uname -m 2>/dev/null || echo Unknown)

DETECTED_OS := Unknown
HOST_ARCH   := $(UNAME_M)
EXE_EXT     :=

ifeq ($(OS),Windows_NT)
    DETECTED_OS := windows
    EXE_EXT     := .exe
    ifeq ($(shell which clang++ 2>/dev/null),)
        CXX := g++
    endif
    # MinGW / MSYS2 / clang-cl portable defaults
    CXXFLAGS_WIN := -D_WIN32_WINNT=0x0601 -DNOMINMAX -DWIN32_LEAN_AND_MEAN
    CXXFLAGS += $(CXXFLAGS_WIN)
    LDFLAGS  += -static-libgcc -static-libstdc++
else ifeq ($(UNAME_S),Darwin)
    DETECTED_OS := macos
    CXX         := clang++
else ifeq ($(UNAME_S),Linux)
    DETECTED_OS := linux
    CXX         ?= g++
else
    DETECTED_OS := unix
endif

# ---------------------------------------------------------------------------
# Build type selection
# ---------------------------------------------------------------------------
BUILD_TYPE := $(DETECTED_OS)-$(HOST_ARCH)

ifeq ($(DETECTED_OS),macos)
    BUILD_TYPE := macos-universal
endif

BUILD_DIR  := build/$(BUILD_TYPE)
DIST_DIR   := dist
TEST_DIR   := $(DIST_DIR)/test
APP_BUNDLE := $(DIST_DIR)/Costrutto.app

# ---------------------------------------------------------------------------
# SDL3 detection (non-blocking)
# ---------------------------------------------------------------------------
SDL_FOUND  := 0
SDL_TYPE   := none
SDL_CFLAGS :=
SDL_LIBS   :=
SDL_RPATH  :=
SDL_DYLIB  :=
IS_FAT     := 0

ifeq ($(DETECTED_OS),macos)
    ifeq ($(HOST_ARCH),arm64)
        BREW_PREFIX := /opt/homebrew
    else
        BREW_PREFIX := /usr/local
    endif

    ifneq ($(wildcard $(BREW_PREFIX)/lib/libSDL3*.dylib),)
        SDL_CFLAGS := -I$(BREW_PREFIX)/include
        SDL_LIBS   := -L$(BREW_PREFIX)/lib -lSDL3
        SDL_RPATH  := -Wl,-rpath,$(BREW_PREFIX)/lib
        SDL_DYLIB  := $(firstword $(wildcard $(BREW_PREFIX)/lib/libSDL3.dylib \
                                             $(BREW_PREFIX)/lib/libSDL3.*.dylib))
        SDL_FOUND  := 1
        SDL_TYPE   := brew
    endif

    ifeq ($(SDL_FOUND),0)
        ifneq ($(wildcard /usr/local/lib/libSDL3*.dylib),)
            SDL_CFLAGS := -I/usr/local/include
            SDL_LIBS   := -L/usr/local/lib -lSDL3
            SDL_RPATH  := -Wl,-rpath,/usr/local/lib
            SDL_DYLIB  := $(firstword $(wildcard /usr/local/lib/libSDL3.dylib \
                                                 /usr/local/lib/libSDL3.*.dylib))
            SDL_FOUND  := 1
            SDL_TYPE   := brew
        endif
    endif

    ifeq ($(SDL_FOUND),0)
        SDL_CFLAGS := $(shell pkg-config --cflags sdl3 2>/dev/null)
        SDL_LIBS   := $(shell pkg-config --libs   sdl3 2>/dev/null)
        ifneq ($(SDL_LIBS),)
            SDL_FOUND := 1
            SDL_TYPE  := pkg-config
        endif
    endif

    ifneq ($(SDL_DYLIB),)
        FAT_CHECK := $(shell lipo -info $(SDL_DYLIB) 2>/dev/null | \
                      grep -E "arm64.*x86_64|x86_64.*arm64" || true)
        ifneq ($(FAT_CHECK),)
            IS_FAT := 1
        endif
    endif

    LDFLAGS  += -framework Cocoa -framework IOKit -framework CoreVideo \
                $(SDL_LIBS) $(SDL_RPATH)
    CXXFLAGS += $(SDL_CFLAGS)

    ifeq ($(BUILD_TYPE),macos-universal)
        ifeq ($(IS_FAT),1)
            CXXFLAGS += -arch arm64 -arch x86_64
            LDFLAGS  += -arch arm64 -arch x86_64
            $(info → Building UNIVERSAL binary (arm64 + x86_64))
        else
            $(warning )
            $(warning ================================================================)
            $(warning  SDL3 is NOT Universal. Falling back to host architecture.)
            $(warning  Current SDL3 : $(SDL_DYLIB))
            $(warning )
            $(warning  To build a real Universal binary run:)
            $(warning      ./setup_environment-MACOS.sh)
            $(warning )
            $(warning ================================================================)
            $(warning )
            BUILD_TYPE := macos-$(HOST_ARCH)
            BUILD_DIR  := build/$(BUILD_TYPE)
        endif
    endif

    ifeq ($(SDL_FOUND),0)
        $(warning )
        $(warning ================================================================)
        $(warning  SDL3 not found.)
        $(warning  Please run:  ./setup_environment-MACOS.sh)
        $(warning ================================================================)
        $(warning )
    endif

else ifeq ($(DETECTED_OS),linux)
    SDL_CFLAGS := $(shell pkg-config --cflags sdl3 2>/dev/null)
    SDL_LIBS   := $(shell pkg-config --libs   sdl3 2>/dev/null)

    ifeq ($(SDL_LIBS),)
        $(warning )
        $(warning ================================================================)
        $(warning  SDL3 development package not found.)
        $(warning )
        $(warning  Please run:  ./setup_environment-LINUX.sh)
        $(warning )
        $(warning  Or install manually:)
        $(warning      Debian/Ubuntu : sudo apt install libsdl3-dev)
        $(warning      Fedora        : sudo dnf install SDL3-devel)
        $(warning      Arch          : sudo pacman -S sdl3)
        $(warning      openSUSE      : sudo zypper install SDL3-devel)
        $(warning )
        $(warning  The non-SDL targets will still be built.)
        $(warning ================================================================)
        $(warning )
        SDL_LIBS := -lSDL3
    else
        SDL_FOUND := 1
        SDL_TYPE  := pkg-config
        $(info → Linux: SDL3 found via pkg-config)
    endif

    LDFLAGS  += $(SDL_LIBS)
    CXXFLAGS += $(SDL_CFLAGS)

else ifeq ($(DETECTED_OS),windows)
    SDL_CFLAGS := $(shell pkg-config --cflags sdl3 2>/dev/null)
    SDL_LIBS   := $(shell pkg-config --libs   sdl3 2>/dev/null)
    ifeq ($(SDL_LIBS),)
        SDL_LIBS := -lSDL3
    endif
    LDFLAGS  += $(SDL_LIBS)
    CXXFLAGS += $(SDL_CFLAGS)
    SDL_FOUND := 1
    SDL_TYPE  := windows
endif

# ---------------------------------------------------------------------------
# Common phony targets
# ---------------------------------------------------------------------------
.PHONY: dirs app prepare-test verify report clean info help

dirs:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(DIST_DIR)
	@mkdir -p $(TEST_DIR)
ifeq ($(DETECTED_OS),macos)
	@mkdir -p $(APP_BUNDLE)/Contents/MacOS
	@mkdir -p $(APP_BUNDLE)/Contents/Resources
	@mkdir -p $(APP_BUNDLE)/Contents/Frameworks
endif

# ---------------------------------------------------------------------------
# macOS .app bundle + embed dylib (clean rpaths)
# ---------------------------------------------------------------------------
ifeq ($(DETECTED_OS),macos)

APP_EXEC = NQG_CleanRoom
APP_ICON = icon.icns

app-legacy-single: $(BUILD_DIR)/nqg_cleanroom_game$(EXE_EXT) $(APP_ICON) | dirs
	@echo "→ Creating $(APP_BUNDLE)  [$(BUILD_TYPE)]"
	cp $(BUILD_DIR)/nqg_cleanroom_game$(EXE_EXT) $(APP_BUNDLE)/Contents/MacOS/$(APP_EXEC)
	chmod +x $(APP_BUNDLE)/Contents/MacOS/$(APP_EXEC)
	cp $(APP_ICON) $(APP_BUNDLE)/Contents/Resources/$(APP_ICON)
	@if [ -n "$(SDL_DYLIB)" ] && [ -f "$(SDL_DYLIB)" ]; then \
		FW="$(APP_BUNDLE)/Contents/Frameworks"; \
		EXEC="$(APP_BUNDLE)/Contents/MacOS/$(APP_EXEC)"; \
		REAL=$$(python3 -c "import os; print(os.path.realpath('$(SDL_DYLIB)'))" 2>/dev/null || readlink -f "$(SDL_DYLIB)" 2>/dev/null || echo "$(SDL_DYLIB)"); \
		BASE=$$(basename "$$REAL"); \
		echo "  → Embedding $$BASE (from $$REAL)"; \
		cp "$$REAL" "$$FW/$$BASE"; \
		cp "$$REAL" "$$FW/libSDL3.dylib"; \
		cp "$$REAL" "$$FW/libSDL3.0.dylib"; \
		install_name_tool -id "@rpath/$$BASE" "$$FW/$$BASE" 2>/dev/null || true; \
		install_name_tool -id "@rpath/libSDL3.dylib" "$$FW/libSDL3.dylib" 2>/dev/null || true; \
		install_name_tool -id "@rpath/libSDL3.0.dylib" "$$FW/libSDL3.0.dylib" 2>/dev/null || true; \
		install_name_tool -change "$(SDL_DYLIB)" "@rpath/libSDL3.0.dylib" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -change "$$REAL" "@rpath/libSDL3.0.dylib" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -change "/usr/local/lib/libSDL3.dylib" "@rpath/libSDL3.0.dylib" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -change "/usr/local/lib/libSDL3.0.dylib" "@rpath/libSDL3.0.dylib" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -change "/opt/homebrew/lib/libSDL3.dylib" "@rpath/libSDL3.0.dylib" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -change "/opt/homebrew/lib/libSDL3.0.dylib" "@rpath/libSDL3.0.dylib" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -change "@rpath/libSDL3.dylib" "@rpath/libSDL3.0.dylib" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -add_rpath "@executable_path/../Frameworks" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -delete_rpath "/usr/local/lib" "$$EXEC" 2>/dev/null || true; \
		install_name_tool -delete_rpath "/opt/homebrew/lib" "$$EXEC" 2>/dev/null || true; \
	fi
	@printf '%s\n' \
		'<?xml version="1.0" encoding="UTF-8"?>' \
		'<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">' \
		'<plist version="1.0">' \
		'<dict>' \
		'	<key>CFBundleDisplayName</key><string>NQG CleanRoom</string>' \
		'	<key>CFBundleExecutable</key><string>$(APP_EXEC)</string>' \
		'	<key>CFBundleIdentifier</key><string>com.nqg.cleanroom</string>' \
		'	<key>CFBundleIconFile</key><string>$(APP_ICON)</string>' \
		'	<key>CFBundleName</key><string>NQG CleanRoom</string>' \
		'	<key>CFBundlePackageType</key><string>APPL</string>' \
		'	<key>CFBundleVersion</key><string>1.0</string>' \
		'	<key>CFBundleShortVersionString</key><string>1.0</string>' \
		'	<key>LSMinimumSystemVersion</key><string>11.0</string>' \
		'</dict></plist>' > $(APP_BUNDLE)/Contents/Info.plist
	@echo "✓ $(APP_BUNDLE) ready"

else
app-legacy-single:
	@echo "Note: .app bundle is macOS-only (current OS = $(DETECTED_OS))"
endif

# ---------------------------------------------------------------------------
# Verification (binary + library + rpath)
# ---------------------------------------------------------------------------
verify:
ifeq ($(DETECTED_OS),macos)
	@echo "→ Verifying Universal binary and embedded library..."
	@EXEC="$(APP_BUNDLE)/Contents/MacOS/$(APP_EXEC)"; \
	LIB="$(APP_BUNDLE)/Contents/Frameworks/libSDL3.dylib"; \
	echo "  Binary architectures:"; \
	lipo -info "$$EXEC" 2>/dev/null || echo "    (binary not found)"; \
	if [ -f "$$LIB" ]; then \
		echo "  Library architectures:"; \
		lipo -info "$$LIB" 2>/dev/null; \
		echo "  Library install name:"; \
		otool -D "$$LIB" 2>/dev/null | tail -1; \
		echo "  Binary rpaths:"; \
		otool -l "$$EXEC" 2>/dev/null | grep -A2 LC_RPATH | grep path || echo "    (none)"; \
		echo "  Binary dependency on SDL3:"; \
		otool -L "$$EXEC" 2>/dev/null | grep -i sdl3 || echo "    (not found)"; \
		BIN_FAT=$$(lipo -info "$$EXEC" 2>/dev/null | grep -E "arm64.*x86_64|x86_64.*arm64" || true); \
		LIB_FAT=$$(lipo -info "$$LIB"  2>/dev/null | grep -E "arm64.*x86_64|x86_64.*arm64" || true); \
		RPATH_OK=$$(otool -l "$$EXEC" 2>/dev/null | grep -A2 LC_RPATH | grep -q "@executable_path/../Frameworks" && echo yes || echo no); \
		ABS_RPATH=$$(otool -l "$$EXEC" 2>/dev/null | grep -A2 LC_RPATH | grep path | grep -E "/usr/local|/opt/homebrew" || true); \
		ID_OK=$$(otool -D "$$LIB" 2>/dev/null | tail -1 | grep -q "@rpath" && echo yes || echo no); \
		echo ""; \
		if [ -n "$$BIN_FAT" ]; then echo "  ✓ Binary is Universal"; else echo "  ✗ Binary is NOT Universal"; fi; \
		if [ -n "$$LIB_FAT" ]; then echo "  ✓ Library is Universal"; else echo "  ✗ Library is NOT Universal"; fi; \
		if [ "$$RPATH_OK" = "yes" ]; then echo "  ✓ rpath is correct (@executable_path/../Frameworks)"; else echo "  ✗ rpath missing or wrong"; fi; \
		if [ -z "$$ABS_RPATH" ]; then echo "  ✓ No residual absolute rpaths"; else echo "  ✗ Residual absolute rpath still present"; fi; \
		if [ "$$ID_OK" = "yes" ]; then echo "  ✓ Library install name uses @rpath"; else echo "  ✗ Library install name does not use @rpath"; fi; \
	else \
		echo "  (no embedded libSDL3.dylib – skipped library checks)"; \
	fi
	@echo "✓ Verification finished"
else
	@echo "→ Verification skipped (macOS only)"
endif

# ---------------------------------------------------------------------------
# Final report
# ---------------------------------------------------------------------------
report:
	@echo ""
	@echo "================================================================"
	@echo "  NQG BUILD REPORT"
	@echo "================================================================"
	@echo "  OS / Arch     : $(DETECTED_OS) / $(HOST_ARCH)"
	@echo "  Build type    : $(BUILD_TYPE)"
	@echo "  Build dir     : $(BUILD_DIR)"
	@echo "  Dist dir      : $(DIST_DIR)"
	@echo ""
ifeq ($(DETECTED_OS),macos)
	@EXEC="$(APP_BUNDLE)/Contents/MacOS/$(APP_EXEC)"; \
	LIB="$(APP_BUNDLE)/Contents/Frameworks/libSDL3.dylib"; \
	echo "  --- App bundle ---"; \
	echo "  Bundle         : $(APP_BUNDLE)"; \
	if [ -f "$$EXEC" ]; then \
		echo "  Binary         : $$(lipo -info "$$EXEC" 2>/dev/null)"; \
	fi; \
	if [ -f "$$LIB" ]; then \
		echo "  Library        : $$(lipo -info "$$LIB" 2>/dev/null)"; \
		echo "  Install name   : $$(otool -D "$$LIB" 2>/dev/null | tail -1)"; \
		echo "  Rpaths         :"; \
		otool -l "$$EXEC" 2>/dev/null | grep -A2 LC_RPATH | grep path | sed 's/^/    /' || echo "    (none)"; \
		echo "  SDL dependency :"; \
		otool -L "$$EXEC" 2>/dev/null | grep -i sdl3 | sed 's/^/    /' || echo "    (none)"; \
	fi; \
	echo ""
endif
	@echo "  --- Host test binaries ---"
	@ls -1 $(TEST_DIR) 2>/dev/null | sed 's/^/    /' || echo "    (empty)"
	@echo ""
	@echo "  --- Unit tests ---"
	@echo "    (see output above – all suites run automatically)"
	@echo ""
	@echo "================================================================"
	@echo "  BUILD COMPLETE"
	@echo "================================================================"
	@echo ""

# prepare-test is defined in mk/test.mk

# ---------------------------------------------------------------------------
# Clean / Info / Help
# ---------------------------------------------------------------------------
clean:
	rm -rf build dist
	@echo "✓ Clean complete"

info:
	@echo "Detected OS       : $(DETECTED_OS)"
	@echo "Host architecture : $(HOST_ARCH)"
	@echo "Build type        : $(BUILD_TYPE)"
	@echo "Build directory   : $(BUILD_DIR)"
	@echo "SDL type          : $(SDL_TYPE)"
	@echo "SDL found         : $(SDL_FOUND)"
	@echo "SDL dylib         : $(SDL_DYLIB)"
	@echo "SDL is Universal  : $(IS_FAT)"
	@echo "CXXFLAGS          : $(CXXFLAGS)"
	@echo "LDFLAGS           : $(LDFLAGS)"

help:
	@echo "Costrutto / NQG Makefile"
	@echo ""
	@echo "  make                 Full pipeline (core + apps + packages + tests)"
	@echo "  make core            Engine unit tests only (no SDL)"
	@echo "  make apps            Build cleanroom + sample games"
	@echo "  make app             macOS .app bundles (Costrutto + NQG_Sample)"
	@echo "  make ios             iOS unsigned IPAs (needs Xcode + SDL3.framework)"
	@echo "  make dist            Collect binaries/apps into dist/"
	@echo "  make verify          Dynamic link + architecture checks"
	@echo "  make test            Re-run unit tests"
	@echo "  make run             Run sample game"
	@echo "  make run_cleanroom   Run apartment/cleanroom game"
	@echo "  make clean / info"
	@echo ""
	@echo "  make BUILD_TYPE=macos-universal   Fat binary arm64+x86_64"
	@echo "  make BUILD_TYPE=macos-arm64"
	@echo "  make BUILD_TYPE=linux-x86_64"
	@echo ""
	@echo "  SDL3_IOS_FRAMEWORK=/path/to/SDL3.framework make ios"
	@echo ""
	@echo "Setup: ./setup_environment-MACOS.sh | ./setup_environment-LINUX.sh"