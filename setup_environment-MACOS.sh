#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# ============================================================================
# NQG – macOS Environment Setup Script
# Installs a Universal (arm64 + x86_64) SDL3
# Works on both Apple Silicon and Intel Macs
# ============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Colours and helpers
# ---------------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info()    { echo -e "${BLUE}[INFO]${NC}  $*"; }
success() { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ---------------------------------------------------------------------------
# Save original project directory (we must return here at the end)
# ---------------------------------------------------------------------------
PROJECT_DIR="$(pwd)"
info "Project directory: $PROJECT_DIR"

# ---------------------------------------------------------------------------
# Detect architecture
# ---------------------------------------------------------------------------
ARCH="$(uname -m)"
if [[ "$ARCH" == "arm64" ]]; then
    HOST_ARCH="Apple Silicon (arm64)"
    BREW_PREFIX="/opt/homebrew"
elif [[ "$ARCH" == "x86_64" ]]; then
    HOST_ARCH="Intel (x86_64)"
    BREW_PREFIX="/usr/local"
else
    error "Unsupported architecture: $ARCH"
fi

info "Host architecture: $HOST_ARCH"
info "Homebrew prefix  : $BREW_PREFIX"

# ---------------------------------------------------------------------------
# Check Xcode Command Line Tools
# ---------------------------------------------------------------------------
if ! xcode-select -p &>/dev/null; then
    warn "Xcode Command Line Tools not found."
    info "Installing them now (a dialog will appear)..."
    xcode-select --install || true
    error "Please finish the Xcode CLT installation and re-run this script."
fi
success "Xcode Command Line Tools found"

# ---------------------------------------------------------------------------
# Check / Install Homebrew
# ---------------------------------------------------------------------------
if ! command -v brew &>/dev/null; then
    warn "Homebrew not found. Installing..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    
    # Add brew to PATH for this session
    if [[ "$ARCH" == "arm64" ]]; then
        eval "$(/opt/homebrew/bin/brew shellenv)"
    else
        eval "$(/usr/local/bin/brew shellenv)"
    fi
fi

if ! command -v brew &>/dev/null; then
    error "Homebrew installation failed or is not in PATH.
Add this to your ~/.zshrc (or ~/.bash_profile):

  eval \"\$($BREW_PREFIX/bin/brew shellenv)\"

Then open a new terminal and re-run this script."
fi
success "Homebrew found: $(brew --prefix)"

# ---------------------------------------------------------------------------
# Required build tools
# ---------------------------------------------------------------------------
info "Installing / updating build dependencies..."
brew install git pkg-config || true
success "cmake, git, pkg-config ready"

# ---------------------------------------------------------------------------
# Decide install prefix for SDL3
# We always install the Universal build into /usr/local so that both
# architectures can find it with the same path.
# ---------------------------------------------------------------------------
SDL_PREFIX="/usr/local"
info "SDL3 will be installed into: $SDL_PREFIX"

# ---------------------------------------------------------------------------
# Remove any previous non-universal SDL3 to avoid conflicts
# ---------------------------------------------------------------------------
info "Checking for existing SDL3 installations..."

if [[ -f "$SDL_PREFIX/lib/libSDL3.dylib" ]]; then
    CURRENT_ARCHS=$(lipo -info "$SDL_PREFIX/lib/libSDL3.dylib" 2>/dev/null || echo "unknown")
    info "Existing libSDL3.dylib → $CURRENT_ARCHS"

    if echo "$CURRENT_ARCHS" | grep -qE "arm64.*x86_64|x86_64.*arm64"; then
        success "A Universal SDL3 is already installed. Nothing to do."
        echo
        info "You can now run:  make"
        exit 0
    else
        warn "Existing SDL3 is NOT Universal. It will be replaced."
    fi
fi

# Also clean Homebrew SDL3 if present (it is usually single-arch)
if brew list sdl3 &>/dev/null 2>&1; then
    warn "Homebrew sdl3 formula detected – uninstalling it to avoid conflicts..."
    brew uninstall --ignore-dependencies sdl3 || true
fi

# ---------------------------------------------------------------------------
# Build Universal SDL3 from source
# ---------------------------------------------------------------------------
WORK_DIR=$(mktemp -d)
info "Working directory: $WORK_DIR"
cd "$WORK_DIR"

info "Cloning SDL repository (latest stable)..."
git clone --depth 1 https://github.com/libsdl-org/SDL.git
cd SDL

info "Configuring Universal build (arm64 + x86_64)..."
mkdir build && cd build

.. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 \
    -DCMAKE_INSTALL_PREFIX="$SDL_PREFIX" \
    -DSDL_SHARED=ON \
    -DSDL_STATIC=OFF \
    -DSDL_TEST=OFF \
    -DSDL_TESTS=OFF \
    -DSDL_INSTALL_TESTS=OFF

info "Compiling (this may take a few minutes)..."
--build . --parallel "$(sysctl -n hw.ncpu)"

info "Installing into $SDL_PREFIX (requires sudo)..."
sudo --install .

# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------
cd "$PROJECT_DIR"   # ← IMPORTANT: return to project directory

if [[ ! -f "$SDL_PREFIX/lib/libSDL3.dylib" ]]; then
    error "Installation failed: $SDL_PREFIX/lib/libSDL3.dylib not found."
fi

ARCHS=$(lipo -info "$SDL_PREFIX/lib/libSDL3.dylib" 2>/dev/null || echo "ERROR")
info "Installed library architectures: $ARCHS"

if ! echo "$ARCHS" | grep -qE "arm64.*x86_64|x86_64.*arm64"; then
    error "The installed SDL3 is NOT Universal!
Expected both arm64 and x86_64.
Something went wrong during the build.
Please check the output above."
fi

success "Universal SDL3 successfully installed!"
echo
echo "---------------------------------------------------------------"
echo "  libSDL3.dylib  →  $SDL_PREFIX/lib/libSDL3.dylib"
echo "  Headers        →  $SDL_PREFIX/include/SDL3/"
echo "  Architectures  →  arm64 + x86_64"
echo "---------------------------------------------------------------"
echo
info "You can now return to the project and run:"
echo
echo "    make clean"
echo "    make"
echo
success "Setup finished. Project directory restored: $PROJECT_DIR"
# ---------------------------------------------------------------------------
# Build system note (Makefile + mk/ only — no CMake)
# ---------------------------------------------------------------------------
info "Build system: Makefile + mk/platform.mk (unified)"
info "  make core                 → engine tests without SDL"
info "  make nqg_cleanroom_game   → full game (needs SDL3)"
info "iOS: build SDL3.framework for iOS, then use Xcode project or"
info "     cross-compile with clang -isysroot \$(xcrun --sdk iphoneos --show-sdk-path)"
success "Environment ready. Run: make core && make nqg_cleanroom_game"
