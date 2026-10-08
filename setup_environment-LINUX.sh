#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
# ============================================================================
# NQG – Linux Environment Setup Script
# Detects the distribution and installs SDL3 development packages
# Supported package managers:
#   apt, dnf, yum, pacman, zypper, apk, xbps, emerge, pkg (FreeBSD)
# ============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

info()    { echo -e "${BLUE}[INFO]${NC}  $*"; }
success() { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ---------------------------------------------------------------------------
# Save project directory
# ---------------------------------------------------------------------------
PROJECT_DIR="$(pwd)"
info "Project directory : $PROJECT_DIR"
info "Kernel            : $(uname -s) $(uname -m)"

# ---------------------------------------------------------------------------
# Detect package manager and set packages
# ---------------------------------------------------------------------------
PKG_MANAGER=""
UPDATE_CMD=""
INSTALL_CMD=""
PACKAGES=""

if command -v apt-get >/dev/null 2>&1; then
    PKG_MANAGER="apt"
    UPDATE_CMD="sudo apt-get update -y"
    INSTALL_CMD="sudo apt-get install -y"
    # Debian / Ubuntu / Pop!_OS / Mint / elementary …
    PACKAGES="libsdl3-dev pkg-config build-essential git"

elif command -v dnf >/dev/null 2>&1; then
    PKG_MANAGER="dnf"
    UPDATE_CMD="sudo dnf check-update || true"
    INSTALL_CMD="sudo dnf install -y"
    # Fedora / RHEL 8+ / Alma / Rocky
    PACKAGES="SDL3-devel pkgconf-pkg-config gcc-c++ git"

elif command -v yum >/dev/null 2>&1; then
    PKG_MANAGER="yum"
    UPDATE_CMD="sudo yum check-update || true"
    INSTALL_CMD="sudo yum install -y"
    # Older RHEL / CentOS
    PACKAGES="SDL3-devel pkgconfig gcc-c++ git"

elif command -v pacman >/dev/null 2>&1; then
    PKG_MANAGER="pacman"
    UPDATE_CMD="sudo pacman -Sy --noconfirm"
    INSTALL_CMD="sudo pacman -S --noconfirm --needed"
    # Arch / Manjaro / EndeavourOS
    PACKAGES="sdl3 pkgconf base-devel git"

elif command -v zypper >/dev/null 2>&1; then
    PKG_MANAGER="zypper"
    UPDATE_CMD="sudo zypper refresh"
    INSTALL_CMD="sudo zypper install -y"
    # openSUSE
    PACKAGES="SDL3-devel pkgconf-pkg-config gcc-c++ git"

elif command -v apk >/dev/null 2>&1; then
    PKG_MANAGER="apk"
    UPDATE_CMD="sudo apk update"
    INSTALL_CMD="sudo apk add"
    # Alpine
    PACKAGES="sdl3-dev pkgconf build-base git"

elif command -v xbps-install >/dev/null 2>&1; then
    PKG_MANAGER="xbps"
    UPDATE_CMD="sudo xbps-install -S"
    INSTALL_CMD="sudo xbps-install -y"
    # Void Linux
    PACKAGES="SDL3-devel pkg-config base-devel git"

elif command -v emerge >/dev/null 2>&1; then
    PKG_MANAGER="emerge"
    UPDATE_CMD="sudo emerge --sync"
    INSTALL_CMD="sudo emerge -v --ask=n"
    # Gentoo
    PACKAGES="media-libs/libsdl3 virtual/pkgconfig sys-devel/gcc dev-build/dev-vcs/git"

elif command -v pkg >/dev/null 2>&1 && [[ "$(uname -s)" == "FreeBSD" ]]; then
    PKG_MANAGER="pkg"
    UPDATE_CMD="sudo pkg update"
    INSTALL_CMD="sudo pkg install -y"
    # FreeBSD
    PACKAGES="sdl3 pkgconf git"

else
    error "Could not detect a supported package manager.

Supported package managers:
  • apt      (Debian, Ubuntu, Pop!_OS, Mint, …)
  • dnf/yum  (Fedora, RHEL, Alma, Rocky, CentOS)
  • pacman   (Arch, Manjaro, EndeavourOS)
  • zypper   (openSUSE)
  • apk      (Alpine)
  • xbps     (Void)
  • emerge   (Gentoo)
  • pkg      (FreeBSD)

Please install the SDL3 development package manually and make sure
the following commands work:

  pkg-config --exists sdl3
  pkg-config --cflags sdl3
  pkg-config --libs sdl3"
fi

info "Detected package manager : ${BOLD}$PKG_MANAGER${NC}"

# ---------------------------------------------------------------------------
# Check whether SDL3 is already usable
# ---------------------------------------------------------------------------
if pkg-config --exists sdl3 2>/dev/null; then
    VERSION=$(pkg-config --modversion sdl3 2>/dev/null || echo "unknown")
    success "SDL3 is already installed (version ${BOLD}$VERSION${NC})"
    echo
    info "Cflags : $(pkg-config --cflags sdl3)"
    info "Libs   : $(pkg-config --libs sdl3)"
    echo
    success "Nothing to do. You can run:  make"
    exit 0
fi

# ---------------------------------------------------------------------------
# Update package index
# ---------------------------------------------------------------------------
info "Updating package index..."
if ! $UPDATE_CMD; then
    warn "Package index update returned a non-zero exit code (continuing anyway)"
fi

# ---------------------------------------------------------------------------
# Install packages
# ---------------------------------------------------------------------------
info "Installing packages:"
echo "        $PACKAGES"
echo

if ! $INSTALL_CMD $PACKAGES; then
    error "Package installation failed.

Possible reasons:
  • The package name 'SDL3' / 'libsdl3-dev' does not exist yet
    on your distribution version (SDL3 is relatively new).
  • You need to enable an extra repository.
  • Network / mirror problems.

What you can try:
  1. Search for the correct package name:
       apt search sdl3          # Debian/Ubuntu
       dnf search SDL3          # Fedora
       pacman -Ss sdl3          # Arch
  2. Install from source (see the macOS script for a example)
  3. Use a Flatpak / container later for distribution"
fi

# ---------------------------------------------------------------------------
# Final verification
# ---------------------------------------------------------------------------
echo
info "Verifying installation..."

if ! command -v pkg-config >/dev/null 2>&1; then
    error "pkg-config is not available after installation."
fi

if ! pkg-config --exists sdl3 2>/dev/null; then
    error "SDL3 development files were installed but pkg-config
still cannot find the module 'sdl3'.

Debug information:
  pkg-config --list-all | grep -i sdl
  ls /usr/lib*/pkgconfig/sdl3* 2>/dev/null || true
  ls /usr/local/lib*/pkgconfig/sdl3* 2>/dev/null || true

Possible fixes:
  • Run:  hash -r
  • Log out and log back in
  • Check PKG_CONFIG_PATH
  • The package may have installed headers under a different name"
fi

VERSION=$(pkg-config --modversion sdl3)
CFLAGS=$(pkg-config --cflags sdl3)
LIBS=$(pkg-config --libs sdl3)

success "SDL3 successfully installed and detected!"
echo
echo "================================================================"
echo "  SDL3 version : $VERSION"
echo "  Cflags       : $CFLAGS"
echo "  Libs         : $LIBS"
echo "================================================================"
echo
info "You can now build the project:"
echo
echo "    make clean"
echo "    make"
echo
success "Setup finished. Current directory: $PROJECT_DIR"
# ---------------------------------------------------------------------------
# Build system note (Makefile + mk/ only — no CMake)
# ---------------------------------------------------------------------------
info "Build system: Makefile + mk/platform.mk (unified)"
info "  make core          → engine tests without SDL"
info "  make nqg_cleanroom_game → full game (needs SDL3)"
info "  make all           → everything"
success "Environment ready. Run: make core && make nqg_cleanroom_game"
