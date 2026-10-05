CXX = clang++

CXXFLAGS = -O2 -std=c++17 -Wall -Wextra -pthread

LDFLAGS = -framework Cocoa $(shell pkg-config --cflags --libs sdl3)

TARGETS = test_nqg_core test_engine3d test_window_sdl3 test_cleanroom \
          test_matter_physics test_continuum_physics test_sdf nqg_sample_game \
          nqg_cleanroom_game

COMMON_HDRS = nqg_physics_core.hpp nqg_engine3d.hpp nqg_sdf.hpp nqg_earth_environment.hpp \
              nqg_apartment.hpp nqg_air_physics.hpp nqg_matter_physics.hpp \
              nqg_continuum_physics.hpp

all: $(TARGETS) app

test_nqg_core: test_nqg_core.cpp nqg_physics_core.hpp
	$(CXX) $(CXXFLAGS) test_nqg_core.cpp -o test_nqg_core

test_engine3d: test_engine3d.cpp nqg_engine3d.hpp nqg_physics_core.hpp
	$(CXX) $(CXXFLAGS) test_engine3d.cpp -o test_engine3d

test_window_sdl3: test_window_sdl3.cpp nqg_window_sdl3.hpp nqg_engine3d.hpp nqg_physics_core.hpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) test_window_sdl3.cpp -o test_window_sdl3

test_cleanroom: test_cleanroom.cpp $(COMMON_HDRS)
	$(CXX) $(CXXFLAGS) test_cleanroom.cpp -o test_cleanroom

test_matter_physics: test_matter_physics.cpp nqg_matter_physics.hpp nqg_sdf.hpp nqg_air_physics.hpp nqg_engine3d.hpp nqg_physics_core.hpp
	$(CXX) $(CXXFLAGS) test_matter_physics.cpp -o test_matter_physics

test_continuum_physics: test_continuum_physics.cpp nqg_continuum_physics.hpp nqg_sdf.hpp nqg_air_physics.hpp nqg_engine3d.hpp nqg_physics_core.hpp
	$(CXX) $(CXXFLAGS) test_continuum_physics.cpp -o test_continuum_physics

test_sdf: test_sdf.cpp $(COMMON_HDRS)
	$(CXX) $(CXXFLAGS) test_sdf.cpp -o test_sdf

nqg_sample_game: nqg_sample_game.cpp nqg_window_sdl3.hpp nqg_engine3d.hpp nqg_physics_core.hpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) nqg_sample_game.cpp -o nqg_sample_game

nqg_cleanroom_game: nqg_cleanroom_game.cpp $(COMMON_HDRS) nqg_window_sdl3.hpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) nqg_cleanroom_game.cpp -o nqg_cleanroom_game

app: nqg_cleanroom_game icon.icns
	mkdir -p Costrutto.app/Contents/MacOS
	mkdir -p Costrutto.app/Contents/Resources

	cp nqg_cleanroom_game Costrutto.app/Contents/MacOS/NQG_CleanRoom
	cp icon.icns Costrutto.app/Contents/Resources/icon.icns

	chmod +x Costrutto.app/Contents/MacOS/NQG_CleanRoom

	printf '%s\n' \
	'<?xml version="1.0" encoding="UTF-8"?>' \
	'<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">' \
	'<plist version="1.0">' \
	'<dict>' \
	'	<key>CFBundleDisplayName</key>' \
	'	<string>NQG CleanRoom</string>' \
	'	<key>CFBundleExecutable</key>' \
	'	<string>NQG_CleanRoom</string>' \
	'	<key>CFBundleIdentifier</key>' \
	'	<string>com.nqg.cleanroom</string>' \
	'	<key>CFBundleIconFile</key>' \
	'	<string>icon.icns</string>' \
	'	<key>CFBundleName</key>' \
	'	<string>NQG CleanRoom</string>' \
	'	<key>CFBundlePackageType</key>' \
	'	<string>APPL</string>' \
	'	<key>CFBundleVersion</key>' \
	'	<string>1.0</string>' \
	'	<key>CFBundleShortVersionString</key>' \
	'	<string>1.0</string>' \
	'</dict>' \
	'</plist>' \
	> Costrutto.app/Contents/Info.plist

test: all
	./test_nqg_core
	./test_engine3d
	./test_window_sdl3
	./test_cleanroom
	./test_matter_physics
	./test_continuum_physics
	./test_sdf

run: nqg_sample_game
	./nqg_sample_game

run_cleanroom: nqg_cleanroom_game
	./nqg_cleanroom_game

clean:
	rm -f $(TARGETS)
	rm -rf Costrutto.app

.PHONY: all test run run_cleanroom clean
