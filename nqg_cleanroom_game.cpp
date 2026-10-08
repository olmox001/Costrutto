/* SPDX-License-Identifier: GPL-2.0-or-later
 * Engine host + optional SDL client. SDL is never required to run the kernel.
 */
#include "nqg_host.hpp"

#if defined(NQG_WITH_SDL)
#include "nqg_sdl_platform.hpp"
#endif

#include <iostream>

using namespace nqg;
using namespace nqg::host;

int main(int argc, char **argv) {
  bool headless = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--headless" || a == "-H")
      headless = true;
  }

  Host host;
  host.setViewResolution(480, 360);

  std::cout << "NQG Apartment host started (capsule='"
            << host.capsule().name << "')\n";

#if defined(NQG_WITH_SDL)
  if (!headless) {
    using namespace nqg::platform;
    PlatformConfig cfg;
    cfg.title = "NQG Apartment";
    // Presentation starts at the same logical size as the Observer.
    // Retina scaling belongs to SDL's drawable, not to engine render size.
    cfg.width = 480;
    cfg.height = 360;
    cfg.vsync = true;
    cfg.enableAudio = true;
    SdlPlatform client;
    if (!client.init(cfg)) {
      std::cerr << "SDL client failed — falling back to headless host steps\n";
      headless = true;
    } else {
      std::cout << "SDL client connected to capsule camera\n";
      while (client.poll()) {
        const double dt = client.deltaTime();
        host.setCommands(client.commands());
        host.step(dt);
        // HUD resolution follows capsule view; client presents capsule ports only
        client.presentCapsule(host.capsule());
        if (host.shouldQuit())
          break;
      }
      client.shutdown();
      std::cout << "Appartamento chiuso.\n";
      return 0;
    }
  }
#else
  (void)headless;
  headless = true;
#endif

  // Headless host: fixed steps for smoke / CI
  std::cout << "Headless host: 120 steps @ 60 Hz\n";
  host.applyTextCommand("move 0 0.5");
  host.stepN(120, 1.0 / 60.0);
  host.saveFramebufferPPM("/tmp/nqg_headless.ppm");
  std::cout << host.logDump(8);
  std::cout << "Framebuffer saved /tmp/nqg_headless.ppm\n";
  return 0;
}
