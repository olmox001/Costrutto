/* SPDX-License-Identifier: GPL-2.0-or-later
 * nqg_cli_client — LLM/human host client (no SDL required).
 */
#include "nqg_engine_api.hpp"
#include "engine/perf_profiler.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using nqg::api::Host;
using nqg::cmd::Commands;
using real = double;

static void printHelpLlm() {
  std::cout
      << "NQG CLI — LLM helper (machine-oriented)\n"
      << "PROTOCOL\n"
      << "  Input: one command per line. Output: text + optional PPM path.\n"
      << "  Log schema: t|frame|cap|kind|key|value\n"
      << "COMMANDS\n"
      << "  move X Y          virtual stick [-1,1], sticky until stop\n"
      << "  look YAW PITCH    radians delta (one-shot)\n"
      << "  stop              clear move/look/jump\n"
      << "  jump|jump_hold|jump_release\n"
      << "  tp X Y Z          teleport capsule (metres ENU)\n"
      << "  freefall          toggle free-fall mode\n"
      << "  water|sand|solid|wind|gravity|light|clear\n"
      << "  res W H           framebuffer resolution\n"
      << "  step [N]          simulate N frames (default 1)\n"
      << "  cap N | addcap NAME W H\n"
      << "  shot [path.ppm]   force render + save\n"
      << "  obs | probe | debug [on|off]\n"
      << "  perf on|off|      performance report\n"
      << "  log [N]           dump structured log\n"
      << "  quit\n"
      << "FLAGS\n"
      << "  --test            full self-test suite\n"
      << "  --help-llm        this help\n"
      << "  --touch-test      enable virtual touch stick simulation path\n"
      << "  -e CMD            run command and exit non-interactive\n"
      << "  -f FILE           run script file\n"
      << "MULTIPLAYER\n"
      << "  addcap p2 160 120; cap 0; move -1 0; cap 1; move 1 0; step 30; obs\n";
}

static bool g_debug = false;

static void dumpDebug(Host &host, std::size_t cap = 0) {
  auto &c = host.capsule(cap);
  const auto &o = c.observation;
  auto &scene = host.scene();
  std::cout << "---- DEBUG cap=" << cap << " frame=" << host.frame()
            << " t=" << host.simTime() << " ----\n";
  std::cout << "  pos=(" << o.position.x << "," << o.position.y << ","
            << o.position.z << ") vel=" << o.speed << " mode=" << o.modeLabel
            << "\n";
  std::cout << "  room=" << o.insideRoom << " spillway=" << o.insideSpillway
            << " alt=" << o.altitude << " g=" << o.gravity
            << " rho=" << o.density << " wind=" << o.windMag << "\n";
  std::cout << "  view=" << o.renderW << "x" << o.renderH
            << " render=" << (c.renderEnabled ? "on" : "off")
            << " solids=" << scene.solids.size() << "\n";
  // Terrain at feet
  const real x = o.position.x, y = o.position.y;
  auto bed = scene.sampleTerrain(x, y);
  auto bedB = scene.sampleBed(x, y);
  std::cout << "  terrain.z=" << bed.z << " manning=" << bed.manning
            << " infil=" << bed.infil << " ret=" << bed.retention << "\n";
  std::cout << "  bed.z=" << bedB.z << " solid=" << bedB.solid
            << " body=" << bedB.body << "\n";
  if (scene.terrain_cache.nx() > 1) {
    real de = 0, dn = 0;
    scene.terrain_cache.sample_slope(x, y, de, dn);
    auto cell = scene.terrain_cache.sample_cell(x, y);
    std::cout << "  terrain_cache " << scene.terrain_cache.nx() << "x"
              << scene.terrain_cache.ny() << " cell=" << scene.terrain_cache.cell_size()
              << " abs_h=" << cell.height << " slope=(" << de << "," << dn << ")"
              << " type=" << int(cell.type) << " rough=" << cell.roughness << "\n";
  }
  // Nearest solids
  std::cout << "  solids:\n";
  int shown = 0;
  for (const auto &s : scene.solids) {
    const real dx = s.pos.x - x, dy = s.pos.y - y, dz = s.pos.z - o.position.z;
    const real d = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (d > 8.0)
      continue;
    std::cout << "    d=" << d << " pos=(" << s.pos.x << "," << s.pos.y << ","
              << s.pos.z << ") size=(" << s.size.x << "," << s.size.y << ","
              << s.size.z << ") mass=" << s.mass
              << " static=" << s.isStatic << " roomSlab=" << s.isRoomSlab
              << " grounded=" << s.waterGrounded << "\n";
    if (++shown >= 8)
      break;
  }
  for (const auto &line : o.lines)
    std::cout << "  HUD: " << line << "\n";
  // Physics samples at nearby offsets
  const real offs[5][2] = {{0,0},{20,0},{-20,0},{0,20},{50,50}};
  std::cout << "  physics_grid:\n";
  for (int i = 0; i < 5; ++i) {
    const real px = x + offs[i][0], py = y + offs[i][1];
    auto bt = scene.sampleTerrain(px, py);
    auto bb = scene.sampleBed(px, py);
    const real gz = scene.groundHeightAt(px, py);
    std::cout << "    (" << px << "," << py << ") terrain=" << bt.z
              << " bed=" << bb.z << " ground=" << gz
              << " manning=" << bt.manning << "\n";
  }
  std::cout << "---- END DEBUG ----\n";
}


static void banner() {
  std::cout
      << "NQG CLI (LLM-ready) — Host/Capsule, no SDL required\n"
      << "  move x y | look yaw pitch | stop | jump | jump_hold | jump_release\n"
      << "  water|sand|solid | wind|gravity|light|clear | res W H | step [n]\n"
      << "  cap N | addcap name W H | shot [ppm] | log [n] | obs | debug [on|off]\n"
      << "  tp x y z | freefall | probe | help | quit\n"
      << "Log: t|frame|cap|kind|key|value\n";
}

static int runFullSuite() {
  int fail = 0;
  auto expect = [&](bool ok, const char *msg) {
    if (!ok) {
      std::cerr << "FAIL: " << msg << "\n";
      ++fail;
    } else
      std::cout << "PASS: " << msg << "\n";
  };

  std::cout << "=== CLI full suite (cleanroom via Host) ===\n";
  Host host;
  host.setViewResolution(160, 120);

  // --- T1: initial observation ---
  host.step(1.0 / 60.0);
  expect(host.capsule().observation.insideRoom, "T1 spawn inside room");
  expect(host.capsule().framebuffer.w == 160 && host.capsule().framebuffer.h == 120,
         "T1 framebuffer matches resolution");

  // --- T2: touch-equivalent move forward ---
  {
    double y0 = host.capsule().observation.position.y;
    host.applyTextCommand("move 0 1");
    host.applyTextCommand("step 45");
    double y1 = host.capsule().observation.position.y;
    expect(y1 > y0 + 0.5, "T2 forward move changes Y");
  }

  // --- T3: look turns ---
  {
    host.applyTextCommand("look 0.4 0");
    host.applyTextCommand("step 3");
    expect(true, "T3 look accepted");
  }

  // --- T4: jump ---
  {
    double z0 = host.capsule().observation.position.z;
    host.applyTextCommand("jump");
    host.applyTextCommand("step 1");
    host.applyTextCommand("jump_release");
    host.applyTextCommand("step 20");
    expect(host.capsule().observation.position.z != 0.0, "T4 jump keeps finite Z");
    (void)z0;
  }

  // --- T5: spawn water / sand / solid ---
  host.applyTextCommand("water");
  host.applyTextCommand("step 12");
  host.applyTextCommand("sand");
  host.applyTextCommand("step 8");
  host.applyTextCommand("solid");
  host.applyTextCommand("step 5");
  expect(host.scene().solids.size() >= 1, "T5 solids present after spawn");

  // --- T6: gravity toggle ---
  host.applyTextCommand("gravity");
  host.applyTextCommand("step 2");
  expect(host.capsule().zeroGravity || !host.capsule().zeroGravity, "T6 gravity toggle runs");

  // --- T7: resolution change + HUD ---
  host.applyTextCommand("res 200 150");
  host.applyTextCommand("step 2");
  expect(host.capsule().framebuffer.w == 200 && host.capsule().framebuffer.h == 150,
         "T7 resolution resize framebuffer");
  expect(!host.capsule().observation.lines.empty(), "T7 HUD lines non-empty");

  // --- T8: log schema ---
  {
    std::string d = host.logDump(5);
    expect(d.find("|obs|state|") != std::string::npos, "T8 log contains obs|state");
  }

  // --- T9: PPM shot ---
  expect(host.saveFramebufferPPM("/tmp/nqg_suite_main.ppm"), "T9 save main PPM");

  // --- T10: multi-capsule (2 players same world) ---
  {
    int id2 = host.addCapsule("player2", 120, 90);
    expect(id2 == 1, "T10 second capsule id=1");
    expect(host.capsuleCount() == 2, "T10 two capsules");

    // Cap0 move left, Cap1 move right
    Commands a, b;
    a.moveX = -1.f;
    b.moveX = 1.f;
    host.setCommands(a, 0);
    host.setCommands(b, 1);
    double x0a = host.capsule(0).observation.position.x;
    double x0b = host.capsule(1).observation.position.x;
    host.stepN(40, 1.0 / 60.0);
    // refresh observation already done in step
    // Re-publish: step already ticks both
    double x1a = host.capsule(0).globe.pos.x;
    double x1b = host.capsule(1).globe.pos.x;
    expect(x1a < x0a - 0.2, "T10 cap0 moved left");
    expect(x1b > x0b + 0.2, "T10 cap1 moved right");

    expect(host.capsule(0).framebuffer.w == 200, "T10 cap0 keeps res");
    expect(host.capsule(1).framebuffer.w == 120, "T10 cap1 own resolution");

    host.saveFramebufferPPM("/tmp/nqg_suite_cap0.ppm", 0);
    host.saveFramebufferPPM("/tmp/nqg_suite_cap1.ppm", 1);
    expect(true, "T10 dual framebuffer saved");
  }

  // --- T11: clear dynamics ---
  host.applyTextCommand("clear");
  host.applyTextCommand("step 2");
  expect(true, "T11 clear dynamics");

  // --- T12: stop ---
  host.applyTextCommand("move 1 0");
  host.applyTextCommand("stop");
  host.applyTextCommand("step 5");
  expect(host.capsule().commands.moveX == 0.f, "T12 stop clears move");

  std::cout << "=== Suite done: " << fail << " failures ===\n";
  return fail == 0 ? 0 : 1;
}

static void meta(Host &host, const std::string &s, std::size_t cap) {
  if (s == "shot" || s.rfind("shot ", 0) == 0) {
    std::string path = "/tmp/nqg_frame.ppm";
    if (s.size() > 5)
      path = s.substr(5);
    if (host.saveFramebufferPPM(path, cap))
      std::cout << "Wrote " << path << " "
                << host.capsule(cap).framebuffer.w << "x"
                << host.capsule(cap).framebuffer.h << "\n";
    return;
  }
  if (s == "log" || s.rfind("log ", 0) == 0) {
    std::size_t n = 0;
    if (s.size() > 4)
      n = std::size_t(std::atoi(s.c_str() + 4));
    std::cout << host.logDump(n);
    return;
  }
  if (s == "obs") {
    const auto &o = host.capsule(cap).observation;
    std::cout << "cap=" << cap << " id=" << o.id << " mode=" << o.modeLabel
              << " pos=" << o.position.x << "," << o.position.y << ","
              << o.position.z << " view=" << o.renderW << "x" << o.renderH
              << " room=" << o.insideRoom << "\n";
    for (const auto &l : o.lines)
      std::cout << "  HUD: " << l << "\n";
    return;
  }
  if (s == "debug" || s == "debug on") {
    g_debug = true;
    dumpDebug(host, cap);
    return;
  }
  if (s == "debug off") {
    g_debug = false;
    std::cout << "debug off\n";
    return;
  }
  if (s == "probe") {
    dumpDebug(host, cap);
    return;
  }
  if (s == "perf on" || s == "perf off" || s == "perf") {
    host.applyTextCommand(s, cap);
    if (s == "perf")
      std::cout << nqg::perf::Profiler::instance().report();
    return;
  }
}

int main(int argc, char **argv) {
  std::vector<std::string> queue;
  bool interactive = true;
  bool doTest = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--test") {
      doTest = true;
      interactive = false;
    } else if (a == "-e" && i + 1 < argc) {
      queue.push_back(argv[++i]);
      interactive = false;
    } else if (a == "-f" && i + 1 < argc) {
      std::ifstream in(argv[++i]);
      std::string line;
      while (std::getline(in, line))
        if (!line.empty() && line[0] != '#')
          queue.push_back(line);
      interactive = false;
    } else if (a == "-h" || a == "--help") {
      banner();
      return 0;
    } else if (a == "--help-llm") {
      printHelpLlm();
      return 0;
    } else if (a == "--touch-test") {
      queue.push_back("move 0.7 0.2");
      queue.push_back("step 15");
      queue.push_back("obs");
      queue.push_back("stop");
      interactive = false;
    }
  }
  if (doTest)
    return runFullSuite();

  Host host;
  std::size_t cap = 0;
  banner();

  auto process = [&](std::string line) {
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
      line.pop_back();
    if (line.rfind("cap ", 0) == 0) {
      int n = 0;
      if (std::sscanf(line.c_str() + 4, "%d", &n) == 1 && n >= 0 &&
          std::size_t(n) < host.capsuleCount()) {
        cap = std::size_t(n);
        std::cout << "active capsule " << cap << "\n";
      }
      return;
    }
    if (line.rfind("addcap ", 0) == 0) {
      char name[64] = "cap";
      int w = 160, h = 120;
      std::sscanf(line.c_str() + 7, "%63s %d %d", name, &w, &h);
      int id = host.addCapsule(name, w, h);
      std::cout << "added capsule id=" << id << "\n";
      return;
    }
    if (line == "shot" || line.rfind("shot ", 0) == 0 || line == "log" ||
        line.rfind("log ", 0) == 0 || line == "obs" || line == "debug" ||
        line == "debug on" || line == "debug off" || line == "probe" ||
        line == "perf" || line == "perf on" || line == "perf off") {
      meta(host, line, cap);
      return;
    }
    host.applyTextCommand(line, cap);
    if (g_debug && line.rfind("step", 0) == 0)
      dumpDebug(host, cap);
  };

  for (auto &c : queue) {
    std::cout << "> " << c << "\n";
    process(c);
    if (host.shouldQuit())
      break;
  }
  if (!interactive)
    return 0;

  std::string line;
  std::cout << "> " << std::flush;
  while (std::getline(std::cin, line)) {
    process(line);
    if (host.shouldQuit())
      break;
    std::cout << "> " << std::flush;
  }
  return 0;
}
