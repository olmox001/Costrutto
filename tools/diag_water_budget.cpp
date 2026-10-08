// Diagnostica bilancio acqua: dove va il volume nel tempo, per canale.
#include "nqg_engine_api.hpp"
#include <cstdio>
using nqg::api::Host;
int main(int argc, char **argv) {
  int frames = argc > 1 ? std::atoi(argv[1]) : 60;
  Host h;
  auto &sc = h.scene();
  auto &F = sc.water.flow;
  auto line = [&](const char *tag) {
    const double fl = F.totalVolume(), sp = sc.spray.volume();
    const auto &a = F.absorbedBy;
    std::printf("%-6s t=%.2f flow=%.4f spray=%.4f absorbed=%.4f | infil=%.4f ret=%.4f compact=%.4f bedRise=%.4f noNb=%.4f | sum=%.4f wet=%zu\n",
      tag, h.simTime(), fl, sp, F.absorbedVolume, a.infiltration, a.retention,
      a.compaction, a.bedRise, a.noNeighbour, fl + sp + F.absorbedVolume, F.wetCells());
  };
  line("t0");
  for (int i = 0; i < frames; i += 10) { h.stepN(10, 1.0/60.0); line("step"); }
  return 0;
}
