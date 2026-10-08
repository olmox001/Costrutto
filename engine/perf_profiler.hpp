/* SPDX-License-Identifier: GPL-2.0-or-later
 * Lightweight performance instrumentation for Host / CLI / engine steps.
 */
#pragma once
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <algorithm>

namespace nqg {
namespace perf {

using clock = std::chrono::steady_clock;

struct ScopeStat {
  uint64_t calls = 0;
  double total_ms = 0;
  double max_ms = 0;
};

class Profiler {
public:
  static Profiler &instance() {
    static Profiler p;
    return p;
  }
  void enabled(bool on) { on_ = on; }
  bool enabled() const { return on_; }
  void begin(const char *name) {
    if (!on_) return;
    std::lock_guard<std::mutex> lock(mu_);
    stack_.push_back({name, clock::now()});
  }
  void end(const char *name) {
    if (!on_) return;
    const auto t1 = clock::now();
    std::lock_guard<std::mutex> lock(mu_);
    if (stack_.empty()) return;
    auto &top = stack_.back();
    if (top.name != name) { stack_.pop_back(); return; }
    const double ms = std::chrono::duration<double, std::milli>(t1 - top.t0).count();
    stack_.pop_back();
    auto &s = stats_[name];
    s.calls++;
    s.total_ms += ms;
    if (ms > s.max_ms) s.max_ms = ms;
  }
  void reset() {
    std::lock_guard<std::mutex> lock(mu_);
    stats_.clear();
  }
  std::string report() const {
    std::lock_guard<std::mutex> lock(mu_);
    std::vector<std::pair<std::string, ScopeStat>> rows(stats_.begin(), stats_.end());
    std::sort(rows.begin(), rows.end(),
              [](const auto &a, const auto &b) { return a.second.total_ms > b.second.total_ms; });
    std::ostringstream os;
    os << "perf_report calls total_ms max_ms name\n";
    for (const auto &r : rows)
      os << r.second.calls << " " << r.second.total_ms << " " << r.second.max_ms
         << " " << r.first << "\n";
    return os.str();
  }
private:
  struct Frame { const char *name; clock::time_point t0; };
  bool on_ = false;
  mutable std::mutex mu_;
  std::vector<Frame> stack_;
  std::unordered_map<std::string, ScopeStat> stats_;
};

struct Scoped {
  const char *name;
  explicit Scoped(const char *n) : name(n) { Profiler::instance().begin(n); }
  ~Scoped() { Profiler::instance().end(name); }
};

} // namespace perf
} // namespace nqg

#define NQG_PERF_SCOPE(name) ::nqg::perf::Scoped nqg_perf_scoped_##__LINE__(name)
