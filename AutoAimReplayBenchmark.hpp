#pragma once

#if __has_include("ReplayBenchmark.hpp")
#include "ReplayBenchmark.hpp"
#else

#include <array>
#include <cstdint>
#include <limits>

namespace AutoAimReplayBenchmark
{
inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct DetectionRecord
{
  int color{-1};
  int type{-1};
  int number{-1};
  double confidence{kNaN};
  bool pnp_valid{false};
  double pnp_error_px{kNaN};
  std::array<double, 8> corners{};
  std::array<double, 3> translation{};
};

inline constexpr bool Enabled() { return false; }
inline constexpr bool WaitForPipelineReady() { return true; }
inline constexpr bool WaitForAimer(uint64_t) { return true; }
inline void RecordCaptureStart(uint64_t) {}
inline void RecordCaptureDecode(uint64_t, double, double) {}
inline void RecordCaptureCommit(uint64_t, double) {}
inline void RecordSync(uint64_t) {}
inline void RecordSyncDrop() {}
inline void RecordDetectorStart(uint64_t) {}
inline void RecordDetector(uint64_t, double, double, double, double, double, double,
                           double, uint32_t, uint32_t)
{
}
inline void RecordDetection(uint64_t, DetectionRecord) {}
inline void RecordTrackerEnqueue(uint64_t) {}
inline void RecordTrackerQueued(uint64_t, double) {}
inline void RecordTrackerOverwrite() {}
inline void RecordTrackerStart(uint64_t) {}
inline void RecordTracker(uint64_t, double, bool, int, const std::array<double, 3>&,
                          const std::array<double, 3>&, double, double, double, double,
                          double)
{
}
inline void RecordAimerStart(uint64_t) {}
inline void RecordMpc(uint64_t, bool, bool, bool, int, int, int, int, int, int, int, int,
                      double, double, double, double)
{
}
inline void RecordAimer(uint64_t, double, bool, bool, bool, bool,
                        const std::array<double, 8>&)
{
}
inline void MarkSourceComplete(uint64_t, bool) {}
}  // namespace AutoAimReplayBenchmark

#endif
