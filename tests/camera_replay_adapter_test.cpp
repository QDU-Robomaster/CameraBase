#include <cstdint>
#include <cstdlib>
#include <type_traits>

#include "AutoAimReplayBenchmark.hpp"

using ExpectedRecordPipelineTiming = void (*)(uint64_t, int64_t, int64_t, int64_t,
                                              int64_t, int64_t, int64_t, int64_t, int64_t,
                                              int64_t, int64_t, int64_t, int64_t, int64_t,
                                              int64_t, int64_t, int64_t, uint32_t,
                                              uint32_t, uint64_t, bool, uint64_t);
using ExpectedRecordAsyncPipelineTiming = void (*)(uint64_t, uint64_t, uint64_t, int64_t,
                                                   int64_t, uint64_t, uint32_t, uint32_t,
                                                   uint32_t, uint32_t, bool, int64_t,
                                                   int64_t, int64_t, int64_t, uint64_t,
                                                   int64_t);
using ExpectedRecordDetectorPipelineCounters = void (*)(uint64_t, uint64_t, uint64_t,
                                                        uint64_t, uint64_t, uint64_t,
                                                        uint64_t, uint64_t, uint64_t,
                                                        uint64_t);
using ExpectedRecordTrackerQueueAdmission = void (*)(uint64_t, uint64_t, double, bool,
                                                     uint32_t, uint32_t, uint32_t);
using ExpectedRecordTrackerWorkerService = void (*)(uint64_t, uint64_t, uint64_t, double);

static_assert(std::is_same_v<decltype(&AutoAimReplayBenchmark::RecordPipelineTiming),
                             ExpectedRecordPipelineTiming>);
static_assert(std::is_same_v<decltype(&AutoAimReplayBenchmark::RecordAsyncPipelineTiming),
                             ExpectedRecordAsyncPipelineTiming>);
static_assert(
    std::is_same_v<decltype(&AutoAimReplayBenchmark::RecordDetectorPipelineCounters),
                   ExpectedRecordDetectorPipelineCounters>);
static_assert(
    std::is_same_v<decltype(&AutoAimReplayBenchmark::RecordTrackerQueueAdmission),
                   ExpectedRecordTrackerQueueAdmission>);
static_assert(
    std::is_same_v<decltype(&AutoAimReplayBenchmark::RecordTrackerWorkerService),
                   ExpectedRecordTrackerWorkerService>);
static_assert(
    std::is_same_v<decltype(&AutoAimReplayBenchmark::RecordPipelineNoFree), void (*)()>);
static_assert(std::is_same_v<decltype(&AutoAimReplayBenchmark::PipelineNoFreeCount),
                             uint64_t (*)()>);
static_assert(!AutoAimReplayBenchmark::Enabled());
static_assert(AutoAimReplayBenchmark::PipelineNoFreeCount() == 0U);

int main()
{
  AutoAimReplayBenchmark::RecordPipelineTiming(
      uint64_t{}, int64_t{}, int64_t{}, int64_t{}, int64_t{}, int64_t{}, int64_t{},
      int64_t{}, int64_t{}, int64_t{}, int64_t{}, int64_t{}, int64_t{}, int64_t{},
      int64_t{}, int64_t{}, int64_t{}, uint32_t{}, uint32_t{}, uint64_t{}, false,
      uint64_t{});
  AutoAimReplayBenchmark::RecordAsyncPipelineTiming(
      uint64_t{}, uint64_t{}, uint64_t{}, int64_t{}, int64_t{}, uint64_t{}, uint32_t{},
      uint32_t{}, uint32_t{}, uint32_t{}, false, int64_t{}, int64_t{}, int64_t{},
      int64_t{}, uint64_t{}, int64_t{});
  AutoAimReplayBenchmark::RecordPipelineNoFree();
  AutoAimReplayBenchmark::RecordDetectorPipelineCounters(
      uint64_t{}, uint64_t{}, uint64_t{}, uint64_t{}, uint64_t{}, uint64_t{}, uint64_t{},
      uint64_t{}, uint64_t{}, uint64_t{});
  AutoAimReplayBenchmark::RecordTrackerQueueAdmission(uint64_t{}, uint64_t{}, 0.0, false,
                                                      uint32_t{}, uint32_t{}, uint32_t{});
  AutoAimReplayBenchmark::RecordTrackerWorkerService(uint64_t{}, uint64_t{}, uint64_t{},
                                                     0.0);
  return EXIT_SUCCESS;
}
