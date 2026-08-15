#include <array>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "CameraBase.hpp"
#include "libxr.hpp"

namespace
{
constexpr CameraTypes::FrameLayout kLayout{2, 2, 6, CameraTypes::Encoding::BGR8};
using Camera = CameraBase<kLayout>;
using Frame = Camera::ImageFrame;
using Pool = CameraBaseDetail::SharedObjectPool<Frame, 2U>;
using Handle = Pool::SharedHandle;

static_assert(std::is_same_v<decltype(std::declval<Handle&>().Get()), const Frame*>);
static_assert(
    std::is_same_v<decltype(std::declval<Handle&>().operator->()), const Frame*>);
static_assert(std::is_same_v<decltype(*std::declval<Handle&>()), const Frame&>);

constexpr Camera::CameraCalibration MakeCalibration()
{
  Camera::CameraCalibration calibration{};
  calibration.native_width = 2U;
  calibration.native_height = 2U;
  calibration.camera_matrix = {1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0};
  return calibration;
}

constexpr Camera::FrameGeometry MakeGeometry()
{
  return {
      .width = 2U,
      .height = 2U,
      .step = 6U,
      .roi_offset_x_native = 0U,
      .roi_offset_y_native = 0U,
      .decimation_x = 1U,
      .decimation_y = 1U,
      .flags = CameraTypes::FRAME_GEOMETRY_NONE,
      .reserved = 0U,
      .sample_phase_x_native = 0.0F,
      .sample_phase_y_native = 0.0F,
  };
}

constexpr Camera::FrameGeometry MakeNarrowGeometry()
{
  auto geometry = MakeGeometry();
  geometry.flags = CameraTypes::FRAME_GEOMETRY_REVERSE_X;
  return geometry;
}

class TestCamera final : public Camera
{
 public:
  explicit TestCamera(LibXR::HardwareContainer& hw)
      : Camera(hw, MakeCalibration(), "camera_base_profile_test",
               "camera_base_profile_image_test", "camera_base_profile_imu_test")
  {
  }

  void SetExposure(double) override {}
  void SetGain(double) override {}

  std::span<const CameraProfile> Profiles() const noexcept override { return profiles_; }

  LibXR::ErrorCode SwitchProfile(ProfileId id, AppliedProfile& applied) override
  {
    for (const auto& profile : profiles_)
    {
      if (profile.id == id)
      {
        applied = {.id = profile.id, .geometry = profile.geometry};
        return LibXR::ErrorCode::OK;
      }
    }
    return LibXR::ErrorCode::NOT_SUPPORT;
  }

  void DiscardWritableForTest() noexcept { DiscardWritableImage(); }

 private:
  inline static constexpr std::array<CameraProfile, 2U> profiles_{{
      {.id = ProfileId::WIDE, .geometry = MakeGeometry(), .trigger_period_us = 10000U},
      {.id = ProfileId::NARROW,
       .geometry = MakeNarrowGeometry(),
       .trigger_period_us = 5000U},
  }};
};

using ProfilesMethod =
    std::span<const Camera::CameraProfile> (Camera::*)() const noexcept;
using SwitchProfileMethod = LibXR::ErrorCode (Camera::*)(Camera::ProfileId,
                                                         Camera::AppliedProfile&);
static_assert(std::is_same_v<decltype(&Camera::Profiles), ProfilesMethod>);
static_assert(std::is_same_v<decltype(&Camera::SwitchProfile), SwitchProfileMethod>);

void Expect(bool condition, const char* label)
{
  if (!condition)
  {
    std::cerr << label << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void TestCopyRetainsOneSlot()
{
  Pool pool;
  Handle first;
  Expect(pool.Available() == 2U, "new pool must expose every slot");
  Expect(pool.Acquire(first) == LibXR::ErrorCode::OK, "first acquire must succeed");
  Expect(first.Valid(), "acquired handle must be valid");
  Expect(first.UseCount() == 1U, "new handle must own one reference");
  Expect(pool.Acquire(first) == LibXR::ErrorCode::STATE_ERR,
         "acquire must reject an already valid output handle");

  Frame* writable = pool.GetWritable(first);
  Expect(writable != nullptr, "the unique pool owner must expose writable storage");
  writable->data[0] = 37U;
  const Frame* frame_address = first.Get();
  Handle second = first;
  Expect(first.UseCount() == 2U, "copy must increment the reference count");
  Expect(second.Get() == frame_address, "copy must refer to the same image bytes");
  Expect(second->data[0] == 37U, "copy must observe the same image bytes");

  second.Reset();
  Expect(first.UseCount() == 1U, "reset must decrement the reference count");
  Expect(pool.Available() == 1U, "slot must remain borrowed while one owner exists");
  first.Reset();
  Expect(pool.Available() == 2U, "last reset must return the slot");
}

void TestAssignmentBoundaries()
{
  Pool first_pool;
  Pool second_pool;
  Handle destination;
  Handle source;
  Expect(first_pool.Acquire(destination) == LibXR::ErrorCode::OK,
         "assignment destination acquire must succeed");
  Expect(second_pool.Acquire(source) == LibXR::ErrorCode::OK,
         "assignment source acquire must succeed");
  const Frame* destination_address = destination.Get();
  const Frame* source_address = source.Get();

  Handle& destination_alias = destination;
  destination = destination_alias;
  Expect(destination.Get() == destination_address && destination.UseCount() == 1U,
         "copy self-assignment must preserve one owner");
  Handle* destination_pointer = &destination;
  destination = std::move(*destination_pointer);
  Expect(destination.Get() == destination_address && destination.UseCount() == 1U,
         "move self-assignment must preserve one owner");

  destination = source;
  Expect(destination.Get() == source_address && source.UseCount() == 2U,
         "cross-pool copy assignment must retain the source slot");
  Expect(first_pool.Available() == 2U,
         "cross-pool copy assignment must release the old destination slot");
  source.Reset();

  Handle replacement;
  Expect(first_pool.Acquire(replacement) == LibXR::ErrorCode::OK,
         "cross-pool move destination acquire must succeed");
  replacement = std::move(destination);
  Expect(!destination.Valid() && replacement.Get() == source_address,
         "cross-pool move assignment must transfer the source slot");
  Expect(first_pool.Available() == 2U,
         "cross-pool move assignment must release the old destination slot");
  replacement.Reset();
  Expect(second_pool.Available() == 2U,
         "assignment test must return the transferred slot to its source pool");
}

void TestWritableOwnershipBoundary()
{
  Pool pool;
  Pool other_pool;
  Handle owner;
  Handle invalid;
  Expect(pool.Acquire(owner) == LibXR::ErrorCode::OK,
         "writable boundary test acquire must succeed");
  Expect(pool.GetWritable(owner) != nullptr,
         "the originating pool must accept its unique owner");
  Expect(other_pool.GetWritable(owner) == nullptr,
         "a different pool must reject the handle");
  Expect(pool.GetWritable(invalid) == nullptr, "an invalid handle must not be writable");

  Handle shared = owner;
  Expect(pool.GetWritable(owner) == nullptr,
         "a shared slot must not expose writable storage");
  Expect(pool.GetWritable(shared) == nullptr,
         "no copy of a shared slot may expose writable storage");
  shared.Reset();
  Expect(pool.GetWritable(owner) != nullptr,
         "releasing the copy must restore unique writable access");
}

void TestPoolExhaustionAndRecovery()
{
  Pool pool;
  Handle first;
  Handle second;
  Handle unavailable;
  Expect(pool.Acquire(first) == LibXR::ErrorCode::OK, "first slot must be available");
  Expect(pool.Acquire(second) == LibXR::ErrorCode::OK, "second slot must be available");
  Expect(pool.Acquire(unavailable) == LibXR::ErrorCode::EMPTY,
         "acquire must report an exhausted pool");

  first.Reset();
  Expect(pool.Acquire(unavailable) == LibXR::ErrorCode::OK,
         "released slot must become acquirable again");
  Expect(unavailable.Get() != nullptr, "recovered handle must expose valid storage");
}

void TestConcurrentCopiesAndLastRelease()
{
  CameraBaseDetail::SharedObjectPool<Frame, 2U> pool;
  using SingleHandle = CameraBaseDetail::SharedObjectPool<Frame, 2U>::SharedHandle;
  SingleHandle root;
  Expect(pool.Acquire(root) == LibXR::ErrorCode::OK,
         "concurrency test acquire must succeed");

  constexpr std::size_t kThreadCount = 8U;
  constexpr std::size_t kCopyIterations = 2000U;
  std::atomic<bool> start{false};
  std::vector<std::thread> workers;
  workers.reserve(kThreadCount);
  for (std::size_t index = 0; index < kThreadCount; ++index)
  {
    workers.emplace_back(
        [owned = SingleHandle(root), &start]() mutable
        {
          while (!start.load(std::memory_order_acquire))
          {
            std::this_thread::yield();
          }
          for (std::size_t iteration = 0; iteration < kCopyIterations; ++iteration)
          {
            SingleHandle copy = owned;
            Expect(copy.Get() == owned.Get(),
                   "concurrent copy must retain the same slot");
          }
        });
  }

  start.store(true, std::memory_order_release);
  for (auto& worker : workers)
  {
    worker.join();
  }
  Expect(root.UseCount() == 1U,
         "all worker-owned references must be released after join");

  SingleHandle last_owner = std::move(root);
  std::thread final_releaser([owned = std::move(last_owner)]() mutable
                             { owned.Reset(); });
  final_releaser.join();
  Expect(pool.Available() == 2U,
         "last destruction on another thread must return the slot");
}

void TestConcurrentLastReleaseAndReacquire()
{
  Pool pool;
  Handle blocker;
  Expect(pool.Acquire(blocker) == LibXR::ErrorCode::OK,
         "reacquire test blocker must acquire one slot");

  constexpr std::size_t kIterations = 64U;
  for (std::size_t iteration = 0; iteration < kIterations; ++iteration)
  {
    Handle retiring;
    Expect(pool.Acquire(retiring) == LibXR::ErrorCode::OK,
           "reacquire test must borrow the remaining slot");
    std::atomic<bool> release{false};
    std::thread releaser(
        [owned = std::move(retiring), &release]() mutable
        {
          while (!release.load(std::memory_order_acquire))
          {
            std::this_thread::yield();
          }
          owned.Reset();
        });

    release.store(true, std::memory_order_release);
    Handle reacquired;
    LibXR::ErrorCode result = LibXR::ErrorCode::EMPTY;
    while ((result = pool.Acquire(reacquired)) == LibXR::ErrorCode::EMPTY)
    {
      std::this_thread::yield();
    }
    Expect(result == LibXR::ErrorCode::OK,
           "slot must become acquirable after concurrent last release");
    releaser.join();
    reacquired.Reset();
  }

  blocker.Reset();
  Expect(pool.Available() == 2U, "reacquire loop must return every slot to the pool");
}

struct TopicContext
{
  Handle retained{};
  const Frame* frame_address{};
  uint32_t call_count{};
};

void RetainFrame(bool, TopicContext* context, const Handle* borrowed)
{
  Expect(borrowed != nullptr, "topic callback must receive a borrowed handle pointer");
  Expect(borrowed->Valid(), "borrowed handle must be valid during callback");
  ++context->call_count;
  context->frame_address = borrowed->Get();
  context->retained = *borrowed;
}

void TestTopicCallbackRetainsFrame()
{
  Pool pool;
  Handle publisher;
  Expect(pool.Acquire(publisher) == LibXR::ErrorCode::OK,
         "topic publisher acquire must succeed");
  Frame* writable = pool.GetWritable(publisher);
  Expect(writable != nullptr, "topic publisher must initially own writable storage");
  writable->data[0] = 91U;
  const Frame* published_address = publisher.Get();

  using Payload = const Handle*;
  LibXR::Topic topic(
      LibXR::Topic::FindOrCreate<Payload>("camera_shared_frame_single_owner_test"));
  TopicContext context;
  auto callback = LibXR::Topic::Callback::Create(RetainFrame, &context);
  topic.RegisterCallback(callback);

  Payload message = &publisher;
  topic.Publish(message);
  Expect(context.call_count == 1U, "topic callback must run synchronously");
  Expect(context.retained.Valid(), "callback copy must retain frame ownership");
  Expect(context.frame_address == published_address,
         "topic delivery must not copy image storage");
  Expect(context.retained->data[0] == 91U, "retained handle must preserve image bytes");
  Expect(publisher.UseCount() == 2U,
         "publisher and callback must each own one reference");

  publisher.Reset();
  Expect(pool.Available() == 1U,
         "callback owner must keep exactly one pool slot borrowed");
  Expect(context.retained->data[0] == 91U,
         "callback owner must outlive the publisher handle");
  context.retained.Reset();
  Expect(pool.Available() == 2U, "slot must return after the callback owner releases it");
}

void TestMultipleCallbacksShareOneFrame()
{
  Pool pool;
  Handle publisher;
  Expect(pool.Acquire(publisher) == LibXR::ErrorCode::OK,
         "fan-out publisher acquire must succeed");
  const Frame* published_address = publisher.Get();

  using Payload = const Handle*;
  LibXR::Topic topic(
      LibXR::Topic::FindOrCreate<Payload>("camera_shared_frame_fanout_test"));
  TopicContext first;
  TopicContext second;
  auto first_callback = LibXR::Topic::Callback::Create(RetainFrame, &first);
  auto second_callback = LibXR::Topic::Callback::Create(RetainFrame, &second);
  topic.RegisterCallback(first_callback);
  topic.RegisterCallback(second_callback);

  Payload message = &publisher;
  topic.Publish(message);
  Expect(first.call_count == 1U && second.call_count == 1U,
         "each callback must run once");
  Expect(first.frame_address == published_address &&
             second.frame_address == published_address,
         "all callbacks must share the same image storage");
  Expect(publisher.UseCount() == 3U,
         "publisher and two callbacks must own three references");

  publisher.Reset();
  first.retained.Reset();
  Expect(pool.Available() == 1U, "remaining callback must keep the shared slot borrowed");
  second.retained.Reset();
  Expect(pool.Available() == 2U, "last callback release must return the shared slot");
}

void TestProfileInterface(TestCamera& camera)
{
  const auto first_view = camera.Profiles();
  const auto second_view = camera.Profiles();
  Expect(!first_view.empty(), "profile table must not be empty");
  Expect(
      first_view.data() == second_view.data() && first_view.size() == second_view.size(),
      "profile table storage and order must remain stable");
  Expect(first_view.size() == 2U, "test camera must expose both fixed profiles");
  Expect(first_view[0].id == Camera::ProfileId::WIDE,
         "first profile must describe the construction-time profile");
  Expect(first_view[0].id != first_view[1].id, "profile ids must be unique");
  Expect(first_view[0].trigger_period_us != 0U && first_view[1].trigger_period_us != 0U,
         "profile trigger periods must be non-zero");

  Camera::AppliedProfile applied{};
  Expect(camera.SwitchProfile(Camera::ProfileId::WIDE, applied) == LibXR::ErrorCode::OK,
         "switching to the current profile must succeed");
  Expect(applied.id == Camera::ProfileId::WIDE,
         "same-profile switch must fill the applied result");
  Expect(CameraTypes::SameFrameGeometry(applied.geometry, first_view[0].geometry),
         "same-profile switch must report the applied geometry");
  Expect(camera.SwitchProfile(Camera::ProfileId::WIDE, applied) == LibXR::ErrorCode::OK,
         "repeating a same-profile switch must succeed");

  Expect(camera.SwitchProfile(Camera::ProfileId::NARROW, applied) == LibXR::ErrorCode::OK,
         "supported profile switch must succeed");
  Expect(applied.id == Camera::ProfileId::NARROW,
         "supported switch must report the selected id");
  Expect(CameraTypes::SameFrameGeometry(applied.geometry, first_view[1].geometry),
         "supported switch must report the selected geometry");

  const Camera::AppliedProfile before_failure = applied;
  Expect(camera.SwitchProfile(static_cast<Camera::ProfileId>(UINT8_MAX), applied) ==
             LibXR::ErrorCode::NOT_SUPPORT,
         "unsupported profile must return NOT_SUPPORT");
  Expect(applied.id == before_failure.id &&
             CameraTypes::SameFrameGeometry(applied.geometry, before_failure.geometry),
         "failed switch must leave the applied output unchanged");
}

void TestDiscardWritableImage(TestCamera& camera)
{
  auto* writable = camera.GetWritableImage();
  Expect(writable != nullptr, "camera must start with one writable slot");
  Expect(camera.AvailableImageSlots() == Camera::image_slot_count - 1U,
         "the producer-owned writable slot must not count as available");
  writable->geometry = MakeGeometry();
  writable->data[0] = 77U;

  camera.DiscardWritableForTest();
  Expect(camera.AvailableImageSlots() == Camera::image_slot_count,
         "discard must immediately return the uncommitted slot");
  camera.DiscardWritableForTest();
  Expect(camera.AvailableImageSlots() == Camera::image_slot_count,
         "discard without a writable slot must be a no-op");

  writable = camera.GetWritableImage();
  Expect(writable != nullptr, "get writable must reacquire a slot after discard");
  writable->geometry = MakeNarrowGeometry();
  writable->data[0] = 91U;
  Expect(CameraTypes::SameFrameGeometry(writable->geometry, MakeNarrowGeometry()) &&
             writable->data[0] == 91U,
         "the reacquired slot must remain writable through CameraBase");
  Expect(camera.AvailableImageSlots() == Camera::image_slot_count - 1U,
         "reacquired slot must again be held by the producer");
}
}  // namespace

int main()
{
  LibXR::PlatformInit();
  TestCopyRetainsOneSlot();
  TestAssignmentBoundaries();
  TestWritableOwnershipBoundary();
  TestPoolExhaustionAndRecovery();
  TestConcurrentCopiesAndLastRelease();
  TestConcurrentLastReleaseAndReacquire();
  TestTopicCallbackRetainsFrame();
  TestMultipleCallbacksShareOneFrame();

  LibXR::RamFS ramfs;
  LibXR::HardwareContainer hw(LibXR::Entry<LibXR::RamFS>{ramfs, {"ramfs"}});
  TestCamera camera(hw);
  TestProfileInterface(camera);
  TestDiscardWritableImage(camera);

  // PlatformInit 的 STDIO 线程按进程生命周期运行；不要在测试退出时伪造并发 teardown。
  std::_Exit(EXIT_SUCCESS);
}
