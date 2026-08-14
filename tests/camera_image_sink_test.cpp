#include <atomic>
#include <cstdlib>
#include <iostream>
#include <thread>
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

  Frame* frame_address = first.Get();
  first->data[0] = 37U;
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
  publisher->data[0] = 91U;
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
}  // namespace

int main()
{
  LibXR::PlatformInit();
  TestCopyRetainsOneSlot();
  TestPoolExhaustionAndRecovery();
  TestConcurrentCopiesAndLastRelease();
  TestConcurrentLastReleaseAndReacquire();
  TestTopicCallbackRetainsFrame();
  TestMultipleCallbacksShareOneFrame();

  // PlatformInit 的 STDIO 线程按进程生命周期运行；不要在测试退出时伪造并发 teardown。
  std::_Exit(EXIT_SUCCESS);
}
