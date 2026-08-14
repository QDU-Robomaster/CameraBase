#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "CameraBase.hpp"

namespace
{
constexpr CameraTypes::FrameLayout kLayout{2, 2, 6, CameraTypes::Encoding::BGR8};
using Camera = CameraBase<kLayout>;
using Frame = Camera::ImageFrame;
using Sink = CameraBaseDetail::ImageSinkState<Frame>;

void Expect(bool condition, const char* label)
{
  if (!condition)
  {
    std::cerr << label << '\n';
    std::exit(EXIT_FAILURE);
  }
}

struct CommitContext
{
  Sink* sink{};
  Frame* committed{};
  Frame* next{};
  uint64_t assigned_token{};
  uint64_t token_on_entry{};
  uint32_t call_count{};
  bool ready_during_callback{true};
  Frame* writable_during_callback{};
  CameraBaseDetail::ImageCommitResult recursive_commit_result{
      CameraBaseDetail::ImageCommitResult::COMMITTED};
};

void CommitAdapter(bool, CommitContext* context, Frame*& next_image)
{
  ++context->call_count;
  context->ready_during_callback = context->sink->Ready();
  context->writable_during_callback = context->sink->WritableImage();
  context->recursive_commit_result = context->sink->Commit();
  context->token_on_entry = context->committed->publish_token;
  context->committed->publish_token = context->assigned_token;
  next_image = context->next;
}

void TestRegistrationPublishesInitialLease()
{
  Sink sink;
  Frame initial{};
  initial.publish_token = 91U;
  CommitContext context{.sink = &sink, .committed = &initial, .next = &initial};
  const auto callback = Sink::CommitCallback::Create(CommitAdapter, &context);

  Expect(sink.Register(nullptr, callback) ==
             CameraBaseDetail::ImageSinkRegistrationResult::INVALID_ARGUMENT,
         "null initial image must be rejected");
  Expect(sink.Register(&initial, {}) ==
             CameraBaseDetail::ImageSinkRegistrationResult::INVALID_ARGUMENT,
         "empty callback must be rejected");
  Expect(sink.Register(&initial, callback) ==
             CameraBaseDetail::ImageSinkRegistrationResult::REGISTERED,
         "valid sink registration must succeed");
  Expect(sink.Ready(), "registered sink must be ready");
  Expect(sink.WritableImage() == &initial, "initial lease must be published");
  Expect(initial.publish_token == 0U,
         "producer-owned initial lease must have token zero");
  Expect(sink.Register(&initial, callback) ==
             CameraBaseDetail::ImageSinkRegistrationResult::ALREADY_REGISTERED,
         "sink registration must be one-shot");
}

void TestCommitRotatesLeaseSynchronously()
{
  Sink sink;
  Frame first{};
  Frame second{};
  second.publish_token = 92U;
  CommitContext context{
      .sink = &sink,
      .committed = &first,
      .next = &second,
      .assigned_token = 41U,
  };
  const auto callback = Sink::CommitCallback::Create(CommitAdapter, &context);
  Expect(sink.Register(&first, callback) ==
             CameraBaseDetail::ImageSinkRegistrationResult::REGISTERED,
         "rotation registration must succeed");

  first.publish_token = 99U;
  Expect(sink.Commit() == CameraBaseDetail::ImageCommitResult::COMMITTED,
         "commit must acquire the next lease");
  Expect(context.call_count == 1U, "commit callback must run exactly once");
  Expect(!context.ready_during_callback,
         "producer write access must be revoked during callback");
  Expect(context.writable_during_callback == nullptr,
         "callback must not observe a writable producer slot");
  Expect(
      context.recursive_commit_result == CameraBaseDetail::ImageCommitResult::NOT_READY,
      "recursive commit must fail without invoking the callback again");
  Expect(context.token_on_entry == 0U,
         "CameraBase must clear the producer token before sink publication");
  Expect(first.publish_token == 41U, "published frame token must remain intact");
  Expect(second.publish_token == 0U, "next producer lease must have token zero");
  Expect(sink.WritableImage() == &second, "next lease must become writable");
}

void TestDroppedFrameMayReuseCommittedSlot()
{
  Sink sink;
  Frame frame{};
  CommitContext context{
      .sink = &sink,
      .committed = &frame,
      .next = &frame,
      .assigned_token = 42U,
  };
  const auto callback = Sink::CommitCallback::Create(CommitAdapter, &context);
  Expect(sink.Register(&frame, callback) ==
             CameraBaseDetail::ImageSinkRegistrationResult::REGISTERED,
         "reuse registration must succeed");
  Expect(sink.Commit() == CameraBaseDetail::ImageCommitResult::COMMITTED,
         "same-slot drop must still return a writable lease");
  Expect(context.call_count == 1U, "same-slot callback must run exactly once");
  Expect(sink.WritableImage() == &frame, "dropped slot must be reusable");
  Expect(frame.publish_token == 0U, "reused producer slot must clear stale token");
}

void TestNullNextLeaseFailsClosed()
{
  Sink sink;
  Frame frame{};
  CommitContext context{
      .sink = &sink,
      .committed = &frame,
      .next = nullptr,
      .assigned_token = 43U,
  };
  const auto callback = Sink::CommitCallback::Create(CommitAdapter, &context);
  Expect(sink.Register(&frame, callback) ==
             CameraBaseDetail::ImageSinkRegistrationResult::REGISTERED,
         "null-next registration must succeed");
  Expect(sink.Commit() == CameraBaseDetail::ImageCommitResult::NO_WRITABLE_IMAGE,
         "null next lease must be reported");
  Expect(context.call_count == 1U, "null-next callback must run exactly once");
  Expect(!sink.Ready(), "null next lease must leave the producer disabled");
  Expect(sink.WritableImage() == nullptr,
         "committed slot must not be exposed again after null handoff");
  Expect(sink.Commit() == CameraBaseDetail::ImageCommitResult::NOT_READY,
         "failed-closed sink must reject another commit");
  Expect(context.call_count == 1U,
         "failed-closed commit must not invoke the callback again");
}

void TestRegistrationPublicationAcrossThreads()
{
  Sink sink;
  Frame frame{};
  frame.publish_token = 44U;
  CommitContext context{.sink = &sink, .committed = &frame, .next = &frame};
  const auto callback = Sink::CommitCallback::Create(CommitAdapter, &context);
  std::atomic<bool> observed{false};

  std::thread producer(
      [&]()
      {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!sink.Ready() && std::chrono::steady_clock::now() < deadline)
        {
          std::this_thread::yield();
        }
        observed.store(
            sink.Ready() && sink.WritableImage() == &frame && frame.publish_token == 0U,
            std::memory_order_release);
      });
  Expect(sink.Register(&frame, callback) ==
             CameraBaseDetail::ImageSinkRegistrationResult::REGISTERED,
         "cross-thread registration must succeed");
  producer.join();
  Expect(observed.load(std::memory_order_acquire),
         "ready publication must expose initialized lease state");
}
}  // namespace

int main()
{
  TestRegistrationPublishesInitialLease();
  TestCommitRotatesLeaseSynchronously();
  TestDroppedFrameMayReuseCommittedSlot();
  TestNullNextLeaseFailsClosed();
  TestRegistrationPublicationAcrossThreads();
  return EXIT_SUCCESS;
}
