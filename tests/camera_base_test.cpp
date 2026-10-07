#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>

#include "CameraBase.hpp"
#include "libxr.hpp"

namespace
{
void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

constexpr CameraTypes::CameraCalibration CALIBRATION{
    1440,
    1080,
    2328.69,
    2328.67,
    733.36,
    540.62,
    {-0.0918, 0.4640, 0.0026, 0.0010, -0.4751}};

class FakeCamera : public CameraBase
{
 public:
  FakeCamera(const char* name, SlotPolicy policy)
      : CameraBase(CALIBRATION, {0.5, 0.5}, name, policy)
  {
    StartCapture();
  }
  ~FakeCamera() override { StopCapture(); }

  /// 先停采集，再让订阅者放掉帧，最后析构相机（池须比句柄活得久）。
  void Stop() { StopCapture(); }
  CameraTypes::FrameGeometry Geometry() const { return CurrentGeometry(); }

  std::atomic<uint32_t> grabs{0};
  std::atomic<uint32_t> after_publish{0};
  std::atomic<uint32_t> last_published_counter{0};
  std::vector<CameraTypes::FrameGeometry> applied;
  std::vector<CameraTypes::FrameGeometry> moved;
  /// 模拟外触发停下：取图阻塞在 GrabFrame 里 / Simulates a stopped trigger: the grab
  /// blocks inside GrabFrame.
  std::atomic<bool> paused{false};
  std::atomic<bool> waiting{false};

 protected:
  bool GrabFrame(ImageFrame& frame) override
  {
    while (paused.load() && CaptureRunning())
    {
      waiting.store(true);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    waiting.store(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const uint32_t n = grabs.fetch_add(1);
    frame.frame_counter = n;
    frame.timestamp_us = LibXR::MicrosecondTimestamp(1000ULL * (n + 1));
    frame.data[0] = static_cast<uint8_t>(n);
    return true;
  }

  LibXR::ErrorCode ApplyView(const CameraTypes::FrameGeometry& geometry) override
  {
    applied.push_back(geometry);
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode ApplyOffset(const CameraTypes::FrameGeometry& geometry) override
  {
    moved.push_back(geometry);
    return LibXR::ErrorCode::OK;
  }

  void OnPublished(const SharedFrame& frame) override
  {
    Expect(frame.Valid(), "OnPublished receives the published frame");
    last_published_counter.store(frame->frame_counter);
    after_publish.fetch_add(1);
  }
};

// 订阅者：在回调里复制句柄，可选择一直攥着不放。LibXR 的回调注册后不能注销，订阅者
// 必须活到进程结束，所以测试里用 Make() 在堆上创建且不释放。
struct Subscriber
{
  std::mutex mutex;
  std::vector<SharedFrame> held;
  std::atomic<bool> keep{false};
  std::atomic<uint32_t> received{0};

  explicit Subscriber(const char* topic_name)
  {
    LibXR::Topic topic(LibXR::Topic::Find(topic_name));
    auto callback = LibXR::Topic::Callback::Create(
        [](bool, Subscriber* self, ImageTopicPayload payload)
        {
          Expect(payload != nullptr && payload->Valid(),
                 "published frame holds an image");
          std::lock_guard<std::mutex> lock(self->mutex);
          self->received.fetch_add(1);
          if (self->keep)
          {
            self->held.push_back(*payload);
          }
          else
          {
            self->held.assign(1, *payload);  // 只留最新一帧 / keep only the latest
          }
        },
        this);
    topic.RegisterCallback(callback);
  }

  static Subscriber& Make(const char* topic_name) { return *new Subscriber(topic_name); }

  void Clear()
  {
    std::lock_guard<std::mutex> lock(mutex);
    held.clear();
  }

  SharedFrame Latest()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return held.empty() ? SharedFrame{} : held.back();
  }
};

void WaitFor(const std::atomic<uint32_t>& counter, uint32_t target)
{
  for (int i = 0; i < 2000 && counter.load() < target; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  Expect(counter.load() >= target, "timed out waiting");
}

void TestPublishStampsGeometryAndCalibration()
{
  FakeCamera camera("cam_a", CameraBase::SlotPolicy::DROP);
  Expect(LibXR::Topic::Find("cam_a_image") != nullptr, "camera creates <name>_image");
  Subscriber& sub = Subscriber::Make("cam_a_image");
  WaitFor(sub.received, 5);
  SharedFrame frame = sub.Latest();
  Expect(frame->geometry == CameraBase::WIDE_GEOMETRY, "WIDE geometry stamped");
  Expect(frame->calibration == &camera.Calibration(),
         "calibration pointer is the camera's");
  camera.Stop();
  // 每帧发布之后调用一次 OnPublished / OnPublished runs once after every publication.
  Expect(camera.after_publish.load() == sub.received.load(), "OnPublished per frame");
  Expect(camera.last_published_counter.load() == sub.Latest()->frame_counter,
         "OnPublished sees the frame just published");
  sub.Clear();
}

void TestSwitchView()
{
  FakeCamera camera("cam_b", CameraBase::SlotPolicy::DROP);
  Subscriber& sub = Subscriber::Make("cam_b_image");
  WaitFor(sub.received, 3);
  Expect(camera.SwitchView(View::NARROW) == LibXR::ErrorCode::OK, "switch ok");
  const CameraTypes::FrameGeometry narrow{400, 284, 1};
  Expect(camera.applied.size() == 1 && camera.applied[0] == narrow, "narrow centred");
  const uint32_t after_switch = sub.received.load();
  WaitFor(sub.received, after_switch + 3);
  Expect(sub.Latest()->geometry == narrow, "frames after the switch carry NARROW");
  Expect(camera.SwitchView(View::NARROW) == LibXR::ErrorCode::OK, "same view is a no-op");
  Expect(camera.applied.size() == 1, "no hardware call for the same view");
  camera.Stop();
  sub.Clear();
}

void TestMoveNarrow()
{
  FakeCamera camera("cam_m", CameraBase::SlotPolicy::DROP);
  Subscriber& sub = Subscriber::Make("cam_m_image");
  WaitFor(sub.received, 3);
  // WIDE 下只记下位置，不动硬件 / In WIDE only the position is stored.
  Expect(camera.MoveNarrow({0.0, 0.0}) == LibXR::ErrorCode::OK, "move in WIDE ok");
  Expect(camera.moved.empty(), "no hardware move in WIDE");
  const CameraTypes::FrameGeometry corner{0, 0, 1};
  Expect(camera.NarrowGeometryNow() == corner, "stored NARROW window");
  Expect(camera.SwitchView(View::NARROW) == LibXR::ErrorCode::OK, "switch ok");
  Expect(camera.applied.back() == corner, "switch uses the stored window");
  // 取图阻塞时移动窗口：恢复后发布的帧都是新窗口 / Move while the grab is blocked:
  // every frame published after resuming carries the new window.
  camera.paused.store(true);
  while (!camera.waiting.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const uint32_t before = sub.received.load();
  Expect(camera.MoveNarrow({1.0, 1.0}) == LibXR::ErrorCode::OK, "move in NARROW ok");
  const CameraTypes::FrameGeometry far{800, 568, 1};
  Expect(camera.moved.size() == 1 && camera.moved[0] == far, "offset written");
  Expect(camera.Geometry() == far, "current geometry follows the move");
  camera.paused.store(false);
  WaitFor(sub.received, before + 3);
  Expect(sub.Latest()->geometry == far, "frames after the move carry the new window");
  Expect(camera.MoveNarrow({1.0, 1.0}) == LibXR::ErrorCode::OK, "same window is a no-op");
  Expect(camera.moved.size() == 1, "no hardware call for the same window");
  camera.Stop();
  sub.Clear();
  // 原生 (720, 540) 居中 / Native (720, 540) is the centre.
  const NarrowPosition centre = CameraBase::CenteredOn(CALIBRATION, {720.0, 540.0});
  Expect(std::abs(centre.u - 0.5) < 1e-9 && std::abs(centre.v - 0.5) < 1e-9, "centred");
  const NarrowPosition edge = CameraBase::CenteredOn(CALIBRATION, {-50.0, 2000.0});
  Expect(edge.u == 0.0 && edge.v == 1.0, "clamped to the sensor");
}

void TestDropPolicyKeepsGrabbing()
{
  FakeCamera camera("cam_c", CameraBase::SlotPolicy::DROP);
  Subscriber& sub = Subscriber::Make("cam_c_image");
  sub.keep = true;
  WaitFor(sub.received, 2);  // 两个槽都被攥住 / both slots held
  const uint32_t received = sub.received.load();
  const uint32_t grabs = camera.grabs.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  Expect(sub.received.load() == received, "no publish while both slots are held");
  Expect(camera.grabs.load() > grabs + 5, "live camera keeps grabbing into scratch");
  {
    std::lock_guard<std::mutex> lock(sub.mutex);
    sub.keep = false;
    sub.held.clear();
  }
  WaitFor(sub.received, received + 3);
  camera.Stop();
  sub.Clear();
}

void TestWaitPolicyStopsGrabbing()
{
  FakeCamera camera("cam_d", CameraBase::SlotPolicy::WAIT);
  Subscriber& sub = Subscriber::Make("cam_d_image");
  sub.keep = true;
  WaitFor(sub.received, 2);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const uint32_t grabs = camera.grabs.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  Expect(camera.grabs.load() <= grabs + 1, "replay waits instead of grabbing");
  {
    std::lock_guard<std::mutex> lock(sub.mutex);
    sub.keep = false;
    sub.held.clear();
  }
  WaitFor(sub.received, 4);
  camera.Stop();
  sub.Clear();
}
}  // namespace

int main()
{
  LibXR::PlatformInit();
  TestPublishStampsGeometryAndCalibration();
  TestSwitchView();
  TestMoveNarrow();
  TestDropPolicyKeepsGrabbing();
  TestWaitPolicyStopsGrabbing();
  std::puts("camera_base_test passed");
  return 0;
}
