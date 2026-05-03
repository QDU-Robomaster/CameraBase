#pragma once

#include "logger.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

/**
 * @brief CameraBase 在线标定辅助器。
 *
 * CameraBase 持有图像发布闸门。标定激活时，CameraBase::CommitImage() 会先把帧交给
 * ProcessFrame()；如果该帧被标定流程接管，就不再调用后级 CameraFrameSync 的发布回调。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CameraBaseCalibration
{
 public:
  static constexpr int kGShangDictionaryBits = 5;
  static constexpr int kGShangMarkerCells = kGShangDictionaryBits + 2;
  static constexpr int kGShangSquareCells = kGShangDictionaryBits + 4;
  static constexpr int kMinimumCalibrationViews = 8;
  static constexpr int kDefaultRecommendedViews = 120;
  static constexpr int kDefaultMaxStoredViews = 300;
  static constexpr bool kSupportsImageEncoding =
      CameraInfoV.encoding == CameraTypes::Encoding::BGR8 ||
      CameraInfoV.encoding == CameraTypes::Encoding::RGB8 ||
      CameraInfoV.encoding == CameraTypes::Encoding::BGRA8 ||
      CameraInfoV.encoding == CameraTypes::Encoding::RGBA8 ||
      CameraInfoV.encoding == CameraTypes::Encoding::MONO8;

  struct Config
  {
    double marker_mm = 25.0;
    int cols = 8;
    int rows = 6;
    int min_markers = 12;
    int recommended_views = kDefaultRecommendedViews;
    int max_stored_views = kDefaultMaxStoredViews;
    int process_stride = 3;
    uint64_t min_accept_interval_us = 200000;
    double max_homography_rms = 2.5;
    double max_reprojection_rms = 3.0;
    double min_sharpness_score = 30.0;
    double min_sharpness_best_ratio = 0.20;
    double min_center_delta_norm = 0.020;
    double min_scale_delta_log = 0.05;
    double min_angle_delta_deg = 3.0;
  };

  bool Start(std::string_view marker_size_text, int cols, int rows,
             std::string_view camera_name)
  {
    double marker_mm = 0.0;
    if (!ParseMarkerMm(marker_size_text, marker_mm))
    {
      const std::string marker_size(marker_size_text);
      XR_LOG_ERROR("相机标定：标记尺寸无效：%s",
                   marker_size.c_str());
      return false;
    }

    if (cols < 2 || rows < 2)
    {
      XR_LOG_ERROR("相机标定：标定板行列无效 cols=%d rows=%d",
                   cols, rows);
      return false;
    }

    if constexpr (!kSupportsImageEncoding)
    {
      XR_LOG_ERROR("相机标定：当前图像编码暂不支持 encoding=%u",
                   static_cast<unsigned>(CameraInfoV.encoding));
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    config_ = MakeConfig(marker_mm, cols, rows);
    board_ = MakeGShangMarkerMap(config_);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    output_dir_ = BuildOutputDir(camera_name, config_);
    debug_dir_ = (std::filesystem::path(output_dir_) / "debug").string();
    std::error_code ec;
    std::filesystem::create_directories(debug_dir_, ec);
    if (ec)
    {
      XR_LOG_ERROR("相机标定：创建输出目录失败 %s: %s",
                   output_dir_.c_str(), ec.message().c_str());
      return false;
    }

    accepted_views_.clear();
    ResetCountersLocked();
    active_ = true;
    active_fast_.store(true, std::memory_order_release);

    XR_LOG_INFO("相机标定已启动：标记=%.3f mm 标定板=%dx%d "
                "方格=%.3f mm 最少标记=%d 建议视角=%d "
                "最多视角=%d 输出=%s",
                config_.marker_mm, config_.cols, config_.rows,
                SquareMm(config_), config_.min_markers,
                config_.recommended_views, config_.max_stored_views,
                output_dir_.c_str());
    return true;
  }

  /**
   * @return true 表示该帧被标定流程接管，正常图像发布必须跳过。
   */
  bool ProcessFrame(const uint8_t* data, uint64_t timestamp_us)
  {
    if (!active_fast_.load(std::memory_order_acquire))
    {
      return false;
    }

    FrameSnapshot snapshot;
    const FrameAction action = PrepareFrame(data, timestamp_us, snapshot);
    if (action == FrameAction::kPublish)
    {
      return false;
    }
    if (action == FrameAction::kSwallow)
    {
      return true;
    }

    Detection detection;
    const cv::Mat image = MakeImageView(data);
    if (image.empty())
    {
      LogUnsupportedEncodingOnce();
      return true;
    }
    if (!DetectUsableView(image, snapshot, detection))
    {
      return true;
    }

    StoredView stored;
    if (StoreDetection(snapshot, detection, stored))
    {
      SaveDebugImage(image, detection, stored);
    }
    return true;
  }

  void Stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    active_fast_.store(false, std::memory_order_release);
    XR_LOG_INFO("相机标定已停止：视角=%llu 输出=%s",
                static_cast<unsigned long long>(accepted_views_.size()),
                output_dir_.c_str());
  }

  bool SaveAndStop()
  {
    std::vector<View> views;
    Config config;
    std::string output_dir;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (static_cast<int>(accepted_views_.size()) < kMinimumCalibrationViews)
      {
        XR_LOG_WARN("相机标定：至少需要 %d 个视角，当前只有 %llu 个",
                    kMinimumCalibrationViews,
                    static_cast<unsigned long long>(accepted_views_.size()));
        return false;
      }

      // 保存命令会结束本次标定。即使数值求解失败，也恢复图像发布，
      // 操作手可以调整姿态后重新开始一轮。
      active_ = false;
      active_fast_.store(false, std::memory_order_release);
      views = accepted_views_;
      config = config_;
      output_dir = output_dir_;
    }

    CalibrationOutput output;
    if (!CalibrateAndWrite(views, config, output_dir, output))
    {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    finished_ = true;
    last_rms_ = output.rms;
    last_saved_yaml_ = output.yaml_path;
    return true;
  }

  std::string StatusString() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream os;
    os << "标定 激活=" << (active_ ? 1 : 0)
       << " 完成=" << (finished_ ? 1 : 0)
       << " 视角=" << accepted_views_.size()
       << " 建议=" << config_.recommended_views
       << " 上限=" << config_.max_stored_views
       << " 可保存="
       << (accepted_views_.size() >= kMinimumCalibrationViews ? 1 : 0)
       << " 已处理=" << processed_frames_
       << " 已检测=" << detected_frames_
       << " 模糊拒绝=" << sharpness_rejected_frames_
       << " 重复拒绝=" << duplicate_rejected_frames_
       << " 已截断发布=" << swallowed_frames_
       << " 标定板=" << config_.cols << "x" << config_.rows
       << " 标记mm=" << config_.marker_mm
       << " 方格mm=" << SquareMm(config_)
       << " 最佳清晰度=" << best_sharpness_score_
       << " 输出=" << output_dir_;
    if (last_rms_ >= 0.0)
    {
      os << " rms=" << last_rms_ << " yaml=" << last_saved_yaml_;
    }
    return os.str();
  }

 private:
  using MarkerCorners = std::array<cv::Point3f, 4>;
  using BoardMap = std::map<int, MarkerCorners>;

  struct View
  {
    uint64_t frame_index = 0;
    uint64_t timestamp_us = 0;
    int used_markers = 0;
    double homography_rms = 0.0;
    double sharpness_score = 0.0;
    double center_x_norm = 0.0;
    double center_y_norm = 0.0;
    double scale_norm = 0.0;
    double angle_deg = 0.0;
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
  };

  struct FrameSnapshot
  {
    Config config;
    BoardMap board;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params;
    uint64_t frame_index = 0;
    uint64_t timestamp_us = 0;
  };

  struct Detection
  {
    int used_markers = 0;
    double homography_rms = 0.0;
    double sharpness_score = 0.0;
    double center_x_norm = 0.0;
    double center_y_norm = 0.0;
    double scale_norm = 0.0;
    double angle_deg = 0.0;
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Mat marker_ids;
  };

  struct StoredView
  {
    std::string debug_dir;
    std::size_t view_number = 0;
    uint64_t frame_index = 0;
    int used_markers = 0;
    double homography_rms = 0.0;
    double sharpness_score = 0.0;
  };

  struct CalibrationOutput
  {
    double rms = -1.0;
    std::string yaml_path;
  };

  enum class FrameAction
  {
    kPublish,
    kSwallow,
    kProcess,
  };

  static Config MakeConfig(double marker_mm, int cols, int rows)
  {
    Config config{};
    config.marker_mm = marker_mm;
    config.cols = cols;
    config.rows = rows;

    const int marker_count = (rows * cols) / 2;
    config.min_markers = std::max(4, std::min(marker_count, (marker_count * 2 + 2) / 3));
    return config;
  }

  void ResetCountersLocked()
  {
    raw_frame_index_ = 0;
    swallowed_frames_ = 0;
    processed_frames_ = 0;
    detected_frames_ = 0;
    last_accept_timestamp_us_ = 0;
    last_rms_ = -1.0;
    last_saved_yaml_.clear();
    finished_ = false;
    unsupported_encoding_logged_ = false;
    recommended_views_logged_ = false;
    max_views_logged_ = false;
    best_sharpness_score_ = 0.0;
    sharpness_rejected_frames_ = 0;
    duplicate_rejected_frames_ = 0;
  }

  FrameAction PrepareFrame(const uint8_t* data, uint64_t timestamp_us,
                           FrameSnapshot& snapshot)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_)
    {
      return FrameAction::kPublish;
    }

    ++swallowed_frames_;
    ++raw_frame_index_;
    if (data == nullptr || config_.process_stride <= 0 ||
        (raw_frame_index_ % static_cast<uint64_t>(config_.process_stride)) != 0)
    {
      return FrameAction::kSwallow;
    }

    ++processed_frames_;
    snapshot.config = config_;
    snapshot.board = board_;
    snapshot.dictionary = dictionary_;
    snapshot.detector_params = detector_params_;
    snapshot.frame_index = raw_frame_index_;
    snapshot.timestamp_us = timestamp_us;
    return FrameAction::kProcess;
  }

  bool DetectUsableView(const cv::Mat& image, const FrameSnapshot& snapshot,
                        Detection& detection)
  {
    std::vector<std::vector<cv::Point2f>> rejected;
    try
    {
      cv::aruco::detectMarkers(image, snapshot.dictionary,
                               detection.marker_corners, detection.marker_ids,
                               snapshot.detector_params, rejected);
    }
    catch (const cv::Exception& e)
    {
      XR_LOG_ERROR("相机标定：detectMarkers 失败：%s", e.what());
      return false;
    }

    if (!CollectPoints(detection.marker_corners, detection.marker_ids,
                       snapshot.board, detection.object_points,
                       detection.image_points, detection.used_markers))
    {
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++detected_frames_;
    }

    detection.homography_rms =
        HomographyRms(detection.object_points, detection.image_points);
    if (detection.used_markers < snapshot.config.min_markers ||
        detection.homography_rms > snapshot.config.max_homography_rms)
    {
      return false;
    }

    FillDetectionQuality(image, detection);
    return true;
  }

  bool StoreDetection(const FrameSnapshot& snapshot, const Detection& detection,
                      StoredView& stored)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_)
    {
      return false;
    }

    if (last_accept_timestamp_us_ != 0 &&
        snapshot.timestamp_us > last_accept_timestamp_us_ &&
        snapshot.timestamp_us - last_accept_timestamp_us_ <
            config_.min_accept_interval_us)
    {
      return false;
    }

    if (static_cast<int>(accepted_views_.size()) >= config_.max_stored_views)
    {
      if (!max_views_logged_)
      {
        XR_LOG_WARN("相机标定：已达到最多存储视角 %d，请执行 cali save 或 cali stop",
                    config_.max_stored_views);
        max_views_logged_ = true;
      }
      return false;
    }

    best_sharpness_score_ = std::max(best_sharpness_score_, detection.sharpness_score);
    const double sharpness_threshold =
        std::max(config_.min_sharpness_score,
                 best_sharpness_score_ * config_.min_sharpness_best_ratio);
    if (detection.sharpness_score < sharpness_threshold)
    {
      ++sharpness_rejected_frames_;
      return false;
    }

    if (IsNearDuplicateLocked(detection))
    {
      ++duplicate_rejected_frames_;
      return false;
    }

    View view;
    view.frame_index = snapshot.frame_index;
    view.timestamp_us = snapshot.timestamp_us;
    view.used_markers = detection.used_markers;
    view.homography_rms = detection.homography_rms;
    view.sharpness_score = detection.sharpness_score;
    view.center_x_norm = detection.center_x_norm;
    view.center_y_norm = detection.center_y_norm;
    view.scale_norm = detection.scale_norm;
    view.angle_deg = detection.angle_deg;
    view.object_points = detection.object_points;
    view.image_points = detection.image_points;
    accepted_views_.push_back(std::move(view));
    last_accept_timestamp_us_ = snapshot.timestamp_us;

    stored.debug_dir = debug_dir_;
    stored.view_number = accepted_views_.size();
    stored.frame_index = snapshot.frame_index;
    stored.used_markers = detection.used_markers;
    stored.homography_rms = detection.homography_rms;
    stored.sharpness_score = detection.sharpness_score;

    XR_LOG_INFO("相机标定：接受视角 %llu，标记=%d H-rms=%.3f 清晰度=%.1f ts=%llu",
                static_cast<unsigned long long>(accepted_views_.size()),
                detection.used_markers, detection.homography_rms,
                detection.sharpness_score,
                static_cast<unsigned long long>(snapshot.timestamp_us));

    if (!recommended_views_logged_ &&
        static_cast<int>(accepted_views_.size()) >= config_.recommended_views)
    {
      XR_LOG_INFO("相机标定：已达到建议视角数 %d，可继续移动取样，或执行 cali save",
                  config_.recommended_views);
      recommended_views_logged_ = true;
    }
    return true;
  }

  void LogUnsupportedEncodingOnce()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (unsupported_encoding_logged_)
    {
      return;
    }
    unsupported_encoding_logged_ = true;
    XR_LOG_ERROR("相机标定：当前图像编码暂不支持 encoding=%u",
                 static_cast<unsigned>(CameraInfoV.encoding));
  }

  static double SquareMm(const Config& config)
  {
    return config.marker_mm * static_cast<double>(kGShangSquareCells) /
           static_cast<double>(kGShangMarkerCells);
  }

  static bool ParseMarkerMm(std::string_view text, double& marker_mm)
  {
    std::string value(text);
    if (value.size() >= 2)
    {
      const std::string suffix = value.substr(value.size() - 2);
      if (suffix == "mm" || suffix == "MM")
      {
        value.resize(value.size() - 2);
      }
    }

    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str() || *end != '\0' || parsed <= 0.0 ||
        !std::isfinite(parsed))
    {
      return false;
    }

    marker_mm = parsed;
    return true;
  }

  static std::string Sanitize(std::string_view text)
  {
    std::string out;
    out.reserve(text.size());
    for (char ch : text)
    {
      if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
          (ch >= '0' && ch <= '9') || ch == '_' || ch == '-')
      {
        out.push_back(ch);
      }
      else
      {
        out.push_back('_');
      }
    }
    return out.empty() ? "camera" : out;
  }

  static std::string TimeStampString()
  {
    std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);

    char buf[32]{};
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
    return std::string(buf);
  }

  static std::string BuildOutputDir(std::string_view camera_name,
                                    const Config& config)
  {
    std::ostringstream leaf;
    leaf << TimeStampString() << "_" << Sanitize(camera_name) << "_"
         << std::fixed << std::setprecision(0) << config.marker_mm << "mm_"
         << config.cols << "x" << config.rows;
    return (std::filesystem::path("runs") / "camera_calib" / leaf.str()).string();
  }

  /**
   * @brief 生成 GShang 在线工具对应的 ChArUco 三维角点表。
   *
   * GShang 工具在 (row + col) 为奇数的棋盘格中放置 ArUco-original 5-bit 标记。
   * 该工具里的 markerSize 不是完整方格宽度：5-bit 标记外侧还有白边和棋盘边界，
   * 因此实际方格宽度为 square_mm = marker_mm * 9 / 7。
   */
  static BoardMap MakeGShangMarkerMap(const Config& config)
  {
    const double cell_mm = config.marker_mm / static_cast<double>(kGShangMarkerCells);
    const double square_mm = cell_mm * static_cast<double>(kGShangSquareCells);
    const double marker_offset_mm = cell_mm;

    BoardMap map;
    int marker_id = 0;
    for (int row = 0; row < config.rows; ++row)
    {
      for (int col = 0; col < config.cols; ++col)
      {
        if (((row + col) % 2) == 0)
        {
          continue;
        }

        const float x0 = static_cast<float>(col * square_mm + marker_offset_mm);
        const float y0 = static_cast<float>(row * square_mm + marker_offset_mm);
        const float x1 = x0 + static_cast<float>(config.marker_mm);
        const float y1 = y0 + static_cast<float>(config.marker_mm);
        map[marker_id++] = {cv::Point3f{x0, y0, 0.0F},
                            cv::Point3f{x1, y0, 0.0F},
                            cv::Point3f{x1, y1, 0.0F},
                            cv::Point3f{x0, y1, 0.0F}};
      }
    }
    return map;
  }

  static bool CollectPoints(const std::vector<std::vector<cv::Point2f>>& corners,
                            const cv::Mat& ids, const BoardMap& board,
                            std::vector<cv::Point3f>& object_points,
                            std::vector<cv::Point2f>& image_points,
                            int& used_markers)
  {
    object_points.clear();
    image_points.clear();
    used_markers = 0;
    if (ids.empty())
    {
      return false;
    }

    for (int i = 0; i < ids.rows; ++i)
    {
      if (i >= static_cast<int>(corners.size()) || corners[i].size() < 4)
      {
        continue;
      }

      const int id = ids.at<int>(i, 0);
      const auto marker = board.find(id);
      if (marker == board.end())
      {
        continue;
      }

      for (int corner = 0; corner < 4; ++corner)
      {
        object_points.push_back(marker->second[corner]);
        image_points.push_back(corners[i][corner]);
      }
      ++used_markers;
    }
    return used_markers > 0;
  }

  static cv::Mat MakeGrayImage(const cv::Mat& image)
  {
    if (image.channels() == 1)
    {
      return image;
    }

    cv::Mat gray;
    if (image.channels() == 3)
    {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else if (image.channels() == 4)
    {
      cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    }
    return gray;
  }

  static double MarkerSharpnessScore(
      const cv::Mat& image, const std::vector<std::vector<cv::Point2f>>& corners)
  {
    const cv::Mat gray = MakeGrayImage(image);
    if (gray.empty())
    {
      return 0.0;
    }

    cv::Mat marker_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
    for (const auto& marker : corners)
    {
      if (marker.size() < 4)
      {
        continue;
      }

      std::array<cv::Point, 4> polygon{};
      for (int i = 0; i < 4; ++i)
      {
        polygon[static_cast<std::size_t>(i)] =
            cv::Point{static_cast<int>(std::lround(marker[static_cast<std::size_t>(i)].x)),
                      static_cast<int>(std::lround(marker[static_cast<std::size_t>(i)].y))};
      }
      cv::fillConvexPoly(marker_mask, polygon.data(), static_cast<int>(polygon.size()),
                         cv::Scalar{255});
    }

    if (cv::countNonZero(marker_mask) < 16)
    {
      return 0.0;
    }

    cv::dilate(marker_mask, marker_mask, cv::Mat{}, {-1, -1}, 1);
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F, 3);

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(laplacian, mean, stddev, marker_mask);
    return stddev[0] * stddev[0];
  }

  static double NormalizeBoardAngle(double angle_deg, const cv::Size2f& size)
  {
    if (size.width < size.height)
    {
      angle_deg += 90.0;
    }

    while (angle_deg < 0.0)
    {
      angle_deg += 180.0;
    }
    while (angle_deg >= 180.0)
    {
      angle_deg -= 180.0;
    }
    return angle_deg;
  }

  static double AngleDeltaDeg(double lhs, double rhs)
  {
    double delta = std::fabs(lhs - rhs);
    while (delta >= 180.0)
    {
      delta -= 180.0;
    }
    return delta > 90.0 ? 180.0 - delta : delta;
  }

  static void FillDetectionQuality(const cv::Mat& image, Detection& detection)
  {
    detection.sharpness_score =
        MarkerSharpnessScore(image, detection.marker_corners);

    if (detection.image_points.empty() || CameraInfoV.width == 0 ||
        CameraInfoV.height == 0)
    {
      return;
    }

    const cv::Rect bounds = cv::boundingRect(detection.image_points);
    const double image_width = static_cast<double>(CameraInfoV.width);
    const double image_height = static_cast<double>(CameraInfoV.height);
    detection.center_x_norm =
        (static_cast<double>(bounds.x) + static_cast<double>(bounds.width) * 0.5) /
        image_width;
    detection.center_y_norm =
        (static_cast<double>(bounds.y) + static_cast<double>(bounds.height) * 0.5) /
        image_height;
    detection.scale_norm =
        std::sqrt(std::max(0.0, static_cast<double>(bounds.area())) /
                  std::max(1.0, image_width * image_height));

    if (detection.image_points.size() >= 4)
    {
      const cv::RotatedRect rect = cv::minAreaRect(detection.image_points);
      detection.angle_deg = NormalizeBoardAngle(rect.angle, rect.size);
    }
  }

  bool IsNearDuplicateLocked(const Detection& detection) const
  {
    for (const View& view : accepted_views_)
    {
      const double center_delta =
          std::hypot(detection.center_x_norm - view.center_x_norm,
                     detection.center_y_norm - view.center_y_norm);
      const double scale_delta =
          std::fabs(std::log((detection.scale_norm + 1e-6) /
                             (view.scale_norm + 1e-6)));
      const double angle_delta =
          AngleDeltaDeg(detection.angle_deg, view.angle_deg);
      if (center_delta < config_.min_center_delta_norm &&
          scale_delta < config_.min_scale_delta_log &&
          angle_delta < config_.min_angle_delta_deg)
      {
        return true;
      }
    }
    return false;
  }

  static double HomographyRms(const std::vector<cv::Point3f>& object_points,
                              const std::vector<cv::Point2f>& image_points)
  {
    if (object_points.size() < 4 || object_points.size() != image_points.size())
    {
      return 1e9;
    }

    std::vector<cv::Point2f> object_xy;
    object_xy.reserve(object_points.size());
    for (const auto& point : object_points)
    {
      object_xy.emplace_back(point.x, point.y);
    }

    const cv::Mat homography = cv::findHomography(object_xy, image_points, 0);
    if (homography.empty())
    {
      return 1e9;
    }

    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(object_xy, projected, homography);

    double sum2 = 0.0;
    for (std::size_t i = 0; i < image_points.size(); ++i)
    {
      const cv::Point2f delta = projected[i] - image_points[i];
      sum2 += delta.dot(delta);
    }
    return std::sqrt(sum2 / static_cast<double>(image_points.size()));
  }

  cv::Mat MakeImageView(const uint8_t* data) const
  {
    constexpr int width = static_cast<int>(CameraInfoV.width);
    constexpr int height = static_cast<int>(CameraInfoV.height);
    constexpr std::size_t step = static_cast<std::size_t>(CameraInfoV.step);

    if constexpr (CameraInfoV.encoding == CameraTypes::Encoding::BGR8 ||
                  CameraInfoV.encoding == CameraTypes::Encoding::RGB8)
    {
      return cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(data), step);
    }
    else if constexpr (CameraInfoV.encoding == CameraTypes::Encoding::BGRA8 ||
                       CameraInfoV.encoding == CameraTypes::Encoding::RGBA8)
    {
      return cv::Mat(height, width, CV_8UC4, const_cast<uint8_t*>(data), step);
    }
    else if constexpr (CameraInfoV.encoding == CameraTypes::Encoding::MONO8)
    {
      return cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(data), step);
    }
    else
    {
      return {};
    }
  }

  static bool CalibrateViews(const std::vector<View>& views,
                             cv::Mat& camera_matrix, cv::Mat& distortion,
                             double& rms, std::vector<cv::Mat>& rvecs,
                             std::vector<cv::Mat>& tvecs)
  {
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    object_points.reserve(views.size());
    image_points.reserve(views.size());
    for (const View& view : views)
    {
      object_points.push_back(view.object_points);
      image_points.push_back(view.image_points);
    }

    rms = cv::calibrateCamera(
        object_points, image_points,
        cv::Size(static_cast<int>(CameraInfoV.width),
                 static_cast<int>(CameraInfoV.height)),
        camera_matrix, distortion, rvecs, tvecs);
    return true;
  }

  static double ReprojectionRms(const View& view, const cv::Mat& camera_matrix,
                                const cv::Mat& distortion,
                                const cv::Mat& rvec, const cv::Mat& tvec)
  {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(view.object_points, rvec, tvec,
                      camera_matrix, distortion, projected);

    double sum2 = 0.0;
    for (std::size_t i = 0; i < view.image_points.size(); ++i)
    {
      const cv::Point2f delta = projected[i] - view.image_points[i];
      sum2 += delta.dot(delta);
    }
    return std::sqrt(sum2 / static_cast<double>(view.image_points.size()));
  }

  static std::vector<double> PerViewRms(const std::vector<View>& views,
                                        const cv::Mat& camera_matrix,
                                        const cv::Mat& distortion,
                                        const std::vector<cv::Mat>& rvecs,
                                        const std::vector<cv::Mat>& tvecs)
  {
    std::vector<double> rms;
    rms.reserve(views.size());
    for (std::size_t i = 0; i < views.size(); ++i)
    {
      rms.push_back(ReprojectionRms(views[i], camera_matrix, distortion,
                                    rvecs[i], tvecs[i]));
    }
    return rms;
  }

  static std::vector<View> FilterByRms(const std::vector<View>& views,
                                       const std::vector<double>& per_view_rms,
                                       double max_rms)
  {
    std::vector<View> filtered;
    for (std::size_t i = 0; i < views.size(); ++i)
    {
      if (i < per_view_rms.size() && per_view_rms[i] <= max_rms)
      {
        filtered.push_back(views[i]);
      }
    }
    return filtered;
  }

  static std::vector<View> FilterBySharpness(const std::vector<View>& views,
                                             const Config& config)
  {
    double best_score = 0.0;
    for (const View& view : views)
    {
      best_score = std::max(best_score, view.sharpness_score);
    }

    const double threshold =
        std::max(config.min_sharpness_score,
                 best_score * config.min_sharpness_best_ratio);

    std::vector<View> filtered;
    filtered.reserve(views.size());
    for (const View& view : views)
    {
      if (view.sharpness_score >= threshold)
      {
        filtered.push_back(view);
      }
    }
    return filtered;
  }

  static bool CalibrateAndWrite(const std::vector<View>& input_views,
                                const Config& config,
                                const std::string& output_dir,
                                CalibrationOutput& output)
  {
    std::vector<View> calibration_views = FilterBySharpness(input_views, config);
    if (calibration_views.size() >= kMinimumCalibrationViews &&
        calibration_views.size() < input_views.size())
    {
      XR_LOG_INFO("相机标定：清晰度预筛选保留 %llu/%llu 个视角",
                  static_cast<unsigned long long>(calibration_views.size()),
                  static_cast<unsigned long long>(input_views.size()));
    }
    else
    {
      calibration_views = input_views;
    }

    cv::Mat camera_matrix;
    cv::Mat distortion;
    double rms = 0.0;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    try
    {
      CalibrateViews(calibration_views, camera_matrix, distortion, rms, rvecs, tvecs);
    }
    catch (const cv::Exception& e)
    {
      XR_LOG_ERROR("相机标定：calibrateCamera 失败：%s", e.what());
      return false;
    }

    std::vector<View> final_views = calibration_views;
    std::vector<double> final_per_view_rms =
        PerViewRms(calibration_views, camera_matrix, distortion, rvecs, tvecs);

    const std::vector<View> filtered =
        FilterByRms(calibration_views, final_per_view_rms, config.max_reprojection_rms);
    if (filtered.size() >= kMinimumCalibrationViews &&
        filtered.size() < calibration_views.size())
    {
      cv::Mat filtered_camera_matrix;
      cv::Mat filtered_distortion;
      double filtered_rms = 0.0;
      std::vector<cv::Mat> filtered_rvecs;
      std::vector<cv::Mat> filtered_tvecs;
      try
      {
        CalibrateViews(filtered, filtered_camera_matrix, filtered_distortion,
                       filtered_rms, filtered_rvecs, filtered_tvecs);
      }
      catch (const cv::Exception& e)
      {
        XR_LOG_WARN("相机标定：剔除离群视角后的标定失败：%s", e.what());
      }

      if (!filtered_camera_matrix.empty())
      {
        camera_matrix = filtered_camera_matrix;
        distortion = filtered_distortion;
        rms = filtered_rms;
        final_views = filtered;
        final_per_view_rms = PerViewRms(final_views, camera_matrix, distortion,
                                        filtered_rvecs, filtered_tvecs);
      }
    }

    std::error_code ec;
    std::filesystem::create_directories(output_dir, ec);
    if (ec)
    {
      XR_LOG_ERROR("相机标定：创建输出目录失败 %s: %s",
                   output_dir.c_str(), ec.message().c_str());
      return false;
    }

    const std::filesystem::path yaml_path =
        std::filesystem::path(output_dir) / "calibration.yml";
    const std::filesystem::path csv_path =
        std::filesystem::path(output_dir) / "views.csv";
    const std::filesystem::path snippet_path =
        std::filesystem::path(output_dir) / "camera_info_snippet.txt";

    WriteCalibrationYaml(yaml_path, config, camera_matrix, distortion, rms,
                         static_cast<int>(final_views.size()));
    WriteViewsCsv(csv_path, final_views, final_per_view_rms);
    WriteCameraInfoSnippet(snippet_path, camera_matrix, distortion);

    output.rms = rms;
    output.yaml_path = yaml_path.string();
    XR_LOG_PASS("相机标定结果已保存：视角=%llu rms=%.4f yaml=%s",
                static_cast<unsigned long long>(final_views.size()), rms,
                output.yaml_path.c_str());
    return true;
  }

  static void WriteCalibrationYaml(const std::filesystem::path& path,
                                   const Config& config,
                                   const cv::Mat& camera_matrix,
                                   const cv::Mat& distortion,
                                   double rms, int views)
  {
    cv::FileStorage fs(path.string(), cv::FileStorage::WRITE);
    fs << "image_width" << static_cast<int>(CameraInfoV.width);
    fs << "image_height" << static_cast<int>(CameraInfoV.height);
    fs << "rows" << config.rows;
    fs << "cols" << config.cols;
    fs << "marker_mm" << config.marker_mm;
    fs << "square_mm" << SquareMm(config);
    fs << "dictionary" << "aruco_original";
    fs << "generator" << "GShang ChArUco: square = marker * 9 / 7";
    fs << "views" << views;
    fs << "rms" << rms;
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_coefficients" << distortion;
  }

  static void WriteViewsCsv(const std::filesystem::path& path,
                            const std::vector<View>& views,
                            const std::vector<double>& per_view_rms)
  {
    std::ofstream csv(path);
    csv << "frame_index,timestamp_us,used_markers,homography_rms,"
           "sharpness_score,center_x_norm,center_y_norm,scale_norm,angle_deg,"
           "per_view_reprojection_rms\n";
    for (std::size_t i = 0; i < views.size(); ++i)
    {
      csv << views[i].frame_index << ","
          << views[i].timestamp_us << ","
          << views[i].used_markers << ","
          << views[i].homography_rms << ","
          << views[i].sharpness_score << ","
          << views[i].center_x_norm << ","
          << views[i].center_y_norm << ","
          << views[i].scale_norm << ","
          << views[i].angle_deg << ","
          << (i < per_view_rms.size() ? per_view_rms[i] : -1.0) << "\n";
    }
  }

  static void WriteCameraInfoSnippet(const std::filesystem::path& path,
                                     const cv::Mat& camera_matrix,
                                     const cv::Mat& distortion)
  {
    std::array<double, 14> distortion_values{};
    const cv::Mat flat = distortion.reshape(1, 1);
    for (int i = 0; i < std::min(flat.cols, static_cast<int>(distortion_values.size())); ++i)
    {
      distortion_values[static_cast<std::size_t>(i)] = flat.at<double>(0, i);
    }

    std::ofstream out(path);
    out << std::setprecision(17);
    out << "camera_matrix: [";
    for (int i = 0; i < 9; ++i)
    {
      if (i != 0)
      {
        out << ", ";
      }
      out << camera_matrix.at<double>(i / 3, i % 3);
    }
    out << "]\n";

    out << "distortion_model: CameraTypes::DistortionModel::PLUMB_BOB\n";
    out << "distortion_coefficients: [";
    for (std::size_t i = 0; i < distortion_values.size(); ++i)
    {
      if (i != 0)
      {
        out << ", ";
      }
      out << distortion_values[i];
    }
    out << "]\n";

    out << "projection_matrix: ["
        << camera_matrix.at<double>(0, 0) << ", 0.0, "
        << camera_matrix.at<double>(0, 2) << ", 0.0, 0.0, "
        << camera_matrix.at<double>(1, 1) << ", "
        << camera_matrix.at<double>(1, 2)
        << ", 0.0, 0.0, 0.0, 1.0, 0.0]\n";
  }

  static void SaveDebugImage(const cv::Mat& image, const Detection& detection,
                             const StoredView& stored)
  {
    cv::Mat debug = image.clone();
    if (!detection.marker_ids.empty())
    {
      cv::aruco::drawDetectedMarkers(debug, detection.marker_corners,
                                     detection.marker_ids);
    }

    std::ostringstream label;
    label << "view=" << stored.view_number
          << " markers=" << stored.used_markers
          << " H=" << std::fixed << std::setprecision(3)
          << stored.homography_rms
          << " S=" << std::setprecision(1) << stored.sharpness_score;
    cv::putText(debug, label.str(), {20, 45}, cv::FONT_HERSHEY_SIMPLEX,
                1.0, {0, 255, 0}, 2, cv::LINE_AA);

    std::ostringstream name;
    name << "view_" << std::setw(4) << std::setfill('0') << stored.view_number
         << "_frame_" << stored.frame_index << ".jpg";
    const std::filesystem::path path =
        std::filesystem::path(stored.debug_dir) / name.str();

    try
    {
      cv::imwrite(path.string(), debug);
    }
    catch (const cv::Exception& e)
    {
      XR_LOG_WARN("相机标定：写入调试图失败 %s: %s",
                  path.string().c_str(), e.what());
    }
  }

  mutable std::mutex mutex_;
  std::atomic<bool> active_fast_{false};

  bool active_{false};
  bool finished_{false};
  bool unsupported_encoding_logged_{false};
  bool recommended_views_logged_{false};
  bool max_views_logged_{false};

  Config config_{};
  BoardMap board_{};
  cv::Ptr<cv::aruco::Dictionary> dictionary_{};
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_{};
  std::vector<View> accepted_views_{};

  std::string output_dir_{};
  std::string debug_dir_{};
  std::string last_saved_yaml_{};

  double last_rms_{-1.0};
  double best_sharpness_score_{0.0};
  uint64_t raw_frame_index_{0};
  uint64_t swallowed_frames_{0};
  uint64_t processed_frames_{0};
  uint64_t detected_frames_{0};
  uint64_t sharpness_rejected_frames_{0};
  uint64_t duplicate_rejected_frames_{0};
  uint64_t last_accept_timestamp_us_{0};
};
