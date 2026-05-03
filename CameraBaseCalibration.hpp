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

template <CameraTypes::CameraInfo CameraInfoV>
class CameraBaseCalibration
{
 public:
  static constexpr int gshang_dictionary_bits = 5;
  static constexpr int gshang_marker_cells = gshang_dictionary_bits + 2;
  static constexpr int gshang_square_cells = gshang_dictionary_bits + 4;
  static constexpr int min_calibration_views = 8;

  struct Config
  {
    double marker_mm = 25.0;
    int cols = 8;
    int rows = 6;
    int min_markers = 12;
    int target_views = 50;
    int process_stride = 3;
    uint64_t min_accept_interval_us = 250000;
    double max_homography_rms = 2.5;
    double max_reprojection_rms = 3.0;
  };

  struct Candidate
  {
    uint64_t frame_index = 0;
    uint64_t timestamp_us = 0;
    int used_markers = 0;
    double homography_rms = 0.0;
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
  };

  bool Start(std::string_view marker_size_text, int cols, int rows,
             std::string_view camera_name)
  {
    double marker_mm = 0.0;
    if (!ParseMarkerMm(marker_size_text, marker_mm))
    {
      const std::string marker_size(marker_size_text);
      XR_LOG_ERROR("Camera calibration: invalid marker size: %s",
                   marker_size.c_str());
      return false;
    }
    if (cols < 2 || rows < 2)
    {
      XR_LOG_ERROR("Camera calibration: invalid board geometry cols=%d rows=%d",
                   cols, rows);
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    cfg_ = Config{};
    cfg_.marker_mm = marker_mm;
    cfg_.cols = cols;
    cfg_.rows = rows;
    const int marker_count = (rows * cols) / 2;
    cfg_.min_markers = std::max(4, std::min(marker_count, (marker_count * 2 + 2) / 3));

    board_ = MakeGShangCharucoMap(cfg_.rows, cfg_.cols, cfg_.marker_mm);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    output_dir_ = BuildOutputDir(camera_name, cfg_);
    debug_dir_ = (std::filesystem::path(output_dir_) / "debug").string();
    std::error_code ec;
    std::filesystem::create_directories(debug_dir_, ec);
    if (ec)
    {
      XR_LOG_ERROR("Camera calibration: failed to create output dir %s: %s",
                   output_dir_.c_str(), ec.message().c_str());
      return false;
    }

    accepted_.clear();
    last_rms_ = -1.0;
    last_saved_yaml_.clear();
    swallowed_frames_ = 0;
    processed_frames_ = 0;
    detected_frames_ = 0;
    frame_index_ = 0;
    last_accept_timestamp_us_ = 0;
    unsupported_encoding_logged_ = false;
    finished_ = false;
    active_ = true;
    active_fast_.store(true, std::memory_order_release);

    XR_LOG_INFO("Camera calibration started: marker=%.3f mm cols=%d rows=%d "
                "square=%.3f mm min_markers=%d target_views=%d output=%s",
                cfg_.marker_mm, cfg_.cols, cfg_.rows, SquareMm(cfg_),
                cfg_.min_markers, cfg_.target_views, output_dir_.c_str());
    return true;
  }

  bool ProcessFrame(const uint8_t* data, uint64_t timestamp_us)
  {
    if (!active_fast_.load(std::memory_order_acquire))
    {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_)
    {
      return false;
    }

    ++swallowed_frames_;
    ++frame_index_;
    if (data == nullptr || cfg_.process_stride <= 0 ||
        (frame_index_ % static_cast<uint64_t>(cfg_.process_stride)) != 0)
    {
      return true;
    }

    cv::Mat image = MakeImageView(data);
    if (image.empty())
    {
      if (!unsupported_encoding_logged_)
      {
        XR_LOG_ERROR("Camera calibration: unsupported CameraBase encoding=%u",
                     static_cast<unsigned>(CameraInfoV.encoding));
        unsupported_encoding_logged_ = true;
      }
      return true;
    }

    ++processed_frames_;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::Mat ids;
    try
    {
      cv::aruco::detectMarkers(image, dictionary_, corners, ids, detector_params_, rejected);
    }
    catch (const cv::Exception& e)
    {
      XR_LOG_ERROR("Camera calibration: detectMarkers failed: %s", e.what());
      return true;
    }

    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    int used_markers = 0;
    if (!CollectPoints(corners, ids, board_, object_points, image_points, used_markers))
    {
      return true;
    }
    ++detected_frames_;

    const double h_rms = HomographyRms(object_points, image_points);
    if (used_markers < cfg_.min_markers || h_rms > cfg_.max_homography_rms)
    {
      return true;
    }
    if (last_accept_timestamp_us_ != 0 &&
        timestamp_us > last_accept_timestamp_us_ &&
        timestamp_us - last_accept_timestamp_us_ < cfg_.min_accept_interval_us)
    {
      return true;
    }

    Candidate candidate;
    candidate.frame_index = frame_index_;
    candidate.timestamp_us = timestamp_us;
    candidate.used_markers = used_markers;
    candidate.homography_rms = h_rms;
    candidate.object_points = std::move(object_points);
    candidate.image_points = std::move(image_points);
    accepted_.push_back(std::move(candidate));
    last_accept_timestamp_us_ = timestamp_us;

    const Candidate& accepted = accepted_.back();
    SaveDebugImage(image, corners, ids, accepted);
    XR_LOG_INFO("Camera calibration accepted view %llu/%d: markers=%d H-rms=%.3f ts=%llu",
                static_cast<unsigned long long>(accepted_.size()), cfg_.target_views,
                accepted.used_markers,
                accepted.homography_rms,
                static_cast<unsigned long long>(accepted.timestamp_us));

    if (static_cast<int>(accepted_.size()) >= cfg_.target_views)
    {
      const bool ok = CalibrateAndWrite();
      active_ = false;
      active_fast_.store(false, std::memory_order_release);
      finished_ = ok;
      XR_LOG_INFO("Camera calibration auto-finished: ok=%d output=%s",
                  ok ? 1 : 0, output_dir_.c_str());
    }

    return true;
  }

  void Stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    active_fast_.store(false, std::memory_order_release);
    XR_LOG_INFO("Camera calibration stopped: views=%llu output=%s",
                static_cast<unsigned long long>(accepted_.size()), output_dir_.c_str());
  }

  bool SaveAndStop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool ok = CalibrateAndWrite();
    if (ok)
    {
      active_ = false;
      active_fast_.store(false, std::memory_order_release);
      finished_ = true;
    }
    return ok;
  }

  std::string StatusString() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream os;
    os << "Calibration active=" << (active_ ? 1 : 0)
       << " finished=" << (finished_ ? 1 : 0)
       << " views=" << accepted_.size() << "/" << cfg_.target_views
       << " processed=" << processed_frames_
       << " detected=" << detected_frames_
       << " swallowed=" << swallowed_frames_
       << " board=" << cfg_.cols << "x" << cfg_.rows
       << " marker_mm=" << cfg_.marker_mm
       << " square_mm=" << SquareMm(cfg_)
       << " output=" << output_dir_;
    if (last_rms_ >= 0.0)
    {
      os << " rms=" << last_rms_ << " yaml=" << last_saved_yaml_;
    }
    return os.str();
  }

 private:
  using BoardMap = std::map<int, std::array<cv::Point3f, 4>>;

  static double SquareMm(const Config& cfg)
  {
    return cfg.marker_mm * static_cast<double>(gshang_square_cells) /
           static_cast<double>(gshang_marker_cells);
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
    if (end == value.c_str() || *end != '\0' || parsed <= 0.0 || !std::isfinite(parsed))
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
    if (out.empty())
    {
      out = "camera";
    }
    return out;
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

  static std::string BuildOutputDir(std::string_view camera_name, const Config& cfg)
  {
    std::ostringstream leaf;
    leaf << TimeStampString() << "_" << Sanitize(camera_name) << "_"
         << std::fixed << std::setprecision(0) << cfg.marker_mm << "mm_"
         << cfg.cols << "x" << cfg.rows;
    return (std::filesystem::path("runs") / "camera_calib" / leaf.str()).string();
  }

  static BoardMap MakeGShangCharucoMap(int rows, int cols, double marker_mm)
  {
    const double cell_mm = marker_mm / static_cast<double>(gshang_marker_cells);
    const double square_mm = cell_mm * static_cast<double>(gshang_square_cells);
    const double marker_offset_mm = cell_mm;

    BoardMap map;
    int marker_id = 0;
    for (int r = 0; r < rows; ++r)
    {
      for (int c = 0; c < cols; ++c)
      {
        if (((r + c) % 2) == 0)
        {
          continue;
        }

        const float x0 = static_cast<float>(c * square_mm + marker_offset_mm);
        const float y0 = static_cast<float>(r * square_mm + marker_offset_mm);
        const float x1 = x0 + static_cast<float>(marker_mm);
        const float y1 = y0 + static_cast<float>(marker_mm);
        map[marker_id++] = {cv::Point3f{x0, y0, 0.0F},
                            cv::Point3f{x1, y0, 0.0F},
                            cv::Point3f{x1, y1, 0.0F},
                            cv::Point3f{x0, y1, 0.0F}};
      }
    }
    return map;
  }

  static bool CollectPoints(const std::vector<std::vector<cv::Point2f>>& corners,
                            const cv::Mat& ids,
                            const BoardMap& board,
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
      const auto it = board.find(id);
      if (it == board.end())
      {
        continue;
      }
      for (int corner = 0; corner < 4; ++corner)
      {
        object_points.push_back(it->second[corner]);
        image_points.push_back(corners[i][corner]);
      }
      ++used_markers;
    }
    return used_markers > 0;
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

    const cv::Mat h = cv::findHomography(object_xy, image_points, 0);
    if (h.empty())
    {
      return 1e9;
    }

    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(object_xy, projected, h);
    double sum2 = 0.0;
    for (std::size_t i = 0; i < image_points.size(); ++i)
    {
      const cv::Point2f delta = projected[i] - image_points[i];
      sum2 += delta.dot(delta);
    }
    return std::sqrt(sum2 / static_cast<double>(image_points.size()));
  }

  static double ReprojectionRms(const std::vector<cv::Point3f>& object_points,
                                const std::vector<cv::Point2f>& image_points,
                                const cv::Mat& camera_matrix,
                                const cv::Mat& distortion,
                                const cv::Mat& rvec,
                                const cv::Mat& tvec)
  {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, distortion, projected);

    double sum2 = 0.0;
    for (std::size_t i = 0; i < image_points.size(); ++i)
    {
      const cv::Point2f delta = projected[i] - image_points[i];
      sum2 += delta.dot(delta);
    }
    return std::sqrt(sum2 / static_cast<double>(image_points.size()));
  }

  static std::vector<Candidate> FilterViews(const std::vector<Candidate>& views,
                                            const std::vector<double>& per_view_rms,
                                            double max_rms)
  {
    std::vector<Candidate> filtered;
    for (std::size_t i = 0; i < views.size(); ++i)
    {
      if (i < per_view_rms.size() && per_view_rms[i] <= max_rms)
      {
        filtered.push_back(views[i]);
      }
    }
    return filtered;
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

  bool RunCalibration(const std::vector<Candidate>& views,
                      cv::Mat& camera_matrix,
                      cv::Mat& distortion,
                      double& rms,
                      std::vector<cv::Mat>& rvecs,
                      std::vector<cv::Mat>& tvecs) const
  {
    if (static_cast<int>(views.size()) < min_calibration_views)
    {
      XR_LOG_WARN("Camera calibration: need at least %d views, got %llu",
                  min_calibration_views,
                  static_cast<unsigned long long>(views.size()));
      return false;
    }

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    object_points.reserve(views.size());
    image_points.reserve(views.size());
    for (const auto& view : views)
    {
      object_points.push_back(view.object_points);
      image_points.push_back(view.image_points);
    }

    try
    {
      rms = cv::calibrateCamera(object_points, image_points,
                                cv::Size(static_cast<int>(CameraInfoV.width),
                                         static_cast<int>(CameraInfoV.height)),
                                camera_matrix, distortion, rvecs, tvecs);
    }
    catch (const cv::Exception& e)
    {
      XR_LOG_ERROR("Camera calibration: calibrateCamera failed: %s", e.what());
      return false;
    }
    return true;
  }

  bool CalibrateAndWrite()
  {
    cv::Mat camera_matrix;
    cv::Mat distortion;
    double rms = 0.0;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    if (!RunCalibration(accepted_, camera_matrix, distortion, rms, rvecs, tvecs))
    {
      return false;
    }

    std::vector<double> per_view_rms;
    per_view_rms.reserve(accepted_.size());
    for (std::size_t i = 0; i < accepted_.size(); ++i)
    {
      per_view_rms.push_back(ReprojectionRms(accepted_[i].object_points,
                                             accepted_[i].image_points,
                                             camera_matrix, distortion,
                                             rvecs[i], tvecs[i]));
    }

    std::vector<Candidate> final_views = accepted_;
    std::vector<double> final_per_view_rms = per_view_rms;
    std::vector<Candidate> filtered =
        FilterViews(accepted_, per_view_rms, cfg_.max_reprojection_rms);
    if (filtered.size() >= min_calibration_views && filtered.size() < accepted_.size())
    {
      cv::Mat filtered_camera_matrix;
      cv::Mat filtered_distortion;
      double filtered_rms = 0.0;
      std::vector<cv::Mat> filtered_rvecs;
      std::vector<cv::Mat> filtered_tvecs;
      if (RunCalibration(filtered, filtered_camera_matrix, filtered_distortion,
                         filtered_rms, filtered_rvecs, filtered_tvecs))
      {
        camera_matrix = filtered_camera_matrix;
        distortion = filtered_distortion;
        rms = filtered_rms;
        final_views = filtered;
        final_per_view_rms.clear();
        for (std::size_t i = 0; i < final_views.size(); ++i)
        {
          final_per_view_rms.push_back(ReprojectionRms(final_views[i].object_points,
                                                       final_views[i].image_points,
                                                       camera_matrix, distortion,
                                                       filtered_rvecs[i],
                                                       filtered_tvecs[i]));
        }
      }
    }

    std::error_code ec;
    std::filesystem::create_directories(output_dir_, ec);
    if (ec)
    {
      XR_LOG_ERROR("Camera calibration: failed to create output dir %s: %s",
                   output_dir_.c_str(), ec.message().c_str());
      return false;
    }

    const std::filesystem::path yaml_path =
        std::filesystem::path(output_dir_) / "calibration.yml";
    const std::filesystem::path csv_path =
        std::filesystem::path(output_dir_) / "views.csv";
    const std::filesystem::path snippet_path =
        std::filesystem::path(output_dir_) / "camera_info_snippet.txt";

    WriteCalibrationYaml(yaml_path, camera_matrix, distortion, rms,
                         static_cast<int>(final_views.size()));
    WriteViewsCsv(csv_path, final_views, final_per_view_rms);
    WriteCameraInfoSnippet(snippet_path, camera_matrix, distortion);

    last_rms_ = rms;
    last_saved_yaml_ = yaml_path.string();
    XR_LOG_PASS("Camera calibration saved: views=%llu rms=%.4f yaml=%s",
                static_cast<unsigned long long>(final_views.size()), rms,
                last_saved_yaml_.c_str());
    return true;
  }

  void WriteCalibrationYaml(const std::filesystem::path& path,
                            const cv::Mat& camera_matrix,
                            const cv::Mat& distortion,
                            double rms,
                            int views) const
  {
    cv::FileStorage fs(path.string(), cv::FileStorage::WRITE);
    fs << "image_width" << static_cast<int>(CameraInfoV.width);
    fs << "image_height" << static_cast<int>(CameraInfoV.height);
    fs << "rows" << cfg_.rows;
    fs << "cols" << cfg_.cols;
    fs << "marker_mm" << cfg_.marker_mm;
    fs << "square_mm" << SquareMm(cfg_);
    fs << "dictionary" << "aruco_original";
    fs << "generator" << "GShang ChArUco: square = marker * 9 / 7";
    fs << "views" << views;
    fs << "rms" << rms;
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_coefficients" << distortion;
  }

  void WriteViewsCsv(const std::filesystem::path& path,
                     const std::vector<Candidate>& views,
                     const std::vector<double>& per_view_rms) const
  {
    std::ofstream csv(path);
    csv << "frame_index,timestamp_us,used_markers,homography_rms,per_view_reprojection_rms\n";
    for (std::size_t i = 0; i < views.size(); ++i)
    {
      csv << views[i].frame_index << ","
          << views[i].timestamp_us << ","
          << views[i].used_markers << ","
          << views[i].homography_rms << ","
          << (i < per_view_rms.size() ? per_view_rms[i] : -1.0) << "\n";
    }
  }

  void WriteCameraInfoSnippet(const std::filesystem::path& path,
                              const cv::Mat& camera_matrix,
                              const cv::Mat& distortion) const
  {
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
    const cv::Mat flat = distortion.reshape(1, 1);
    for (int i = 0; i < std::min(flat.cols, 14); ++i)
    {
      if (i != 0)
      {
        out << ", ";
      }
      out << flat.at<double>(0, i);
    }
    out << "]\n";

    out << "projection_matrix: ["
        << camera_matrix.at<double>(0, 0) << ", 0.0, "
        << camera_matrix.at<double>(0, 2) << ", 0.0, 0.0, "
        << camera_matrix.at<double>(1, 1) << ", "
        << camera_matrix.at<double>(1, 2) << ", 0.0, 0.0, 0.0, 1.0, 0.0]\n";
  }

  void SaveDebugImage(const cv::Mat& image,
                      const std::vector<std::vector<cv::Point2f>>& corners,
                      const cv::Mat& ids,
                      const Candidate& candidate) const
  {
    cv::Mat debug = image.clone();
    if (!ids.empty())
    {
      cv::aruco::drawDetectedMarkers(debug, corners, ids);
    }

    std::ostringstream label;
    label << "view=" << accepted_.size()
          << " markers=" << candidate.used_markers
          << " H=" << std::fixed << std::setprecision(3)
          << candidate.homography_rms;
    cv::putText(debug, label.str(), {20, 45}, cv::FONT_HERSHEY_SIMPLEX,
                1.0, {0, 255, 0}, 2, cv::LINE_AA);

    std::ostringstream name;
    name << "view_" << std::setw(4) << std::setfill('0') << accepted_.size()
         << "_frame_" << candidate.frame_index << ".jpg";
    const std::filesystem::path path = std::filesystem::path(debug_dir_) / name.str();
    try
    {
      cv::imwrite(path.string(), debug);
    }
    catch (const cv::Exception& e)
    {
      XR_LOG_WARN("Camera calibration: failed to write debug image %s: %s",
                  path.string().c_str(), e.what());
    }
  }

  mutable std::mutex mutex_;
  std::atomic<bool> active_fast_{false};
  bool active_{false};
  bool finished_{false};
  bool unsupported_encoding_logged_{false};
  Config cfg_{};
  BoardMap board_{};
  cv::Ptr<cv::aruco::Dictionary> dictionary_{};
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_{};
  std::vector<Candidate> accepted_{};
  std::string output_dir_{};
  std::string debug_dir_{};
  std::string last_saved_yaml_{};
  double last_rms_{-1.0};
  uint64_t frame_index_{0};
  uint64_t swallowed_frames_{0};
  uint64_t processed_frames_{0};
  uint64_t detected_frames_{0};
  uint64_t last_accept_timestamp_us_{0};
};
