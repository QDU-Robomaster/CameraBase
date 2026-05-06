#pragma once

#include "logger.hpp"

#include <array>
#include <cerrno>
#include <cctype>
#include <ctime>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

/**
 * @brief CameraBase 原始图像内录配置。
 */
struct CameraBaseRecordingParam
{
  bool enable = false;  ///< 是否在 CameraBase 生产者侧记录每帧原始图像。
  std::string_view output_dir = {};  ///< 为空时自动创建 runs/camera_record/...。
  bool overwrite = false;  ///< false 时拒绝覆盖已有同 stem 内录包。
  uint32_t flush_every_frames = 30;  ///< 每隔多少帧 flush 一次，0 表示只依赖析构。
};

/**
 * @brief CameraBase 生产者侧图像内录和断电恢复。
 */
template <auto CameraInfoV>
class CameraBaseRecording
{
 public:
  static constexpr std::size_t image_bytes =
      static_cast<std::size_t>(CameraInfoV.step) * static_cast<std::size_t>(CameraInfoV.height);

  bool Enabled() const { return enabled_; }

  std::string_view OutputDirView() const { return output_dir_; }

  const char* OutputDir() const { return output_dir_.c_str(); }

  std::string_view FileStemView() const { return file_stem_; }

  const char* FileStem() const { return file_stem_.c_str(); }

  void Open(std::string_view camera_name, const CameraBaseRecordingParam& param)
  {
    name_ = camera_name.empty() ? "camera" : std::string(camera_name);
    RecoverInterruptedRecordings(RecordingRoot());
    if (!param.enable)
    {
      return;
    }

    file_stem_ = MakeRecordingFileStem(name_);
    final_output_dir_ = param.output_dir.empty() ? MakeDefaultRecordingDir(file_stem_)
                                                 : std::string(param.output_dir);

    const std::filesystem::path final_dir(final_output_dir_);
    const std::filesystem::path temp_dir = MakeTempRecordingDir(final_dir);
    output_dir_ = temp_dir.string();
    marker_path_ = MakeRecordingMarkerPath(final_dir, file_stem_);
    flush_every_frames_ = param.flush_every_frames;

    const auto final_parent = ParentOrCurrent(final_dir);
    RecoverInterruptedRecordings(final_parent);

    std::error_code ec;
    std::filesystem::create_directories(final_parent, ec);
    if (ec)
    {
      XR_LOG_ERROR("CameraBase(%s): create recording dir failed %s: %s",
                   name_.c_str(), final_parent.string().c_str(), ec.message().c_str());
      throw std::runtime_error("CameraBase: create recording dir failed");
    }

    if (param.overwrite)
    {
      std::filesystem::remove_all(final_dir, ec);
      ec.clear();
      std::filesystem::remove_all(temp_dir, ec);
      ec.clear();
      std::filesystem::remove(marker_path_, ec);
      ec.clear();
    }
    else if (std::filesystem::exists(final_dir) || std::filesystem::exists(temp_dir) ||
             std::filesystem::exists(marker_path_))
    {
      XR_LOG_ERROR("CameraBase(%s): recording package already exists final=%s temp=%s",
                   name_.c_str(), final_dir.string().c_str(), temp_dir.string().c_str());
      throw std::runtime_error("CameraBase: recording package already exists");
    }

    WriteRecordingMarker(temp_dir, final_dir);
    std::filesystem::create_directories(temp_dir, ec);
    if (ec)
    {
      XR_LOG_ERROR("CameraBase(%s): create temp recording dir failed %s: %s",
                   name_.c_str(), temp_dir.string().c_str(), ec.message().c_str());
      throw std::runtime_error("CameraBase: create temp recording dir failed");
    }

    const std::filesystem::path dir(output_dir_);
    frames_.open(dir / (file_stem_ + "_frames.bin"), std::ios::binary | std::ios::trunc);
    csv_.open(dir / (file_stem_ + "_frames.csv"), std::ios::out | std::ios::trunc);
    if (!frames_.is_open() || !csv_.is_open())
    {
      XR_LOG_ERROR("CameraBase(%s): open recording files failed in %s",
                   name_.c_str(), output_dir_.c_str());
      throw std::runtime_error("CameraBase: open recording files failed");
    }

    WriteCameraInfo(dir / (file_stem_ + "_camera_info.yaml"));
    csv_ << "frame_index,camera_timestamp_us,offset_bytes,size_bytes\n";
    enabled_ = true;
    XR_LOG_PASS(
        "CameraBase(%s): recording enabled temp=%s final=%s stem=%s bytes_per_frame=%u",
        name_.c_str(), output_dir_.c_str(), final_output_dir_.c_str(),
        file_stem_.c_str(), static_cast<unsigned>(image_bytes));
  }

  void Close()
  {
    if (!enabled_)
    {
      return;
    }
    if (csv_.is_open())
    {
      csv_.flush();
      csv_.close();
    }
    if (frames_.is_open())
    {
      frames_.flush();
      frames_.close();
    }
    FinalizePackage();
    enabled_ = false;
  }

  bool Record(const std::array<uint8_t, image_bytes>& data, uint64_t timestamp_us)
  {
    if (!enabled_)
    {
      return true;
    }

    const auto offset_pos = frames_.tellp();
    if (offset_pos == std::streampos(-1))
    {
      XR_LOG_ERROR("CameraBase(%s): recording tellp failed", name_.c_str());
      return false;
    }
    const auto offset = static_cast<std::streamoff>(offset_pos);

    frames_.write(reinterpret_cast<const char*>(data.data()),
                  static_cast<std::streamsize>(image_bytes));
    csv_ << frame_index_ << "," << static_cast<unsigned long long>(timestamp_us)
         << "," << static_cast<unsigned long long>(offset) << ","
         << static_cast<unsigned long long>(image_bytes) << "\n";
    if (!frames_.good() || !csv_.good())
    {
      XR_LOG_ERROR("CameraBase(%s): recording write failed at frame=%llu",
                   name_.c_str(), static_cast<unsigned long long>(frame_index_));
      return false;
    }

    ++frame_index_;
    if (flush_every_frames_ != 0 && (frame_index_ % flush_every_frames_) == 0)
    {
      frames_.flush();
      csv_.flush();
    }
    return true;
  }

 private:
  struct Marker
  {
    std::string stem;
    std::filesystem::path temp_dir;
    std::filesystem::path final_dir;
  };

  static std::string SanitizeName(std::string_view camera_name)
  {
    if (camera_name.empty())
    {
      return "camera";
    }

    std::string safe;
    safe.reserve(camera_name.size());
    for (char ch : camera_name)
    {
      const bool ok = (ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') ||
                      (ch >= 'a' && ch <= 'z') || ch == '_' || ch == '-';
      safe.push_back(ok ? ch : '_');
    }
    return safe;
  }

  static std::string MakeTimestamp()
  {
    std::time_t now = std::time(nullptr);
    std::tm local{};
#if defined(_WIN32)
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif

    std::ostringstream stamp;
    stamp << std::put_time(&local, "%Y%m%d_%H%M%S");
    return stamp.str();
  }

  static std::string MakeRecordingFileStem(std::string_view camera_name)
  {
    return MakeTimestamp() + "_" + SanitizeName(camera_name);
  }

  static std::filesystem::path RecordingRoot()
  {
    return std::filesystem::path("runs") / "camera_record";
  }

  static std::filesystem::path ParentOrCurrent(const std::filesystem::path& path)
  {
    const auto parent = path.parent_path();
    return parent.empty() ? std::filesystem::path(".") : parent;
  }

  static std::string MakeDefaultRecordingDir(std::string_view file_stem)
  {
    return (RecordingRoot() / std::string(file_stem)).string();
  }

  static std::filesystem::path MakeTempRecordingDir(const std::filesystem::path& final_dir)
  {
    return ParentOrCurrent(final_dir) / (final_dir.filename().string() + ".tmp");
  }

  static std::filesystem::path MakeRecordingMarkerPath(
      const std::filesystem::path& final_dir, std::string_view stem)
  {
    return ParentOrCurrent(final_dir) / (std::string(stem) + ".recording");
  }

  static std::string Trim(std::string value)
  {
    while (!value.empty() &&
           std::isspace(static_cast<unsigned char>(value.front())) != 0)
    {
      value.erase(value.begin());
    }
    while (!value.empty() &&
           std::isspace(static_cast<unsigned char>(value.back())) != 0)
    {
      value.pop_back();
    }
    return value;
  }

  static bool ParseMarkerLine(const std::string& line, const char* key, std::string& value)
  {
    const std::string prefix = std::string(key) + ":";
    if (line.rfind(prefix, 0) != 0)
    {
      return false;
    }
    value = Trim(line.substr(prefix.size()));
    return true;
  }

  static bool LoadMarker(const std::filesystem::path& marker_path, Marker& marker)
  {
    std::ifstream input(marker_path);
    if (!input.is_open())
    {
      return false;
    }

    std::string line;
    while (std::getline(input, line))
    {
      std::string value;
      if (ParseMarkerLine(line, "stem", value))
      {
        marker.stem = value;
      }
      else if (ParseMarkerLine(line, "temp_dir", value))
      {
        marker.temp_dir = value;
      }
      else if (ParseMarkerLine(line, "final_dir", value))
      {
        marker.final_dir = value;
      }
    }
    return !marker.stem.empty() && !marker.temp_dir.empty() && !marker.final_dir.empty();
  }

  void WriteRecordingMarker(const std::filesystem::path& temp_dir,
                            const std::filesystem::path& final_dir) const
  {
    std::ofstream marker(marker_path_, std::ios::out | std::ios::trunc);
    if (!marker.is_open())
    {
      XR_LOG_ERROR("CameraBase(%s): open recording marker failed %s",
                   name_.c_str(), marker_path_.string().c_str());
      throw std::runtime_error("CameraBase: open recording marker failed");
    }

    marker << "recording_state: active\n";
    marker << "stem: " << file_stem_ << "\n";
    marker << "temp_dir: " << temp_dir.string() << "\n";
    marker << "final_dir: " << final_dir.string() << "\n";
    marker.flush();
    if (!marker.good())
    {
      XR_LOG_ERROR("CameraBase(%s): write recording marker failed %s",
                   name_.c_str(), marker_path_.string().c_str());
      throw std::runtime_error("CameraBase: write recording marker failed");
    }
  }

  void WriteCameraInfo(const std::filesystem::path& path) const
  {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
      XR_LOG_ERROR("CameraBase(%s): open camera_info.yaml failed %s",
                   name_.c_str(), path.string().c_str());
      throw std::runtime_error("CameraBase: open recording camera info failed");
    }

    out << "width: " << static_cast<unsigned>(CameraInfoV.width) << "\n";
    out << "height: " << static_cast<unsigned>(CameraInfoV.height) << "\n";
    out << "step: " << static_cast<unsigned>(CameraInfoV.step) << "\n";
    out << "encoding: " << static_cast<unsigned>(CameraInfoV.encoding) << "\n";
    out << "image_bytes: " << static_cast<unsigned long long>(image_bytes) << "\n";
    out << "camera_name: " << name_ << "\n";
    out << "recording_stem: " << file_stem_ << "\n";
  }

  static void TrimTextFileToLastNewline(const std::filesystem::path& path)
  {
    std::error_code ec;
    if (!std::filesystem::exists(path, ec))
    {
      return;
    }

    std::ifstream input(path, std::ios::binary);
    if (!input.is_open())
    {
      return;
    }
    std::string data((std::istreambuf_iterator<char>(input)),
                     std::istreambuf_iterator<char>());
    if (data.empty() || data.back() == '\n')
    {
      return;
    }

    const auto last_newline = data.find_last_of('\n');
    const auto keep_size = last_newline == std::string::npos
                               ? 0U
                               : static_cast<uintmax_t>(last_newline + 1);
    std::filesystem::resize_file(path, keep_size, ec);
  }

  static bool ParseUint64Token(const std::string& token, uint64_t& value)
  {
    std::string trimmed = Trim(token);
    if (trimmed.empty())
    {
      return false;
    }
    errno = 0;
    char* end = nullptr;
    const auto parsed = std::strtoull(trimmed.c_str(), &end, 10);
    if (errno != 0 || end == trimmed.c_str() || *end != '\0')
    {
      return false;
    }
    value = static_cast<uint64_t>(parsed);
    return true;
  }

  static bool ParseFrameCsvRow(const std::string& line, uint64_t& offset, uint64_t& size)
  {
    std::array<uint64_t, 4> values{};
    std::stringstream stream(line);
    std::string token;
    std::size_t index = 0;
    while (std::getline(stream, token, ','))
    {
      if (index >= values.size() || !ParseUint64Token(token, values[index]))
      {
        return false;
      }
      ++index;
    }
    if (index != values.size())
    {
      return false;
    }
    offset = values[2];
    size = values[3];
    return true;
  }

  static void RepairFrameFiles(const std::filesystem::path& dir, std::string_view stem)
  {
    const auto frames_path = dir / (std::string(stem) + "_frames.bin");
    const auto csv_path = dir / (std::string(stem) + "_frames.csv");
    std::error_code ec;
    if (!std::filesystem::exists(frames_path, ec) ||
        !std::filesystem::exists(csv_path, ec))
    {
      return;
    }

    TrimTextFileToLastNewline(csv_path);
    const auto raw_size = std::filesystem::file_size(frames_path, ec);
    if (ec)
    {
      return;
    }

    std::ifstream input(csv_path);
    if (!input.is_open())
    {
      return;
    }

    std::string line;
    std::ostringstream repaired;
    uint64_t valid_raw_end = 0;
    if (std::getline(input, line))
    {
      repaired << line << "\n";
    }
    while (std::getline(input, line))
    {
      if (line.empty())
      {
        continue;
      }

      uint64_t offset = 0;
      uint64_t size = 0;
      if (!ParseFrameCsvRow(line, offset, size))
      {
        break;
      }
      if (size == 0 || offset > raw_size || size > raw_size - offset)
      {
        break;
      }

      repaired << line << "\n";
      valid_raw_end = offset + size;
    }

    std::ofstream output(csv_path, std::ios::out | std::ios::trunc);
    output << repaired.str();
    output.flush();
    if (valid_raw_end < raw_size)
    {
      std::filesystem::resize_file(frames_path, valid_raw_end, ec);
    }
  }

  static void RepairPackage(const std::filesystem::path& dir, std::string_view stem)
  {
    RepairFrameFiles(dir, stem);
    TrimTextFileToLastNewline(dir / (std::string(stem) + "_imu.csv"));
    TrimTextFileToLastNewline(dir / (std::string(stem) + "_sync.csv"));
  }

  static std::filesystem::path PickRecoveryFinalDir(const std::filesystem::path& final_dir)
  {
    std::error_code ec;
    if (!std::filesystem::exists(final_dir, ec))
    {
      return final_dir;
    }

    for (uint32_t index = 1; index < 1000; ++index)
    {
      auto candidate = final_dir;
      candidate += "_recovered_" + std::to_string(index);
      if (!std::filesystem::exists(candidate, ec))
      {
        return candidate;
      }
    }
    return final_dir;
  }

  static void RecoverInterruptedRecordings(const std::filesystem::path& root)
  {
    std::error_code ec;
    if (root.empty() || !std::filesystem::exists(root, ec) ||
        !std::filesystem::is_directory(root, ec))
    {
      return;
    }

    for (const auto& entry : std::filesystem::directory_iterator(root, ec))
    {
      if (ec || !entry.is_regular_file(ec) || entry.path().extension() != ".recording")
      {
        continue;
      }

      Marker marker{};
      if (!LoadMarker(entry.path(), marker))
      {
        XR_LOG_WARN("CameraBase: skip invalid recording marker %s",
                    entry.path().string().c_str());
        continue;
      }

      if (std::filesystem::exists(marker.temp_dir, ec))
      {
        RepairPackage(marker.temp_dir, marker.stem);
        const auto final_dir = PickRecoveryFinalDir(marker.final_dir);
        std::filesystem::rename(marker.temp_dir, final_dir, ec);
        if (ec)
        {
          XR_LOG_WARN("CameraBase: recover recording rename failed temp=%s final=%s err=%s",
                      marker.temp_dir.string().c_str(), final_dir.string().c_str(),
                      ec.message().c_str());
          ec.clear();
          continue;
        }
        XR_LOG_PASS("CameraBase: recovered interrupted recording %s",
                    final_dir.string().c_str());
      }
      else if (std::filesystem::exists(marker.final_dir, ec))
      {
        XR_LOG_PASS("CameraBase: recording already finalized %s",
                    marker.final_dir.string().c_str());
      }

      std::filesystem::remove(entry.path(), ec);
      ec.clear();
    }
  }

  void FinalizePackage()
  {
    const std::filesystem::path temp_dir(output_dir_);
    const std::filesystem::path final_dir(final_output_dir_);
    RepairPackage(temp_dir, file_stem_);

    std::error_code ec;
    std::filesystem::rename(temp_dir, final_dir, ec);
    if (ec)
    {
      XR_LOG_ERROR("CameraBase(%s): finalize recording failed temp=%s final=%s err=%s",
                   name_.c_str(), temp_dir.string().c_str(), final_dir.string().c_str(),
                   ec.message().c_str());
      return;
    }

    std::filesystem::remove(marker_path_, ec);
    output_dir_ = final_output_dir_;
    XR_LOG_PASS("CameraBase(%s): recording finalized dir=%s",
                name_.c_str(), output_dir_.c_str());
  }

  bool enabled_{false};
  uint32_t flush_every_frames_{30};
  uint64_t frame_index_{0};
  std::string name_{"camera"};
  std::string output_dir_{};
  std::string final_output_dir_{};
  std::string file_stem_{};
  std::filesystem::path marker_path_{};
  std::ofstream frames_{};
  std::ofstream csv_{};
};
