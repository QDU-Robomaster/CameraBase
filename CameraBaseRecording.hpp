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
 *
 * @tparam CameraInfoV 编译期相机信息，提供图像尺寸、stride、编码和内参。
 */
template <auto CameraInfoV>
class CameraBaseRecording
{
 public:
  /**
   * @brief 单帧原始图像字节数。
   */
  static constexpr std::size_t image_bytes =
      static_cast<std::size_t>(CameraInfoV.step) * static_cast<std::size_t>(CameraInfoV.height);

  /**
   * @brief 当前是否正在记录图像帧。
   */
  bool Enabled() const { return enabled_; }

  /**
   * @brief 当前输出目录视图；记录中返回临时目录，Finalize 后返回最终目录。
   */
  std::string_view OutputDirView() const { return output_dir_; }

  /**
   * @brief 当前输出目录 C 字符串，供旧接口和日志使用。
   */
  const char* OutputDir() const { return output_dir_.c_str(); }

  /**
   * @brief 当前记录包文件名前缀。
   */
  std::string_view FileStemView() const { return file_stem_; }

  /**
   * @brief 当前记录包文件名前缀 C 字符串。
   */
  const char* FileStem() const { return file_stem_.c_str(); }

  /**
   * @brief 初始化内录目录、断电恢复标记和输出文件。
   *
   * 未启用内录时仍会扫描默认根目录，整理上次异常断电留下的 `.recording`
   * 标记和 `.tmp` 目录。
   *
   * @param camera_name 相机名，用于生成默认记录包 stem。
   * @param param 内录运行参数。
   */
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

  /**
   * @brief 关闭输出文件，并把 `.tmp` 目录整理成最终记录包。
   */
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

  /**
   * @brief 写入一帧原始图像和对应帧索引 CSV。
   *
   * @param data 图像原始字节，布局必须与 `CameraInfoV` 一致。
   * @param timestamp_us 图像传感器侧时间戳，单位微秒。
   * @return true 表示写入成功或未启用内录；false 表示文件写入失败。
   */
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
  /**
   * @brief `.recording` 标记文件中保存的恢复信息。
   */
  struct Marker
  {
    std::string stem;  ///< 记录包文件名前缀。
    std::filesystem::path temp_dir;  ///< 断电前正在写入的临时目录。
    std::filesystem::path final_dir;  ///< 恢复完成后应得到的最终目录。
  };

  /**
   * @brief 把相机名整理成可用于文件名的安全字符串。
   */
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

  /**
   * @brief 生成本地时间戳字符串，格式为 `YYYYMMDD_HHMMSS`。
   */
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

  /**
   * @brief 根据时间戳和相机名生成记录包文件名前缀。
   */
  static std::string MakeRecordingFileStem(std::string_view camera_name)
  {
    return MakeTimestamp() + "_" + SanitizeName(camera_name);
  }

  /**
   * @brief 默认记录根目录。
   */
  static std::filesystem::path RecordingRoot()
  {
    return std::filesystem::path("runs") / "camera_record";
  }

  /**
   * @brief 返回路径父目录；无父目录时返回当前目录。
   */
  static std::filesystem::path ParentOrCurrent(const std::filesystem::path& path)
  {
    const auto parent = path.parent_path();
    return parent.empty() ? std::filesystem::path(".") : parent;
  }

  /**
   * @brief 根据 stem 生成默认最终记录目录。
   */
  static std::string MakeDefaultRecordingDir(std::string_view file_stem)
  {
    return (RecordingRoot() / std::string(file_stem)).string();
  }

  /**
   * @brief 根据最终目录生成同级 `.tmp` 临时目录路径。
   */
  static std::filesystem::path MakeTempRecordingDir(const std::filesystem::path& final_dir)
  {
    return ParentOrCurrent(final_dir) / (final_dir.filename().string() + ".tmp");
  }

  /**
   * @brief 根据最终目录和 stem 生成同级 `.recording` 标记路径。
   */
  static std::filesystem::path MakeRecordingMarkerPath(
      const std::filesystem::path& final_dir, std::string_view stem)
  {
    return ParentOrCurrent(final_dir) / (std::string(stem) + ".recording");
  }

  /**
   * @brief 去掉字符串首尾空白。
   */
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

  /**
   * @brief 解析标记文件中的 `key: value` 行。
   */
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

  /**
   * @brief 从 `.recording` 标记文件读取恢复信息。
   */
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

  /**
   * @brief 写入断电恢复标记文件。
   */
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

  /**
   * @brief 写入当前 CameraInfo 的轻量 YAML 描述。
   */
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

  /**
   * @brief 将文本文件裁剪到最后一个完整换行，丢弃断电产生的半行。
   */
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

  /**
   * @brief 解析无符号 64 位整数 token。
   */
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

  /**
   * @brief 解析帧索引 CSV 的一行，返回 raw 偏移和大小。
   */
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

  /**
   * @brief 修复 raw 图像文件和帧索引 CSV 的断电尾部。
   */
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

  /**
   * @brief 修复记录包内所有允许存在半行尾部的文件。
   */
  static void RepairPackage(const std::filesystem::path& dir, std::string_view stem)
  {
    RepairFrameFiles(dir, stem);
    TrimTextFileToLastNewline(dir / (std::string(stem) + "_imu.csv"));
    TrimTextFileToLastNewline(dir / (std::string(stem) + "_sync.csv"));
  }

  /**
   * @brief 选择恢复后的最终目录，避免覆盖已经存在的完整记录包。
   */
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

  /**
   * @brief 扫描目录中的 `.recording` 标记并恢复未完成记录包。
   */
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

  /**
   * @brief 修复当前临时包并将其重命名为最终目录。
   */
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

  bool enabled_{false};  ///< 是否已经打开并正在写入记录文件。
  uint32_t flush_every_frames_{30};  ///< 每隔多少帧主动 flush，0 表示不按帧 flush。
  uint64_t frame_index_{0};  ///< 下一帧写入 CSV 的连续帧号。
  std::string name_{"camera"};  ///< 当前相机名副本，仅用于日志和 camera_info。
  std::string output_dir_{};  ///< 当前写入目录；记录中为 `.tmp`，完成后为最终目录。
  std::string final_output_dir_{};  ///< 记录包最终目录。
  std::string file_stem_{};  ///< 记录包内所有文件共享的前缀。
  std::filesystem::path marker_path_{};  ///< 断电恢复标记文件路径。
  std::ofstream frames_{};  ///< 原始图像二进制输出流。
  std::ofstream csv_{};  ///< 图像帧索引 CSV 输出流。
};
