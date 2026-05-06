#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 相机基础类型、像素编码和图像/IMU 同步 ABI
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "message.hpp"
#include "logger.hpp"
#include "ramfs.hpp"

#include <array>
#include <cerrno>
#include <cctype>
#include <ctime>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

/**
 * @class CameraTypes
 * @brief 相机静态类型定义容器。
 */
class CameraTypes
{
 public:
  /**
   * @enum Encoding
   * @brief 图像像素编码格式。
   */
  enum Encoding : uint8_t
  {
    INVALID = 0,   ///< 无效或未初始化的编码占位。
    RGB8,          ///< 8 位 RGB 三通道。
    BGR8,          ///< 8 位 BGR 三通道。
    RGBA8,         ///< 8 位 RGBA 四通道。
    BGRA8,         ///< 8 位 BGRA 四通道。
    RGB16,         ///< 16 位 RGB 三通道。
    BGR16,         ///< 16 位 BGR 三通道。
    RGBA16,        ///< 16 位 RGBA 四通道。
    BGRA16,        ///< 16 位 BGRA 四通道。
    MONO8,         ///< 8 位单通道灰度图。
    MONO16,        ///< 16 位单通道灰度图。
    BAYER_RGGB8,   ///< 8 位 Bayer，RGGB 排列。
    BAYER_GRBG8,   ///< 8 位 Bayer，GRBG 排列。
    BAYER_GBRG8,   ///< 8 位 Bayer，GBRG 排列。
    BAYER_BGGR8,   ///< 8 位 Bayer，BGGR 排列。
    BAYER_RGGB16,  ///< 16 位 Bayer，RGGB 排列。
    BAYER_GRBG16,  ///< 16 位 Bayer，GRBG 排列。
    BAYER_GBRG16,  ///< 16 位 Bayer，GBRG 排列。
    BAYER_BGGR16,  ///< 16 位 Bayer，BGGR 排列。
    YUV422         ///< 打包 YUV 4:2:2。
  };

  /**
   * @enum DistortionModel
   * @brief 相机畸变模型。
   */
  enum class DistortionModel : uint8_t
  {
    NONE = 0,             ///< 无畸变模型。
    PLUMB_BOB,            ///< Brown-Conrady / plumb_bob。
    RATIONAL_POLYNOMIAL,  ///< OpenCV 扩展有理多项式模型。
    EQUIDISTANT,          ///< 等距鱼眼模型。
    FOV,                  ///< FOV 畸变模型。
    OMNI,                 ///< 统一全向模型。
    EXTENDED_UNIFIED,     ///< 扩展统一相机模型。
    DOUBLE_SPHERE,        ///< 双球模型。
    THIN_PRISM,           ///< 薄棱镜模型。
    UNKNOWN               ///< 未知或自定义模型。
  };

  /**
   * @struct CameraInfo
   * @brief 编译期静态相机描述。
   */
  struct CameraInfo
  {
    uint32_t width{};  ///< 图像宽度，单位像素。
    uint32_t height{};  ///< 图像高度，单位像素。
    uint32_t step{};  ///< 每行字节数，即 stride。
    Encoding encoding{};  ///< 像素编码格式。
    std::array<double, 9> camera_matrix;  ///< 3x3 相机内参矩阵 K，按行优先存储。
    DistortionModel distortion_model{};  ///< 畸变模型类型。
    std::array<double, 14> distortion_coefficients;  ///< 畸变参数，布局跟随 ROS CameraInfo。
    std::array<double, 9> rectification_matrix;  ///< 3x3 校正旋转矩阵 R，按行优先存储。
    std::array<double, 12> projection_matrix;  ///< 3x4 投影矩阵 P，按行优先存储。
  };

  /**
   * @struct PnPDistCoeffs
   * @brief PnP 使用的固定尺寸畸变系数描述。
   *
   * @note 这里保持为纯静态数据，便于编译期生成，再由运行时封装成 `cv::Mat`。
   */
  struct PnPDistCoeffs
  {
    std::array<double, 8> values{};  ///< 当前 PnP 后端会消费的畸变系数。
    uint8_t size{};  ///< `values` 中有效系数个数。
    bool uses_rational_polynomial_extension{};  ///< 是否命中 8 项 rational 扩展路径。
    bool requires_undistort_first{};  ///< 当前模型是否应先去畸变再进入 PnP。
  };

  /**
   * @brief 按当前静态相机模型生成 PnP 所需的固定畸变系数描述。
   *
   * @note 当前只直接支持 OpenCV 常用 pinhole / rational 两类输入。
   *       其他模型后续应先做去畸变，再按无畸变 pinhole 进入 PnP。
   */
  [[nodiscard]] static constexpr PnPDistCoeffs BuildPnPDistCoeffs(
      const CameraInfo& info)
  {
    PnPDistCoeffs dc{};
    switch (info.distortion_model)
    {
      case DistortionModel::NONE:
        break;

      case DistortionModel::PLUMB_BOB:
        dc.values[0] = info.distortion_coefficients[0];
        dc.values[1] = info.distortion_coefficients[1];
        dc.values[2] = info.distortion_coefficients[2];
        dc.values[3] = info.distortion_coefficients[3];
        dc.values[4] = info.distortion_coefficients[4];
        dc.size = 5;
        break;

      case DistortionModel::RATIONAL_POLYNOMIAL:
        dc.values[0] = info.distortion_coefficients[0];
        dc.values[1] = info.distortion_coefficients[1];
        dc.values[2] = info.distortion_coefficients[2];
        dc.values[3] = info.distortion_coefficients[3];
        dc.values[4] = info.distortion_coefficients[4];
        dc.values[5] = info.distortion_coefficients[5];
        dc.values[6] = info.distortion_coefficients[6];
        dc.values[7] = info.distortion_coefficients[7];
        dc.size = 8;
        dc.uses_rational_polynomial_extension = true;
        break;

      case DistortionModel::EQUIDISTANT:
      case DistortionModel::FOV:
      case DistortionModel::OMNI:
      case DistortionModel::EXTENDED_UNIFIED:
      case DistortionModel::DOUBLE_SPHERE:
      case DistortionModel::THIN_PRISM:
      case DistortionModel::UNKNOWN:
      default:
        dc.requires_undistort_first = true;
        break;
    }
    return dc;
  }
};

#include "CameraBaseIntrinsicSanity.hpp"
#include "CameraBaseCalibration.hpp"

/**
 * @class CameraBase
 * @brief 编译期绑定相机静态信息的基类。
 */
template <CameraTypes::CameraInfo CameraInfoV>
class CameraBase
{
 public:
  using Encoding = CameraTypes::Encoding;
  using DistortionModel = CameraTypes::DistortionModel;
  using CameraInfo = CameraTypes::CameraInfo;
  static constexpr std::size_t image_alignment = 64;

  // 这一份静态相机描述会被整条视觉链共享，后续模块可直接把分辨率、
  // 内参、畸变模型当作编译期常量使用。
  static inline constexpr CameraInfo camera_info = CameraInfoV;
  static constexpr std::size_t image_bytes =
      static_cast<std::size_t>(camera_info.step) * static_cast<std::size_t>(camera_info.height);

  /**
   * @struct ImageFrame
   * @brief 固定尺寸的原始图像帧载荷。
   */
  struct alignas(image_alignment) ImageFrame
  {
    LibXR::MicrosecondTimestamp timestamp_us;  ///< 图像传感器侧时间戳，单位微秒。
    alignas(image_alignment) std::array<uint8_t, image_bytes> data;  ///< 图像字节负载。
  };

  /**
   * @struct ImuStamped
   * @brief 与图像同步搬运的位姿与惯导采样。
   */
  struct ImuStamped
  {
    LibXR::MicrosecondTimestamp timestamp_us;  ///< 对应图像的传感器侧时间戳，单位微秒。
    std::array<float, 4> rotation_wxyz;  ///< 姿态四元数，顺序为 wxyz。
    std::array<float, 3> translation_xyz;  ///< 相机平移，单位米。
    std::array<float, 3> angular_velocity_xyz;  ///< 角速度，单位 rad/s。
    std::array<float, 3> linear_acceleration_xyz;  ///< 线加速度，单位 m/s^2。
  };

  /// 图像生产者提交一帧后，由 sink 返回下一个可写槽位。
  using ImageCommitCallback = LibXR::Callback<ImageFrame*&>;

  /**
   * @brief CameraBase 原始图像内录配置。
   */
  struct RecordingParam
  {
    bool enable = false;  ///< 是否在 CameraBase 生产者侧记录每帧原始图像。
    std::string_view output_dir = {};  ///< 为空时自动创建 runs/camera_record/...。
    bool overwrite = false;  ///< false 时拒绝覆盖已有同 stem 内录文件。
    uint32_t flush_every_frames = 30;  ///< 每隔多少帧 flush 一次，0 表示只依赖析构。
  };

  // 共享图像和 imu 都会跨模块搬运，这里只保留真正影响 ABI 的约束。
  static_assert(sizeof(LibXR::MicrosecondTimestamp) == sizeof(uint64_t),
                "CameraBase timestamp ABI must stay 64-bit");
  static_assert(alignof(LibXR::MicrosecondTimestamp) == alignof(uint64_t),
                "CameraBase timestamp ABI alignment changed");
  static_assert(std::is_standard_layout_v<LibXR::MicrosecondTimestamp>,
                "CameraBase timestamp wrapper must stay standard layout");
  static_assert(camera_info.width > 0, "CameraBase requires non-zero width");
  static_assert(camera_info.height > 0, "CameraBase requires non-zero height");
  static_assert(camera_info.step > 0, "CameraBase requires non-zero step");
  static_assert(image_bytes > 0, "CameraBase requires non-zero image bytes");
  static_assert(CameraBaseIntrinsicSanity::CameraInfoReasonable(camera_info),
                "CameraBase compile-time CameraInfo intrinsics are unreasonable");
  static_assert(std::is_standard_layout_v<ImageFrame>,
                "CameraBase::ImageFrame must be standard layout");
  static_assert(std::is_standard_layout_v<ImuStamped>,
                "CameraBase::ImuStamped must be standard layout");
  static_assert(alignof(ImageFrame) >= image_alignment,
                "CameraBase::ImageFrame alignment is too small");
  static_assert(offsetof(ImageFrame, data) % image_alignment == 0,
                "CameraBase::ImageFrame image payload must stay aligned");

  CameraBase(LibXR::HardwareContainer& hw, std::string_view name = "camera",
             std::string_view image_topic_name = "camera_image",
             std::string_view imu_topic_name = "camera_imu",
             RecordingParam recording = {})
      : name_(name),
        image_topic_name_(image_topic_name),
        imu_topic_name_(imu_topic_name),
        cmd_file_(LibXR::RamFS::CreateFile(name_.c_str(), CommandFun, this)),
        imu_topic_(LibXR::Topic::FindOrCreate<ImuStamped>(imu_topic_name_.c_str()))
  {
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
    OpenRecording(recording);
  }

  virtual ~CameraBase() { CloseRecording(); }

  virtual void SetExposure(double exposure) = 0;
  virtual void SetGain(double gain) = 0;

  std::string_view NameView() const { return name_; }
  const char* Name() const { return name_.c_str(); }

  std::string_view ImageTopicNameView() const { return image_topic_name_; }
  const char* ImageTopicName() const { return image_topic_name_.c_str(); }

  std::string_view ImuTopicNameView() const { return imu_topic_name_; }
  const char* ImuTopicName() const { return imu_topic_name_.c_str(); }

  bool RecordingEnabled() const { return recording_enabled_; }

  std::string_view RecordingOutputDirView() const { return recording_output_dir_; }

  const char* RecordingOutputDir() const { return recording_output_dir_.c_str(); }

  std::string_view RecordingFileStemView() const { return recording_file_stem_; }

  const char* RecordingFileStem() const { return recording_file_stem_.c_str(); }

  /// 发布已经由同步模块对齐完成的 IMU 包。
  void PublishImu(ImuStamped imu) { imu_topic_.Publish(imu); }

  /// 注册唯一图像 sink；具体相机只写入当前租借槽位，不拥有后级队列。
  bool RegisterImageSink(ImageFrame* initial_image, ImageCommitCallback commit_callback)
  {
    if (initial_image == nullptr || commit_callback.Empty())
    {
      XR_LOG_ERROR("CameraBase(%s): image sink registration got invalid argument", name_.c_str());
      return false;
    }
    if (!image_commit_callback_.Empty())
    {
      XR_LOG_ERROR("CameraBase(%s): image sink already registered", name_.c_str());
      return false;
    }

    writable_image_ = initial_image;
    image_commit_callback_ = commit_callback;
    return true;
  }

  bool ImageSinkReady() const
  {
    return !image_commit_callback_.Empty() && writable_image_ != nullptr;
  }

  ImageFrame* GetWritableImage() { return ImageSinkReady() ? writable_image_ : nullptr; }

  bool CommitImage()
  {
    if (!ImageSinkReady() || image_commit_callback_.Empty())
    {
      return false;
    }

    if (calibration_.ProcessFrame(writable_image_->data.data(),
                                  static_cast<uint64_t>(writable_image_->timestamp_us)))
    {
      return true;
    }

    if (!RecordImageFrame(*writable_image_))
    {
      return false;
    }

    ImageFrame* next_image = nullptr;
    image_commit_callback_.Run(false, next_image);
    if (next_image == nullptr)
    {
      XR_LOG_ERROR("CameraBase(%s): image sink callback returned null writable image",
                   name_.c_str());
      return false;
    }

    writable_image_ = next_image;
    return true;
  }

  static bool ParsePositiveIntArg(const char* text, int& value)
  {
    if (text == nullptr || *text == '\0')
    {
      return false;
    }

    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed <= 0 ||
        parsed > static_cast<long>(std::numeric_limits<int>::max()))
    {
      return false;
    }

    value = static_cast<int>(parsed);
    return true;
  }

  // 调试/bring-up 阶段的命令入口。
  static int CommandFun(CameraBase* self, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Camera: %s\n\n">(self->name_.c_str());
      LibXR::STDIO::Printf<"用法:\r\n">();
      LibXR::STDIO::Printf<"  set_exposure <曝光>\r\n">();
      LibXR::STDIO::Printf<"  set_gain <增益>\r\n">();
      LibXR::STDIO::Printf<"  cali <标记尺寸mm> <列数> <行数>  例：cali 25mm 8 6\r\n">();
      LibXR::STDIO::Printf<"  cali status\r\n">();
      LibXR::STDIO::Printf<"  cali save\r\n">();
      LibXR::STDIO::Printf<"  cali stop\r\n">();
      return 0;
    }
    else if (strcmp(argv[1], "cali") == 0)
    {
      if (argc == 5)
      {
        int cols = 0;
        int rows = 0;
        if (!ParsePositiveIntArg(argv[3], cols) ||
            !ParsePositiveIntArg(argv[4], rows))
        {
          LibXR::STDIO::Printf<"标定板列数和行数必须是正整数：cols=%s rows=%s\r\n">(
              argv[3], argv[4]);
          return -1;
        }
        return self->calibration_.Start(argv[2], cols, rows, self->name_) ? 0 : -1;
      }

      if (argc == 2 || (argc == 3 && strcmp(argv[2], "status") == 0))
      {
        const std::string status = self->calibration_.StatusString();
        LibXR::STDIO::Printf<"%s\r\n">(status.c_str());
        return 0;
      }

      if (argc == 3 && strcmp(argv[2], "save") == 0)
      {
        if (self->calibration_.SaveAndStop())
        {
          std::exit(EXIT_SUCCESS);
        }
        return -1;
      }

      if (argc == 3 && strcmp(argv[2], "stop") == 0)
      {
        self->calibration_.Stop();
        return 0;
      }
    }
    else if (argc == 3)
    {
      if (strcmp(argv[1], "set_exposure") == 0)
      {
        self->SetExposure(atof(argv[2]));
        return 0;
      }
      if (strcmp(argv[1], "set_gain") == 0)
      {
        self->SetGain(atof(argv[2]));
        return 0;
      }
    }

    LibXR::STDIO::Printf<"未知命令：%s\n">(argv[1]);
    return -1;
  }

 protected:
  static std::string SanitizeRecordingName(std::string_view camera_name)
  {
    std::string safe;
    if (camera_name.empty())
    {
      return "camera";
    }
    for (char ch : camera_name)
    {
      const bool ok = (ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') ||
                      (ch >= 'a' && ch <= 'z') || ch == '_' || ch == '-';
      safe.push_back(ok ? ch : '_');
    }
    return safe;
  }

  static std::string MakeRecordingTimestamp()
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
    return MakeRecordingTimestamp() + "_" + SanitizeRecordingName(camera_name);
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

  static std::filesystem::path MakeTempRecordingDir(
      const std::filesystem::path& final_dir)
  {
    return ParentOrCurrent(final_dir) / (final_dir.filename().string() + ".tmp");
  }

  static std::filesystem::path MakeRecordingMarkerPath(
      const std::filesystem::path& final_dir, std::string_view stem)
  {
    return ParentOrCurrent(final_dir) / (std::string(stem) + ".recording");
  }

  void OpenRecording(const RecordingParam& recording)
  {
    RecoverInterruptedRecordings(RecordingRoot());
    if (!recording.enable)
    {
      return;
    }

    recording_file_stem_ = MakeRecordingFileStem(name_);
    recording_final_output_dir_ = recording.output_dir.empty()
                                      ? MakeDefaultRecordingDir(recording_file_stem_)
                                      : std::string(recording.output_dir);
    const std::filesystem::path final_dir(recording_final_output_dir_);
    const std::filesystem::path temp_dir = MakeTempRecordingDir(final_dir);
    recording_output_dir_ = temp_dir.string();
    recording_marker_path_ = MakeRecordingMarkerPath(final_dir, recording_file_stem_);
    recording_flush_every_frames_ = recording.flush_every_frames;

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

    if (recording.overwrite)
    {
      std::filesystem::remove_all(final_dir, ec);
      ec.clear();
      std::filesystem::remove_all(temp_dir, ec);
      ec.clear();
      std::filesystem::remove(recording_marker_path_, ec);
      ec.clear();
    }
    else if (std::filesystem::exists(final_dir) || std::filesystem::exists(temp_dir) ||
             std::filesystem::exists(recording_marker_path_))
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

    const std::filesystem::path dir(recording_output_dir_);
    const auto frames_path = dir / (recording_file_stem_ + "_frames.bin");
    const auto csv_path = dir / (recording_file_stem_ + "_frames.csv");

    recording_frames_.open(frames_path, std::ios::binary | std::ios::trunc);
    recording_csv_.open(csv_path, std::ios::out | std::ios::trunc);
    if (!recording_frames_.is_open() || !recording_csv_.is_open())
    {
      XR_LOG_ERROR("CameraBase(%s): open recording files failed in %s",
                   name_.c_str(), recording_output_dir_.c_str());
      throw std::runtime_error("CameraBase: open recording files failed");
    }

    WriteRecordingCameraInfo(dir / (recording_file_stem_ + "_camera_info.yaml"));
    recording_csv_ << "frame_index,camera_timestamp_us,offset_bytes,size_bytes\n";
    recording_enabled_ = true;
    XR_LOG_PASS("CameraBase(%s): recording enabled temp=%s final=%s stem=%s bytes_per_frame=%u",
                name_.c_str(), recording_output_dir_.c_str(),
                recording_final_output_dir_.c_str(), recording_file_stem_.c_str(),
                static_cast<unsigned>(image_bytes));
  }

  void CloseRecording()
  {
    if (!recording_enabled_)
    {
      return;
    }
    if (recording_csv_.is_open())
    {
      recording_csv_.flush();
      recording_csv_.close();
    }
    if (recording_frames_.is_open())
    {
      recording_frames_.flush();
      recording_frames_.close();
    }
    FinalizeRecordingPackage();
    recording_enabled_ = false;
  }

  void WriteRecordingCameraInfo(const std::filesystem::path& path) const
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
    out << "recording_stem: " << recording_file_stem_ << "\n";
  }

  bool RecordImageFrame(const ImageFrame& frame)
  {
    if (!recording_enabled_)
    {
      return true;
    }

    const auto offset_pos = recording_frames_.tellp();
    if (offset_pos == std::streampos(-1))
    {
      XR_LOG_ERROR("CameraBase(%s): recording tellp failed", name_.c_str());
      return false;
    }
    const auto offset = static_cast<std::streamoff>(offset_pos);

    recording_frames_.write(reinterpret_cast<const char*>(frame.data.data()),
                            static_cast<std::streamsize>(image_bytes));
    recording_csv_ << recording_frame_index_ << ","
                   << static_cast<unsigned long long>(frame.timestamp_us) << ","
                   << static_cast<unsigned long long>(offset) << ","
                   << static_cast<unsigned long long>(image_bytes) << "\n";
    if (!recording_frames_.good() || !recording_csv_.good())
    {
      XR_LOG_ERROR("CameraBase(%s): recording write failed at frame=%llu",
                   name_.c_str(),
                   static_cast<unsigned long long>(recording_frame_index_));
      return false;
    }

    ++recording_frame_index_;
    if (recording_flush_every_frames_ != 0 &&
        (recording_frame_index_ % recording_flush_every_frames_) == 0)
    {
      recording_frames_.flush();
      recording_csv_.flush();
    }
    return true;
  }

  struct RecordingMarker
  {
    std::string stem;
    std::filesystem::path temp_dir;
    std::filesystem::path final_dir;
  };

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

  static bool ParseMarkerLine(const std::string& line, const char* key,
                              std::string& value)
  {
    const std::string prefix = std::string(key) + ":";
    if (line.rfind(prefix, 0) != 0)
    {
      return false;
    }
    value = Trim(line.substr(prefix.size()));
    return true;
  }

  static bool LoadRecordingMarker(const std::filesystem::path& marker_path,
                                  RecordingMarker& marker)
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
    std::ofstream marker(recording_marker_path_, std::ios::out | std::ios::trunc);
    if (!marker.is_open())
    {
      XR_LOG_ERROR("CameraBase(%s): open recording marker failed %s",
                   name_.c_str(), recording_marker_path_.string().c_str());
      throw std::runtime_error("CameraBase: open recording marker failed");
    }
    marker << "recording_state: active\n";
    marker << "stem: " << recording_file_stem_ << "\n";
    marker << "temp_dir: " << temp_dir.string() << "\n";
    marker << "final_dir: " << final_dir.string() << "\n";
    marker.flush();
    if (!marker.good())
    {
      XR_LOG_ERROR("CameraBase(%s): write recording marker failed %s",
                   name_.c_str(), recording_marker_path_.string().c_str());
      throw std::runtime_error("CameraBase: write recording marker failed");
    }
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
    const auto keep_size =
        last_newline == std::string::npos ? 0U : static_cast<uintmax_t>(last_newline + 1);
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

  static bool ParseFrameCsvRow(const std::string& line, uint64_t& offset,
                               uint64_t& size)
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

  static void RepairFrameFiles(const std::filesystem::path& dir,
                               std::string_view stem)
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

  static void RepairRecordingPackage(const std::filesystem::path& dir,
                                     std::string_view stem)
  {
    RepairFrameFiles(dir, stem);
    TrimTextFileToLastNewline(dir / (std::string(stem) + "_imu.csv"));
    TrimTextFileToLastNewline(dir / (std::string(stem) + "_sync.csv"));
  }

  static std::filesystem::path PickRecoveryFinalDir(
      const std::filesystem::path& requested_final)
  {
    std::error_code ec;
    if (!std::filesystem::exists(requested_final, ec))
    {
      return requested_final;
    }
    for (uint32_t index = 1; index < 1000; ++index)
    {
      auto candidate = requested_final;
      candidate += "_recovered_" + std::to_string(index);
      if (!std::filesystem::exists(candidate, ec))
      {
        return candidate;
      }
    }
    return requested_final;
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
      if (ec || !entry.is_regular_file(ec) ||
          entry.path().extension() != ".recording")
      {
        continue;
      }

      RecordingMarker marker{};
      if (!LoadRecordingMarker(entry.path(), marker))
      {
        XR_LOG_WARN("CameraBase: skip invalid recording marker %s",
                    entry.path().string().c_str());
        continue;
      }

      if (std::filesystem::exists(marker.temp_dir, ec))
      {
        RepairRecordingPackage(marker.temp_dir, marker.stem);
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

  void FinalizeRecordingPackage()
  {
    const std::filesystem::path temp_dir(recording_output_dir_);
    const std::filesystem::path final_dir(recording_final_output_dir_);
    RepairRecordingPackage(temp_dir, recording_file_stem_);

    std::error_code ec;
    std::filesystem::rename(temp_dir, final_dir, ec);
    if (ec)
    {
      XR_LOG_ERROR("CameraBase(%s): finalize recording failed temp=%s final=%s err=%s",
                   name_.c_str(), temp_dir.string().c_str(), final_dir.string().c_str(),
                   ec.message().c_str());
      return;
    }
    std::filesystem::remove(recording_marker_path_, ec);
    recording_output_dir_ = recording_final_output_dir_;
    XR_LOG_PASS("CameraBase(%s): recording finalized dir=%s",
                name_.c_str(), recording_output_dir_.c_str());
  }

  std::string name_;
  std::string image_topic_name_;
  std::string imu_topic_name_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Topic imu_topic_;
  ImageFrame* writable_image_{nullptr};
  ImageCommitCallback image_commit_callback_{};
  CameraBaseCalibration<CameraInfoV> calibration_{};
  bool recording_enabled_{false};
  uint32_t recording_flush_every_frames_{30};
  uint64_t recording_frame_index_{0};
  std::string recording_output_dir_{};
  std::string recording_final_output_dir_{};
  std::string recording_file_stem_{};
  std::filesystem::path recording_marker_path_{};
  std::ofstream recording_frames_{};
  std::ofstream recording_csv_{};
};
