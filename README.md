# CameraBase

CameraBase 提供相机模块共用的 C++ 类型。

它不打开相机，不分配图像缓存。

## 类型

`CameraTypes::CameraInfo` 描述一台相机：

- `width`：图像宽度，单位像素
- `height`：图像高度，单位像素
- `step`：每行字节数
- `encoding`：像素格式
- `camera_matrix`：3x3 内参矩阵，按行存储
- `distortion_model`：畸变模型
- `distortion_coefficients`：畸变参数
- `rectification_matrix`：3x3 校正矩阵，按行存储
- `projection_matrix`：3x4 投影矩阵，按行存储

`CameraBase<Info>::ImageFrame` 表示一帧图像：

- `timestamp_us`：图像时间戳，单位微秒
- `data`：固定大小图像数据，大小为 `Info.step * Info.height`

`CameraBase<Info>::ImuStamped` 表示一帧同步 IMU 数据：

- `timestamp_us`：时间戳，单位微秒
- `rotation_wxyz`：四元数，顺序为 `w, x, y, z`
- `translation_xyz`：平移，单位米
- `angular_velocity_xyz`：角速度，单位 `rad/s`
- `linear_acceleration_xyz`：线加速度，单位 `m/s^2`

## 相机信息检查

`CameraBase<Info>` 会在编译期检查 `CameraInfo` 是否明显异常。

检查内容包括：

- 图像宽高不能为 0
- `step` 不能为 0
- 内参矩阵中的焦距和主点要落在合理范围内
- `fx / fy` 比例不能明显异常
- 投影矩阵不能和内参矩阵明显冲突

这些检查只用于发现错误配置，不会修改 `CameraInfo`。

## 图像槽

相机通过外部提供的图像槽写图。

流程是：

1. 其他模块调用 `RegisterImageSink(initial_image, callback)`
2. 相机调用 `GetWritableImage()`
3. 相机写入 `timestamp_us` 和 `data`
4. 相机调用 `CommitImage()`
5. 回调返回下一帧可写槽位

`CommitImage()` 之后，旧槽位不能继续写。

## 命令

CameraBase 会创建一个 RamFS 命令文件，名字和相机实例名相同。

支持命令：

```text
set_exposure <值>
set_gain <值>
```

单位和范围由具体相机实现决定。

## 如何实现相机

继承 `CameraBase<Info>`，并实现：

```cpp
void SetExposure(double exposure) override;
void SetGain(double gain) override;
```

采集时只写当前可写槽位：

```cpp
auto* image = GetWritableImage();
if (image == nullptr) {
  return;
}

image->timestamp_us = timestamp;
// 写入 image->data

CommitImage();
```

## 说明

- `step` 是字节数，不是像素数
- `ImageFrame` 和 `ImuStamped` 是标准布局类型
- CameraBase 只保存当前可写图像指针，不保存队列
- 相机内参放在 `CameraInfo`，外参不放在这里
