# CameraBase

CameraBase 提供相机模块共用的 C++ 类型。

它不打开相机，不分配图像缓存。

## 类型

`CameraTypes::FrameLayout` 描述编译期固定的图像存储布局：

- `width`：图像宽度，单位像素
- `height`：图像高度，单位像素
- `step`：每行字节数
- `encoding`：像素格式

`CameraTypes::CameraCalibration` 描述原生传感器坐标系下的标定：

- `native_width / native_height`：标定对应的原生传感器尺寸
- `camera_matrix`：3x3 内参矩阵，按行存储
- `distortion_model`：畸变模型
- `distortion_coefficients`：畸变参数
- `rectification_matrix`：3x3 校正矩阵，按行存储
- `projection_matrix`：3x4 投影矩阵，按行存储

`CameraTypes::FrameGeometry` 描述一帧图像到原生传感器坐标的映射。它携带
`epoch`、当前帧尺寸和步长、原生 ROI 偏移、横纵下采样倍率、翻转标志和采样相位。
坐标映射为：

```text
oriented = reverse ? extent - 1 - frame : frame
native = roi_offset + sample_phase + decimation * oriented
```

同一镜头只保存一份 `CameraCalibration`；ROI、下采样和翻转不修改内参，而是由每帧
`FrameGeometry` 表达。

`CameraBase<FrameLayoutV>::ImageFrame` 表示一帧图像：

- `timestamp_us`：图像时间戳，单位微秒
- `geometry`：当前帧按值携带的 `FrameGeometry`
- `data`：固定大小图像数据，大小为 `FrameLayoutV.step * FrameLayoutV.height`

`CameraBase<FrameLayoutV>::ImuStamped` 表示一帧同步 IMU 数据：

- `timestamp_us`：时间戳，单位微秒
- `rotation_wxyz`：四元数，顺序为 `w, x, y, z`
- `translation_xyz`：平移，单位米
- `angular_velocity_xyz`：角速度，单位 `rad/s`
- `linear_acceleration_xyz`：线加速度，单位 `m/s^2`

## 配置检查

`CameraBase<FrameLayoutV>` 在编译期检查帧布局，并在构造时检查按值传入的
`CameraCalibration`。`CameraTypes::ValidateFrameGeometry(...)` 用于检查每帧采样几何。

检查内容包括：

- 布局宽高和 `step` 非零，且 `step` 能容纳一行像素
- 内参矩阵中的焦距和主点要落在合理范围内
- `fx / fy` 比例不能明显异常
- 投影矩阵不能和内参矩阵明显冲突
- `FrameGeometry` 的 `epoch`、下采样倍率和采样相位有效
- 当前帧尺寸和 `step` 与 `FrameLayoutV` 一致，映射结果不超出原生标定范围

这些检查只拒绝错误配置，不会缩放或改写标定。

## 图像槽

相机通过外部提供的图像槽写图。

流程是：

1. 其他模块调用 `RegisterImageSink(initial_image, callback)`
2. 相机调用 `GetWritableImage()`
3. 相机写入 `timestamp_us`、`geometry` 和 `data`
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

继承 `CameraBase<FrameLayoutV>`，构造时把原生标定传给基类，并实现：

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
image->geometry = frame_geometry;
// 写入 image->data

CommitImage();
```

## 说明

- `step` 是字节数，不是像素数
- `FrameGeometry` 固定为 40 字节，按值进入 `ImageFrame`
- `ImageFrame` 和 `ImuStamped` 是标准布局类型，`ImageFrame` 可平凡复制
- CameraBase 只保存当前可写图像指针，不保存队列
- 原生内参放在不可变 `CameraCalibration`，相机外参不放在这里
