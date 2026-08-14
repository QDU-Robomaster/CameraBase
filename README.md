# CameraBase

CameraBase 提供相机模块共用的 C++ 类型和单生产者图像槽交接协议。它不打开相机、
不分配图像缓存、不保存队列，也不创建工作线程。

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

当前构造期 sanity 只检查原生尺寸、内参矩阵和畸变系数；`rectification_matrix` 与
`projection_matrix` 仅按值携带，不在本模块内验证。

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

- `timestamp_us`：源传感器或录制文件的采样时间，单位微秒
- `publish_token`：sink 在发布前分配的进程内非零标识；生产者持有槽位时为零
- `geometry`：当前帧按值携带的 `FrameGeometry`
- `data`：固定大小图像数据，大小为 `FrameLayoutV.step * FrameLayoutV.height`

`CameraBase<FrameLayoutV>::ImuStamped` 表示一帧同步 IMU 数据：

- `timestamp_us`：时间戳，单位微秒
- `publish_token`：对应共享图像的发布标识
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
- `FrameGeometry` 的 `epoch`、下采样倍率和采样相位有效
- 当前帧尺寸和 `step` 与 `FrameLayoutV` 一致，映射结果不超出原生标定范围

这些检查只拒绝错误配置，不会缩放或改写标定。

## 发布契约

相机通过外部 sink 提供的图像槽写图。注册和提交顺序是：

1. 其他模块调用 `RegisterImageSink(initial_image, callback)`
2. 相机调用 `GetWritableImage()`
3. 相机写入 `timestamp_us`、`geometry` 和 `data`，不写 `publish_token`
4. 相机调用 `CommitImage()`
5. CameraBase 撤销当前写权限，并在同一个采集线程内同步调用 sink
6. sink 完成发布前准备、决定发布或丢帧，并返回下一帧独占可写槽位

`CommitImage()` 返回 true 只表示生产者拿到下一写租约，不表示当前帧已发布。发布失败、
没有共享槽、几何拒绝或预处理拒绝均由 sink 记账。未发布时 sink 可以返回原槽位直接
复用；一旦发布，就必须返回不会与任何读者并发访问的槽位。回调返回 nullptr 会使接口
失效关闭，旧槽位不会再次暴露给生产者。

## 时间戳与发布标识

`ImageFrame::timestamp_us` 是同步所用的源采样时间，不是主机到达、解码、预处理或
发布时间。实时相机应使用设备时钟换算值并保留其采样语义。CameraBase 不比较时间戳，
不重映射时钟域，也不处理复位、回绕、回退或跨源排序；同步 sink 按自己的时间线策略
接受、复位或丢帧。确定性回放必须保持输入记录的原始顺序和时间，包括原始重复值，
不能为了制造单调时间线而改写数据。

`publish_token` 只区分同一进程内的发布尝试，不能代替时间戳、帧号或持久化 ID。sink
在发布尝试前给图像分配非零 token；同步模块生成 `ImuStamped` 时复制同一个 token。
CameraBase 在每次把槽位交给生产者前将 token 清零。

## 预处理与背压

必须先于图像发布完成的工作，例如把原图准备到 Detector 自有输入槽，应在同步 commit
回调内、实际 `Publish()` 之前执行。CameraBase 不执行算法预处理，也不启动额外线程；
它只保证该回调同步完成后才归还下一写租约。

CameraBase 自身没有缓存深度和丢帧策略。sink 可以在回调内等待形成背压，也可以拒绝
发布并返回原槽位形成 drop-and-reuse。具体策略及计数由 sink 所属模块定义。

## 线程与生命周期

sink 只注册一次，不支持注销或替换。注册可以发生在采集线程已经开始轮询之后；初始
槽位和 callback 通过原子就绪门闩发布。注册完成后，`GetWritableImage()` 与
`CommitImage()` 必须始终由同一个采集线程调用，callback 也在该线程内同步执行。
callback 可以只读查询 `ImageSinkReady()`，但不得重新注册 sink、递归提交或尝试生产
下一帧。

CameraBase、sink、callback 目标和槽位池按进程生命周期存在。CameraBase 析构不注销
sink、不等待 callback，也不停止派生模块的工作线程；本模块不定义 stop token 或 join
流程。

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

if (!CommitImage()) {
  // sink 尚未注册，或违反了下一写槽位契约
  return;
}
```

## 说明

- `step` 是字节数，不是像素数
- `YUV422` 只保证每像素两字节且宽度为偶数，打包顺序仍由具体源约定；当前主链不用它
- `FrameGeometry` 固定为 40 字节，按值进入 `ImageFrame`
- `ImageFrame` 和 `ImuStamped` 均为标准布局、可平凡复制类型
- `publish_token` 改变了两种载荷的 ABI；所有参与模块必须使用同一 CameraBase 版本重编译，
  并随同一进程一起重启，不能和旧布局混用
- CameraBase 只保存一个 sink 的当前可写图像租约，不保存队列
- 原生内参放在不可变 `CameraCalibration`，相机外参不放在这里
