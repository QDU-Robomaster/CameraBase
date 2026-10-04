# CameraBase 内部机制 / Internals

本页记录 CameraBase 图像槽与所有权的实现细节，用于维护模块和编写相机驱动。使用模块时依赖的约定见 [README](../README.md)。

This page records the implementation details of the CameraBase image slots and ownership, for maintaining the Module and writing camera drivers. The conventions that users of the Module rely on are in the [README](../README.md).

## 1. 图像槽对象池 / Image Slot Pool

CameraBase 用 `LibXR::MPMCObjectPool<ImageFrame>` 管理两个进程内图像槽。`SharedFrame` 是对其中一个槽位的引用计数句柄，复制句柄只增加引用计数，最后一个句柄析构时槽位归还对象池。像素不被复制，每帧也没有堆内存分配。

CameraBase manages the two in-process image slots with `LibXR::MPMCObjectPool<ImageFrame>`. A `SharedFrame` is a reference-counted handle to one of the slots; copying a handle only increments the reference count, and the slot returns to the pool when the last handle is destroyed. Pixels are not copied and no heap memory is allocated per frame.

## 2. 所有权在回调中建立 / Ownership Taken in the Callback

图像所有权在同步 `Topic::Callback` 内通过复制 `SharedFrame` 建立。`Topic::SyncSubscriber` 可能错过发布，`Topic::QueuedSubscriber` 在队列已满时丢弃消息，而 `Topic::Publish()` 不返回逐订阅者的接收结果，因此订阅者通过回调持有图像所有权。

Image ownership is established inside the synchronous `Topic::Callback` by copying the `SharedFrame`. A `Topic::SyncSubscriber` can miss a publication and a `Topic::QueuedSubscriber` drops messages when its queue is full, while `Topic::Publish()` returns no per-subscriber result, so subscribers take image ownership through the callback.

## 3. 背压 / Back Pressure

固定的两个图像槽构成有界背压：一个槽由采集线程写入，另一个槽由下游异步链路持有。工作槽位拒绝接收时，回调中的副本随即析构。等待、丢帧和计数策略由相机或订阅方模块定义。

The two fixed image slots form a bounded back pressure: one slot is written by the capture thread and the other is held by the downstream asynchronous chain. When the worker slot rejects the handle, the copy in the callback is destroyed immediately. Waiting, frame dropping and counting policies are defined by the camera or the subscribing Module.

## 4. 生命周期与析构 / Lifetime and Destruction

CameraBase、回调目标、槽位池和工作线程按进程生命周期存在。CameraBase 的析构函数只销毁对象；销毁前所有 `SharedFrame` 已释放，回调注销和工作线程的停止在析构之外完成。

CameraBase, callback targets, the slot pool and worker threads live for the whole process. The CameraBase destructor only destroys the object; all `SharedFrame` handles are released before destruction, and unregistering callbacks and stopping worker threads happen outside the destructor.
