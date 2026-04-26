# CameraBase

`CameraBase` is the front-half camera boundary module for the current
Webots/Linux auto-aim pipeline.

It does not implement a concrete camera driver by itself. Instead, it defines
the compile-time camera description and the producer-side raw frame / IMU ABI
that concrete camera modules use.

## Runtime Role

- compile-time source of truth for image geometry and calibration:
  - `CameraTypes::CameraInfo`
- producer-side payload definitions:
  - `CameraBase<Info>::ImageFrame`
  - `CameraBase<Info>::ImuStamped`
- producer-side image sink registration and commit boundary
- IMU topic publish helper for concrete camera implementations

## Current Boundary

- `CameraBase<Info>` owns raw producer-side types and sink registration
- `WebotsCamera<Info>` or other concrete camera modules:
  - fill `ImuStamped`
  - write image bytes into the registered `ImageFrame`
  - call `CommitImage()`
- `CameraFrameSync<Info>` owns the shared-image publish and sync bridge
- downstream modules should consume the sync boundary instead of rebuilding a
  mixed monolithic camera payload

## Public Contract

- `CameraTypes::CameraInfo`
  - fixed-size, compile-time camera metadata
  - width / height / stride / encoding
  - camera matrix, distortion model, rectification matrix, projection matrix
- `CameraBase<Info>::ImageFrame`
  - fixed-size image payload derived from compile-time `CameraInfo`
  - aligned for shared-memory-safe transport
- `CameraBase<Info>::ImuStamped`
  - timestamped rotation, translation, angular velocity, linear acceleration
- image sink API:
  - `RegisterImageSink(...)`
  - `ImageSinkReady()`
  - `GetWritableImage()`
  - `CommitImage()`

## Notes

- image payload size is derived at compile time from `CameraInfo.step *
  CameraInfo.height`
- this module keeps the front-half ABI trivial / standard-layout so the shared
  memory boundary stays predictable
- `CameraTypes::BuildPnPDistCoeffs(...)` provides the compile-time PnP-facing
  distortion description helper
