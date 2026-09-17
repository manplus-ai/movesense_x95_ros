# MoveSense X95 ROS Driver (movesense_x95_ros)

> **Status: 0.1.1 — early release.** Interfaces (topics / parameters) may
> still change before 1.0.

ROS 1 (Noetic) driver for the MoveSense X95-series stereo depth camera. It
connects to the camera over Ethernet (via the MoveSense X95 SDK), receives
frames and IMU on dedicated threads, and publishes:

- left / right stereo images
- depth image + depth camera_info
- color image + color camera_info (active A / AP cameras only)
- IMU (~200 Hz)
- YOLO detections (`vision_msgs/Detection2DArray`)

The node auto-detects the camera form factor (A / AP / P) and adapts the
output (see [Camera types](#camera-types)).

## Requirements

| Item    | Requirement               |
| ------- | ------------------------- |
| OS      | Ubuntu 20.04 (x86-64)     |
| ROS     | Noetic                    |
| Build   | catkin (`catkin_make`)  |
| Network | same subnet as the camera |

## Dependencies

```bash
# ROS packages
sudo apt install ros-noetic-roscpp ros-noetic-sensor-msgs \
                 ros-noetic-std-msgs ros-noetic-tf2-ros libopencv-dev
# vision_msgs (if not packaged for your ROS, build from source into the workspace):
#   git clone -b noetic-devel https://github.com/ros-perception/vision_msgs.git
```

### MoveSense X95 SDK

This driver is a thin ROS wrapper around the **MoveSense X95 SDK**, which does
all the camera-side work: device discovery, the four network channels, frame
and IMU reception, parameter configuration and calibration read-back. The SDK
is a **separate repository, not bundled here** - install it first:

- Source: https://github.com/manplus-ai/MoveSense_X95_SDK/

```bash
git clone https://github.com/manplus-ai/MoveSense_X95_SDK.git
cd MoveSense_X95_SDK
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install          # installs to /usr/local by default
```

The package then locates it through `find_package(MoveSense_X95_SDK REQUIRED)`
and links the imported target `MoveSense::X95_SDK`; nothing else is needed as
long as the SDK is installed into a prefix CMake searches (`/usr/local` by
default). Two public headers are used: `<movesense/Simou3Camera.h>` (camera
API) and `<movesense/Simou3CalibLayout.h>` (calibration blob field offsets,
used here to parse intrinsics / baseline / color and IMU extrinsics).

The SDK library itself has no third-party dependencies.

> **Version compatibility — read this before mixing versions.**
> This driver is built and tested against **MoveSense X95 SDK 0.2.3**, and
> requires **0.2.3 or newer**: the ROI parameters call `setRoi()`, which was
> introduced in 0.2.3, and the calibration parsing includes
> `<movesense/Simou3CalibLayout.h>` (0.2.1). Older SDKs will not compile
> against this package (0.2.0 and earlier also differ elsewhere - the
> `movesense` namespace only appeared in 0.2.0). ROI additionally needs
> camera firmware that implements it; on older firmware the node still runs
> as long as no ROI is enabled.
>
> Until the SDK reaches **1.0.0 its public API is still changing
> substantially between releases**, so always pair this driver with the SDK
> version it was released against instead of mixing versions. Check what is
> installed with `simou3_get_version_string()` from
> `<movesense/Simou3Camera.h>`, or by looking at
> `/usr/local/lib/libMoveSense_X95_SDK.so.*`.

OpenCV (from ROS Noetic) is used for NV21→BGR color conversion and IMU
calibration XML I/O.

## Build

```bash
mkdir -p ~/catkin_ws/src
cp -r movesense_x95_ros ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
```

## Run

```bash
source devel/setup.bash
roslaunch movesense_x95_ros movesense_x95.launch camera_ip:=<your-camera-ip>
```

View data:

```bash
rostopic list | grep movesense
rosrun rqt_image_view rqt_image_view      # left / right / color / depth
rostopic echo /movesense/imu
rostopic echo /movesense/detections
```

## Published topics

| Topic                                 | Type                             | Notes                                                                                                                      |
| ------------------------------------- | -------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| `/movesense/left/image_rect_raw`    | `sensor_msgs/Image`            | left rectified —`mono8` (A/AP) or `bgr8` (P)                                                                          |
| `/movesense/right/image_rect_raw`   | `sensor_msgs/Image`            | right rectified —`mono8` (A/AP) or `bgr8` (P)                                                                         |
| `/movesense/color/image_rect_color` | `sensor_msgs/Image`            | `bgr8`, **active A/AP only** (RGB lens)                                                                            |
| `/movesense/color/camera_info`      | `sensor_msgs/CameraInfo`       | color intrinsics, active A/AP only                                                                                         |
| `/movesense/depth/image_rect_raw`   | `sensor_msgs/Image`            | `16UC1`, unit = mm; rendered from the **RIGHT** camera viewpoint (pixel-aligned with `right/image_rect_raw`)     |
| `/movesense/depth/camera_info`      | `sensor_msgs/CameraInfo`       | rectified, distortion = 0;**color** intrinsics + color frame_id when `registration=1`, stereo intrinsics otherwise |
| `/movesense/imu`                    | `sensor_msgs/Imu`              | ~200 Hz, raw accel/gyro (no orientation)                                                                                   |
| `/movesense/detections`             | `vision_msgs/Detection2DArray` | YOLO boxes (image coordinate frame)                                                                                        |

Topics are only advertised when their stream is enabled: `left` / `right`
require `enable_stereo:=true` (off by default), `color*` requires
`enable_color` on an A/AP camera, and `detections` requires
`enable_detection`. A topic that is not advertised never appears in
`rostopic list`.

When a ROI is enabled for a stream (see
[Region of interest](#region-of-interest-roi)), its image topic carries the
cropped size instead of the configured stream resolution.

## Camera types

The node queries `getCameraType()` and adapts automatically:

| Type   | Stereo (left/right)   | Color topic         | Detection source |
| ------ | --------------------- | ------------------- | ---------------- |
| A / AP | `mono8` (grayscale) | RGB lens →`bgr8` | RGB              |
| P      | `bgr8` (NV21→BGR)  | not published       | right eye        |

Depth is always rendered from the right camera viewpoint. On P cameras it
stays there regardless of `registration`, since there is no color stream to
register to.

The shipped launch defaults (`enable_color=true`, `registration=1`,
`doe_power=255`) target **active A / AP** cameras. On a **passive P**
camera set `enable_color:=false` and `enable_stereo:=true` — P has no RGB
lens, and the rectified stereo pair (published as `bgr8`) is its image
source. Depth→color registration also has no effect without an RGB lens:
on P the depth stays in the stereo frame and its camera_info always uses
stereo intrinsics, regardless of the `registration` value. P has no DOE
projector either, so `doe_power` is not applicable. The node ignores all
three on P (with a warning) instead of sending them to the camera.

## Frames

The node broadcasts the static TF tree below once at startup (set
`publish_tf:=false` to suppress it, e.g. when your own URDF already provides
these transforms):

```
movesense_link
└── movesense_infra1_frame
      └── movesense_infra1_optical_frame        <- left eye, reference frame of all calibration
            ├── movesense_infra2_optical_frame  <- right eye  (+baseline on x)
            ├── movesense_color_optical_frame   <- color lens
            │     └── movesense_imu_frame       <- IMU, on an A camera
            └── movesense_imu_frame             <- IMU, on an AP / P camera
```

**The IMU frame hangs off a different parent depending on the camera type**,
because the two models are calibrated against different cameras:

| Type | IMU parent | Calibration source |
| ---- | ---------- | ------------------ |
| **A** | `movesense_color_optical_frame` | IMU segment of the **color** calibration blob |
| **AP** / **P** | `movesense_infra1_optical_frame` | IMU segment of the **stereo** calibration blob |

The node picks the parent from the type reported by the camera; it does not
guess from whichever segment happens to be filled in. On an A camera the color
frame must be part of the tree (color stream on, or depth registered),
otherwise the IMU transform is skipped with a warning.

- `infra1` = **left** eye, `infra2` = **right** eye. Every extrinsic stored in
  the camera is expressed relative to the left eye, so `infra1_optical_frame`
  is the parent of the other sensor frames.
- `movesense_link` → `movesense_infra1_frame` is identity; the
  `*_frame` → `*_optical_frame` step carries the standard REP-103 body-to-optical
  rotation (x right, y down, z forward), so the calibration numbers are used
  as-is without any hand conversion.
- All camera frames are **rectified** frames: the published images are already
  rectified and their distortion is zero, so no undistortion or rectification
  rotation is needed downstream.
- Right eye and color are pure translations from the left eye (rectified stereo
  is row-aligned, and the color rectification rotation is already baked into the
  camera's own calibration), so their rotations are identity.
- The color and IMU transforms are only broadcast when the calibration actually
  contains those extrinsics (and, for color, when the color frame is in use);
  otherwise the node logs a warning and skips that transform. The startup log
  line `[TfPublisher] IMU frame attached to <frame>` tells you which parent was
  used.

Depth does **not** get a frame of its own — it follows `registration`, exactly
like its `camera_info`. Depth is rendered from the **right** camera viewpoint
(most stereo cameras use the left eye as reference, this one does not): with
`registration=0` the depth image lives in `movesense_infra2_optical_frame`;
with `registration=1` it is reprojected into `movesense_color_optical_frame`.

Frame names are configurable (`base_frame_id`, `cam0_frame_id`,
`cam1_frame_id`, `color_frame_id`, `imu_frame_id`, `det_frame_id`); the
intermediate `movesense_infra1_frame` follows `cam1_frame_id`.

## Resolution & downsampling

Each of stereo / depth / color is either **1280×960** (full) or **640×480**
(downsampled); other sizes are rejected.

**Color is independent.** It has its own rectification group on the camera, so
`color_width` / `color_height` can be set freely and never constrains stereo or
depth. A full-resolution color stream alongside a downsampled depth stream
(1280×960 color + 640×480 depth) is a supported combination.

`downsample_mode` selects **where** the stereo/depth pipeline downsamples, and
affects only those two streams:

- **pre** (`0`) — downsample first, then match. Matching runs on the small
  image: faster, allows a higher frame rate, lower range accuracy.
- **post** (`1`) — match first, then downsample the result. More accurate, but
  the frame-rate ceiling is lower and DPU load is higher.

Valid combinations (**stereo ≥ depth**):

| stereo | depth | `downsample_mode` |
| ------ | ----- | ----------------- |
| 640×480 | 640×480 | `0` or `1`, both valid |
| 1280×960 | 640×480 | `1` only — pre would shrink the stereo pair too |
| 640×480 | 1280×960 | unsupported, the node exits with an error |
| 1280×960 | 1280×960 | nothing is downsampled, the mode is ignored |

The default is `0` (pre), matching the default 640×480 stereo and depth
streams. `-1` (auto) is also accepted: post when stereo is larger than depth,
pre otherwise.

## Region of interest (ROI)

Each stream can be cropped **on the camera**, before it is sent over the
network: only the window is transmitted, and the image topic then carries the
cropped size. Six streams have their own window (`left_raw`, `right_raw`,
`color_raw`, `left_rect`, `right_rect`, `color_rect`); depth cannot be cropped.
This node publishes the rectified streams only, so `left_rect` / `right_rect` /
`color_rect` are the ones that change a topic - the raw entries are forwarded
to the camera as well but do not affect anything this node publishes.

Per stream: `roi_<stream>_enable` plus `roi_<stream>_x1 / _y1 / _x2 / _y2`.
The window is `[x1, x2) x [y1, y2)` in **0-based pixels of that stream's
current output size** - 640x480 coordinates when the stream is downsampled,
1280x960 otherwise - and the published image is `(x2-x1) x (y2-y1)`. Rules:
`x1 < x2`, `y1 < y2`, at least **16x16**, **all four values even**, and the
window must lie inside the stream. The SDK rejects the first four locally; the
camera rejects a window outside the stream.

```bash
roslaunch movesense_x95_ros movesense_x95.launch enable_stereo:=true \
    roi_left_rect_enable:=true roi_left_rect_x2:=256 roi_left_rect_y2:=256 \
    roi_right_rect_enable:=true roi_right_rect_x2:=256 roi_right_rect_y2:=256
```

Behavior:

- On startup the node sends all six entries, disabled ones included, so a
  window left behind by a previous client is cleared. Enabled windows are
  cleared again when the node exits.
- An enabled window that the SDK or the camera rejects stops the node with an
  error that names the stream and the reason.
- On camera firmware without ROI support the node logs one warning and keeps
  running, as long as no ROI is enabled.
- ROI does not change the camera's frame-rate ceiling: that is decided by the
  full / downsampled stream configuration, not by the window size.

**Current limitation:** `camera_info` and `/movesense/detections` are not yet
adjusted for the window. `camera_info` still describes the uncropped stream
(subtract `x1` / `y1` from the principal point yourself), and detection boxes
stay in full-image coordinates.

## Launch parameters

| Parameter                                     | Default             | Description                                                                                                 |
| --------------------------------------------- | ------------------- | ----------------------------------------------------------------------------------------------------------- |
| `camera_ip`                                 | see launch          | camera IP                                                                                                   |
| `align_time_on_start`                       | `false`           | run one NTP time alignment on startup                                                                       |
| `downsample_mode`                           | `0`               | 0 = pre (downsample, then match) / 1 = post (match, then downsample) / -1 = auto; stereo + depth only        |
| `depth_fps`                                 | `25`              | 5 ~ 25                                                                                                      |
| `stereo_width` / `stereo_height`          | `640` / `480`   | 1280×960 or 640×480                                                                                       |
| `depth_width` / `depth_height`            | `640` / `480`   | 1280×960 or 640×480                                                                                       |
| `color_width` / `color_height`            | `640` / `480`   | 1280×960 or 640×480 (A/AP)                                                                                |
| `enable_stereo`                             | `false`           | subscribe left/right                                                                                        |
| `enable_depth`                              | `true`            | subscribe depth                                                                                             |
| `enable_color`                              | `true`            | subscribe color (A/AP only)                                                                                 |
| `enable_imu`                                | `true`            | subscribe IMU                                                                                               |
| `enable_detection`                          | `false`           | subscribe YOLO seg                                                                                          |
| `stereo_auto_expo`                          | `false`           | auto exposure on/off                                                                                        |
| `stereo_exposure_us` / `stereo_gain_x`    | `5000` / `1`    | manual exposure (µs) / gain (×)                                                                           |
| `stereo_max_exposure_us` / `..._min_...`  | `65535` / `100` | effective exposure range (-1 = keep camera setting)                                                         |
| `stereo_max_gain_x` / `stereo_min_gain_x` | `16` / `1`      | effective gain range (-1 = keep camera setting)                                                             |
| `color_*` (same set as stereo)              | —                  | color exposure/gain (A/AP only)                                                                             |
| `doe_power`                                 | `255`             | DOE projector 0..255 (0 = off); -1 = keep camera setting                                                    |
| `registration`                              | `1`               | depth→color registration: 1 on / 0 off / -1 keep (also selects which intrinsics go into depth camera_info) |
| `roi_<stream>_enable`                       | `false`           | crop this stream on the camera; `<stream>` = `left_raw` / `right_raw` / `color_raw` / `left_rect` / `right_rect` / `color_rect` (see [ROI](#region-of-interest-roi)) |
| `roi_<stream>_x1` / `_y1` / `_x2` / `_y2`   | `0`               | window `[x1,x2)×[y1,y2)` in that stream's output pixels; all even, at least 16×16                             |
| `imu_calib_dir`                             | `""`              | if set, export stereo↔IMU extrinsics XML on startup                                                        |
| `publish_tf`                                | `true`            | broadcast the static TF tree (see[Frames](#frames))                                                          |
| `base_frame_id`                             | `movesense_link`  | TF tree root                                                                                                |
| topic / frame_id overrides                    | see launch          | per-stream topic names and`frame_id`s                                                                     |

## IMU calibration export (optional)

Set `imu_calib_dir` to a writable directory; on connect the node writes four
XMLs per calibrated segment: `r_IMU.xml` (R 3×3), `t_IMU.xml` (t, mm),
`ts_IMU.xml` (timeshift, ms), `noise_IMU.xml` (gyro/accel noise density +
random walk).

The camera stores **two** IMU extrinsic segments, and they are written to
separate subdirectories with identical file names (same layout as the vendor
calibration tool):

| Subdirectory | Segment | Reference camera |
| ------------ | ------- | ---------------- |
| `<imu_calib_dir>/xml`     | IMU segment of the stereo blob | **left** rectified camera |
| `<imu_calib_dir>/xml_rgb` | IMU segment of the color blob  | **color** rectified camera |

Only calibrated segments are written: an all-zero or non-finite segment means
"not calibrated" and that subdirectory is skipped. Typically an A camera has
only `xml_rgb` and an AP / P camera only `xml`. Reading the color segment
requires the color calibration to be read, which the node does whenever
`imu_calib_dir` is set (on non-passive cameras), even if the color stream is off.

Notes on the format:

- The files are OpenCV `cv::FileStorage` XML; the values are exported
  verbatim from the calibration stored on the camera (same content as the
  vendor calibration tool exports).
- Field names follow [kalibr](https://github.com/ethz-asl/kalibr)
  conventions (`timeshift_cam_imu`, `*_noise_density`, `*_random_walk`),
  but the units are **mm** for `t_IMU` and **ms** for `ts_IMU` — convert to
  m / s before feeding a kalibr-style camchain yaml (e.g. for OpenVINS /
  ORB-SLAM3 VIO configs).
- Reference frame: **which camera the extrinsic is relative to is decided by
  the subdirectory**, so never mix the two. `xml` is relative to the left
  rectified camera, `xml_rgb` to the color rectified camera; both are
  expressed in that camera's optical frame (x right, y down, z forward).
  `r_IMU` / `t_IMU` are the rotation and translation parts of kalibr's
  `T_cam_imu`: `R` maps vectors from the IMU frame into that camera's frame,
  `t` is the IMU origin expressed in that camera's frame (mm). `ts_IMU`
  follows `t_imu = t_cam + ts` (ms). Note that depth is right-eye based while
  neither IMU segment is, so a VIO config that mixes them needs the baseline
  and the color extrinsic to bridge the frames.
- Published IMU timestamps do **not** have this timeshift applied. Apply it
  yourself if your pipeline needs it.

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) and
[NOTICE](NOTICE).

---

中文说明见 [README_CN.md](README_CN.md)。
