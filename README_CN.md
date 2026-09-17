# MoveSense X95 ROS 驱动 (movesense_x95_ros)

> **状态:0.1.1 — 早期版本。** 接口(话题 / 参数)在 1.0 之前可能仍有变动。

MoveSense X95 系列双目深度相机的 ROS 1(Noetic)驱动。通过以太网(借助
MoveSense X95 SDK)连接相机,用独立线程收帧 / 收 IMU,并发布:

- 左 / 右双目矫正图像
- 深度图 + 深度相机内参
- 彩色图 + 彩色相机内参(仅主动式 A / AP)
- IMU(约 200 Hz)
- YOLO 检测(`vision_msgs/Detection2DArray`)

节点会自动识别相机形态(A / AP / P)并相应调整输出(见[相机类型](#相机类型))。

## 运行环境

| 项       | 要求                    |
| -------- | ----------------------- |
| 操作系统 | Ubuntu 20.04(x86-64)    |
| ROS      | Noetic                  |
| 编译     | catkin(`catkin_make`) |
| 网络     | 与相机同网段            |

## 依赖

```bash
# ROS 包
sudo apt install ros-noetic-roscpp ros-noetic-sensor-msgs \
                 ros-noetic-std-msgs ros-noetic-tf2-ros libopencv-dev
# vision_msgs(若 apt 没有,克隆源码进工作区一起编):
#   git clone -b noetic-devel https://github.com/ros-perception/vision_msgs.git
```

### MoveSense X95 SDK

本驱动是 **MoveSense X95 SDK** 的一层薄 ROS 封装,相机侧的活都由 SDK 完成:
设备发现、四条网络通道、收帧与收 IMU、参数配置、标定回读。SDK 是**独立仓库、
不随本包分发**,必须先安装:

- 源码:https://github.com/manplus-ai/MoveSense_X95_SDK/

```bash
git clone https://github.com/manplus-ai/MoveSense_X95_SDK.git
cd MoveSense_X95_SDK
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install          # 默认装到 /usr/local
```

之后本包通过 `find_package(MoveSense_X95_SDK REQUIRED)` 找到它并链接导出目标
`MoveSense::X95_SDK`;只要装在 CMake 会搜索的前缀下(默认 `/usr/local`)就无需
额外配置。用到两个公开头:`<movesense/Simou3Camera.h>`(相机 API)与
`<movesense/Simou3CalibLayout.h>`(标定数组字段偏移,本驱动据此解析内参 / 基线 /
彩色与 IMU 外参)。

SDK 库本身无第三方依赖。

> **版本对应 —— 混用版本前务必先看这里。**
> 本驱动基于 **MoveSense X95 SDK 0.2.3** 编译并测试,要求 **0.2.3 或更高**:
> ROI 参数用到的 `setRoi()` 是 0.2.3 才引入的,标定解析用到的
> `<movesense/Simou3CalibLayout.h>` 是 0.2.1 引入的。更低版本的 SDK 无法与
> 本包一起编译(0.2.0 及更早还有别的差异 —— `movesense` 命名空间也是 0.2.0
> 才有的)。ROI 还需要相机固件支持;老固件上只要不启用 ROI,节点照常运行。
>
> 在 SDK 发布 **1.0.0 之前,其公开 API 在各版本之间仍会有较大变动**,请始终让
> 本驱动与其对应的 SDK 版本配套使用,不要混用。查看当前装的是哪个版本:调用
> `<movesense/Simou3Camera.h>` 里的 `simou3_get_version_string()`,或直接看
> `/usr/local/lib/libMoveSense_X95_SDK.so.*`。

OpenCV(ROS Noetic 自带)用于 NV21→BGR 彩色转换和 IMU 标定 XML 读写。

## 编译

```bash
mkdir -p ~/catkin_ws/src
cp -r movesense_x95_ros ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
```

## 运行

```bash
source devel/setup.bash
roslaunch movesense_x95_ros movesense_x95.launch camera_ip:=<相机IP>
```

看数据:

```bash
rostopic list | grep movesense
rosrun rqt_image_view rqt_image_view      # 左 / 右 / 彩色 / 深度
rostopic echo /movesense/imu
rostopic echo /movesense/detections
```

## 发布的话题

| 话题                                  | 类型                             | 说明                                                                                 |
| ------------------------------------- | -------------------------------- | ------------------------------------------------------------------------------------ |
| `/movesense/left/image_rect_raw`    | `sensor_msgs/Image`            | 左目矫正 —`mono8`(A/AP)或 `bgr8`(P)                                             |
| `/movesense/right/image_rect_raw`   | `sensor_msgs/Image`            | 右目矫正 —`mono8`(A/AP)或 `bgr8`(P)                                             |
| `/movesense/color/image_rect_color` | `sensor_msgs/Image`            | `bgr8`,**仅主动式 A/AP**(RGB 镜头)                                           |
| `/movesense/color/camera_info`      | `sensor_msgs/CameraInfo`       | 彩色内参,仅 A/AP                                                                     |
| `/movesense/depth/image_rect_raw`   | `sensor_msgs/Image`            | `16UC1`,单位 mm;以**右目**视角渲染(与 `right/image_rect_raw` 逐像素对齐)   |
| `/movesense/depth/camera_info`      | `sensor_msgs/CameraInfo`       | 已矫正,畸变=0;`registration=1` 时为**彩色**内参 + 彩色 frame_id,否则双目内参 |
| `/movesense/imu`                    | `sensor_msgs/Imu`              | 约 200 Hz,原始加速度/角速度(无姿态)                                                  |
| `/movesense/detections`             | `vision_msgs/Detection2DArray` | YOLO 检测框(图像坐标系)                                                              |

话题只在对应流启用时才 advertise:`left` / `right` 需 `enable_stereo:=true`
(默认关),`color*` 需在 A/AP 机型上开 `enable_color`,`detections` 需
`enable_detection`。未 advertise 的话题不会出现在 `rostopic list` 里。

某一路启用了 ROI(见[感兴趣区域](#感兴趣区域roi))时,该路图像话题的尺寸是
裁剪后的尺寸,而不是配置的流分辨率。

## 相机类型

节点查询 `getCameraType()` 并自动适配:

| 类型   | 双目(左/右)         | color 话题          | 检测来源 |
| ------ | ------------------- | ------------------- | -------- |
| A / AP | `mono8`(灰度)     | RGB 镜头 →`bgr8` | RGB      |
| P      | `bgr8`(NV21→BGR) | 不发布              | 右目     |

深度始终以右目视角渲染。P 型没有可配准的彩色流,无论 `registration`
取值,深度都保持在右目视角。

自带 launch 的默认值(`enable_color=true`、`registration=1`、`doe_power=255`)
面向**主动式 A / AP** 相机。**被动式 P** 相机请设 `enable_color:=false`
并 `enable_stereo:=true` —— P 没有 RGB 镜头,矫正双目图(以 `bgr8` 发布)
就是它的图像来源。没有 RGB 镜头,深度→彩色配准也不生效:P 型的深度始终在
双目坐标系下,camera_info 恒用双目内参,与 `registration` 取值无关。P 也
没有 DOE 投射器,`doe_power` 同样不适用。这三项在 P 上会被节点忽略并打
警告日志,不会下发给相机。

## 坐标系(Frames)

节点启动时广播一次下面这棵静态 TF 树(若你自己的 URDF 已提供这些变换,
可用 `publish_tf:=false` 关闭):

```
movesense_link
└── movesense_infra1_frame
      └── movesense_infra1_optical_frame        <- 左目,所有标定的参考系
            ├── movesense_infra2_optical_frame  <- 右目(x 方向 +基线)
            ├── movesense_color_optical_frame   <- 彩色镜头
            │     └── movesense_imu_frame       <- IMU(A 型)
            └── movesense_imu_frame             <- IMU(AP / P 型)
```

**IMU 坐标系挂在哪个父节点下,按相机型号不同**,因为两类机型是对不同相机
标定的:

| 型号 | IMU 父节点 | 外参来源 |
| ---- | ---------- | -------- |
| **A** | `movesense_color_optical_frame` | **彩色**标定数组的 IMU 段 |
| **AP** / **P** | `movesense_infra1_optical_frame` | **双目**标定数组的 IMU 段 |

节点按相机上报的型号选父节点,不会去猜"哪一段填了就用哪一段"。A 型机上
彩色坐标系必须在树里(开了彩色流,或深度做了配准),否则 IMU 变换会跳过
并打印警告。

- `infra1` = **左目**,`infra2` = **右目**。相机内存储的所有外参都以左目
  为基准,因此 `infra1_optical_frame` 是其它传感器坐标系的父节点。
- `movesense_link` → `movesense_infra1_frame` 为单位变换;
  `*_frame` → `*_optical_frame` 这一步承载 REP-103 标准的机体系→光学系
  旋转(x 右、y 下、z 前),因此标定数值可原样使用,无需任何手工换算。
- 所有相机坐标系都是**校正后(rectified)**的坐标系:发布的图像已完成校正、
  畸变为零,下游无需再做去畸变或校正旋转。
- 右目与彩色相对左目都是**纯平移**(校正后双目行对齐;彩色的校正旋转已烘焙
  进相机自身的标定),故旋转为单位阵。
- 彩色与 IMU 的变换只在标定中确实含有对应外参时才广播(彩色还需该坐标系
  确实被使用);否则节点打印警告并跳过该变换。启动日志里的
  `[TfPublisher] IMU frame attached to <frame>` 会告诉你实际挂在了谁下面。

深度**不单独建坐标系** —— 它跟随 `registration`,与其 `camera_info` 一致。
深度图以**右目**视角渲染(主流双目相机以左目为基准,本机型与惯例相反):
`registration=0` 时深度图位于 `movesense_infra2_optical_frame`;
`registration=1` 时重投影到 `movesense_color_optical_frame`。

坐标系名称均可配置(`base_frame_id`、`cam0_frame_id`、`cam1_frame_id`、
`color_frame_id`、`imu_frame_id`、`det_frame_id`);中间的
`movesense_infra1_frame` 跟随 `cam1_frame_id` 生成。

## 分辨率与降采样

双目 / 深度 / 彩色每路只能是 **1280×960**(全)或 **640×480**(降采样),
其它尺寸报错退出。

**彩色是独立的。** 相机端中目有自己的矫正组, `color_width` / `color_height` 可以
单独配, 不受双目/深度牵制, 也不会反过来影响它们。彩色全分辨率 + 深度降采样
(1280×960 彩色 + 640×480 深度)是受支持的组合。

`downsample_mode` 决定双目/深度这条链在**哪一步**降采样, 只影响这两路:

- **前降**(`0`) — 先降采样, 再匹配深度。匹配跑在小图上, 更快、能上更高帧率, 测距精度低一些。
- **后降**(`1`) — 先匹配, 再对结果降采样。更准, 但帧率上限更低、DPU 负载更高。

合法组合(**双目 ≥ 深度**):

| 双目 | 深度 | `downsample_mode` |
| ---- | ---- | ----------------- |
| 640×480 | 640×480 | `0` / `1` 都可以 |
| 1280×960 | 640×480 | 只能 `1` —— 前降会把双目一起降下去 |
| 640×480 | 1280×960 | 不支持, 节点报错退出 |
| 1280×960 | 1280×960 | 没有降采样, 该项不起作用 |

默认 `0`(前降), 与默认的 640×480 双目/深度相配。也接受 `-1`(自动): 双目大于深度时用后降,
其余用前降。

## 感兴趣区域(ROI)

每一路图像都可以**在相机端**裁剪后再发送:网络上只传窗口内的部分,图像话题
的尺寸也随之变为裁剪后的尺寸。六路各有自己的窗口(`left_raw`、`right_raw`、
`color_raw`、`left_rect`、`right_rect`、`color_rect`),深度不能裁剪。本节点只
发布矫正流,所以真正影响话题的是 `left_rect` / `right_rect` / `color_rect`;
raw 三路的配置同样会下发给相机,但不影响本节点发布的任何内容。

每路参数:`roi_<stream>_enable` 加 `roi_<stream>_x1 / _y1 / _x2 / _y2`。窗口为
`[x1, x2) × [y1, y2)`,坐标是**该路当前输出尺寸下的 0 基像素** —— 该路降采样
时按 640×480 算,否则按 1280×960 —— 发布出的图像尺寸为 `(x2-x1) × (y2-y1)`。
规则:`x1 < x2`、`y1 < y2`、至少 **16×16**、**四个值都是偶数**、窗口不能超出
该路图像。前四条由 SDK 本地检查,超出图像由相机拒绝。

```bash
roslaunch movesense_x95_ros movesense_x95.launch enable_stereo:=true \
    roi_left_rect_enable:=true roi_left_rect_x2:=256 roi_left_rect_y2:=256 \
    roi_right_rect_enable:=true roi_right_rect_x2:=256 roi_right_rect_y2:=256
```

行为:

- 启动时六路全部下发,包括关闭的,这样上一个客户端留在相机上的窗口会被清掉。
  节点退出时再把启用过的窗口清一遍。
- 启用的窗口若被 SDK 或相机拒绝,节点报错退出,日志里写明是哪一路、什么原因。
- 相机固件不支持 ROI 时,节点打一条警告继续运行,前提是没有启用任何 ROI。
- ROI 不改变相机的帧率上限:上限由各路全分辨率 / 降采样的配置决定,与窗口
  大小无关。

**当前限制:**`camera_info` 与 `/movesense/detections` 尚未随窗口调整。
`camera_info` 描述的仍是未裁剪的整路图像(主点请自行减去 `x1` / `y1`),
检测框仍是整图坐标。

## Launch 参数

| 参数                                          | 默认                | 说明                                                                      |
| --------------------------------------------- | ------------------- | ------------------------------------------------------------------------- |
| `camera_ip`                                 | 见 launch           | 相机 IP                                                                   |
| `align_time_on_start`                       | `false`           | 启动时做一次 NTP 对时                                                     |
| `downsample_mode`                           | `0`               | 0=前降(先降再匹配) / 1=后降(先匹配再降) / -1=自动;只影响双目与深度        |
| `depth_fps`                                 | `25`              | 5 ~ 25                                                                    |
| `stereo_width` / `stereo_height`          | `640` / `480`   | 1280×960 或 640×480                                                     |
| `depth_width` / `depth_height`            | `640` / `480`   | 1280×960 或 640×480                                                     |
| `color_width` / `color_height`            | `640` / `480`   | 1280×960 或 640×480(A/AP)                                               |
| `enable_stereo`                             | `false`           | 订阅左右目                                                                |
| `enable_depth`                              | `true`            | 订阅深度                                                                  |
| `enable_color`                              | `true`            | 订阅彩色(仅 A/AP)                                                         |
| `enable_imu`                                | `true`            | 订阅 IMU                                                                  |
| `enable_detection`                          | `false`           | 订阅 YOLO seg                                                             |
| `stereo_auto_expo`                          | `false`           | 自动曝光开关                                                              |
| `stereo_exposure_us` / `stereo_gain_x`    | `5000` / `1`    | 手动曝光(µs)/ 增益(×)                                                   |
| `stereo_max_exposure_us` / `..._min_...`  | `65535` / `100` | 有效曝光区间(-1 = 沿用相机配置)                                           |
| `stereo_max_gain_x` / `stereo_min_gain_x` | `16` / `1`      | 有效增益区间(-1 = 沿用相机配置)                                           |
| `color_*`(与 stereo 同一套)                 | —                  | 彩色曝光/增益(仅 A/AP)                                                    |
| `doe_power`                                 | `255`             | DOE 投射器 0..255(0=关);-1=沿用相机配置                                   |
| `registration`                              | `1`               | 深度→彩色配准:1 开 / 0 关 / -1 沿用(同时决定深度 camera_info 用哪套内参) |
| `roi_<stream>_enable`                       | `false`           | 在相机端裁剪该路;`<stream>` = `left_raw` / `right_raw` / `color_raw` / `left_rect` / `right_rect` / `color_rect`(见[ROI](#感兴趣区域roi)) |
| `roi_<stream>_x1` / `_y1` / `_x2` / `_y2`   | `0`               | 窗口 `[x1,x2)×[y1,y2)`,该路输出像素坐标;都是偶数、至少 16×16                |
| `imu_calib_dir`                             | `""`              | 非空则启动时导出双目↔IMU 外参 XML                                        |
| `publish_tf`                                | `true`            | 广播静态 TF 树(见[坐标系](#坐标系frames))                                  |
| `base_frame_id`                             | `movesense_link`  | TF 树根节点                                                               |
| 话题 / frame_id 覆盖项                        | 见 launch           | 各路话题名与`frame_id`                                                  |

## IMU 标定外参导出(可选)

把 `imu_calib_dir` 设为一个可写目录,节点连上相机后,**每一段已标定的外参**
各写 4 个 XML:`r_IMU.xml`(R 3×3)、`t_IMU.xml`(t,mm)、`ts_IMU.xml`
(时间偏移,ms)、`noise_IMU.xml`(陀螺/加速度噪声密度 + 随机游走)。

相机里存了**两套** IMU 外参,导出时按子目录分开、文件名相同(与厂商标定
工具的布局一致):

| 子目录 | 外参段 | 参考相机 |
| ------ | ------ | -------- |
| `<imu_calib_dir>/xml`     | 双目标定数组的 IMU 段 | **左目**校正相机 |
| `<imu_calib_dir>/xml_rgb` | 彩色标定数组的 IMU 段 | **彩色**校正相机 |

只导出已标定的段:全 0 或非有限值视为"未标定",该子目录不写。通常 A 型只有
`xml_rgb`,AP / P 型只有 `xml`。读彩色段需要先读彩色标定,只要设了
`imu_calib_dir`,节点就会去读(非被动式机型),即使彩色流没开。

格式说明:

- 文件为 OpenCV `cv::FileStorage` XML;数值原样导出自相机内存储的标定
  (与厂商标定工具导出的内容一致)。
- 字段命名沿用 [kalibr](https://github.com/ethz-asl/kalibr) 惯例
  (`timeshift_cam_imu`、`*_noise_density`、`*_random_walk`),但单位不同:
  `t_IMU` 为 **mm**、`ts_IMU` 为 **ms** —— 用于 kalibr 风格 camchain yaml
  (如 OpenVINS / ORB-SLAM3 的 VIO 配置)前需换算成 m / s。
- 参考系:**外参相对哪个相机由子目录决定**,两套千万不要混用。`xml` 相对
  左目校正相机,`xml_rgb` 相对彩色校正相机,都在该相机的光学系
  (x右 / y下 / z前)下表达。`r_IMU` / `t_IMU` 是 kalibr `T_cam_imu` 的
  旋转与平移:`R` 把 IMU 系向量变换到该相机系,`t` 是 IMU 原点在该相机系
  下的坐标(mm)。`ts_IMU` 语义为 `t_imu = t_cam + ts`(ms)。注意深度是以
  右目为基准的,两套 IMU 外参都不是,所以混用它们的 VIO 配置需要用基线和
  彩色外参把坐标系接起来。
- 发布的 IMU 时间戳**没有**应用这个时间偏移。若你的管线需要,请自行补偿。

## 许可

采用 Apache License 2.0。见 [LICENSE](LICENSE) 与 [NOTICE](NOTICE)。
