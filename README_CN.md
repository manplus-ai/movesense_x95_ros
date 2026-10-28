# MoveSense X95 ROS 驱动 (movesense_x95_ros)

> **发布版本：0.5.2.20261028，早期版本。** 1.0 前话题和参数可能调整。

发布标识由 SDK 0.5.2 和日期组成。ROS 要求三段包版本，因此 package.xml 与 CMake project 使用 0.5.2；VERSION、包导出信息及可执行程序的 --version 输出保留完整的 0.5.2.20261028。

适用于 X95 A / AP / P 的 ROS 1 Noetic 驱动，要求 **MoveSense X95 SDK 0.5.2 或以上**。本版已使用 SDK 0.5.2 编译并在 X95 AP 相机上测试，实测范围见下文。

## 环境与编译

| 项目 | 要求 |
| --- | --- |
| 系统 / ROS | Ubuntu 20.04，ROS 1 Noetic，x86-64 |
| 语言 | C++17 |
| 依赖 | OpenCV、catkin、roscpp、sensor_msgs、std_msgs、vision_msgs、geometry_msgs、tf2、tf2_ros |
| 相机网络 | TCP 5001 / 5002；主机对时使用 UDP 5000；可选在线探测使用 UDP 5004 |

先安装独立的 [SDK](https://github.com/manplus-ai/MoveSense_X95_SDK)：

```bash
cmake -S MoveSense_X95_SDK -B sdk-build -DCMAKE_BUILD_TYPE=Release
cmake --build sdk-build --parallel 4
sudo cmake --install sdk-build
sudo ldconfig
```

将本仓库放进 ROS 工作空间的 `src`，用 rosdep 安装 ROS 依赖，然后在工作空间根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make install
source install/setup.bash
```

CMake 使用 `find_package(MoveSense_X95_SDK 0.5.2 REQUIRED)`，链接 `MoveSense::X95_SDK`。SDK 安装在自定义位置时，传入 `-DMoveSense_X95_SDK_DIR=<prefix>/lib/cmake/MoveSense_X95_SDK`，并保证运行时能找到其动态库。安装内容包括节点程序、launch 和中英文 README。

例如，SDK 安装在 `/path/to/sdk-install` 时：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DMoveSense_X95_SDK_DIR=/path/to/sdk-install/lib/cmake/MoveSense_X95_SDK
catkin_make install
export LD_LIBRARY_PATH=/path/to/sdk-install/lib:$LD_LIBRARY_PATH
```

SDK 头文件和库直接来自已安装的 SDK 包，不复制到本仓库；CMake 导入目标会提供对应的包含路径和链接依赖。本地 `include/movesense_x95/` 存放本驱动的公共实现，需要保留。

## 运行

```bash
source install/setup.bash
roslaunch movesense_x95_ros movesense_x95.launch camera_ip:=192.168.1.70
```

`rosrun movesense_x95_ros movesense_x95_ros --version` 输出完整发布标识，不连接相机。

默认订阅 RGB、深度和原始 IMU，使用前降采样、640×480、请求 25 FPS。节点会回读并打印实际帧率；实际吞吐量取决于固件、订阅组合和带宽。

P 型没有独立 RGB 镜头和 DOE，使用：

```bash
roslaunch movesense_x95_ros movesense_x95.launch enable_color:=false enable_stereo:=true registration:=0
```

双目全分辨率、深度半分辨率需要后降采样：

```bash
roslaunch movesense_x95_ros movesense_x95.launch enable_stereo:=true stereo_width:=1280 stereo_height:=960 downsample_mode:=1
```

## 话题与单位

默认话题前缀为 `/movesense`，话题和 frame 名称可以覆盖。

| 话题后缀 | 消息 | 含义 |
| --- | --- | --- |
| `left/image_rect_raw`、`right/image_rect_raw` | Image | 双目校正图，根据 SDK 图像格式解码为 mono8 或 bgr8 |
| `color/image_rect_color`、`color/camera_info` | Image / CameraInfo | 校正 RGB 及对应内参，仅 A/AP |
| `depth/image_rect_raw`、`depth/camera_info` | Image / CameraInfo | 16UC1，固定为**毫米**，使用所选配准视角的内参 |
| `imu` | Imu | 原始比力量测 m/s²、角速度 rad/s，无姿态 |
| `detections` | Detection2DArray | 源图像应用所配置 ROI 后的像素坐标；A/AP 为 RGB，P 为右目 |

未启用的流不创建发布话题。目前没有左右目 CameraInfo 话题，完整几何参数可以通过 XML 导出。

启用检测不会自动启用源图像话题。依赖缺失标定的 CameraInfo 和 TF 不发布；没有有效 RGB 标定时，配准深度无法开始发布。

开流前查询 SDK 深度单位，并将计数四舍五入为整数毫米。0 仍表示无效；超过 65535 mm 的距离置 0。此 16UC1 话题不保留亚毫米精度。

`registration=1` 为 RGB 视角深度，使用 RGB 内参；`0` 为右目视角。配准深度要求有效的 RGB 标定。SDK 没有读取配准开关的接口，因此 `registration=-1` 仅在关闭深度时允许。P 型深度始终使用双目视角。

## 参数

参数在启动时生效。修改后需重新启动节点。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `camera_ip` | 192.168.1.70 | 相机 IP |
| `align_time_on_start` | true | 首批数据建立相机时间基准后对齐 SDK 时间戳；失败则停止初始化 |
| `connection_probe` | true | 每 2 秒通过 UDP 5004 探测在线状态，连续 3 次失败后重连 |
| `trigger_mode` | 0 | 0 自动；2 外触发；-1 保留并读取当前模式 |
| `first_frame_wait_ms` | 5000 | 等待首图；IMU-only 模式等待首个非零 IMU 时间戳，1…60000 ms |
| `depth_fps` | 25 | 请求 5…25 FPS，日志打印实际值 |
| `stereo_width/height` | 640 / 480 | 640×480 或 1280×960 |
| `depth_width/height` | 640 / 480 | 640×480 或 1280×960 |
| `color_width/height` | 640 / 480 | RGB 独立输出分辨率：640×480 或 1280×960 |
| `downsample_mode` | 0 | 0 前降采样；1 后降采样；-1 按所需几何选择 |
| `enable_stereo` | false | 左右校正图 |
| `enable_depth/color/imu` | true | 对应流订阅 |
| `enable_detection` | false | 检测订阅 |
| `registration` | 1 | 0 / 1；-1 仅在关闭深度时允许 |
| `doe_power` | 255 | A/AP 投射器等级 0…255；-1 保留当前值 |
| `imu_calib_dir` | 空 | XML 导出目录，空值不导出 |
| `publish_tf` | true | 发布静态 TF 树 |
| `det_frame_id` | 空 | 自动选择：A/AP 为 RGB，P 为右目；支持显式覆盖 |

节点没有软触发服务，不支持模式 1。设置 `trigger_mode=1` 会在启动参数校验时退出；保留相机已有的软触发模式会使每次连接尝试失败。启用图像时，收到首帧后才读取标定，因此外触发模式需在 `first_frame_wait_ms` 内提供输入脉冲；自动或外触发模式启动首帧超时，均结束当前连接尝试并继续重试。初始化完成后，允许继续等待后续外触发脉冲。

双目全尺寸、深度半尺寸要求后降采样；双目半尺寸、深度全尺寸不支持。ROI 不改变各路配置的整图分辨率。

以下后缀分别搭配 `stereo_` 和 `color_`：

| 后缀 | 默认值 | 含义 |
| --- | --- | --- |
| `auto_expo` | false | 自动曝光 |
| `exposure_us` | 5000 | 手动曝光，1…65535 微秒 |
| `gain_x` | 1 | 手动增益倍数，1…16 |
| `min_exposure_us / max_exposure_us` | 100 / 65535 | 曝光范围，1…65535 微秒；-1 保留对应限值 |
| `min_gain_x / max_gain_x` | 1 / 16 | 增益范围，1…16；-1 保留对应限值 |

手动值必须处于已配置范围，已配置的最小值不能大于最大值。相机命令失败会结束当前初始化尝试；先设置几何，再设置帧率，开流后回读实际帧率。

话题覆盖项：`cam0_topic`（右）、`cam1_topic`（左）、`color_topic`、`color_info_topic`、`depth_topic`、`depth_info_topic`、`det_topic`、`imu_topic`。
Frame 覆盖项：`base_frame_id`、`cam0_frame_id`、`cam1_frame_id`、`color_frame_id`、`det_frame_id`、`imu_frame_id`。默认名称见 launch。

## ROI 与检测坐标

参数形式为 `roi_<stream>_enable/x1/y1/x2/y2`，stream 可取 `left_raw`、`right_raw`、`color_raw`、`left_rect`、`right_rect`、`color_rect`。

各路 `enable` 默认为 `false`，四个坐标均默认为 `0`。

坐标是该路应用 ROI 之前的完整输出图像中的零起点像素；校正图按配置的输出分辨率设置窗口。窗口为半开区间 `[x1,x2) × [y1,y2)`，至少 16×16，四个坐标必须为偶数。深度不支持裁剪。节点只发布校正图，raw ROI 仍对应相机中独立的原始图流。

RGB CameraInfo 描述实际裁剪图：宽高更新，K/P 主点减去窗口原点；ROI 字段保持为空，避免下游重复减偏移。实际图像尺寸与预期几何不一致时不发布 CameraInfo。检测框按 SEG 的 coordW/coordH 缩放，再减去源图 ROI 原点并裁到图像范围；空框、反向框、非有限值和异常 SEG 报文会被丢弃。

每次连接时下发六路 ROI；启用的窗口必须得到相机接受。不支持 ROI 的固件仅允许所有窗口关闭。正常退出时清除本节点启用过的窗口；已知断联时跳过不可达的清窗命令，在重连成功后重新下发所需窗口。

## 时间戳、连接与退出

默认在首帧到达后完成主机对时，IMU-only 模式则等待首个 IMU 样本，避免相机启动时重设 PTS 基准使偏移失效。之后清空 SDK 旧缓存，并过滤仍在途的旧设备时间域样本。PTS 为 0 的样本丢弃。IMU 标定中的相机时间偏移**不会自动应用**。

`align_time_on_start=false` 明确保留设备 PTS，用于诊断，不能与主机时间域的其他传感器或动态 TF 混用。实时相机节点要求 `use_sim_time=false`。

启动时相机不在线，或运行中发生断联，ROS 节点都保持运行。SDK 数据连接关闭，或独立 UDP 在线探测连续 3 次失败后，会停止当前相机会话；清理后每隔 1 秒尝试连接，直到相机恢复或用户停止节点。SDK 建连与设备初始化耗时不包含在这一重试间隔中。

每次连接成功都会重新下发配置、等待首批数据、对时、读取标定并重建 CameraInfo/TF，再恢复发布。各会话使用全新的队列和缓存，不沿用旧连接的数据。节点 PID 不变；发布端点可能重建，已有订阅者会自动重新匹配。

下游 TF 缓冲可能保留已收到的静态变换。更换相机或使标定失效时，可能需要清空或重启下游缓冲；后续会话不再发送某条 TF，并不能撤销消费者已缓存的旧变换。

UDP 5004 在线探测间隔为 2 秒，也能识别 TCP 没有收到关闭通知的静默断网。相机仍在线时的普通图像超时，包括等待外触发，不视为断联。网络禁止 UDP 5004 时可设 connection_probe=false，此时关闭静默断网检测，仅保留 TCP 关闭检测；仅过滤 TCP、UDP 仍可达的故障也无法单靠 UDP 发现。

关闭 `connection_probe` 不会关闭主机对时对 UDP 5000 的独立依赖。

启动校验拒绝的参数会使节点退出；连接或会话初始化失败会继续重试。正常关闭会停止重试、join 线程并关闭相机；关闭 IMU 后退出同样安全，不再使用 detach 强制退出线程。

## 标定与 XML 导出

通过 SDK 0.5.2 结构化接口读取标定，旧固件由 SDK 回退读取。相机内参、逆校正矩阵、基线、IMU 旋转/噪声及可用内参会独立校验。

```bash
roslaunch movesense_x95_ros movesense_x95.launch imu_calib_dir:=/path/to/calibration
```

导出目录包含 `calibration_status.xml` 和：

| 子目录 | 相机文件 | IMU 参考系 |
| --- | --- | --- |
| `xml` | M1、D1、M2、D2、IR_1、IR_2、t_P2 | 原始左目 |
| `xml_rgb` | M2、D2、t_P2、iR_2、T_reg、R、T | 原始 RGB，仅 A/AP |

表内文件名均带 `.xml`。有效 IMU 段增加 `r_IMU.xml`、`t_IMU.xml`、`ts_IMU.xml`、`noise_IMU.xml`、`intrinsics_IMU.xml`。A 优先使用 RGB-IMU 标定，AP/P 优先使用双目-IMU 标定；首选段不可用时回退到另一套有效标定，TF 参考系按实际存储的有效外参选择。

导出的图像矩阵保留 SDK 的 1280×960 标定图像域，不会按输出降采样或 ROI 改写；CameraInfo 则描述配置的输出图像，并在 RGB 裁剪开启时补偿窗口原点。

- `r_IMU.xml` / `t_IMU.xml` 是 **IMU → 原始相机**，保留 SDK 数值。平移和基线单位 mm，时偏单位 ms，符号为 `t_imu = t_cam + timeshift`。按下游要求将平移从 mm 换为 m、时偏从 ms 换为 s。
- `IR_1/IR_2` 是 `(K_rect R_rect)^-1`，不是旋转矩阵。RGB 使用 `R_C_Lr = iR_2 K_out`、`t_C_Lr = R_C_Lr T_reg`，不依据保留的 rotation 字段判断格式。
- `intrinsics_IMU.xml` 含 `model=scale-misalignment`、`has_intrinsics`；可用时包含四个 3×3 矩阵 `accel_M`、`gyro_M`、`gyro_A`、`gyro_C`。旧设备可能有有效外参但没有这些矩阵；此时 `has_intrinsics=0`，不写矩阵节点。
- IMU 话题保留原始量测语义，读取矩阵不会自动修正样本或改变 IMU 发布轴系。
- 静态 TF 会将原始相机外参与 raw→rectified 旋转组合，因此导出的 raw 外参与发布的校正相机 TF 数值有意不同。
- 读取 `calibration_status.xml` 的 `export_complete=1` 和各项有效标志。重复使用导出目录时，本次失效的已有本导出器 XML 会写入 `valid=0` 并清除旧参数节点，其他用户文件保留。

完整字段和补偿公式见 [SDK 标定参数说明](https://github.com/manplus-ai/MoveSense_X95_SDK/blob/movesense-0.5.2/doc/CALIBRATION_CN.md)。

## 公共实现

不依赖 ROS 的声明位于 `include/movesense_x95/`，非模板实现放在对应源文件：`src/calibration_data.cpp`、`calibration_export.cpp`、`camera_config.cpp`、`camera_session.cpp`、`data_receiver.cpp`、`image_geometry.cpp`、`seg_result.cpp`、`session_supervisor.cpp`。仅模板实现继续保留在头文件。这些文件和公共 CMake 模块在 ROS1/ROS2 两仓中保持字节完全一致。ROS 文件负责日志、参数、消息和节点接口适配。

两个仓库并列存在时，CMake 校验公共文件清单和 SHA256；出现差异直接停止配置。单独克隆任一仓也能构建。`MOVESENSE_VERIFY_PEER_CORE=OFF` 只关闭跨仓校验。

离线回归覆盖标定/TF 组合、坏参数、深度单位、ROI、检测坐标、SEG 边界、未初始化退出及重试状态机。

AP 实机使用每项 8～20 秒稳态采集的短时功能验收，覆盖 640×480 前降采样、双目全尺寸而 RGB/深度半尺寸的后降采样、默认 RGB+深度 25 FPS、RGB ROI 和关闭 IMU。启动时相机不在线、相机服务重启、模拟 20 秒网络中断均能在同一 ROS 进程 PID 中恢复。测试相机有有效的旧版 RGB-IMU 外参，但未配置完整 IMU 内参矩阵，四矩阵路径由离线回归覆盖；本次未测试外触发脉冲、物理拔线、长时间稳定性或其他相机型号。

## 许可

Apache-2.0，见 [LICENSE](LICENSE) 和 [NOTICE](NOTICE)。
