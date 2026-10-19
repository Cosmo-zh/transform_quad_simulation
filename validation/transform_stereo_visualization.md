# 可视化自动仿真与按次保存数据

在仓库根目录运行：

```sh
python3 Tools/transform_stereo_sitl_check.py --flight --rviz
```

`--rviz` 自动同时打开 Gazebo 和 RViz2，左右相机各一个 Image 窗口，预设话题和
Best Effort QoS，无需手动添加显示项。使用系统 Python 将 Gazebo Transport 的原始
RGB/BGR 图像发布成 ROS `sensor_msgs/msg/Image`，保留仿真时间戳。
默认以 5 Hz、480×300 预览，降低同时打开两个三维程序的负载；识别和图像留存仍用
原始 960×600 图像。可用 `--rviz-fps 10 --rviz-scale 1` 提高预览帧率和恢复原分辨率，
但需要更多资源。缩放后会重新计算图像步长，原分辨率转发时保留原步长和行尾填充。
只显示原始图像，尚未叠加识别框、门宽或折叠比例；不是深度图或点云显示。

本机 ROS 2 Humble 的一次性依赖安装：

```sh
sudo apt-get install ros-humble-rviz2
```

本项目的 OpenCV 可独立安装到已忽略的目录，重启后仍保留：

```sh
python3 -m pip install --target logs/transform_stereo_deps --no-deps opencv-python-headless==4.11.0.86
```

启动脚本优先使用该目录；旧的 `/tmp/transform-stereo-deps` 仅作为兼容回退。

脚本自动加载 `/opt/ros/humble/setup.bash`，可用 `--ros-setup /路径/setup.bash` 指定环境。
需要系统 Python 能导入 `rclpy`、`sensor_msgs`、`gz.transport13` 和 `gz.msgs10`。
不依赖 `ros_gz_bridge`，避免 Humble 默认桥接包与本项目 Gazebo Harmonic 版本不匹配。
启动前检查 ROS/RViz 依赖，缺失时保留失败记录并在启动飞控前退出。

图像桥接、RViz 与本次 Gazebo 使用同一 `GZ_PARTITION`，ROS 默认仅本机通信。
显示进程使用较低 CPU 优先级，不参与飞行控制。关闭 RViz 或 Gazebo 会中止本次运行，
请求伴随程序处理退出/降落并保留已有数据；结束后清理本次启动的显示和桥接进程。
Gazebo 使用项目自带的 900×600、无抗锯齿配置，不修改用户个人 GUI 配置。
OpenCV 明确限制为 1 个工作线程；图像解码、识别和 PNG 写盘在独立进程中完成，
通过有界队列传递最新测量，过期测量丢弃，视觉进程异常会中止飞行。
飞行进程保留 500 ms 循环保护；运行锁防止同时启动多个本脚本实例争用端口和资源。
图形模式默认 `--sim-speed 0.5`，无图形模式默认 1；速度参数以仿真时间为单位，
位置目标推进、阶段等待和默认运行时长同步换算。显式 `--duration` 仍表示墙钟秒数。
半速让一次相同飞行需要约两倍实际时间；`--sim-speed 1` 可恢复实时速度，需另行验证机器负载。

仅需要 Gazebo 时仍可使用：

```sh
python3 Tools/transform_stereo_sitl_check.py --flight --gui
```

程序打开 Gazebo 三维窗口，运行 5103 双目自动通行，并请求视角跟随无人机。
飞行结束后关闭本次窗口和仿真实例。需要桌面会话；未加 `--gui` 时仍可只在后台运行。
相机跟随在后台发送，不能阻塞飞行与数据采集；可在 Gazebo 中自行选取无人机、调整视角。

```sh
# 后台自动仿真，同样保存数据
python3 Tools/transform_stereo_sitl_check.py --flight

# 自定义保存位置和双目图像采样间隔（秒）
python3 Tools/transform_stereo_sitl_check.py --flight --gui \
  --output-root logs/my_experiments --image-interval 0.5
```

默认每秒保存一对左右图像，属于定时采样图片。`--image-interval 0` 关闭连续图像保存，
仍保留首次成功识别时的 `left.png` / `right.png`。每次使用独立时间戳目录，不覆盖前次数据。

## 保存位置与内容

默认目录为 `logs/transform_stereo/transform_stereo_日期_时间_微秒/`。
它位于 `build/` 之外，清理构建产物时仍保留。仓库已忽略 `/logs`，数据不会自动加入提交。

```text
本次运行目录/
  run.json                 启止时间、运行参数、退出码、分析是否完成
  environment.json         本次独立 Gazebo 通信分区和模型配置
  result.json              飞行/测量判定；失败时记录原因
  console.txt              PX4 / Gazebo 服务端输出
  companion.txt            伴随控制器输出
  gui.txt                  Gazebo 图形界面和相机跟随输出（--gui）
  rviz.txt                 RViz2 输出（--rviz）
  ros_bridge.txt           图像桥接输出（--rviz）
  ros_images.json          两路发布计数、当前与峰值订阅数（--rviz）
  stereo.rviz              本次使用的左右 Image 显示配置（--rviz）
  gazebo.config            本次使用的 Gazebo 显示配置（--gui / --rviz）
  ros_logs/                本次 ROS 日志（--rviz）
  log/日期/*.ulg            PX4 原始飞行日志
  parameters.bson          本次 PX4 参数
  events.jsonl             状态变化、门洞测量和飞行遥测
  vision.json              识别帧数、保存图片对数、OpenCV 线程数和最大处理/写盘耗时
  resources.jsonl          运行中的可用内存、交换空间和系统负载
  ground_truth.jsonl       独立 Gazebo 真值（飞行模式）
  frames/                  定时左右图像，文件名含仿真时间戳
  analysis/
    summary.json           结果汇总与各类数据样本数
    trajectory.png         真值轨迹、高度、实际机臂折叠进度
    measurements.csv       门宽、距离、中心偏差、朝向和不确定度
    telemetry.csv          PX4 NED 位置/速度、姿态、关节角、接触计数
    ground_truth.csv       Gazebo 世界位置和四元数
  analysis.txt             自动分析输出
  manifest.json            数据文件列表、大小和 SHA-256
```

所有模式都创建结果和清单。启动失败、飞行失败或 Ctrl+C 中断后，保留已采集数据；
尚未产生的日志或图像不会凭空生成。分析文件缺数据时明确标注无样本，
分析失败也保留原始文件并在 `run.json` 中记录，不把缺失数据判作飞行成功。

`measurements.csv` 长度单位为米，角度为弧度。`telemetry.csv` 的 `north/east/down`
遵循 PX4 NED；`ground_truth.csv` 的 `x/y/z` 遵循 Gazebo 世界 ENU，二者不能直接同轴比较。
机臂 `arm1_rad`～`arm4_rad` 为实际关节角（弧度）。
`cmd1_rad`～`cmd4_rad` 为实际收到的 Gazebo 舵机话题值；5103 当前仅启用
`servo_0`，四个关节 PID 共用它，因此 `cmd2_rad`～`cmd4_rad` 留空。

可在仿真结束后重新生成分析：

```sh
python3 Tools/analyze_transform_stereo.py logs/transform_stereo/本次目录名
```

## 验证

记录相关回归测试覆盖：启动失败仍保存结果、分析失败仍保留原始数据、
CSV 坐标约定、截断尾行处理；与现有双目测试一起执行：

```sh
PYTHONPATH=logs/transform_stereo_deps python3 -m unittest discover \
  -s Tools/tests -p 'test_transform*.py' -v
```

ROS 图像转换测试与本机端到端传输测试（需本机通信权限）：

```sh
RUN_STEREO_ROS_INTEGRATION=1 /usr/bin/python3 -m unittest discover \
  -s Tools/tests -p test_transform_ros_bridge.py -v
```

覆盖 RGB/BGR、行尾填充、原始时间戳、非法载荷拒绝及独立分区/ROS 域下的双路接收。
RViz 日志的 `Stereo is NOT SUPPORTED` 是 OpenGL 立体渲染提示，不表示两路 Image 显示失败。

2026-10-19 验证：ROS 转换及本机端到端 4 项测试通过。低负载静态运行
`logs/transform_stereo/transform_stereo_20261019_150623_858439/`，左右各发布 8 帧，
均有 RViz 订阅；12 帧门宽中位数 600 mm，运行和分析均成功。
可用 `python3 Tools/transform_stereo_sitl_check.py --rviz` 重现静态短测，不解锁飞行。
之前带两个图形界面的飞行触发过控制循环超时或遥测过期，用户反馈电脑卡顿；
上述静态成功本身不能作为两门飞行成功的证据；后续回归见下节。

## 机构与控制循环诊断（2026-10-19）

5103 模型现在使用一个折叠输入，四个关节仍有各自的物理 PID，保持与真实单舵机机构一致。
只有 `SIM_GZ_SV_FUNC1=301`，其余三个输出禁用；旧 5101/5102 配置不变。
5103 的三个角速度控制增益系数设为 0.2，原配置在这轮测试中出现过实际姿态振荡。
机臂未到位仍在 25 秒超时；机臂已到位时允许等待机体停稳至 60 秒，
原到位误差、速度条件、姿态和接触保护保持不变。

```sh
# 固定基座，直接验证四条机臂全行程，不启动 PX4、不飞行
python3 Tools/transform_stereo_mechanism_check.py

# PX4 原地起飞，折叠到 87.5%，展开、降落，不穿门
python3 Tools/transform_stereo_sitl_check.py --fold-check
```

静态测试完成 0→0.3→0.875→1→0→0.875→0。单输入下四臂均到位。
悬停折叠回归 `logs/transform_stereo/transform_stereo_20261019_172823_661205/` 成功，
四臂实际角度约 -0.905/-0.905/-0.928/-0.928 rad，目标 -0.9163 rad，随后展开并降落。
门体接触为 0，控制循环最大间隔约 51.6 ms。

对中计算使用 `right*cos(normal_yaw) - (forward+0.08)*sin(normal_yaw)`，
表示机身相对门平面的横向偏差，0.08 m 为相机前向安装位置。
只看图像中心偏差 `right` 会在机头偏航时误判居中。
近距离保留左右视差差异用于估计门朝向，穿越时以门平面偏差反馈并在门前暂停修正偏航；
横向预算、关节、姿态和接触保护均保留。`measurements.csv` 新增 `lateral_error` 列。

最终完整图形回归已通过：
`logs/transform_stereo/transform_stereo_20261019_192056_825527/`，两门通行、88.5% 折叠、
展开和降落成功；门体接触 0，循环最大间隔 52 ms，保存 262 对双目图片。
配置、原始证据与统计见 [修复回归记录](stereo_20261019_stability/README.md)。

2026-10-18 实跑记录：

- 9 项测试通过；图形模式启动、相机跟随订阅、自动起飞和第一道门通行已验证。
- 数据目录：`logs/transform_stereo/transform_stereo_20261018_143730_487852/`。
  保存 102 对左右图像、1419 条测量、522 条遥测、5622 条真值及 ULog；
  CSV 和轨迹图生成成功，清单内 225 个文件的 SHA-256 复核全部通过。
- 本轮飞行结果为失败：第二道门测得约 439.85 mm，目标折叠 87.5%，
  机臂 1/2 到达约 86.4%，机臂 3/4 仍接近展开，触发 `Folding/settling timeout`。
  该执行器/仿真问题尚待排查；不能把可视化与数据保存验证当作两门飞行成功。
