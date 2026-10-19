# 可折叠四旋翼项目进展与交接说明

更新时间：2026-10-19。仓库根目录：`/home/zjy/PX4-Autopilot`。
以下文件路径均相对于仓库根目录。本文用于向后续 ChatGPT 或开发者交接，
记录当前代码、已验证结果及待办，不能把几何计算或场景加载视为穿门飞行验证。

## 1. 项目目标和当前结论

项目基于 PX4，为一个舵机通过太阳轮/齿轮机构同步驱动四条机臂的四旋翼，
实现飞行中的折叠和展开。机臂在水平面内绕各自铰点旋转，电机推力轴保持竖直。
折叠时通过动态更新电机位置，协调机构动作与 PX4 控制分配。

原展开构型为 X 型，固定机头方向时折叠后的横向投影反而增大。
现已增加十字型：电机 1 的机臂定义为正前方，保留原接线和旋向对应。
截至 2026-10-19，双目机型 5103 已在 Gazebo 与 RViz 同时运行时完成
600/440 mm 两门自动通行、88.5% 折叠、展开和降落，门体接触为 0。
这是带标记训练门与理想双目的成功回归；100% 折叠穿越原 360 mm 门及实物通行尚未验证。
第 2～8 节保留早期 5101/5102 与实物配置记录，后续新增和修复见第 9～12 节。
本次上传的新增、修改与复现步骤汇总见 [项目变更说明](PROJECT_CHANGES.md)。

| 工作项 | 当前状态 |
| --- | --- |
| `transform_control` 折叠控制模块 | 已实现；实物原 X 型版本曾确认运行 |
| 折叠时更新控制分配几何 | 已实现 |
| 原 X 型空中折叠/展开 | 旧项目记录中已有仿真验证 |
| 电机 1 朝前的十字构型 | 已实现实物配置与独立仿真配置 |
| V6C 十字型固件 | 已编译；未确认已刷入实物 |
| 十字型 30% 折叠悬停 | 5101 完成三轮折叠/展开及降落，日志验证通过 |
| 十字型通行世界、桨盘碰撞模型 | 已生成，通过 SDF 校验与实际加载检查 |
| 手动通过第一道练习门 | 用户反馈已通过，但有漂移，容易碰框 |
| Position 漂移量化 | 已分析 2026-10-12 的一份日志 |
| 将第一道门加宽至 600 mm | 5103 已使用独立 600/440 mm 世界；5102 保持 440/360 mm |
| 折叠稳定性和第二道门通行 | 5103 已验证 88.5% 折叠通过 440 mm 门；原 360 mm 门仍未验证 |
| 自动飞行、自动折叠、通行成功判定 | 5102 未实现；5103 已新增原型，验证见第 9 节 |
| 构型航向补偿 | 仅有设计文档，尚未接入控制代码 |
| 双目识别与自动通行原型（5103） | 已完成 600/440 mm 两门自动通行，最新图形回归成功；见第 9、12 节 |
| Gazebo、RViz 与按次记录 | 一条命令启动三维视角、双目实时预览及记录；见第 10～12 节 |

## 2. 构型与机型入口

### 十字型电机布局

以下为俯视图，机头向上；PX4 FRD 机体系 X 向前、Y 向右、Z 向下：

```text
                电机1 / 下标0
                     前
                     |
电机3 / 下标2 左 ---- 中心 ---- 右 电机4 / 下标3
                     |
                     后
                电机2 / 下标1
```

| 电机 | 展开 X / Y（m） | `CA_ROTOR*_KM` |
| --- | --- | --- |
| 1 / 0 | +0.140007143 / 0 | -0.05 |
| 2 / 1 | -0.140007143 / 0 | -0.05 |
| 3 / 2 | 0 / -0.140007143 | +0.05 |
| 4 / 3 | 0 / +0.140007143 | +0.05 |

控制几何将原 X 型的完整机构旋转 -45 度，电机位置与铰点一起变换。
Gazebo FLU 下相应旋转为 +45 度，不改变折叠方向和电机编号。

### 机型索引

| 编号 | 用途 | 配置路径 |
| --- | --- | --- |
| 4099 | 原 X 型动态折叠仿真 | `ROMFS/px4fmu_common/init.d-posix/airframes/4099_gz_transform_quad` |
| 4100 | 原 X 型静态折叠参考 | `ROMFS/px4fmu_common/init.d-posix/airframes/4100_gz_transform_quad_folded` |
| 4101 | 实物十字型，QGC 名称 `Transform Quad Plus Real RC Safe` | `ROMFS/px4fmu_common/init.d/airframes/4101_transform_quad_real` |
| 5101 | 十字型悬停与折叠仿真 | `ROMFS/px4fmu_common/init.d-posix/airframes/5101_gz_transform_quad_plus` |
| 5102 | 十字型通行测试，带桨盘碰撞体 | `ROMFS/px4fmu_common/init.d-posix/airframes/5102_gz_transform_quad_plus_collision` |
| 5103 | 双目识别与自动折叠通行原型 | `ROMFS/px4fmu_common/init.d-posix/airframes/5103_gz_transform_quad_plus_stereo` |

4099 在新进程中使用模块默认值：X 型、折叠上限 1.0、全行程 12 秒。
5101/5102 使用 `geometry plus`、`limit 0.30`、`transition 6.0`。
5102 引用 5101 配置，并指定通行世界和带碰撞体的模型。

## 3. 控制实现

核心代码：

- [折叠控制模块](src/modules/transform_control/transform_control.cpp)
- [几何计算](src/modules/transform_control/TransformGeometry.hpp)
- [模块说明](src/modules/transform_control/README.md)
- [构建配置](src/modules/transform_control/CMakeLists.txt)
- [模块开关](src/modules/transform_control/Kconfig)

控制流程：输入命令 → 目标折叠进度 → 限幅/匀速推进 → 舵机指令与动态电机坐标 → 控制分配。
PX4 原有姿态、角速度闭环继续运行，模块不直接取代飞行控制器。

- 进度 `p=0` 表示展开，`p=1` 表示模型定义的完整折叠。
- 机臂转角为 `60° × p`，电机位置按铰点旋转计算。
- 舵机指令约 50 Hz；运动中几何参数约 10 Hz 更新，并在到达目标时更新。
- 更新八个 `CA_ROTOR0..3_PX/PY` 参数后通知控制分配器重建效能矩阵。
- `transition` 是完整 0→1 的时间；6 秒设置下，0→0.30 约需 1.8 秒。
- 输入支持终端、手动控制按钮 bit0、RC AUX，以及外部动作命令。
- 外部 `DO_SET_ACTUATOR` 使用 `param7=99` 标识折叠动作；内部舵机指令使用 `param7=0`、功能 301。
- 显式 `fold/open` 命令会保持控制权，执行 `manual` 才恢复手动输入。
- `normal` 同时驱动舵机与几何；`servo_only`、`geometry_only` 为隔离测试模式。
- `geometry x/plus` 需在模块启动前配置，运行中不能切换。

相关 PX4 接口文件：

| 路径 | 作用 |
| --- | --- |
| `src/modules/control_allocator/ControlAllocator.cpp` | 处理参数变化、重建分配矩阵 |
| `src/modules/control_allocator/ControlAllocator.hpp` | 参数更新订阅周期为 100 ms |
| `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp` | 根据电机位置、推力轴和 KM 计算效能 |
| `src/lib/mixer_module/functions/FunctionActuatorSet.hpp` | 接收执行器命令并提供功能 301 等输出 |
| `boards/px4/fmu-v6c/default.px4board` | 启用 `CONFIG_MODULES_TRANSFORM_CONTROL=y` |

当前没有实际舵机角度反馈，几何使用的是指令进度；也没有随折叠更新重心、惯量或控制增益。
`status` 显示模块配置和命令来源，不显示实际机构位置。

## 4. 仿真模型和世界文件

| 路径 | 内容 |
| --- | --- |
| `Tools/simulation/gz/models/transform_quad/` | 原 X 型模型和 STL 网格 |
| `Tools/simulation/gz/models/transform_quad_folded/` | 原静态折叠参考模型 |
| `Tools/simulation/gz/models/transform_quad_plus/` | 十字型模型，保留原 base/IMU 安装姿态 |
| `Tools/simulation/gz/models/transform_quad_plus_collision/` | 通行专用模型，增加桨盘圆柱碰撞体 |
| `Tools/simulation/gz/tools/generate_transform_plus.py` | 从原模型生成十字型模型 |
| `Tools/simulation/gz/tools/generate_transform_passage.py` | 生成新通行世界和专用碰撞模型 |
| `Tools/simulation/gz/worlds/transform_challenge_world.sdf` | 旧场景，整条通道在世界中旋转 45 度 |
| `Tools/simulation/gz/worlds/transform_plus_passage.sdf` | 新十字型通行场景，沿世界 +X |

两个生成脚本会覆盖对应生成文件。后续修改门宽、模型或场景时，需同步修改生成脚本，
避免再次生成时丢失手工修改。新模型的网格 URI 仍引用原 `transform_quad/meshes/`。

### 新世界的布局

全部路线中心位于世界 `Y=0`；门洞中心高度 `Z=1.60 m`。

| 元素 | 世界 X | 尺寸/用途 |
| --- | --- | --- |
| 起飞区 | 0 m | 模型初始生成位置 `(0,0,0.35)` |
| 第一练习门 | 3 m | 净宽 440 mm |
| 黄色折叠等待区 | 5 m | 停稳并折叠 |
| 第二验证门 | 7 m | 净宽 360 mm |
| 蓝色展开等待区 | 9 m | 离开门后展开 |
| 降落区 | 12 m | 降落 |

两门净高 700 mm，下沿 1.25 m、上沿 1.95 m，厚 250 mm。
地面刻度、颜色区和展开/折叠投影示意均为视觉提示，没有自动触发功能。
5102 专用模型消除了原 `base_link` 额外 90 度偏航，使零生成偏航下机头对准世界 +X。
世界 ENU 的 +X 向东，对应 PX4 本地 NED 的 +Y；不要直接把世界 X 当作 PX4 北向 X。

### 通行宽度

从 STL 顶点测得桨盘最大半径约 63.54 mm。专用模型使用半径 63.55 mm、厚度 6 mm
的圆柱近似旋转桨盘碰撞包络，门体也有碰撞体。按 SDF 关节几何计算：

| 折叠进度 | 含桨盘横向宽度 | 360 mm 门单侧理论余量 |
| --- | --- | --- |
| 0 | 407.10 mm | -23.55 mm |
| 0.30 | 398.83 mm | -19.42 mm |
| 0.80 | 350.97 mm | 4.51 mm |
| 1.00 | 322.24 mm | 18.88 mm |

上述计算假定水平、对准通道、达到目标构型。30% 折叠不能通过第二道门。
第一道门在展开时每侧仅约 16.45 mm 余量，用户已反馈手动 Position 通行容易撞框。
建议把练习门加宽至 600 mm，目前代码仍为 440 mm，尚未修改。

原 SDF 铰点尺寸与控制代码略有差异。例如完整折叠电机中心宽度，SDF 约 195.14 mm，
控制代码约 192.33 mm。后续需结合实际机械尺寸统一；计算宽度不等于实测关节跟踪结果。
圆柱碰撞体不模拟桨叶破损或近壁气动效应。

## 5. 已有验证与证据

### 5101：三轮 30% 折叠/展开

2026-10-11，3 m 高度起飞、悬停、三轮 0→0.30→0、降落。
最大绝对横滚约 0.369°、俯仰约 0.259°，动作及后续 3 秒窗口中
最大水平位移约 0.0659 m、高度变化约 0.0249 m，电机输出约 0.5730～0.5801，
力矩目标达成率 100%，ULog 无丢帧。每次动作状态消息间隔约 1.78 秒。

- [验证报告](validation/transform_plus_sitl.md)
- 控制台：`build/transform_plus_check_20261011_125845/console.txt`
- 飞行日志：`build/transform_plus_check_20261011_125845/log/2026-10-11/04_59_14.ulg`
- 自动测试：`Tools/transform_plus_sitl_check.py`
- 日志分析：`Tools/analyze_transform_plus_log.py`

该结果不覆盖 100% 折叠、带碰撞模型穿门或实物飞行。
控制台出现过 MAVLink 命令确认队列丢失消息，外部动作确认可靠性仍需检查。

### 5102：场景加载和用户手动飞行

SDF 校验、门洞尺寸计算及实际加载检查通过，模型和全部场景元素成功生成。
加载检查没有解锁或飞行。用户随后反馈 QGC Position 模式下通过第一道门，
但存在小幅漂移，容易碰框。完整折叠及第二道门通行尚无成功验证记录。

- [场景说明](validation/transform_plus_passage.md)
- 加载控制台：`build/transform_plus_check_20261011_132944/console.txt`

### Position 漂移分析

分析文件是 2026-10-12 北京时间 13:55:42 的日志，时长约 105.06 秒，
机型 5102。用户完成起飞、前进、修正、悬停和降落。
筛选摇杆各轴绝对值小于 0.02 的中立段，排除前 3 秒制动后得到：

| 指标 | 第一段 | 第二段 |
| --- | --- | --- |
| 日志起点后分析区间 | 65.584～70.376 s | 87.392～94.376 s |
| 最大估计水平位置跟踪误差 | 10.8 cm | 11.6 cm |
| Gazebo 真值横向变化范围 | 13.2 cm | 6.5 cm |
| 最大估计水平速度 | 6.0 cm/s | 6.5 cm/s |

两段中位置目标固定且摇杆接近零，真值也显示实际运动。它们是短时段移动范围，
不能据此认定持续单向漂移或直接归因于 PID。位置和航向重置在启动初期，
不在上述窗口；窗口中电机无饱和，力矩目标可达，无 failsafe、无日志丢帧。
尚未排除先前操纵或接触后的恢复过程，需要空旷区域 20～30 秒连续松杆补测。

- 原始日志：`build/px4_sitl_default/rootfs/log/2026-10-12/05_55_42.ulg`
- [分析脚本](validation/drift_20261012/analyze.py)
- [量化数据](validation/drift_20261012/metrics.json)
- [曲线图](validation/drift_20261012/position_drift.png)

此日志是已分析的那份，不代表后续运行中仍为最新日志。

## 6. 编译、启动与诊断

以下命令在仓库根目录执行。GUI 复现前先退出其他正在使用的 PX4/Gazebo，
避免连接到已有世界或同名实例。

```sh
# 编译仿真
make px4_sitl_default

# 原 X 型
PX4_SYS_AUTOSTART=4099 make px4_sitl gz_transform_quad

# 十字型普通仿真
PX4_SYS_AUTOSTART=5101 make px4_sitl gz_transform_quad_plus

# 十字型通行场景，专用碰撞模型
PX4_SYS_AUTOSTART=5102 make px4_sitl gz_transform_quad_plus_collision_transform_plus_passage
```

自动验证脚本使用独立参数目录和 Gazebo transport partition：

```sh
python3 Tools/transform_plus_sitl_check.py
python3 Tools/transform_plus_sitl_check.py --passage-smoke
```

前者执行 5101 起降与三轮 30% 折叠；后者只检查 5102 场景加载。
自动脚本依赖 `pexpect`，分析脚本依赖 `pyulog`、`numpy`，漂移绘图还依赖 `matplotlib`。

PX4 控制台诊断：

```sh
param show SYS_AUTOSTART
transform_control status
transform_control geometry
listener manual_control_setpoint 10
listener vehicle_local_position 10
```

仿真折叠命令：

```sh
transform_control fold
transform_control open
transform_control manual
```

若在 5102 中验证完整折叠，先在空旷位置验证悬停，再考虑穿门；这是待验证操作：

```sh
transform_control transition 12
transform_control limit 1.0
transform_control fold
```

完整动作约 12 秒；这些终端设置不是写入机型脚本的持久配置。
普通 SITL 日志位于 `build/px4_sitl_default/rootfs/log/`；自动脚本证据位于
`build/transform_plus_check_日期_时间/`。`build/` 中产物和日志可能被清理，交接时应保留原始证据。

## 7. 实物配置状态

硬件目标为 Pixhawk FMU-V6C / Holybro 6C mini；电机信号接 AUX1～AUX4，
折叠舵机接 MAIN1，功能为 301。实物脚本默认 `rc_aux`、`pwm_open_high`、
上限 0.30、全行程 6 秒。通道 6/VrB 控制折叠比例，通道 7/SwA 使能。
RC 比例先从 [-1,1] 映射至 [0,1] 再截断到上限，并非把旋钮全行程缩放到 [0,0.30]。

MAIN1 配置范围为 750～1200 μs，展开 1200 μs，失效/未使能值也为 1200 μs。
实际输出受解锁、输出反向等设置影响，不应只凭指令认为机构已到位。

```sh
make px4_fmu-v6c_default
```

固件路径：`build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`，旁边有 `.bin` 与 `.elf`。
已完成一次十字型 V6C 编译；本次阶段未执行刷机。
此前用户在原 X 型固件上确认 `SYS_AUTOSTART=4101` 且模块 running，
不能据此认定实物现在已经运行新的十字型构型。

若飞控箭头朝电机 1 且无其他安装旋转，板卡旋转为 0；若保留原 X 型安装，
箭头朝电机 1/3 之间，则对应 `SENS_BOARD_ROT=7`（Yaw 315°）。须按实际安装核对，
脚本没有自动修改该参数。仿真中的 100% 折叠设置不可直接推断为实物可用。

## 8. 早期 5102 后续建议与交接约束

以下为 5102 原场景的建议。5103 已在独立训练场景完成视觉对中、折叠与通行，
当前剩余重点为重复试验统计成功率、验证原 360 mm 门、统一几何尺寸及实物验证。

1. 先将练习门由 440 mm 加宽到 600 mm，同步修改生成脚本、世界文件和场景说明。
2. 空旷位置做 20～30 秒连续松杆 Position 悬停，比较目标、估计和 Gazebo 真值；
   区分制动、位置保持误差、估计漂移和碰框后恢复，不预先认定 PID 故障。
3. 在带桨盘碰撞模型上验证完整折叠悬停，检查姿态、位置、关节角和电机余量。
4. 再验证第二道门的低速居中通行，记录接触状态与飞行日志；必要时增加自动位置控制。
5. 统一 SDF 与控制代码的机械尺寸，补充实际角度反馈或可观测状态。

航向补偿设计在 `src/modules/transform_control/HEADING_COMPENSATION_DESIGN.md`，
当前尚未实现；不要按文档中的未来 `heading on`、`transform_status` 接口假定现有固件支持。
旧 X 型实验说明在 `morphing_uav_inflight_fold.md`。

Gazebo 文件位于 `Tools/simulation/gz` 子模块。发布时需先提交、推送 `gz-models` 分支，
再更新并推送父仓库 `px4-transform-quad` 分支中的子模块指针。
拉取项目时使用 `git clone --recurse-submodules --branch px4-transform-quad`，
已有仓库更新后执行 `git submodule update --init --recursive`，避免模型与控制代码不一致。

如果后续 ChatGPT 只能读取本文而不能访问本地仓库，应让用户提供对应源码或日志，
不能根据路径声称已读取文件、修改代码或执行验证。当前用户关注的是仿真漂移与折叠通行，
实物验证应在仿真问题明确后继续。

## 9. 2026-10-17：双目识别与自动通行原型

用户已授权开始实现。新增独立机型 `5103_gz_transform_quad_plus_stereo`，
模型 `transform_quad_plus_stereo`，世界 `transform_stereo_passage`。
原 5102 世界保持 440/360 mm；新训练世界为 600/440 mm，避免混淆验证条件。

- 理想双目基线 120 mm、960×600、15 Hz，固定在机身，首版识别青绿色门边线。
- 双目图像三角化测宽；根据含桨盘包络、±5° 航向偏差和默认每侧 40 mm
  余量选择折叠比例。保留旧两档接口，新增终端 `target 0..1` 和外部
  `DO_SET_ACTUATOR param7=100 / param1=比例`；仍受折叠上限约束。
- 伴随控制器通过 MAVLink Offboard 完成起飞、低速接近、近距离复测对中、折叠、
  关节到位确认、穿越、展开、降落。默认仅感知，`--execute` 才飞行，并核对 5103。
- 利用视觉位移与本地里程计位移估计坐标航向对齐；这不是前述构型航向补偿设计的实现。
- 独立进程记录 Gazebo 真值用于事后验收，不向控制器传递真值。
- 静态双目验证：600 mm 门连续 12 帧中位值 600.0 mm；440 mm 门为 446.7 mm。
  SITL 编译、SDF 校验和 6 项自动测试通过。
- 2026-10-17 完成一次完整自动飞行：展开通过 600 mm 门，双目测得第二门
  440.625 mm，自动选择 87% 折叠并通过 440 mm 门，随后展开并在门后就地降落。
  接触通道收到 2254 条消息，门体接触为 0；独立真值确认两门洞内穿越。
  ULog 无丢帧，动力工作窗口最大横滚/俯仰约 0.492°/1.047°，电机控制最大约 0.636，
  力矩和推力目标达成率均为 100%。
- 原始运行：`build/transform_stereo_20261017_115627_528215/`。
  压缩原始日志、真值、事件、结果、哈希清单和轨迹图已另存
  `validation/stereo_20261017/flight/`。接触正向测试另有 7 条门体接触证据。
- 这是带标记门、理想双目及当前训练场景的一次成功，不代表稳定成功率、无标记门、
  原 360 mm 门、100% 折叠或实物飞行已经验证。开发版本有失败运行；旧接触订阅无消息
  的完整轨迹运行被判失败，不计入无接触成功证据。

入口：`python3 Tools/transform_stereo_sitl_check.py`（静态，不飞行），
`python3 Tools/transform_stereo_sitl_check.py --flight`（隔离仿真，自动解锁飞行）。
完整说明和验证记录：[双目通行原型](validation/transform_stereo_passage.md)。

## 10. 2026-10-18：可视化与每次运行的数据保存

- `Tools/transform_stereo_sitl_check.py --flight --gui` 增加 Gazebo 三维窗口和后台相机跟随。
- 默认数据位置改为 `logs/transform_stereo/transform_stereo_时间_微秒/`，每次独立保存，
  支持 `--output-root` 自定义位置，构建清理不会删除该目录。
- 自动保留 ULog、参数、控制台、GUI 输出、状态/测量/遥测、真值和默认每秒一对的双目图片；
  结束后生成 CSV、轨迹图、汇总和 SHA-256 清单。
- 启动失败、中断和分析失败都保留已有数据；数据缺失不视为飞行成功。
- 9 项测试通过；图形实跑目录 `logs/transform_stereo/transform_stereo_20261018_143730_487852/`，
  已验证相机跟随、自动起飞、第一门通行，保存 102 对双目图片、ULog、CSV、轨迹图，
  225 个数据文件的 SHA-256 校验通过。
- 本轮第二门前机臂 1/2 折叠而 3/4 未到位，触发 `Folding/settling timeout`，
  已如实记为失败；该问题待查，不覆盖第 9 节已有的一次成功记录。
- 说明与当前验证记录：[可视化与数据保存](validation/transform_stereo_visualization.md)。

## 11. 2026-10-19：一键 Gazebo 与 RViz 双目显示

- 入口：`python3 Tools/transform_stereo_sitl_check.py --flight --rviz`。
  `--rviz` 隐含 `--gui`，自动加载 ROS Humble 环境和预设左右 Image 窗口。
- 新增 `Tools/transform_stereo_ros_bridge.py`、`Tools/transform_stereo_rviz.py`、
  `Tools/transform_stereo.rviz`；直接使用 Gazebo Python 与 rclpy 桥接，无需 ros_gz_bridge。
- 用户已安装 RViz2。默认预览 5 Hz、480×300，可用 `--rviz-fps` 和 `--rviz-scale` 调整。
  感知/留存仍用原始图像；限制数值库线程、降低显示进程优先级、只保留最新帧以降低负载。
- OpenCV 依赖移至忽略目录 `logs/transform_stereo_deps/`，避免 `/tmp` 随重启丢失。
- 每次额外保存 `rviz.txt`、`ros_bridge.txt`、`ros_images.json`、`stereo.rviz` 和 `ros_logs/`；
  ROS 图像发布和订阅统计写入 `run.json`。缺依赖时在启动飞控前失败并保存记录。
- 低负载静态实跑 `logs/transform_stereo/transform_stereo_20261019_150623_858439/` 成功：
  两路各发布 8 帧，均有 RViz 订阅，测得 600 mm 门宽，数据与清单生成。
- 先前图形飞行运行发生过控制循环超时、遥测过期，用户反馈电脑卡住。
  降负载后仅完成静态显示验证，尚未重新证明完整飞行稳定性；原机臂未到位问题仍待排查。

## 12. 2026-10-19：折叠同步与图形运行稳定性修复

- 5103 改为单一 `servo_0` 输入，四个关节保持独立物理 PID，禁用多余的
  `SIM_GZ_SV_FUNC2..4`。生成脚本与模型已同步修改；旧机型及实物配置不变。
- 静态固定基座测试完成 0→0.3→0.875→1→0→0.875→0，两轮测试四臂均正常跟踪。
  新增诊断工具 `Tools/transform_stereo_mechanism_check.py`，不启动 PX4、不解锁飞行。
- `--fold-check` 提供原地起飞、87.5% 折叠、展开和降落回归。
  `logs/transform_stereo/transform_stereo_20261019_172823_661205/` 成功，控制循环最大 51.6 ms。
- 本轮仿真发现实际姿态振荡，仅在 5103 将 `MC_ROLLRATE_K`、`MC_PITCHRATE_K`、
  `MC_YAWRATE_K` 降为 0.2。机臂到位与机体停稳分别限时 25/60 仿真秒，
  到位精度、速度、姿态、碰撞和控制循环保护保持。
- 新增独立视觉进程 `Tools/transform_stereo_vision_worker.py`；解码、识别与 PNG 写盘
  不再阻塞 Offboard，队列有界，只采用新鲜测量，子进程失败会中止飞行。
- 本机 OpenCV 默认 28 个线程，现明确限制为 1；Gazebo 使用 900×600、无抗锯齿配置。
  图形模式默认 `--sim-speed 0.5`，无图形默认 1，控制目标与阶段计时同步换算。
- 新增运行互斥锁，避免并行启动争用端口和资源；记录 `vision.json`、
  `resources.jsonl`、实际 Gazebo 命令、循环最大间隔和中止时反馈，便于事后复核。
- 图形回归 `logs/transform_stereo/transform_stereo_20261019_173903_443579/` 已无接触穿过两门，
  但第二门后横向误差 40.3 mm 超过规划约 40.0 mm，按保护退出，不计作完整任务成功。
  后续改用门平面横向误差而非图像中心偏差，并在门前暂停时修正偏航，
  保留原误差限值，继续完整图形回归。投影测试覆盖偏航时画面居中但机身偏 35 mm 的情况。
- 最终图形回归 `logs/transform_stereo/transform_stereo_20261019_192056_825527/` 成功：
  600/440 mm 两门自动通过，第二门选择 88.5% 折叠，之后展开并降落；
  两次真值穿越在门洞内，门体接触 0 / 2095 条通道消息，ULog 无丢帧。
  控制循环最大间隔 52.0 ms，运行阶段最小可用内存约 2910 MiB；
  保存 262 对双目图片，RViz 两路各发布 1465 帧；554 个文件 SHA-256 校验通过。
- 16 项本地回归通过；4 项 ROS 桥接测试已在系统 Python 中单独通过。
  成功数据、配置快照、压缩原始记录和图表归档：
  [修复与验证记录](validation/stereo_20261019_stability/README.md)。
  完整任务的长期成功率仍需后续重复实验，不能外推至实物或旧 360 mm 门。
