# transform_control 模块说明

## 1. 模块概述

`transform_control` 是为可变形四旋翼新增的 PX4 模块，用于协调机臂变形执行器和 Control Allocation 几何参数。模块在飞行过程中维护统一的折叠进度，并同时完成以下工作：

- 接收终端命令、遥控器输入或 QGroundControl Action 发出的变形命令；
- 按设定速度平滑改变折叠进度；
- 通过 `Actuator Set 1` 驱动仿真舵机关节或实物舵机；
- 根据折叠进度更新四个电机的 `CA_ROTOR*_PX/PY`；
- 提供舵机单独测试和动态几何单独测试模式。

模块源文件位于：

```text
src/modules/transform_control/
├── CMakeLists.txt
├── Kconfig
├── README.md
├── TransformGeometry.hpp
└── transform_control.cpp
```

模块使用 `CONFIG_MODULES_TRANSFORM_CONTROL=y` 加入固件。当前 Pixhawk 6C mini 目标已在 `boards/px4/fmu-v6c/default.px4board` 中启用该模块。

## 2. 工作流程

模块以 50 Hz 运行，控制流程如下：

```text
终端命令 / QGC Action / RC 输入
                │
                ▼
         计算目标折叠进度
                │
                ▼
       变形幅度限制与速度规划
                │
          ┌─────┴─────┐
          ▼           ▼
  Actuator Set 1   CA_ROTOR*_PX/PY
     舵机输出        动态几何更新
          │           │
          ▼           ▼
   机臂实际变形   Control Allocation
```

折叠进度定义为：

- `0.0`：机臂完全展开；
- `1.0`：机臂按当前模型定义完全折叠；
- `0.0 < progress < 1.0`：部分折叠。

`limit` 会限制最大折叠进度。例如实物配置中的 `limit 0.30` 表示遥控器或折叠命令最多只能到达 30% 折叠位置。

## 3. 状态机

模块根据当前进度和目标进度维护以下状态：

| 状态 | 含义 |
| --- | --- |
| `open` | 已完全展开 |
| `folding` | 正在向折叠方向运动 |
| `folded` | 已达到 100% 折叠位置 |
| `unfolding` | 正在向展开方向运动 |
| `partial` | 已到达一个非 0、非 1 的目标进度 |

状态发生变化时会在 PX4 Shell 中输出日志，例如：

```text
INFO  [transform_control] state=folding progress=0.00
INFO  [transform_control] state=partial progress=0.30
```

## 4. 命令接口

所有命令均在 PX4 Shell 或 QGroundControl 的 MAVLink Console 中执行。

### 4.1 启停和状态

```sh
transform_control start
transform_control stop
transform_control status
```

`status` 会显示运行状态、测试模式、输入源、输出映射、折叠上限、过渡时间和当前命令来源。

### 4.2 折叠控制

```sh
transform_control fold
transform_control open
transform_control unfold
transform_control manual
```

- `fold`：折叠到 `limit` 允许的最大进度；
- `open` 或 `unfold`：恢复到展开位置；
- `manual`：退出终端或外部命令控制，重新使用当前手动输入源。

`fold` 和 `open` 会保持为显式命令目标。在执行 `manual` 之前，遥控器输入不会重新取得控制权。

### 4.3 输入源

```sh
transform_control source
transform_control source button
transform_control source rc_aux
```

`button` 模式读取 `manual_control_setpoint.buttons` 的 bit 0：

- button 0 按下：折叠；
- button 0 释放：展开。

`rc_aux` 模式用于实物遥控器：

- `aux2 > 0.5`：允许变形；
- `aux1` 从 `[-1, 1]` 映射为折叠进度 `[0, 1]`；
- 进度最终受 `limit` 限制；
- AUX2 未使能、输入无效或 RC AUX 数据超时 500 ms 时，目标回到展开位置。

当前实物机型将遥控器通道 6 映射到 AUX1（VrB），通道 7 映射到 AUX2（SwA）。

### 4.4 折叠幅度和速度

```sh
transform_control limit
transform_control limit 0.30

transform_control transition
transform_control transition 6.0
```

`limit` 的范围为 `0.0..1.0`。

`transition` 的范围为 `3.0..30.0` 秒，表示从进度 0 运动到进度 1 所需的完整时间。因此设置 `transition 6.0`、`limit 0.30` 时，从完全展开到最大允许位置约需：

```text
6.0 s × 0.30 = 1.8 s
```

### 4.5 执行器输出映射

```sh
transform_control output
transform_control output progress
transform_control output pwm_open_high
transform_control output pwm_open_low
```

| 输出模式 | 展开时输出 | 折叠时输出 | 用途 |
| --- | ---: | ---: | --- |
| `progress` | `0` | `1` | Gazebo 仿真 |
| `pwm_open_high` | `1` | `-1` | 实物舵机，较高 PWM 为展开位置 |
| `pwm_open_low` | `-1` | `1` | 实物舵机，较低 PWM 为展开位置 |

模块通过 `VEHICLE_CMD_DO_SET_ACTUATOR` 发布执行器设定值，`param7=0` 表示 Actuator Set 1。对应输出通道的 Function 必须设置为 `301 (Peripheral via Actuator Set 1)`。

### 4.6 调试模式

```sh
transform_control mode normal
transform_control mode servo_only
transform_control mode geometry_only
```

| 模式 | 舵机输出 | 动态几何更新 | 用途 |
| --- | --- | --- | --- |
| `normal` | 是 | 是 | 正常运行 |
| `servo_only` | 是 | 否 | 单独验证机械变形对飞行的影响 |
| `geometry_only` | 否 | 是 | 单独验证 Control Allocation 动态更新 |

这三个模式主要用于定位折叠失稳来自机械运动还是控制分配变化，不应将测试模式作为正常飞行配置。

## 5. 动态 Control Allocation

机臂折叠后，电机相对机体中心的位置发生变化。若仍使用展开状态的电机力臂，Control Allocation 会根据错误的几何关系计算各电机输出。因此模块根据折叠进度实时更新：

```text
CA_ROTOR0_PX / CA_ROTOR0_PY
CA_ROTOR1_PX / CA_ROTOR1_PY
CA_ROTOR2_PX / CA_ROTOR2_PY
CA_ROTOR3_PX / CA_ROTOR3_PY
```

对于每个电机，模块以机臂铰点为旋转中心，根据当前折叠角计算电机坐标：

```text
angle = 60 deg × progress
rotor_position = hinge_position + R(angle) × (open_position - hinge_position)
```

舵机设定值以 50 Hz 发布；几何参数在进度变化时最多以 10 Hz 更新，并在达到目标位置时执行最后一次更新。运行时参数更新会临时关闭参数自动保存，避免连续插值过程频繁写入非易失存储器，随后通过参数通知触发 Control Allocation 使用新几何。

当前几何算法固定面向四旋翼，展开坐标、铰点坐标和最大 60 度折叠角定义在 `TransformGeometry.hpp` 中。更换机械尺寸或折叠方向时，需要同步修改这些数据。

### 5.1 X 型与十字型

模块默认使用 `geometry x`，与现有 Gazebo 模型一致。实物机型 `4101` 在启动模块前执行 `transform_control geometry plus`，选择电机 1 朝前的十字型机体系。查询当前构型：

```sh
transform_control geometry
transform_control status
```

设置构型只能在模块未运行时执行。构型在模块启动时固定，运行中不允许更改机体系。

十字型坐标由原 X 型的完整机构绕 Z 轴旋转 -45 度得到，电机位置与铰点一起变换，折叠方向、接线与 `KM` 符号保持原有对应关系。

| 实物电机 | PX4 下标 | 展开位置 | X / Y (m) | KM |
| --- | --- | --- | --- | --- |
| 1 | 0 | 前 | +0.140007143 / 0 | -0.05 |
| 2 | 1 | 后 | -0.140007143 / 0 | -0.05 |
| 3 | 2 | 左 | 0 / -0.140007143 | +0.05 |
| 4 | 3 | 右 | 0 / +0.140007143 | +0.05 |

十字型从机头方向看的电机中心横向跨度随折叠进度减小：展开约 280.0 mm，30% 折叠约 270.6 mm，完全折叠约 192.3 mm。实际宽度还需计入桨盘与机身包络。实物折叠上限仍为 0.30。

### 5.2 飞控安装方向

新机头沿电机 1 的机臂。若飞控箭头实际朝向电机 1，板卡无其他安装旋转时使用 `SENS_BOARD_ROT=0`。若飞控保留原 X 型安装方向，即箭头仍朝电机 1 与电机 3 之间，则相对新机头是 Yaw 315 度，对应 `SENS_BOARD_ROT=7`。其他安装方式需按实测方向设置。

机型脚本不自动修改 `SENS_BOARD_ROT`。更新前确认实际安装方向，在地面配置并重启后核对姿态显示；仅修改 Control Allocation 坐标不会改变传感器定义的机头。

机型编号仍为 `SYS_AUTOSTART=4101`，显示名称改为 `Transform Quad Plus Real RC Safe`，分类改为 `Quadrotor +`。模块启动会写入当前构型的全部八个 `CA_ROTOR*_PX/PY` 运行值，即使参数存储中仍有旧 X 型坐标，也会按十字型初始化。

## 6. 外部命令与 QGroundControl Action

模块订阅外部 `VEHICLE_CMD_DO_SET_ACTUATOR` 命令。为避免将普通执行器命令误判为折叠命令，外部变形命令使用 `param7=99` 作为标识：

- `abs(param1) > 0.05`：折叠；
- `abs(param1) <= 0.05`：展开。

收到外部命令后，模块会切换为显式目标，并输出：

```text
INFO  [transform_control] external target progress: 1.000
```

该接口供 QGroundControl 的自定义 MAVLink Action 使用。需要恢复遥控器连续控制时，执行 `transform_control manual`。

双目通行原型新增比例接口：外部 `param7=100`、`param1=0..1`，或终端
`transform_control target 0.85`。比例必须有限且在范围内，实际目标仍受 `limit`
限制、按 `transition` 匀速推进。`param7=99` 保留两档行为，普通舵机输出仍为
`param7=0`；命令日志/确认不代表机构实际到位。新仿真机型 5103 的说明见
`validation/transform_stereo_passage.md`。

## 7. Gazebo 仿真配置

十字型使用独立机型 `5101_gz_transform_quad_plus` 和模型 `transform_quad_plus`，启动命令为：

```sh
PX4_SYS_AUTOSTART=5101 make px4_sitl gz_transform_quad_plus
```

它与实物一样使用 `geometry plus`、`limit 0.30`、`transition 6.0`，舵机输出仍采用仿真专用的 `progress` 映射。已完成三轮 30% 空中折叠/展开和降落，结果及自动验证脚本见 `validation/transform_plus_sitl.md`。以下 4099 的说明保留用于原 X 型仿真。

仿真机型脚本位于：

```text
ROMFS/px4fmu_common/init.d-posix/airframes/4099_gz_transform_quad
```

主要配置包括：

- `SIM_GZ_EC_FUNC1..4 = 101..104`：四个电机；
- `SIM_GZ_SV_FUNC1..4 = 301`：四个仿真关节共用 Actuator Set 1；
- `SIM_GZ_SV_MAXA1..4 = -60`：设定关节折叠角范围；
- `transform_control start`：启动变形模块。

仿真默认使用 `progress` 输出模式，折叠进度直接映射到 Gazebo 舵机关节。四个关节使用同一个执行器设定值，用于模拟实物中一个舵机通过太阳轮和行星轮同步驱动四条机臂。

常用仿真测试命令：

```sh
transform_control status
transform_control fold
transform_control open
```

## 8. 实物配置

实物机型脚本位于：

```text
ROMFS/px4fmu_common/init.d/airframes/4101_transform_quad_real
```

当前硬件连接为：

| 设备 | 飞控接口 | Function |
| --- | --- | ---: |
| 电机 1 | AUX1 | 101 |
| 电机 2 | AUX2 | 102 |
| 电机 3 | AUX3 | 103 |
| 电机 4 | AUX4 | 104 |
| 折叠舵机 | MAIN1 | 301 |

MAIN1 的当前 PWM 配置为：

```text
展开位置：1200 us
折叠方向终点：750 us
失效位置：1200 us
更新频率：50 Hz
```

机型启动时执行：

```sh
transform_control geometry plus
transform_control source rc_aux
transform_control output pwm_open_high
transform_control limit 0.30
transform_control transition 6.0
transform_control start
```

这套配置使 SwA 控制是否允许变形，VrB 控制 0% 到 30% 的折叠程度，并保证模块启动时以展开构型初始化舵机和 Control Allocation。

## 9. 编译与刷写

编译 Pixhawk 6C mini 固件：

```sh
make px4_fmu-v6c_default
```

通过 USB 刷写：

```sh
make px4_fmu-v6c_default upload
```

刷写并选择实物机型后，可在 MAVLink Console 中确认：

```sh
transform_control status
param show SYS_AUTOSTART
```

实物机型对应 `SYS_AUTOSTART=4101`。

## 10. 诊断方法

### 确认模块运行

```sh
transform_control status
```

实物正常配置应包含类似输出：

```text
geometry: plus (motor 1 front)
running, mode: normal, source: rc_aux, output: pwm_open_high,
limit: 0.300, transition: 6.0s, command target: manual
```

### 确认遥控器输入

```sh
listener manual_control_setpoint 10
```

检查 `aux1` 是否随 VrB 连续变化，`aux2` 是否随 SwA 在高低值之间切换。

### 确认舵机输出

```sh
listener actuator_outputs 10
```

根据输出实例和通道确认 MAIN1 是否从展开 PWM 向折叠 PWM 平滑变化。

### 确认动态几何

在折叠过程中多次执行：

```sh
param show CA_ROTOR0_PX
param show CA_ROTOR0_PY
```

坐标应随折叠进度变化。也可以使用 `servo_only` 和 `geometry_only` 对舵机运动与动态分配进行隔离测试。

## 11. 当前限制

- 仅支持四个电机和当前机械结构的几何参数；
- 最大折叠角固定为 60 度；
- 没有舵机位置反馈，模块记录的是指令进度，无法直接确认机械结构是否到达目标位置；
- 动态更新电机位置，不更新机体质量、重心和惯量；
- 外部命令仅提供折叠/展开目标，连续折叠程度由 RC AUX 控制；
- 实物飞行安全上限目前设置为 30%，扩大范围前需要逐级完成悬停和日志验证。
