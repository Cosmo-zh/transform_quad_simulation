# 可变形四旋翼空中折叠仿真说明

本文记录 `transform_quad` 在 PX4 SITL + Gazebo Harmonic 中实现空中折叠/展开的当前方案、关键参数和验证结果。

## 当前状态

已实现并验证：

- 展开状态起飞、悬停和降落。
- 空中触发机臂折叠。
- 折叠过程中动态更新 Control Allocation 的电机位置。
- 折叠完成后继续稳定悬停。
- 空中触发展开，并恢复展开构型飞行。
- QGC MAVLink Actions 一键折叠/展开。
- `manual_control_setpoint.buttons` bit0 诊断触发。
- 实物安全入口：RC AUX2 使能开关 + AUX1/VrB 折叠比例。

最新完整验证日志：

- `build/px4_sitl_default/rootfs/log/2026-07-16/10_54_53.ulg`

该日志中完成 3 次折叠和 3 次展开。每次动作期间都有约 122 次 `parameter_update`，持续约 12 秒，确认动态 Control Allocation 生效。

## 文件结构

主要新增/修改文件：

- `src/modules/transform_control/`
  - PX4 空中变形控制模块。
- `ROMFS/px4fmu_common/init.d-posix/airframes/4099_gz_transform_quad`
  - 主测试机型，展开构型起飞，空中动态折叠/展开。
- `ROMFS/px4fmu_common/init.d-posix/airframes/4100_gz_transform_quad_folded`
  - 静态折叠参考机型，仅用于对比。
- `ROMFS/px4fmu_common/init.d/airframes/4101_transform_quad_real`
  - 实物专用机型，使用真实 PWM/RC 参数，不包含 Gazebo 仿真参数。
- `Tools/simulation/gz/models/transform_quad/`
  - Gazebo 可折叠模型。
- `Tools/simulation/gz/models/transform_quad_folded/`
  - Gazebo 静态折叠参考模型。
- `src/lib/mixer_module/functions/FunctionActuatorSet.hpp`
  - 让 `Peripheral via Actuator Set 1` 能通过 `vehicle_command` 回调及时驱动 Gazebo 舵机输出。
- `src/modules/control_allocator/ControlAllocator.hpp`
  - 将参数更新检查周期从 1 s 缩短到 100 ms，用于折叠过程中的几何更新。

## 仿真和实物配置边界

仿真使用：

- 机型：`4099_gz_transform_quad`
- 位置：`ROMFS/px4fmu_common/init.d-posix/airframes/`
- 输出参数：`SIM_GZ_EC_FUNC*`、`SIM_GZ_SV_FUNC*`
- 默认折叠范围：0 到 1，也就是完整折叠/展开

实物使用：

- 机型：`4101_transform_quad_real`
- 位置：`ROMFS/px4fmu_common/init.d/airframes/`
- 输出参数：`PWM_MAIN_FUNC*`、`PWM_AUX_FUNC*`
- 实物接线：Holybro 6C mini，电机在 AUX1 到 AUX4，太阳轮舵机在 MAIN1
- 默认安全输入：`RC_MAP_AUX1=6` 为 VrB 折叠比例，`RC_MAP_AUX2=7` 为 SwA 使能开关
- 默认折叠范围限制：`transform_control limit 0.30`
- 实物构型：`transform_control geometry plus`，电机 1 前、电机 2 后、电机 3 左、电机 4 右；机型编号仍为 4101，分类为 `Quadrotor +`。
- 仿真 4099/4100 仍为原 X 型；实物十字型几何及飞控安装方向见 `src/modules/transform_control/README.md` 第 5 节。本文中的仿真日志与 X 型坐标不代表十字型实物飞行验证。

不要把 `SIM_GZ_*` 参数写入实物机型，也不要把 `PWM_*` 输出参数写入 Gazebo 机型。

## 控制入口

### QGC Actions

QGC 的 `Transform_Actions.json` 使用 `MAV_CMD_DO_SET_ACTUATOR`：

- `param7 = 99`：外部变形动作命令，由 `transform_control` 接管。
- `param1 != 0`：折叠。
- `param1 = 0`：展开。

`transform_control` 收到外部命令后，再发布内部 `MAV_CMD_DO_SET_ACTUATOR`：

- `param7 = 0`：Actuator Set 1。
- `param1 = progress`：0 表示展开，1 表示折叠。

这样可以避免 QGC 直接驱动舵机，同时让 PX4 模块同步更新舵机和 CA 几何。

### 终端命令

在 PX4 shell 中可用：

```sh
transform_control status
transform_control fold
transform_control open
transform_control manual
transform_control source
transform_control source button
transform_control source rc_aux
transform_control output
transform_control output progress
transform_control output pwm_open_high
transform_control limit
transform_control limit 0.30
transform_control mode normal
transform_control mode servo_only
transform_control mode geometry_only
```

默认模式是：

```sh
normal
```

默认输入源是：

```sh
button
```

也就是兼容原来的 `manual_control_setpoint.buttons` bit0。实物机型会在启动脚本中切换到：

```sh
source rc_aux
output pwm_open_high
limit 0.30
```

`rc_aux` 模式下：

- AUX2 大于 0.5 才允许折叠。
- AUX2 未使能、AUX1 无效、AUX2 无效或 RC 输入超时，目标折叠比例都会回到 0。
- AUX1 从 -1 到 +1 映射到折叠比例 0 到 1，再受 `limit` 限制。

`output pwm_open_high` 用于实物 PWM 舵机：

- 折叠比例 0 输出 `+1`，在 MAIN1 映射到 1200 us，对应展开。
- 折叠比例 1 输出 `-1`，在 MAIN1 映射到 750 us，对应最大折叠。

Gazebo 仿真仍使用默认 `output progress`，不需要切换。

## 诊断模式

`transform_control` 保留 3 种模式：

- `normal`
  - 舵机输出和 CA 几何同时更新。
  - 正常飞行验证使用该模式。
- `servo_only`
  - 只驱动 Gazebo 舵机，不更新 CA 几何。
  - 用于判断物理折叠本身是否稳定。
- `geometry_only`
  - 只更新 CA 几何，不驱动 Gazebo 舵机。
  - 用于隔离分配矩阵问题。

故障定位结果：

- 早期版本中 `servo_only` 稳定，但 `geometry_only` 会失稳。
- 根因是动态 CA 的 `PY` 坐标和折叠角方向与 PX4/Gazebo 坐标约定不一致。
- 修正后 `normal` 模式已稳定。

## Control Allocation 几何

4099 机型展开状态：

```text
CA_ROTOR0_PX  0.099   CA_ROTOR0_PY  0.099   CA_ROTOR0_KM -0.05
CA_ROTOR1_PX -0.099   CA_ROTOR1_PY -0.099   CA_ROTOR1_KM -0.05
CA_ROTOR2_PX  0.099   CA_ROTOR2_PY -0.099   CA_ROTOR2_KM  0.05
CA_ROTOR3_PX -0.099   CA_ROTOR3_PY  0.099   CA_ROTOR3_KM  0.05
```

4100 机型折叠参考状态：

```text
CA_ROTOR0_PX  0.017   CA_ROTOR0_PY  0.119   CA_ROTOR0_KM -0.05
CA_ROTOR1_PX -0.017   CA_ROTOR1_PY -0.119   CA_ROTOR1_KM -0.05
CA_ROTOR2_PX  0.119   CA_ROTOR2_PY -0.017   CA_ROTOR2_KM  0.05
CA_ROTOR3_PX -0.119   CA_ROTOR3_PY  0.017   CA_ROTOR3_KM  0.05
```

`transform_control` 在 12 秒内插值更新 `CA_ROTOR*_PX/PY`。参数通知频率约 100 ms，避免每个 50 Hz 控制循环都重建分配矩阵。

## Gazebo 舵机和电机

4099 机型中：

```text
SIM_GZ_SV_FUNC1 = 301
SIM_GZ_SV_FUNC2 = 301
SIM_GZ_SV_FUNC3 = 301
SIM_GZ_SV_FUNC4 = 301
```

四个 Gazebo 机臂关节同时使用 `Peripheral via Actuator Set 1`，模拟真实机构中一个中心舵机通过齿轮机构同步驱动四个机臂。

舵机输出关系：

- 输出约 500：展开。
- 输出约 1000：折叠。

## 验证结果

最新验证日志：`2026-07-16/10_54_53.ulg`

外部触发：

- 折叠 3 次。
- 展开 3 次。

每次动作统计：

- `parameter_update`：约 122 次。
- 参数更新持续时间：约 12 秒。
- 电机低端饱和：0%。
- 电机高端饱和：0%。
- `unallocated_torque`：0。
- `torque_setpoint_achieved`：100%。
- 姿态扰动：roll/pitch 通常小于 0.5 deg，最大 pitch 变化约 0.91 deg。
- 位置漂移：单次动作水平约 0.05 m 到 0.08 m，高度约 0.01 m 到 0.03 m。

该结果说明：

- QGC Action 触发链路有效。
- 舵机输出有效。
- 动态 CA 几何更新有效。
- 折叠/展开过程没有引入电机饱和或控制分配失效。

## 仿真启动和测试

推荐启动：

```sh
PX4_SYS_AUTOSTART=4099 PX4_GZ_MODEL=transform_quad make px4_sitl gz_transform_quad
```

启动后确认：

```sh
transform_control status
```

应看到 `mode: normal`。

测试流程：

1. QGC 起飞并悬停。
2. 点击 QGC Actions 的折叠动作。
3. 悬停 20 到 30 秒。
4. 点击 QGC Actions 的展开动作。
5. 再悬停 20 到 30 秒。
6. 重复多次。

## 实物验证注意事项

当前结果只证明 SITL/Gazebo 中方案可行，不能直接等同于实物安全。

实物固件需要确认：

- 使用 Holybro 6C mini 时编译目标为 `px4_fmu-v6c_default`，该目标已启用 `CONFIG_MODULES_TRANSFORM_CONTROL=y`。
- 选择机型 `4101_transform_quad_real`。
- 电机输出：`PWM_AUX_FUNC1..4 = 101..104`。
- 折叠舵机输出：`PWM_MAIN_FUNC1 = 301`，`PWM_MAIN_DIS1=1200`，`PWM_MAIN_MIN1=750`，`PWM_MAIN_MAX1=1200`。
- 遥控器 VrB 映射到 `RC_MAP_AUX1=6`，SwA 安全使能开关映射到 `RC_MAP_AUX2=7`。
- 首次实物测试保留 `transform_control limit 0.30`，不要直接完整折叠。

实物前建议按顺序验证：

1. 无桨，上电检查舵机方向和限位，确认 AUX2 未使能时 VrB 不会折叠。
2. 无桨，使能 AUX2 后缓慢旋转 VrB，确认舵机和 `CA_ROTOR*_PX/PY` 同步变化。
3. 有桨绑机，低油门检查折叠过程振动和电机响应。
4. 限高、低高度悬停，先测试展开状态。
5. 低高度、短时间、小范围测试空中折叠，折叠比例先不超过 0.30。
6. 逐步扩大高度、折叠比例和折叠/展开次数。

实物阶段应重新评估惯量、重心、舵机速度、机械间隙和电池/线缆对折叠过程的影响。
