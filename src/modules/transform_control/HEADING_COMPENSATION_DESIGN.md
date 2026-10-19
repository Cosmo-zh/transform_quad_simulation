# 可变形四旋翼构型航向补偿设计

> 本文是原 X 型几何的设计方案，尚未接入控制代码。实物机型 4101 已增加电机 1 朝前的十字型配置，几何计算位于 `TransformGeometry.hpp`。下文的绝对构型前向与横向宽度表针对原 X 型；实现补偿前需按当前构型重新计算基准与包络。

## 1. 问题描述

当前 `transform_control` 模块根据折叠进度完成两项工作：

1. 通过 `Actuator Set 1` 驱动机臂折叠舵机；
2. 动态更新 `CA_ROTOR0..3_PX/PY`，使 Control Allocation 使用当前电机力臂。

四条机臂在当前机械结构中沿同一旋转方向运动。折叠后，四个电机组成的旋翼构型不仅半径发生变化，还会整体相对飞控机体系旋转。飞控的 X 轴和机身物理机头保持不变，因此固定机体航向时，旋翼构型的窄边不会保持在原来的穿越方向，沿洞口横向的投影宽度可能先增大。

本设计增加“构型航向补偿”：折叠时根据电机位置计算旋翼构型相对飞控的偏转角，并让无人机本体执行大小相等、方向相反的偏航；展开时再平滑恢复。这样可以使由电机 1/3 与电机 2/4 两组中点定义的构型前向在世界坐标中保持不变。

## 2. 坐标定义

以下编号采用 PX4 数组下标与实物电机编号的对应关系：

| 实物编号 | PX4 下标 | 参数 |
| --- | ---: | --- |
| 电机 1 | 0 | `CA_ROTOR0_PX/PY` |
| 电机 2 | 1 | `CA_ROTOR1_PX/PY` |
| 电机 3 | 2 | `CA_ROTOR2_PX/PY` |
| 电机 4 | 3 | `CA_ROTOR3_PX/PY` |

设当前四个电机在 PX4 机体系中的位置为：

```text
P1 = (x0, y0)
P2 = (x1, y1)
P3 = (x2, y2)
P4 = (x3, y3)
```

电机 1/3 的中点和电机 2/4 的中点分别为：

```text
P13 = (P1 + P3) / 2
P24 = (P2 + P4) / 2
```

用两个中点之间的向量定义构型前向：

```text
F(progress) = P13 - P24
```

构型相对飞控 X 轴的航向为：

```text
heading(progress) = atan2(F.y, F.x)
```

为避免模型初始安装角进入补偿量，以展开状态作为零点：

```text
geometry_yaw(progress) = wrap_pi(heading(progress) - heading(0))
```

当前几何中 `heading(0) = 0`，但实现时仍应计算展开基准，不应依赖该特例。

## 3. 当前模型的计算结果

根据 `transform_control.cpp` 中现有的展开坐标、铰点坐标和 60 度最大折叠角，构型偏转角如下：

| 折叠进度 | 构型相对飞控旋转 | 机体补偿偏航 |
| ---: | ---: | ---: |
| 0.0 | +0.00 deg | -0.00 deg |
| 0.1 | +3.62 deg | -3.62 deg |
| 0.2 | +7.24 deg | -7.24 deg |
| 0.3 | +10.87 deg | -10.87 deg |
| 0.4 | +14.51 deg | -14.51 deg |
| 0.5 | +18.17 deg | -18.17 deg |
| 0.6 | +21.85 deg | -21.85 deg |
| 0.7 | +25.56 deg | -25.56 deg |
| 0.8 | +29.29 deg | -29.29 deg |
| 0.9 | +33.06 deg | -33.06 deg |
| 1.0 | +36.87 deg | -36.87 deg |

仅计算电机中心的轴向跨度：

| 构型 | 不补偿航向 | 补偿航向后 |
| --- | ---: | ---: |
| 展开 | 0.198 m | 0.198 m |
| 折叠 30% | 约 0.228 m | 约 0.195 m |
| 完全折叠 | 约 0.238 m | 约 0.170 m |

实际包络宽度还需要加上螺旋桨直径，并检查机臂和机身网格的投影。若螺旋桨近似为水平圆盘，则：

```text
总宽度 = 电机中心投影跨度 + 螺旋桨直径
```

当前实物限制为 `limit 0.30`。30% 折叠经过航向补偿后，电机中心跨度只比展开状态减少约 3 mm，因此适合验证控制逻辑，但实际穿越能力提升有限。扩大折叠范围前仍需逐级完成仿真和实物验证。

## 4. 正确的控制含义

飞控板固定安装在中心机架上，折叠过程中没有相对机架转动，所以 PX4 机体系仍应由飞控安装方向定义。构型航向只是一个由电机位置计算得到的附加几何量。

以下做法不适合本问题：

- 动态修改 `SENS_BOARD_ROT`；
- 修改 IMU、磁罗盘或 EKF 的航向测量值；
- 只旋转 `CA_ROTOR*_PX/PY`，但不对应真实电机位置；
- 由 `transform_control` 直接持续发布 `vehicle_attitude_setpoint`，与现有飞行任务争用姿态设定值。

正确做法是：

```text
实际电机位置
     │
     ├──> 更新 CA_ROTOR*_PX/PY
     │
     └──> 计算 geometry_yaw
                    │
                    ▼
            修正 yaw setpoint
                    │
                    ▼
          无人机本体实际反向偏航
```

EKF 继续估计无人机真实物理姿态。飞行控制层通过正常的偏航闭环驱动电机，使机身产生真实偏航运动。

## 5. 推荐的软件架构

推荐增加一个只描述变形状态的 uORB 主题，由 `transform_control` 发布，飞行任务订阅。这样变形模块负责几何，飞行模式负责航向设定值，模块职责清晰。

```text
transform_control
  ├── current_progress
  ├── target_progress
  ├── rotor positions
  └── geometry_yaw
           │
           ▼
    transform_status (uORB)
           │
           ▼
FlightTaskManualAltitude
  ├── 原有遥控器 yaw setpoint
  ├── 构型航向增量补偿
  └── 最终 trajectory_setpoint.yaw
           │
           ▼
mc_pos_control -> mc_att_control -> Control Allocation
```

`FlightTaskManualPosition` 继承 `FlightTaskManualAltitude`，并调用其航向更新逻辑。因此在 `FlightTaskManualAltitude` 中实现一次，即可覆盖当前实物测试使用的 Altitude 模式，也可以覆盖具备 GPS/光流定位后的 Position 模式。

## 6. 需要新增和修改的文件

### 6.1 新增 `msg/TransformStatus.msg`

建议消息定义：

```text
uint64 timestamp

uint8 STATE_OPEN = 0
uint8 STATE_FOLDING = 1
uint8 STATE_FOLDED = 2
uint8 STATE_UNFOLDING = 3
uint8 STATE_PARTIAL = 4

float32 progress
float32 target_progress
float32 geometry_yaw
float32 geometry_yaw_rate

uint8 state
bool heading_compensation_enabled
```

字段含义：

- `progress`：当前实际指令进度；
- `target_progress`：目标折叠进度；
- `geometry_yaw`：构型相对展开状态的航向偏转，单位 rad；
- `geometry_yaw_rate`：构型偏转角速度，单位 rad/s；
- `state`：当前折叠状态；
- `heading_compensation_enabled`：飞行任务是否应应用航向补偿。

同时将 `TransformStatus.msg` 加入 `msg/CMakeLists.txt` 的 `msg_files` 列表。

### 6.2 修改 `src/modules/transform_control/transform_control.cpp`

增加：

```cpp
#include <matrix/matrix/math.hpp>
#include <uORB/topics/transform_status.h>
```

增加一个默认关闭的控制开关：

```cpp
static px4::atomic<bool> heading_compensation_enabled{false};
```

增加构型航向计算函数。该函数必须使用与 Control Allocation 相同的 `rotor_position()` 结果：

```cpp
static float geometry_yaw_from_progress(float progress)
{
    float x[4]{};
    float y[4]{};

    for (int i = 0; i < 4; ++i) {
        rotor_position(i, progress, x[i], y[i]);
    }

    const float p13_x = 0.5f * (x[0] + x[2]);
    const float p13_y = 0.5f * (y[0] + y[2]);
    const float p24_x = 0.5f * (x[1] + x[3]);
    const float p24_y = 0.5f * (y[1] + y[3]);

    const float heading = atan2f(p13_y - p24_y, p13_x - p24_x);

    // open_heading 可在初始化时按 progress=0 计算并保存。
    return matrix::wrap_pi(heading - open_heading);
}
```

在变形线程中增加：

```cpp
uORB::Publication<transform_status_s> transform_status_pub{ORB_ID(transform_status)};
```

每个 50 Hz 控制周期，在更新 `current_progress` 后计算并发布状态：

```cpp
const float geometry_yaw = geometry_yaw_from_progress(current_progress);
const float geometry_yaw_rate = matrix::wrap_pi(geometry_yaw - previous_geometry_yaw) / dt;

transform_status_s status{};
status.timestamp = hrt_absolute_time();
status.progress = current_progress;
status.target_progress = target_progress;
status.geometry_yaw = geometry_yaw;
status.geometry_yaw_rate = geometry_yaw_rate;
status.state = static_cast<uint8_t>(current_state);
status.heading_compensation_enabled = heading_compensation_enabled.load();
transform_status_pub.publish(status);
```

增加命令接口：

```sh
transform_control heading
transform_control heading on
transform_control heading off
```

建议默认值为 `off`。这样升级固件后，原有仿真和实物行为不会自动改变，必须明确启用后才执行航向补偿。

`transform_control status` 也应增加：

```text
heading compensation: on/off
```

### 6.3 修改 `FlightTaskManualAltitude.hpp`

增加订阅和内部状态：

```cpp
#include <uORB/topics/transform_status.h>

uORB::Subscription _transform_status_sub{ORB_ID(transform_status)};
transform_status_s _transform_status{};

bool _transform_heading_initialized{false};
float _last_geometry_yaw{0.f};
```

增加私有或受保护函数：

```cpp
void _applyTransformHeadingCompensation();
void _resetTransformHeadingCompensation();
```

### 6.4 修改 `FlightTaskManualAltitude.cpp`

在 `activate()` 中清空补偿历史：

```cpp
_transform_heading_initialized = false;
_last_geometry_yaw = 0.f;
```

在 `_updateYawSetpoint()` 中，先运行 PX4 原有摇杆航向逻辑，再应用构型补偿：

```cpp
void FlightTaskManualAltitude::_updateYawSetpoint()
{
    _stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint,
                                   _sticks.getYawExpo(), _yaw, _deltatime,
                                   _unaided_yaw);

    _applyTransformHeadingCompensation();
}
```

补偿逻辑建议使用“构型角增量”，而不是每周期覆盖绝对航向：

```cpp
void FlightTaskManualAltitude::_applyTransformHeadingCompensation()
{
    _transform_status_sub.update(&_transform_status);

    const bool fresh = _transform_status.timestamp != 0
                       && hrt_elapsed_time(&_transform_status.timestamp) < 200_ms;

    if (!fresh || !_transform_status.heading_compensation_enabled
        || !PX4_ISFINITE(_transform_status.geometry_yaw)) {
        _transform_heading_initialized = false;
        return;
    }

    if (!_transform_heading_initialized) {
        _last_geometry_yaw = _transform_status.geometry_yaw;
        _transform_heading_initialized = true;
        return;
    }

    const float delta_geometry_yaw = matrix::wrap_pi(
        _transform_status.geometry_yaw - _last_geometry_yaw);

    _last_geometry_yaw = _transform_status.geometry_yaw;

    if (PX4_ISFINITE(_yaw_setpoint)) {
        _yaw_setpoint = matrix::wrap_pi(_yaw_setpoint - delta_geometry_yaw);
        _yawspeed_setpoint -= _transform_status.geometry_yaw_rate;
    }
}
```

该算法的行为是：

- 遥控器偏航杆回中、PX4 正在锁定航向时，折叠角每增加一点，航向设定值就反向移动同样的角度；
- 展开时 `delta_geometry_yaw` 为负，航向设定值自动回到原方向；
- 飞手主动推动偏航杆时，PX4 将 `_yaw_setpoint` 置为 `NAN`，补偿暂时让位给飞手；
- 飞手松开偏航杆后，以当前真实航向重新锁定，不追赶飞手接管期间遗漏的补偿角；
- 主题超时、模块停止或补偿关闭时，只清空内部基准，不产生航向跳变；
- 切换 Altitude/Position 模式时，新飞行任务以当前构型角重新初始化，不产生重复补偿。

实现时应限制补偿角速度，例如限制在 `20 deg/s` 以内。当前完整变形时间最短为 3 秒，理论最大构型补偿速度约为 `12.3 deg/s`，在该限制以内。

### 6.5 修改日志主题

在 `src/modules/logger/logged_topics.cpp` 的默认主题中加入：

```cpp
add_topic("transform_status", 20);
```

这样 ULog 可以同时分析：

- 折叠进度；
- 构型偏转角；
- 航向设定值与真实航向；
- 动态 Control Allocation 参数；
- 四个电机输出。

### 6.6 修改仿真机型启动脚本

文件：

```text
ROMFS/px4fmu_common/init.d-posix/airframes/4099_gz_transform_quad
```

第一轮实现后先保持默认关闭，通过 PX4 Shell 手动启用：

```sh
transform_control heading on
```

确认偏航方向、宽度和飞行稳定性正确后，再考虑将该命令写入仿真机型脚本。

### 6.7 实物机型暂不默认启用

文件：

```text
ROMFS/px4fmu_common/init.d/airframes/4101_transform_quad_real
```

在完成 Gazebo 验证前，不在该脚本中增加 `transform_control heading on`。实物第一次验证也应手动启用，并继续保留 `limit 0.30`。

## 7. 为什么采用增量补偿

也可以在折叠开始时记录航向 `yaw_entry`，然后持续设置：

```text
yaw_setpoint = yaw_entry - geometry_yaw
```

这种绝对方式适合无人值守穿洞任务，但会与飞手的偏航输入发生所有权冲突。手动 Altitude/Position 模式采用增量补偿有以下优点：

- 保留 PX4 原有 `StickYaw` 的航向锁定与 EKF reset 处理；
- 飞手可以随时用偏航杆接管；
- 不需要 `transform_control` 直接控制姿态；
- 模式切换和消息短时中断不会突然恢复旧航向。

Transform Challenge 的全自动任务可以在后续使用绝对方式，由专用 FlightTask 或 Offboard 轨迹控制统一管理位置、航向与折叠时序。

## 8. Transform Challenge 任务逻辑

自动穿洞任务建议按以下状态执行：

```text
TAKEOFF
   │ 起飞到安全高度
   ▼
APPROACH_GATE
   │ 飞到洞口前等待点并对准
   ▼
FOLD_AND_ALIGN
   │ 折叠机臂
   │ yaw = entry_yaw - geometry_yaw
   ▼
CROSS_GATE
   │ 保持折叠与补偿航向
   ▼
UNFOLD_AND_REALIGN
   │ 展开机臂
   │ yaw 平滑恢复 entry_yaw
   ▼
EXIT / LAND
```

进入 `CROSS_GATE` 前应同时满足：

```text
abs(progress - target_progress) < progress_tolerance
abs(yaw_error) < yaw_tolerance
横滚和俯仰角速度低于阈值
```

这样无人机不会在机臂仍在运动或航向尚未对准时进入洞口。

## 9. Gazebo 验证步骤

### 阶段 A：只验证几何角

1. 启动 `transform_quad`，保持落地并锁定；
2. 执行 `transform_control heading off`；
3. 分别设置折叠进度 0.1、0.2、0.3 和 1.0；
4. 用 `listener transform_status` 检查 `geometry_yaw`；
5. 对照本文件第 3 节的角度表。

### 阶段 B：悬停航向补偿

1. 在开阔默认世界起飞并稳定悬停；
2. 启用 `transform_control heading on`；
3. 缓慢折叠到 0.1；
4. 检查机体是否反向偏航约 3.6 度；
5. 依次验证 0.2、0.3；
6. 展开并确认航向恢复；
7. 若偏航方向相反，只修正补偿公式的符号，不修改 EKF 或传感器方向。

### 阶段 C：包络验证

1. 在 Gazebo 顶视图中固定洞口方向；
2. 记录展开状态横向最大包络；
3. 比较相同折叠进度下补偿关闭和开启时的横向包络；
4. 包络计算必须包含旋转中的螺旋桨圆盘；
5. 根据测量结果调整 Transform Challenge 洞口尺寸。

### 阶段 D：穿洞验证

1. 起飞并飞到洞口前等待点；
2. 停稳后开始折叠和航向补偿；
3. 达到折叠进度及航向容差后低速穿越；
4. 完全离开洞口后展开并恢复航向；
5. 检查 ULog 中是否存在电机饱和、偏航误差突增或位置明显偏移。

## 10. 实物验证顺序

实物测试必须在仿真确认补偿符号正确后进行，建议顺序如下：

1. 拆除螺旋桨，在锁定状态检查 `transform_status.geometry_yaw`；
2. 保持 `heading off`，验证舵机和动态 CA 与当前版本行为一致；
3. 安装螺旋桨后，仅测试 `limit 0.10`；
4. 在安全高度悬停，手动执行 `heading on`；
5. 折叠到 0.10，确认机体反向偏航约 3.6 度；
6. 展开并确认航向回到进入折叠前的位置；
7. 检查日志后再逐级增加到 0.20 和 0.30；
8. 在 Altitude 模式下需要飞手持续修正水平漂移，航向补偿不提供水平定点能力。

重点检查以下日志量：

```text
transform_status.progress
transform_status.geometry_yaw
trajectory_setpoint.yaw
vehicle_attitude_setpoint
vehicle_attitude
vehicle_angular_velocity
actuator_motors
control_allocator_status
```

## 11. 完成判据

该功能通过验证应至少满足：

- 构型偏转角与电机坐标计算结果一致；
- 折叠时机体偏航方向与构型旋转方向相反；
- 展开后航向设定值连续恢复，不发生跳变；
- 飞手推动偏航杆时可以立即接管；
- `transform_status` 超时或模块停止时不会产生额外偏航命令；
- 航向补偿开启后的洞口横向包络小于未补偿状态；
- 折叠和恢复过程中无持续电机饱和、发散振荡或位置突变；
- 补偿关闭时，原有 `transform_control` 行为保持不变。

## 12. 实施阶段建议

为便于定位问题，每次只完成并验证一个阶段：

1. 新增 `TransformStatus` uORB 消息并发布构型角，不控制航向；
2. 在地面仿真中核对 0.0 到 1.0 的构型角；
3. 在 `FlightTaskManualAltitude` 中加入默认关闭的增量航向补偿；
4. 在开阔 Gazebo 世界验证悬停折叠与航向恢复；
5. 测量补偿前后的实际三维包络并调整洞口；
6. 实现 Transform Challenge 自动任务状态机；
7. 最后以 0.10、0.20、0.30 的顺序进行实物验证。
