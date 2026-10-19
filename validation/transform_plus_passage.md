# 十字构型折叠通行场景

世界：`Tools/simulation/gz/worlds/transform_plus_passage.sdf`

专用模型：`transform_quad_plus_collision`；专用仿真机型：`5102`。
原世界、4099、5101 和实物 4101 的默认配置保持原样。

## 启动

先退出其他正在使用的 PX4/Gazebo 仿真，再从仓库根目录执行：

```sh
PX4_SYS_AUTOSTART=5102 make px4_sitl gz_transform_quad_plus_collision_transform_plus_passage
```

模型从 `(0,0,0.35)` 生成，机身前向与通道世界 +X 对齐，电机 1 在前。
专用模型消除了原模型 `base_link` 的额外 90 度偏航；四条机臂相对
机身的十字型几何保持不变。世界采用 ENU；世界 +X 对应向东，
PX4 本地 NED 坐标中为 +Y，不能直接把世界 X 当作 PX4 北向 X。

## 路线

全部通行点沿世界 `Y=0`，门洞中心 `Z=1.60 m`。

| 地点 | 世界 X | 用途 |
| --- | --- | --- |
| 绿色起飞区 | 0 m | 起飞并对准路线 |
| 练习门 | 3 m | 净宽 440 mm，展开构型对准练习 |
| 黄色等待区 | 5 m | 悬停、折叠，确认机械动作完成 |
| 折叠验证门 | 7 m | 净宽 360 mm，展开构型无法居中平穿 |
| 蓝色等待区 | 9 m | 完全离开门后展开 |
| 绿色降落区 | 12 m | 降落 |

两扇门厚 250 mm，净高 700 mm，下沿 1.25 m、上沿 1.95 m。
青绿色边线贴近实际碰撞开口，不用较大的装饰框冒充净开口。
地面有每米刻度，路线侧面有展开/完整折叠的等比例桨盘投影示意。
所有地面颜色和刻度只是提示，不自动触发飞行或折叠命令。

## 为什么不是 30% 折叠就能通过

从原 STL 顶点测得最大桨盘半径约 63.54 mm，测试模型采用
63.55 mm 半径、6 mm 厚的圆柱碰撞体近似旋转桨盘。
按 Gazebo 模型关节位置计算的水平总宽：

| 折叠进度 | 包含桨盘的宽度 | 360 mm 门单侧理论余量 |
| --- | --- | --- |
| 0 | 407.10 mm | -23.55 mm |
| 0.30 | 398.83 mm | -19.42 mm |
| 0.80 | 350.97 mm | 4.51 mm |
| 1.00 | 322.24 mm | 18.88 mm |

30% 行程只缩窄约 8 mm，无法同时满足“展开不能过”和合理控制余量。
因此 360 mm 门用于完整折叠的后续仿真。尺寸计算假设机头沿通道、
姿态水平、机构达到目标，不代表闭环穿越已经成功。

5102 仍默认 `limit 0.30`，避免启动即扩大折叠范围。只在该仿真中，
先在门前空旷位置验证完整折叠悬停，再做穿越：

```sh
transform_control transition 12
transform_control limit 1.0
transform_control fold
```

完整折叠需约 12 秒，不能在运动中假定机臂已到位。`status` 不显示
真实舵机角度；应观察模型和状态日志，并确认姿态与位置稳定后再前进。
到达世界 X=9 m 等待区后：

```sh
transform_control open
```

这里的全行程设置不应直接照搬到实物。

## 碰撞验证的边界

专用模型给四个旋翼补充了桨盘碰撞圆柱，门柱与上下横梁都有碰撞体。
其作用是避免原模型因没有桨盘碰撞而出现穿墙假象，不模拟桨叶破损、
气动近壁效应或碰撞后的真实飞行。一次无碰撞穿过还需结合位置、姿态、
关节角和接触状态确认；本场景没有自动判定成功的任务控制器。

## 生成与检查

```sh
python3 Tools/simulation/gz/tools/generate_transform_plus.py
python3 Tools/simulation/gz/tools/generate_transform_passage.py
make px4_sitl_default
python3 Tools/transform_plus_sitl_check.py --passage-smoke
```

SDF 校验与门洞/包络几何计算已通过。加载检查不解锁、不起飞；此前
5101 的 30% 悬停日志不能替代这个带碰撞模型的完整折叠穿越验证。

2026-10-11 加载检查通过，Gazebo 返回全部场景模型及
`transform_quad_plus_collision_0`，生成后落地位置约 `(0,0,0.025)`，
模型姿态为零。控制台证据：
`build/transform_plus_check_20261011_132944/console.txt`。
