# 双目识别与自动折叠通行原型

新增独立 SITL 机型 **5103**，模型 `transform_quad_plus_stereo`，世界
`transform_stereo_passage`。5102 的原通行世界和实物默认配置未改变。

## 实现范围

- 固定在机身上的理想平行双目：120 mm 基线，960×600，15 Hz，水平视场 1.2 rad。
  相机中点在机身 FLU `(0.08, 0, 0.08)` m；支架可视化不增加质量/惯量。
- 首版识别已有的青绿色竖直门边线，在左右图像中匹配内边界，以视差三角化得到
  净宽、中心、距离和粗略朝向。不读取门宽配置、Gazebo 深度相机或世界真值来控制。
  这是带标记矩形门、近水平正面接近的原型，不能泛化为任意无标记通道识别。
- 左右图像必须具有相同仿真时间戳；缺帧、裁切、负视差、异常宽度和大角度拒绝。
  长距离一像素内不可分辨的视差采用共同深度，门前再次测量修正。
  折叠后复核与穿越阶段允许竖向裁切的边线继续提供水平视差，不把裁切边线中点当作门洞高度；
  横向裁切仍拒绝。默认接近速度 0.1 m/s，穿越速度 0.2 m/s，以缩短近门盲行段。
- 从生成模型的关节几何计算包含桨盘的包络，检查 ±5° 航向偏差、每侧默认 40 mm
  余量和测量不确定度，选择最小可行折叠比例；360 mm 门在此设置下会被拒绝。
- 训练门位于世界 X=3/7 m，宽 600/440 mm。第二道训练门比展开包络加余量窄，
  会要求折叠；原 5102 的 360 mm 门不被替换。
- 状态机：起飞 → 识别 → 低速接近 → 门前重新测量/对中 → 折叠 → 关节到位并停稳
  → 折叠后重新视觉对中 → 低速穿越 → 完全离门后展开 → 下一道门 → 降落。
  默认通过两门后在门后展开并就地降落。
- 折叠由 PX4 模块执行，Gazebo 关节角反馈用于确认到位。5103 的关节位置控制 P
  从原模型的 5 提高到 15，以减小旋翼反作用力矩造成的稳态偏角；只作用于新模型。
- 门边线离开视野后的短距离通行使用穿越前冻结的视觉门位姿和 PX4 本地里程计，
  不是全程纯视觉导航。门仍可见时继续修正横向和高度，门身份通过宽度和距离约束。
  视觉/遥测丢失、余量不足、姿态超限、关节跟踪异常或接触会中止。
  穿越误差上限由测得门宽、当前包络和测量不确定度计算，最低规划余量仍为每侧 40 mm。
  若在仍可见且距门足够远的位置出现横向偏差，先暂停前进、视觉居中后再继续；
  接近门框后仍超过可用余量时中止，不在门框内放宽误差限制。
- 接近时以同一静止门的双目相对位移与 PX4 本地位置变化做二维最小二乘旋转对齐，
  减小估计航向与本地平移坐标不一致带来的横向路径误差。需要足够平移和良好拟合，
  建立对齐前不允许开始穿越。这与构型变化引起的航向补偿是不同功能。
- 独立真值记录进程只用于事后验收，不向伴随控制器传递任何数据。

## 文件

| 文件 | 职责 |
| --- | --- |
| `Tools/simulation/gz/tools/generate_transform_stereo.py` | 生成模型和训练世界 |
| `Tools/transform_stereo_perception.py` | 时间同步、标记识别、双目测量和包络规划 |
| `Tools/transform_stereo_passage.py` | MAVLink Offboard 状态机，默认仅感知 |
| `Tools/transform_stereo_sitl_check.py` | 隔离参数目录/Transport partition 的验证入口 |
| `Tools/transform_stereo_truth.py` | 独立记录 Gazebo 真值 |
| `Tools/analyze_transform_stereo.py` | 汇总飞行结果、真值轨迹和实际机臂进度图 |
| `Tools/tests/test_transform_stereo.py` | 独立投影夹具、拒绝路径、余量和标定检查 |

## 运行

2026-10-18 起，支持 `--flight --gui` 可视化，默认按次保存到 `logs/transform_stereo/`，
并自动生成 CSV 和轨迹图。详见 [可视化与数据保存](transform_stereo_visualization.md)。
下文历史验证的 `build/` 路径为当时的原始证据目录。

环境使用现有 Gazebo Harmonic、`gz.transport13`、`gz.msgs10`、NumPy、pymavlink、
pexpect。OpenCV 可隔离安装，不修改系统 Python：

```sh
python3 -m pip install --target logs/transform_stereo_deps opencv-python-headless==4.11.0.86 --no-deps
export PYTHONPATH=logs/transform_stereo_deps:${PYTHONPATH}
python3 Tools/simulation/gz/tools/generate_transform_stereo.py
CCACHE_DISABLE=1 make px4_sitl_default -j4
python3 -m unittest discover -s Tools/tests -p test_transform_stereo.py -v

# 静态双目测宽，不启动 PX4、不解锁、不飞行
python3 Tools/transform_stereo_sitl_check.py
python3 Tools/transform_stereo_sitl_check.py --fixture-x 4.5

# 正向验证接触监测：未供电模型落到门体上，不启动 PX4
python3 Tools/transform_stereo_sitl_check.py --contact-probe

# 新建隔离的 5103 SITL，自动解锁、起飞、通行、降落
python3 Tools/transform_stereo_sitl_check.py --flight
```

当前默认验证输出目录为 `logs/transform_stereo/transform_stereo_时间/`，包含控制台、图像、测量/遥测事件、
`result.json`，飞行模式另有 ULog 和独立 `ground_truth.jsonl`。
飞行验收要求接触通道确实收到过消息，防止把未工作的接触订阅当成“零碰撞”。
`--flight` 需要可用的本机 UDP 14656/14640 端口；不要并行运行多个飞行验证。
静态测试与飞行验收分别记录，静态测宽成功不代表自动飞行成功。
两组静态图像和结果另存于 `validation/stereo_20261017/`，避免随 `build/` 清理丢失。

GUI 手动观察（先关闭占用同一实例的仿真）：

```sh
PX4_SYS_AUTOSTART=5103 make px4_sitl gz_transform_quad_plus_stereo_transform_stereo_passage
```

此命令只启动模型，不自动运行伴随控制器。在此 PX4 控制台额外启动专用 MAVLink：

```sh
param set COM_RC_IN_MODE 4
param set NAV_RCL_ACT 0
param set COM_DL_LOSS_T 300
mavlink start -x -u 14656 -o 14640 -r 2000000 -m onboard -t 127.0.0.1
```

单独运行 `python3 Tools/transform_stereo_passage.py` 仅感知；`--execute` 才允许飞行，
并检查 `SYS_AUTOSTART=5103`。自动验证入口会配置无 RC 输入并等待估计器初始化；
GUI 手动启动时需自行确认对应仿真设置。若使用自定义 `GZ_PARTITION`，感知进程需相同设置。

生成某一飞行目录的轨迹图与汇总：

```sh
python3 Tools/analyze_transform_stereo.py build/transform_stereo_具体时间
```

## 折叠接口

```sh
transform_control target 0.85
```

新命令接受有限的 `[0,1]` 比例，仍受 `limit` 和 `transition` 约束。
MAVLink `MAV_CMD_DO_SET_ACTUATOR` 的 `param7=100`、`param1=比例` 是新增比例协议；
原 `param7=99` 的两档协议保留。普通执行器输出仍使用 `param7=0`。
命令确认不能代替关节到位反馈，本原型使用真实仿真关节角确认。

## 验证记录

2026-10-17：SITL 编译通过，模型及世界 `gz sdf -k` 通过，6 项自动测试通过。

| 测试 | 真值 | 双目中位值 | 误差 | 证据目录 |
| --- | --- | --- | --- | --- |
| 第一门静态、12 帧 | 600 mm | 600.0 mm | 0.0 mm | `build/transform_stereo_20261017_104132_184500/` |
| 第二门静态、12 帧 | 440 mm | 446.7 mm | +6.7 mm | `build/transform_stereo_20261017_104253_059324/` |

第一轮飞行联调因位置目标跳变触发姿态保护，已自动降落，没有通行成功记录。
后续状态及最终飞行证据以本节追加记录为准。

接触监测正向测试通过：`build/transform_stereo_20261017_112111_166639/` 收到
7 条门体接触消息，结果保存在 `validation/stereo_20261017/contact_probe.json`。
`gz-sim8` 的 Contact 系统从 `<contact><topic>` 读取 topic，不能沿用相机的
`<sensor><topic>` 配置。早期 `111602` 运行虽然完成了两门轨迹、展开和降落，
但接触消息计数为 0，最终验收判为失败，不计入无接触通行验证。

### 完整飞行通过：2026-10-17

运行：`build/transform_stereo_20261017_115627_528215/`，自动验收 `success=true`。

| 项目 | 结果 |
| --- | --- |
| 第一门：真值 600 mm | 双目 596.25 mm，保持展开通过 |
| 第二门：真值 440 mm | 双目 440.625 mm，自动折叠 87% 后通过 |
| 门体接触 | 0；接触通道共收到 2254 条消息，已通过正向测试 |
| 真值门平面附近机身中心 Y | 第一门 -16.10 mm；第二门 -16.16 mm |
| 真值门平面附近机身中心 Z | 第一门 1.566 m；第二门 1.558 m |
| 离门后的动作 | 展开、降落、解除解锁 |
| ULog 丢帧 | 0 |
| 动力工作窗口最大绝对横滚/俯仰 | 0.492° / 1.047° |
| 动力工作窗口最大电机控制 | 0.63594（归一化） |
| 力矩/推力目标达成率 | 100% / 100% |

动力工作窗口取四个电机控制值都大于 0.05 的首末时刻之间；不是静态悬停窗口。
姿态极值来自完整速率 ULog，轨迹图中的实际机臂角来自伴随遥测记录。
真值中心在门洞内与接触监测共同用于验收，单看中心轨迹不能代替桨盘碰撞检查。

证据已保留到 [flight/](stereo_20261017/flight/)，包含
[最终结果](stereo_20261017/flight/result.json)、
[汇总](stereo_20261017/flight/summary.json)、
[ULog 指标](stereo_20261017/flight/ulog_metrics.json)、
[轨迹与实际机臂进度图](stereo_20261017/flight/trajectory.png)。
`flight.ulg.gz`、`events.jsonl.gz`、`ground_truth.jsonl.gz`、控制台和伴随输出保存原始证据，
`manifest.json` 保存原始路径、字节数和 SHA-256。解压后可重新分析。

这是当前带标记训练场景的一次完整通过。尚未进行成功率统计、无标记门识别、
传感器噪声/标定误差系统试验、原 360 mm 门或实物验证；原 5102 场景仍保持不变。

实现参考：[Gazebo Transport 13 Python 接口](https://gazebosim.org/api/transport/13/python.html)、
[Gazebo Harmonic 传感器](https://gazebosim.org/docs/harmonic/sensors/)。
