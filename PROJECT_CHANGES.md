# 项目变更说明（截至 2026-10-19）

相对上次主仓库提交 `c2419c5ee4`、Gazebo 模型提交 `b1397eb`，
本次集中提交十字构型、双目自动通行、可视化、数据保存与稳定性修复成果。
完整 PX4 主项目现以同仓库的 `main` 为入口，`px4-transform-quad` 保留此前阶段版本，
`gz-models` 为 `Tools/simulation/gz` 子模块；父仓库记录模型版本。
`main` 保留早期快照的提交历史；完整获取、环境准备、编译、仿真与分析流程见 [README](README.md)。

## 新增功能与文件

| 内容 | 主要文件 | 实现结果 |
| --- | --- | --- |
| 十字构型与独立机型 | `TransformGeometry.hpp`、机型 4101/5101/5102/5103 | 电机 1 朝前；区分实物、折叠悬停、碰撞与双目训练配置 |
| Gazebo 模型与世界 | 子模块中的 `transform_quad_plus*`、`generate_transform_*.py`、`transform_*passage.sdf` | 可重建十字模型、桨盘碰撞模型、双目模型与两门场景 |
| 双目识别与折叠规划 | `Tools/transform_stereo_perception.py` | 从同步左右图像测量标记门宽、距离、朝向，并结合桨盘包络和余量选择折叠比例 |
| 自动通行 | `Tools/transform_stereo_passage.py` | MAVLink Offboard 起飞、接近、对中、折叠、确认到位、穿越、展开和降落 |
| 一键运行 | `Tools/transform_stereo_sitl_check.py` | 隔离仿真、飞行、窗口、诊断与本次数据归档 |
| Gazebo 与 RViz 画面 | `transform_stereo_gui.py`、`transform_stereo_rviz.py`、`transform_stereo_ros_bridge.py` 及配置文件 | 三维跟随视角和 RViz 左右 Image 实时预览 |
| 独立视觉进程 | `Tools/transform_stereo_vision_worker.py` | 图像解码、识别和 PNG 写盘不阻塞飞行控制循环 |
| 诊断与验收 | `transform_stereo_mechanism_check.py`、`transform_stereo_truth.py`、分析脚本与 `Tools/tests/` | 固定基座全行程、悬停折叠、独立真值及数据/桥接回归检查 |
| 文档与验证证据 | `PROJECT_STATUS.md`、`validation/`、模块 README 和航向补偿设计 | 保存阶段进展、配置快照、压缩飞行记录、CSV、图表及哈希清单；航向补偿仍为设计文档 |

## 修改与修复

- `transform_control.cpp` 抽取并扩展构型几何，支持十字布局、连续 `target 0..1`、
  外部比例命令、折叠上限、动作时间、RC AUX 输入及舵机输出模式。
- 注册新机型，在 V6C 默认构建中启用折叠模块，补充实物 RC 配置和使用说明。
  已完成固件编译记录，尚未确认新十字固件已刷入实物。
- 5103 四个关节的独立物理 PID 共用一个 `servo_0` 输入，并禁用多余舵机输出，
  修复本轮测试中机臂折叠不同步的问题。
- 修正视觉对中：使用门平面的机身横向误差，计入相机安装偏置；
  折叠后近距离复核，门前暂停时修正偏航，改善像素量化与竖向裁切处理。
- 降低图形运行负载：OpenCV 单线程、独立视觉进程、有界最新帧队列、
  低帧率预览及较小 Gazebo 窗口。图形模式默认半速，按仿真时间换算动作与等待。
  5103 的三个角速度控制增益系数设为 0.2；其他机型不随此项调整。
- 增加运行互斥锁、视觉/遥测新鲜度检查及资源记录。启动失败、中断、分析失败均保留已有数据。
  每次默认记录到 `logs/transform_stereo/`，生成 ULog、图片、事件、真值、CSV、轨迹图和 SHA-256 清单。
- 为归档 CSV 保留原始换行，保证从 GitHub 克隆后仍能通过归档清单的字节与哈希校验。

## 已有验证结果

2026-10-19 在 Gazebo 与 RViz 同时运行时完成一次完整回归：

- 自动通过 600/440 mm 两门，第二门折叠 88.5%，随后展开、降落。
- 两次独立真值穿越均在门洞内；接触通道收到 2095 条消息，门体接触为 0。
- 控制循环最大间隔约 52 ms，ULog 无丢帧。
- 本地原始运行保存 262 对双目图片，RViz 左右各发布 1465 帧；554 个运行文件校验通过。
- 16 项本地回归与 4 项系统 Python/ROS 桥接测试通过。

完整证据见 [图形稳定性回归](validation/stereo_20261019_stability/README.md)。
本次上传包含精选验证归档与压缩飞行日志；每次实验目录和全部采样图片留在本机忽略的 `logs/` 下。
当前结果覆盖理想双目和带标记的 600/440 mm 门；长期成功率、无标记通道、
原 360 mm 门以及实物通行尚待验证。

## 获取与运行

```sh
git clone --recurse-submodules --branch main \
  https://github.com/Cosmo-zh/transform_quad_simulation.git
cd transform_quad_simulation
make px4_sitl_default -j2
python3 Tools/transform_stereo_sitl_check.py --flight --rviz
```

依赖安装、显示参数、诊断命令和数据格式见
[可视化与按次保存数据](validation/transform_stereo_visualization.md)；
双目算法和自动通行流程见 [双目通行原型](validation/transform_stereo_passage.md)。
