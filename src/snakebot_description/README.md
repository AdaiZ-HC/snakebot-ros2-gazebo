# 随机森林中的金属仿生蛇

ROS 2 Lyrical / Gazebo Sim 10。九个可见蛇身节段、银灰/亮金装甲、黑色反光头部，
加上可控俯仰与红色分叉滑动舌头。my_test_pkg 不参与本包修改。

## 场景

- 原世界整体替换，移除了旧的环形树列、原 P1/P2/P3 和 plane 地面。
- 50×50×0.1 m 实心 Box 泥土地面，顶面 z=0.025 m，场景 grid=false。
  顶面同时覆盖 z=0 附近可能残留的 GUI 网格；出生高度自动与地面高度对齐。
- 32 棵随机树，坐标在 [-14.5,14.5] m 内，树中心离原点至少 3.6 m，彼此至少相隔 2 m。
  树干半径 0.15–0.3 m、高 2–4 m，有简化圆柱碰撞。
- 每棵有 3–5 个尺寸不同、位置错开的球体树冠，颜色为深绿、黄绿或橙红。
- 32 丛小球体灌木，304 片 0.1×0.1×0.005 m 落叶；树根处叶片相互覆盖、分层形成叶堆。
- 所有场景模型静态，树冠、灌木、落叶没有碰撞。晨昏低角度太阳产生长阴影，环境光避免全黑。
- tools/generate_forest.py 同时生成世界与 config/forest_map.yaml，避免地图坐标和真实树干不一致。
  默认种子 73，可以重现，也可以换种子重新生成后构建。

## 机器人与材质

- 8 节身体及 1 节头部，位置控制的 joint_1…joint_8 绕 Z 轴扭动。
- head_pitch_joint 改为可控 revolute，轴为负 Y，正值抬头；范围 -0.22…0.85 rad。
- tongue_joint 为沿头部 +X 的 prismatic，范围 -0.05…0.12 m；tongue_link 无碰撞。
  红色主舌杆加两个分叉圆柱，q=0 时收在头部前端内部。
- ros2_control 和同一个 joint_trajectory_controller 控制全部 10 个关节。
- 银灰/亮金身体 PBR metalness=0.92、roughness=0.22；黑色头部 metalness=0.85、roughness=0.14。
  舌头为非金属红色。URDF visual 自动命名，避免本机转换器漏掉材质覆盖。
- 头颈与舌头重新分配质量和惯量，总质量约 8.003 kg；低摩擦仍为 mu1=mu2=0.05。
- 侧边轮毂纯视觉。真实蛇形机器人常在节段底部布置被动轮以降低摩擦；
  本仿真采用平面运动系统简化整体位移，未建模被动轮。

## 统一行为控制

launch 只启动 snake_behavior.py，不再启动旧 snake_wave.py / snake_roam.py。
单节点拥有全部关节轨迹与自动 /cmd_vel，避免多个控制节点相互覆盖。

身体正常扭动为 0.7 Hz，相邻关节相位差 0.6 rad。常规身体参数切换用 0.3 s
quintic Ease-in-out，进入和退出 COILED / WRAP 均用 1.0 s。底盘速度也按对应时间缓动。
头部仍遵守 URDF 3 rad/s 限速：大幅抬头时自动延长俯仰插值，解析峰值不超过 2.6 rad/s。
频率通过积分得到连续相位，WRAP 的 1.0 Hz 不会令波形突然跳相。
轨迹为 50 Hz，每帧完整发布 10 个关节的位置和速度，发布前检查长度与 NaN/Inf。
JTC 允许非零末点速度，并从上一条指令连续插值，避免此前整条轨迹被拒的问题。

ROAM 线速度范围 0.1–0.35 m/s，角速度范围 ±0.6 rad/s。
DODGE 及脱困为 -0.15 m/s、±1.5 rad/s；安全预测可临时限制危险指令。
以下参数可通过 ROS 参数读取：linear_min、linear_max、angular_max、dodge_linear、
dodge_angular、frequency、transition_time。

每隔随机 4–8 s 作一次行为选择，默认种子 42 可复现：

| 状态 | 选择概率 | 行为 |
|---|---|---|
| ROAM | 40% | 0.4 rad 身体扭动，头部平视 |
| REST | 7% | 停步 2 s，保留 0.2 rad 小幅扭动 |
| COILED | 3% | REST 总触发率 10% 中的 30% 升级；8 个关节收至 +0.65 rad，停留 4–6 s |
| ALERT | 30% | 停步 2 s，抬头 0.8 rad 并快速吐信子，身体振幅 0.35 rad |
| LOW_CRAWL | 20% | 持续 2 s，头部 -0.2 rad、振幅 0.32 rad、目标速度减半 |
| WRAP | 接近树干的 AVOID 中 50% | 先准备安全轨道，随后表演 4 s，身体 0.6 rad / 1.0 Hz，抬头朝树干一侧 |

COILED 和 WRAP 的维持时间不会被下一次随机选择提前打断。结束后平滑回到 ROAM。
9 节与 ±0.7 rad 机械限位决定盘卷呈近环形单圈，不能形成多圈紧密蚊香。
默认种子在前 30 s 包含 COILED；随机概率不保证任意种子都能在 30 s 触发。

### 脉冲式吐舌（不是正弦或慢速伸缩）

每次 ALERT 预先安排 3–5 次脉冲，总时长不超过 2 s：

1. 0.1 s 快速伸出，随机行程 0.06–0.12 m。
2. 停顿 0.05 s。
3. 0.1 s 快速收回。
4. 随机停顿 0.1–0.3 s，随后再伸出。

**舌头按上述关键要求绕过姿态缓动。** 身体常规 0.3 s、特殊状态 1.0 s，头部另受速度限位。
ALERT 结束收回舌头。运行时用实际 tongue_joint 位置确认，不能只看目标消息。

## 避障与脱困

读取与 SDF 同源的树干地图，订阅实际 /odom 和全部关节状态。原 P1/P2/P3 已在森林重构时被
随机树木替换，WRAP 使用地图中真实树干，不对不存在的坐标执行动作。
ROAM → AVOID 后，蛇头到树干表面间隙 <0.6 m 时有 50% 概率启动 WRAP 准备。
安全时缓慢接近；紧急间隙 <0.3 m 优先 DODGE。准备阶段先后退，再转成轨道切线方向。

WRAP 采用经用户确认的安全半径自适应速度。原 0.05 m/s、0.8 rad/s 对应半径仅 0.0625 m，
无法绕粗树干。现按树干半径、约 1.15 m 全身最大包络与 0.18 m 余量设定安全轨道，
以最高 0.35 m/s 沿圆弧前进，角速度根据实际半径计算，最多 0.8 rad/s。
4 s 表演是圆弧缠绕视觉，不承诺绕满 360°；不通过瞬移或禁用碰撞实现。
底盘与树干中心距离硬阈值 0.5 m，同时检查所有身体段未来 0.8 s 的位置、波形及树干碰撞。
头部俯仰抬起，偏航朝树干一侧，始终遵守 ±0.7 rad 机械限位。
WRAP 结束先向外转，再以 0.2 m/s 退出；危险时提前中止并避障。

3 s 指令行程超过 0.15 m，但实际累计位移小于 0.1 m 时判定卡死；
安全包络连续 3 s 禁止运动也触发恢复。恢复执行 2 s 后退与急转，随后返回漫游。
主动 REST / ALERT / COILED 的停车不会因没有位移命令而误判为卡死。
这些逻辑是已知地图的几何避障，不是相机识别或真实攀爬树干。

/snakebot/behavior_state 发布 JSON，包括行为、导航状态、舌头脉冲阶段、目标姿态、
ALERT 编号、脉冲数量、实际位移、恢复次数、WRAP 轨道半径及完成/中止计数。/snakebot/roam_state 保留兼容输出。

## 构建与运行

```bash
cd /data/ros2_ws
source /opt/ros/lyrical/setup.bash
source .venv/bin/activate
python3 -m unittest discover -s src/snakebot_description/tests -v
colcon build --packages-select snakebot_description
source install/setup.bash
ros2 launch snakebot_description snakebot_gazebo.launch.py
```

默认 GUI、NVIDIA / OGRE2、组合模式和统一行为节点均开启。
先 spawn 实体，再依次激活两个控制器，成功后才启动 snake_behavior。
所有仿真节点保持 use_sim_time=True。保留 GUI 窗口至少 30–45 s 以观察随机行为。

```bash
# 换一片森林（修改源世界与地图后再构建）：
/usr/bin/python3 src/snakebot_description/tools/generate_forest.py --seed 91
colcon build --packages-select snakebot_description

# 可选：关闭 GUI 或不用组合模式。
ros2 launch snakebot_description snakebot_gazebo.launch.py gui:=false
ros2 launch snakebot_description snakebot_gazebo.launch.py use_composition:=false
```

## 自检

```bash
ros2 control list_controllers
ros2 topic echo /snakebot/behavior_state
ros2 topic echo /joint_states
ros2 topic echo /joint_trajectory_controller/joint_trajectory
gz topic -e -t /world/snake_world/stats -n 5
nvidia-smi
```

至少观察 30 s：检查网格消失、树冠簇拥、落叶堆、金属高光、红色分叉舌头；
对比实际头部和舌头关节，看是否有 3–5 次断续伸出/停顿/收回。
RTF 读取 GUI 右下角或 stats，不以世界配置的目标 1.0 冒充实测值。
失败即停止并报告；实现说明与单元测试不代替运行验收。

## 方向键遥控

```bash
ros2 launch snakebot_description snakebot_gazebo.launch.py autonomous:=false
# 第二终端：
ros2 run key_teleop key_teleop --ros-args \
  -p use_sim_time:=false -p twist_stamped_enabled:=false -r key_vel:=/cmd_vel
```

统一节点继续关节行为，但不发布自动速度，避免抢占人工遥控。
↑/↓ 前后，←/→ 转向。key_teleop 使用系统时间处理松键超时，不改变仿真节点的时钟。

## 限制

树木与外观均用基础几何体，不是摄影级模型。PBR 高光取决于太阳与观察角度。
平面整体位移继续由 Gazebo Sim VelocityControl 指定，不模拟真实摩擦推进。
原有裁剪和接触抽搐需通过本轮实际数据验收，不能仅凭新缓动算法宣称已消失。

材质兼容依据：[SDFormat 的 URDF 扩展](https://sdformat.org/tutorials/specification/sdformat_urdf_extensions/)。

### 复现近树 WRAP 测试

以下只是初始化出生位置，运行过程中不移动模型位姿。选择地图第一棵真实树干附近，
默认种子 42 的近树选择进入 WRAP 准备，先后退再沿安全圆弧运动：

```bash
ros2 launch snakebot_description snakebot_gazebo.launch.py \
  spawn_x:=-7.33535 spawn_y:=0.06207 spawn_yaw:=0.0
```

测试时应记录所有身体段与树干的距离，不能只凭 base_link 超过 0.5 m 判定无穿模。
