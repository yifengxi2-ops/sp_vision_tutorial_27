# 迷宫导航：MPC 路径跟踪与轨迹平滑

基于 ROS 2 Humble 的一次点击导航项目。使用给定的行为树和 A* 全局规划器，保留 PID 作为基线，新增 MPC 控制器，对全局路径进行碰撞约束下的平滑，并输出底盘速度指令。

固定起点为地图坐标 `(0.900, 0.900)`，指定终点为 `(14.100, 14.100)`。最终运行保留仿真器原有的共用回调组，不通过回调隔离消除阻塞；控制器对短时反馈延迟进行预测和渐进降速，对过期反馈输出零速。

> 当前状态：恢复后的控制器已编译通过，公开插件接口已恢复为初始版本。最终源码尚未完成原版仿真器下的固定起终点完整复测，本文不声称已在长时间阻塞下保证无碰撞到达，也不将诊断实验作为正式成绩。

## 1. 实现与数据流

```text
RViz 目标
  -> 上层/下层行为树
  -> A* 全局规划 -> /global_path
  -> ControllerServer -> MpcController
       代价地图融合 -> 路径平滑 -> 参考速度生成
       反馈时间检查 -> 状态预测 -> MPC 求解 -> 预测碰撞检查
  -> /sentry/cmd_vel -> 仿真器
                         -> /Odometry、TF、局部代价地图
```

主要代码：

| 文件                                                                                                                                                  | 职责                                                    |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| [mpc_controller.hpp](src/sp_controller_server/plugins/mpc_controller.hpp) / [mpc_controller.cpp](src/sp_controller_server/plugins/mpc_controller.cpp) | MPC、轨迹平滑、地图检查和插件实现，全部位于这两个文件中 |
| [pid_controller.cpp](src/sp_controller_server/plugins/pid_controller.cpp)                                                                             | PID 基线控制器                                          |
| [controller_server.cpp](src/sp_controller_server/src/controller_server.cpp)                                                                           | 接收路径与里程计、坐标变换、插件调度及过期反馈停速      |
| [controller_plugin.hpp](src/sp_controller_server/include/sp_controller_server/controller_plugin.hpp)                                                  | 原始公开接口，保持与初始提交一致                        |
| [plugin_description.xml](src/sp_controller_server/plugin_description.xml)                                                                             | PID、MPC 的 pluginlib 注册                              |
| [nav_params.yaml](src/sp_nav_bringup/config/nav_params.yaml)                                                                                          | 话题、坐标系、控制频率与插件选择                        |

## 2. 环境与运行

运行环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10。除原工程依赖外，MPC 使用 Eigen3；仿真器附带的二进制扩展需要与 Python ABI 匹配，不能当作普通缓存删除。

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-pluginlib ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs ros-humble-std-msgs ros-humble-std-srvs \
  ros-humble-rviz2 ros-humble-behaviortree-cpp-v3 \
  qtbase5-dev libopencv-dev libyaml-cpp-dev libeigen3-dev \
  ros-humble-eigen3-cmake-module
pip3 install --user pygame numpy Pillow
```

在仓库根目录构建完整工程：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash bash/sim_nav_all_start.sh
```

[启动脚本](bash/sim_nav_all_start.sh) 会打开导航、静态 TF 和仿真终端，需要图形桌面以及 `gnome-terminal`、`xterm` 或 `konsole` 中的一种。不要重复启动整套节点。

仅修改控制器后，可增量编译：

```bash
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select sp_controller_server --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

重新编译不会替换正在运行的插件实例，需停止并重启相关节点。恢复源码后同样必须重编译，不能直接使用旧安装库。

当前配置选用 `MpcController`。切换 PID 时，将配置中的 `plugin_name` 和 `plugin_type` 同时改为 `PidController`；比较两者时应保持起终点、仿真器和采样方法一致。当前 YAML 没有单独的 `MpcController` 参数块，以下默认参数来自插件的 `configure()`。

启动后确认地图、机器人与路径坐标系正常，再在 RViz 中向指定终点发送一次目标。不得拖动起点、中途修改目标或手动发布 `cmd_vel`。RViz 点击存在坐标误差，记录实验时应保存实际目标坐标，不能仅凭画面认定为指定坐标。

## 3. 控制方法

### 3.1 路径平滑与速度规划

对 A* 路径去除相邻重复点，保持端点固定，通过邻点平滑和原始路径牵引进行有限次迭代。每个候选点最多偏离原始位置 `0.10 m`；点代价和两侧线段代价不能增加，禁止穿过占用区域。

最终按不大于 `0.04 m` 的线段间隔重采样，保留所有折点以避免跨拐角切墙，再次验证整条路径。仅当点击目标与规划终点距离不超过 `0.10 m`，且连接线段可通行时，才将点击目标补到路径末端。

参考速度综合路径曲率、代价值、终点距离以及前后向加减速约束生成。平滑和预测检查是有限模型下的检查，不等于对任意延迟的安全证明。

### 3.2 MPC 模型与优化

状态为地图系二维位置和速度，控制量为期望二维速度。采用一阶速度响应的精确零阶保持离散模型，令 `c = 1 - exp(-dt / tau)`：

```text
p_next = p + tau*c*v + (dt - tau*c)*u
v_next = (1 - c)*v + c*u
```

目标函数包括位置误差、速度误差、控制幅值、相邻控制变化量和终端误差。采用凝聚形式的二次优化，缓存 Eigen LDLT 分解，通过有界迭代的 ADMM 处理速度与指令变化约束。每周期只输出第一个控制量，然后利用下一次反馈滚动求解。

`success` 和 `converged` 含义不同：达到迭代上限但未收敛的结果并不必然被拒绝；仍需满足有限性、可行性、目标值不劣化和后续碰撞验证，才能作为候选输出。

### 3.3 延迟反馈与停速

反馈年龄由当前 ROS 时间减去里程计源时间戳得到，不用消息接收时间代替源时间。短时缺少新反馈时，根据速度外推位置，并估计航向变化；另以 `0.15 s` 固定执行延迟补偿预测起点。

| 反馈年龄            | 行为                                               |
| ------------------- | -------------------------------------------------- |
| `0 <= age <= 0.2 s` | 反馈年龄不额外压低速度，仍受其他速度和碰撞约束限制 |
| `0.2 < age < 0.4 s` | 速度比例按 `(0.4 - age) / 0.2` 线性下降            |
| `age >= 0.4 s`      | 输出零速                                           |

缺失、非有限或时间异常的反馈，地图缺失/过期，以及没有通过检查的轨迹，也会触发零速输出。当前提交不包含实验性的 `holdSafe()`、1.5 秒指令保持检查或前方净空限速。

### 3.4 地图与碰撞检查

全局和局部代价取最大值；未知区域和全局地图外区域按占用处理，局部窗口以外仍使用全局地图。线段检查覆盖穿过的栅格及角点，避免只采样端点造成漏检。

检查对象同时包括求解器产生的控制序列和实际限幅后的首条指令序列，考虑旧指令延迟、模型误差包络以及零速指令下的残余运动。地图已有机器人膨胀，额外包络用于模型误差，不再重复使用完整机器人半径膨胀。

## 4. 关键默认参数

| 参数                                            | 默认值                 | 作用                                                 |
| ----------------------------------------------- | ---------------------- | ---------------------------------------------------- |
| 控制频率                                        | `50 Hz`                | 控制器服务器调度频率                                 |
| `prediction_dt` / `horizon`                     | `0.1 s` / `20`         | 预测时域为 `2 s`                                     |
| `velocity_time_constant`                        | `0.5 s`                | 一阶速度响应时间常数                                 |
| `max_linear_speed` / `cruise_speed`             | `0.35` / `0.30 m/s`    | 速度上限与巡航参考                                   |
| `max_command_acceleration`                      | `0.6 m/s²`             | 正常候选指令变化率约束，安全停速可直接输出零         |
| `lateral_acceleration` / `braking_acceleration` | `0.25` / `0.25 m/s²`   | 曲率限速与参考速度规划                               |
| `actuator_delay`                                | `0.15 s`               | 固定执行延迟建模，不是实测长阻塞上界                 |
| `solver_iterations` / `solver_tolerance`        | `80` / `0.0001`        | 求解迭代预算与容差                                   |
| `goal_tolerance` / `goal_speed_tolerance`       | `0.03 m` / `0.025 m/s` | 插件内部稳定到达条件，不代表整套行为树的统一判定阈值 |
| `global_map_timeout` / `local_map_timeout`      | `3.0 s` / `1.0 s`      | 地图新鲜度要求                                       |

## 5. 观测与测试状态

| 话题                            | 含义                                       |
| ------------------------------- | ------------------------------------------ |
| `/global_path`                  | 原始全局参考路径                           |
| `/mpc/smoothed_path`            | 平滑后的参考路径                           |
| `/mpc/predicted_path`           | 选中求解结果的模型预测路径，非实际执行轨迹 |
| `/mpc/metrics`                  | 诊断数组                                   |
| `/Odometry` / `/sentry/cmd_vel` | 反馈状态与发布的速度指令                   |

诊断数组依次为：到平滑路径的距离、终点距离、实测速度大小、指令速度大小、求解与候选检查阶段耗时（ms）、反馈年龄（s）、速度模型残差、求解迭代数、收敛标志。

注意两个统计口径：

- 诊断数组只在选出有效候选后发布，停速分支不发布，不能用它的年龄分布代表所有过期时段。
- 内部路径误差相对于平滑路径，不能直接当作作业要求的原始全局路径跟踪误差；耗时字段也不是完整控制周期耗时。

### 已有验证与限制

恢复版已通过控制器包 Release 编译，源与安装接口均核对为初始版本，安装库已检查不含撤回的保持安全实验逻辑。

早期曾通过隔离仿真器回调进行诊断并完成导航，但该实验修改了运行条件，因此不作为本提交的正式到达精度或用时成绩。

恢复共用回调组后的诊断中，曾观测到秒级里程计中断，另一次限速实验中最长到达间隔约 `6.51 s`，并发生撞墙，不知是否是虚拟机性能不够的原因。后续增加的 1.5 秒指令保持检查又出现持续停滞，因此已撤回。最终恢复版仍需重新进行固定起终点测试，不能沿用上述不同版本的数据。

长时间阻塞可能同时影响状态发布和速度指令接收。控制器发布零速不代表仿真器已立即处理零速，因此 `0.4 s` 保护只能表述为反馈过期处理，不能保证实际在 `0.4 s` 内停车。

### 最终评测记录方法

从指定起点向指定终点只发送一次目标，同时记录实际目标、里程计、TF、全局路径和速度指令。将位姿统一到 `map` 系后，计算终点欧氏误差与到原始全局路径线段的最短距离，报告均值、P95、最大值及有效样本数。总用时应明确起止事件，失败或未到达的试验不填成功用时。

下面命令仅用于录制，不发送目标或速度；`run_01` 输出目录必须不存在：

```bash
source install/setup.bash
ros2 bag record -o bags/run_01 \
  /goal_pose /Odometry /tf /tf_static /global_path /sentry/cmd_vel \
  /mpc/metrics /mpc/smoothed_path /mpc/predicted_path
```

测试数据仅作本地分析，不应提交大型 bag。已有动态库中仿真器自带的 Python 扩展与本地产生的构建产物应区别对待。

## 6. 已知问题与改进方向

- 固定执行延迟和短时状态外推不覆盖任意长的并发阻塞；无法从发布端保证零速指令被及时执行。
- 地图或路径更新会触发整条路径重新优化，包括已走过的部分；后方新障碍也可能导致暂停，直到重新规划或障碍消失。
- 完成相同运行条件下的 PID/MPC 定量对比， MPC 已在本任务中实测优于 PID。
- RViz 原配置主要展示全局地图与路径；MPC 额外路径已发布，可按需添加 Path 显示，但不默认假设其已出现在界面中。

## 附录：原作业要求

<details>
<summary>展开原始作业说明（保留供对照）</summary>

# 27算法组导航方向招新大作业

## 0\. 环境配置

```Plain Text
// 已装 Ubuntu 22.04 和 ROS2 Humble
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-pluginlib \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-std-srvs \
  ros-humble-rviz2 \
  ros-humble-behaviortree-cpp-v3 \
  qtbase5-dev \
  libopencv-dev \
  libyaml-cpp-dev

pip3 install --user pygame numpy Pillow
```

## 1\. 作业目标

在给定的迷宫地图上，完成 **一次点击导航** 全流程：

1. 机器人从 **固定起点** 出发；

2. 选手在 RViz 中对 **指定终点** 发布一次目标；

3. 上层行为树 `click_nav.xml` 触发导航；

4. 下层行为树 `default_nav_with_fallback.xml` 调用 A\* 规划全局路径；

5. **你实现的控制器插件** 跟踪规划轨迹，使机器人准确、平滑地到达终点。

## 2\. 任务场景

### 2\.1 地图

| 项目     | 数值                             |
| -------- | -------------------------------- |
| 文件     | `maze_map.pgm` / `maze_map.yaml` |
| 分辨率   | `0.05 m/pixel`                   |
| 尺寸     | `15.00 m × 15.00 m`（300×300）   |
| 走廊宽度 | 约 `1.50 m`                      |
| 墙厚     | 约 `0.15 m`<br>                  |

<p align="center">
  <img src="images/image_1.png" alt="仿真 GUI" width="420" />
  <img src="images/image_2.png" alt="代价地图与全局路径" width="420" />
</p>

### 2\.2 固定起终点

|          | **map 系坐标 \(x, y\)** | **说明**                                          |
| -------- | ----------------------- | ------------------------------------------------- |
| **起点** | `(0.900, 0.900)`        | 迷宫左下角通路中心；仿真初始位置已设为此处        |
| **终点** | `(14.100, 14.100)`      | 迷宫右上角通路中心；评测时只允许对该点 click 一次 |

- **不允许拖拽改起点、不允许中途二次改目标；**

- **不允许手动发布 ****`cmd_vel`**** 绕过控制器。**

### 2\.3 仿真器

- **不允许修改仿真器内部代码及参数；**

- **仿真里机器人撞墙便会卡住。**

## 3\. 发放内容与你需要完成的部分

### 3\.1 发放给你的代码

| 包                     | 内容                                                                     |
| ---------------------- | ------------------------------------------------------------------------ |
| `robot_msg`            | 消息 / Action                                                            |
| `sp_map_server`        | ESDF / 全局代价地图                                                      |
| `sp_global_planner`    | A\* 全局规划                                                             |
| `sp_nav_bt`            | 下层 BT：`default_nav_with_fallback.xml`                                 |
| `sp_decision`          | 上层 BT：`click_nav.xml`                                                 |
| `sp_nav_bringup`       | 启动与地图（含 `maze_map`）                                              |
| `sp_nav_sim`           | 拖拽仿真（里程计 \+ TF）                                                 |
| `sp_controller_server` | 控制器框架（`ControllerPlugin` 接口、`controller_node`、pluginlib 加载） |

控制器侧（名称可自定，但须 pluginlib 可加载）：

- `include/sp_controller_server/controller_plugin.hpp`（接口，只读）

- `plugins/<你的控制器>.hpp/.cpp`（由你实现）

- `plugin_description.xml`、`CMakeLists.txt`（需正确注册插件）

- `launch/controller.launch.py`

**注意：代码中已给出 PidController（\.cpp 与 \.hpp）的大体框架，可直接在此基础上修改。**

### 3\.2 你必须完成的工作（代码中已用 TODO 标明）

1. **实现路径跟踪控制器插件**

实现 `ControllerPlugin` 三个接口：

- `configure(...)`：读参数、初始化

- `setPlan(path)`：接收 `/global_path`（经 controller server 转发）

- `computeVelocityCommands(pose, velocity)`：输出 `TwistStamped`（线速度）

2. **修改参数使工程能在迷宫上跑通**

发放包中部分路径 / 话题 / 坐标系 / 为空，直接运行`sim_nav_all_start.sh`不能正确评测迷宫任务，你需要自行排查并修改。

3. **可视化与代码管理**

在 RViz 中至少清晰展示：全局代价地图、全局路径、机器人位姿、目标点等；代码要有清晰的 commit 记录，知道该忽略哪些文件。

## 4\. 评分标准（100 分）

**评测方式：固定起点 → 一次 click 到固定终点 → 记录从目标下发到判定到达的过程。**

| **分项**             | **分值** | **考察内容**                              |
| -------------------- | -------- | ----------------------------------------- |
| **准确到达**         | 25       | 最终位置与终点距离                        |
| **时间**             | 25       | 从收到目标到到达的用时                    |
| **跟踪精度**         | 20       | 对采样位姿计算到全局参考路径的误差        |
| **报告**             | 15       | 清晰的 README 介绍你的项目                |
| **可视化与代码管理** | 15       | RViz 地图、路径、机器人清晰；代码管理清晰 |

## 5\. 加分项（**不超过总分 100**）

- 轨迹平滑（平滑 A\* 轨迹或直接使用其他算法）；

- 自研优于 PID 的控制器（例如使用 LQR 或 MPC）。

## 6\. 提交要求

- 线上提交到自己的 git 仓库对应分支，需包含完整可编译源码；

- 如虚拟机卡顿且自己无法解决，10\.6后可线下到地下室使用小电脑调试（需提前告知群里导航方向管理员）；

- DDL：10\.25。

</details>
