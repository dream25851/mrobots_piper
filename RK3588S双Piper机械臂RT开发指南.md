# RK3588S 双 Piper 机械臂 ROS 2 RT 开发指南

> 状态：勘察与迁移设计基线
> 最近验证日期：2026-08-31
> 目标设备：RK3588S CoolPi 4B / KaihongOS（OpenHarmony）/ ROS 2 Jazzy
> 参考实现：`/home/alpha/physical_ai_runtime/src/bringup/piper_manipulation/rt_launch`

> 2026-09-01 实机更新：双臂 `piper1/piper0` 已按 USB 序列号稳定识别，ROS 反馈、
> 双臂动作和跨机 Fast DDS 均已通过；有线地址为 `192.168.1.18 ↔ 192.168.1.19`。
> 最新证据、运行命令与跨机 DDS 阻塞项以
> `/home/alpha/mrobot/piper_dual_bringup/README.md` 为准。

## 1. 目标与边界

这块 RK3588S 的目标不是运行完整的 `physical_ai_runtime`，而是替代其中的
RT host，只负责两台 Piper follower 机械臂及其原生夹爪：

- 运行双臂 `ros2_control_node`；
- 运行左右臂硬件接口和控制器；
- 发布关节状态与 TF；
- 运行本地 JTC 心跳保护；
- 通过 ROS 2/DDS 接收 workstation 的动作命令。

以下组件继续留在 workstation，不迁移到 RK3588S：

- Execution Manager / RMI；
- cuRobo、MoveIt 规划和策略推理；
- leader/teleoperation 节点；
- 相机、感知、录包和可视化；
- RViz、Foxglove 等 GUI。

`apps/profiles/piper_bimanual.yaml` 当前把部分外部相机也列在 RT host 的职责中，
但本次 RK3588S 角色明确收窄为“只驱动双臂”。部署时应以本节边界为准。

## 2. 已验证的开发板现状

### 2.1 设备与系统

| 项目 | 实测结果 |
| --- | --- |
| HDC 设备 ID | `ec29004133314d38433031a51f413c00` |
| USB 枚举 | Rockchip `2207:5000`，`HDC Device` |
| 板卡模型 | `RK3588S CoolPi 4B Board` |
| Device Tree compatible | `rockchip,khs_3588s_sbc` |
| CPU 架构 | `aarch64`，4×Cortex-A55 + 4×Cortex-A76 |
| 内核 | Linux `6.6.101` |
| 内存 | 约 7.6 GiB |
| `/data` 可用空间 | 约 38 GiB |
| 根文件系统 | `/`、`/vendor` 只读，`/data` 可写 |
| SELinux | Enforcing |

主机通过仓库自带 HDC 连接：

```bash
cd /home/alpha/mrobot
./toolchains/hdc list targets
./toolchains/hdc shell
```

### 2.2 ROS 2 与控制栈

| 组件 | 板端版本/状态 |
| --- | --- |
| ROS 发行版 | Jazzy |
| `rclpy` | 7.1.4，导入成功 |
| `rclcpp` | 28.1.9 |
| `ros2_control` / `controller_manager` | 4.35.0 |
| `ros2_controllers` | 4.30.1 |
| MoveIt 2 | 2.12.3 |
| 默认 RMW | `rmw_fastrtps_cpp` |
| 默认 `ROS_DOMAIN_ID` | 26 |
| CycloneDDS | 类型支持库存在，但 `librmw_cyclonedds_cpp.so` 不存在，当前不能切换 |

实测通过：

```text
rclpy_import=OK
ros2 doctor: All 3 checks passed
C++ demo_nodes_cpp talker -> listener 收发成功
ros2 control CLI 可用
```

`ros2 doctor` 会提示缺少可选的 `rosdistro`，`ros2 control` 的控制器链可视化
会提示缺少可选的 `pygraphviz`。两者均不阻塞控制器和硬件插件运行。

### 2.3 板端运行时布局

```text
OpenHarmony / KaihongOS
├── /、/system、/vendor              系统层，通常只读
├── /bin/run                        M-Robots 环境注入入口
└── /data                           用户可写区
    └── local
        ├── release                 厂商 ROS 2/Python/MoveIt underlay
        ├── ros2_ws                 用户 ROS 2 overlay，当前不存在
        └── robot                   建议放部署配置、入口脚本和日志
```

板端不是标准 Ubuntu ROS 安装。基础 ROS 命令需要经过 `/bin/run`：

```bash
run ros2 doctor
run ros2 pkg list
run python3 -c "import rclpy; print('OK')"
```

文档中的 `/data/local/ros2_env.sh` 当前并不存在，但 `/bin/run` 已能注入基础
Jazzy 环境。未来部署用户 overlay 后，必须显式加载
`/data/local/ros2_ws/install/setup.sh`，不能假设 `run` 会自动发现它。

### 2.4 编译 ABI

板端 ROS 可执行文件使用：

```text
aarch64 + musl + /lib/ld-musl-aarch64.so.1
```

因此不能使用以下产物：

- workstation 上 Pixi/Conda 的 `linux-64` 二进制；
- 普通 Ubuntu `aarch64-linux-gnu`/glibc 二进制；
- 在 x86_64 上直接 `colcon build` 得到的库。

必须使用与 M-Robots 镜像匹配的 `aarch64-linux-ohos`/musl 工具链、sysroot、
ROS 头文件和动态库进行交叉编译。

板端虽然安装了 `colcon`，但当前没有 `cmake`、`make`、`gcc/g++` 或
`clang/clang++`，不能在板上原生编译 C++ ROS 包。

## 3. 目标双机架构

```text
workstation（非 RT）
├── policy / planner / teleop
├── Execution Manager / RMI
├── cameras / perception / recorder
└── ROS_DOMAIN_ID=1 + CycloneDDS
             │
             │ DDS：动作命令、action、状态、TF、heartbeat
             ▼
RK3588S（RT host 角色）
├── robot_state_publisher
├── joint_trajectory_controller_guard × 2
└── controller_manager / ros2_control_node（500 Hz 目标）
    ├── joint_state_broadcaster
    ├── 左臂 JSPC / TSKPC / JTC / gripper forward
    ├── 右臂 JSPC / TSKPC / JTC / gripper forward
    ├── PiperHardwareInterface × 2
    └── PiperGripperInterface × 2
             │
             ├── SocketCAN A ── 左 Piper + 左夹爪
             └── SocketCAN B ── 右 Piper + 右夹爪
```

一个 `ros2_control_node` 管理四个硬件组件：左右臂各一个
`PiperHardwareInterface`，左右夹爪各一个 `PiperGripperInterface`。同一侧的
机械臂和夹爪共享同一条 CAN 总线。

## 4. 必须保持不变的 ROS 接口契约

workstation 端已经依赖 `/execution/<group>/...` 命名。迁移 RT host 时，不应
通过改 topic 名来绕开兼容问题。

| 路由 | 板端控制器 | 对外接口 |
| --- | --- | --- |
| 左臂关节伺服 | `left_arm_jspc` | `/execution/left_arm/joint_reference` |
| 右臂关节伺服 | `right_arm_jspc` | `/execution/right_arm/joint_reference` |
| 左臂笛卡尔伺服 | `left_arm_tskpc` | `/execution/left_arm/pose_reference`、`twist_reference` |
| 右臂笛卡尔伺服 | `right_arm_tskpc` | `/execution/right_arm/pose_reference`、`twist_reference` |
| 左臂轨迹 | `left_arm_jtc` | `/execution/left_arm/follow_joint_trajectory` |
| 右臂轨迹 | `right_arm_jtc` | `/execution/right_arm/follow_joint_trajectory` |
| 左夹爪 | `left_gripper_fwd` | `/execution/left_gripper/joint_reference` |
| 右夹爪 | `right_gripper_fwd` | `/execution/right_gripper/joint_reference` |

启动契约也应保持：

1. `joint_state_broadcaster` 启动为 `active`；
2. JSPC、TSKPC、JTC 和 gripper forward 全部以 `inactive` 启动；
3. workstation 的 Execution Manager 获得 authority 后再激活唯一控制路由；
4. 两个 JTC guard 在 RK3588S 本地运行，workstation heartbeat 丢失时取消轨迹；
5. RT host 不启动 Execution Manager，也不启动 planner。

## 5. 复用现有 `physical_ai_runtime` 的最小包集合

开发阶段仍以 `/home/alpha/physical_ai_runtime` 为源码真源。RK3588S overlay
至少需要以下包/依赖：

| 包或库 | 来源 | 板端现状 | 处理方式 |
| --- | --- | --- | --- |
| `piper_description` | workspace 源码 | 未安装 | 作为数据包交叉构建/安装 |
| `piper_hardware_interface` | workspace 源码 | 未安装 | 对板端 Jazzy ABI 交叉编译 |
| `piper_manipulation_rt_launch` | workspace 源码 | 未安装 | 安装并做 OpenHarmony 适配 |
| `libpiper 0.5.x` | 当前仅 Pixi `linux-64` | 未安装 | 获取源码并为 OHOS/musl 交叉编译 |
| `manipulation_position_controllers 0.3.1` | 当前为预编译包 | 未安装 | 获取准确源码并为板端构建 |
| `joint_trajectory_controller_guard 0.1.0` | 当前为预编译包 | 未安装 | 获取准确源码并为板端构建 |
| 标准 controller_manager/JTC/forward controller | M-Robots underlay | 已安装 | 先决定版本对齐策略 |
| `robot_state_publisher`、`xacro` | M-Robots underlay | 已安装 | 直接复用 |

不要把 workstation 的 `.pixi/envs/...`、`install/` 或 `.so` 直接复制到板上。

## 6. 当前阻塞项与兼容性差异

### 6.1 ros2_control 版本不一致

```text
physical_ai_runtime 锁定：
  controller_manager / ros2_control 4.45.1
  ros2_controllers              4.40.0

RK3588S 当前镜像：
  controller_manager / ros2_control 4.35.0
  ros2_controllers              4.30.1
```

现有 `controllers.yaml` 中使用的：

```yaml
constraints:
  decelerate_on_cancel: true
  <joint>:
    max_deceleration_on_cancel: 40.0
```

在板端 4.30.1 的 JTC 库和安装文件中未找到对应参数名，不能假设该配置可以
原样加载。

可选方案：

1. **基于板端版本移植**：所有自定义包针对 4.35/4.30 重新构建，并为缺失的
   JTC 安全行为做明确回移或替代实现；
2. **整体版本升级**：为 OHOS 构建一套协调一致的 4.45/4.40 overlay，并让
   所有控制器、hardware plugin、controller manager 使用同一套 ABI。

禁止把针对 4.45 编译的 controller/hardware plugin 单独加载进 4.35 的
controller manager。版本策略确定后，应把版本写入板端 deployment manifest。

### 6.2 当前双路板端 CAN

开发板现已通过两只 candleLight USB-CAN 提供两条独立总线：

```text
piper1 -> left follower  -> USB serial 004400424148570D20343133
piper0 -> right follower -> USB serial 004500424148570C20343133
can0   -> SoC rockchip_canfd，保持 DOWN，不用于 Piper
```

两路均已配置为 1 Mbps 并完成双臂并发反馈和动作验收。逻辑名称由 USB 序列号绑定，
不依赖 Linux 临时枚举出的 `can1/can2` 顺序。

### 6.3 CAN 左右映射已现场固定

- follower：左 `piper1`、右 `piper0`；
- 本机 Leader：左 `can0`、右 `can1`；
- `piper1/piper0` 由 USB-CAN 序列号绑定，不依赖枚举顺序。

若更换 USB-CAN，必须重新记录序列号并逐臂确认，不能按临时 `can1/can2` 猜测。

### 6.4 OpenHarmony 没有 Bash

板端有 `/bin/sh`、`taskset` 和 `xacro`，但没有 `bash`。当前
`controller_bringup.launch.py` 用 `ExecuteProcess(["bash", "-c", ...])`
启动 joint-state broadcaster，因此原 launch 文件会失败。

推荐后续把这段改成 POSIX `sh -c`，不要仅为一条命令向 RT 镜像加入 Bash。
修改后应在 `physical_ai_runtime` 中保留 contract test，避免两个平台分叉成
不可维护的 launch 文件。

### 6.5 网络与 DDS 已就绪，持久化仍待完成

当前已验证配置为：

```text
workstation: 192.168.1.18/24
RK3588S:     192.168.1.19/24
ROS_DOMAIN_ID=1
workstation 控制面: Cyclone DDS
RK3588S 数据面:     Fast DDS
```

跨机 topic、service、双臂动作和 RViz 已通过。`192.168.1.19` 与 HDC 8710 当前仍
是运行时配置；重启持久化和正式时间同步服务尚待完成。

## 7. 源码与部署目录约定

### 7.1 开发机

不要在板上手工维护源码。建议从 `physical_ai_runtime` 选择包，建立专用的
目标构建工作区或交叉构建清单：

```text
/home/alpha/physical_ai_runtime/              源码真源
├── src/embodiments/robots/piper/
│   ├── piper_description/
│   └── piper_hardware_interface/
└── src/bringup/piper_manipulation/rt_launch/

/home/alpha/mrobot/ros2_ws/                   建议的板端交叉构建工作区
├── src/                                      受控引用/checkout，不复制改两份
├── build-ohos/
├── install-ohos/
└── toolchain/aarch64-linux-ohos.cmake
```

真正开始实现时，可创建上述工作区；本次勘察没有创建或修改板端代码。

### 7.2 开发板

```text
/data/local/release/                          厂商 underlay，禁止放项目代码
/data/local/ros2_ws/install/                  交叉编译后的 ROS overlay
/data/local/robot/bin/start_piper_rt.sh       POSIX 启动入口
/data/local/robot/config/cyclonedds_rt.xml    板端 DDS 配置
/data/local/robot/config/deployment.yaml      版本、CAN 映射、IP、构建标识
/data/local/robot/log/                        项目日志
```

不要把源码和 build 中间产物部署到产品板；部署 `install`、配置和必要的调试
符号包即可。

## 8. Piper 硬件接口开发原则

现有 `piper_hardware_interface` 已按正确边界拆分：硬件通信属于
`hardware_interface::SystemInterface`，控制器不直接访问 CAN。

在 Jazzy 目标版本上维护这些生命周期行为：

- `on_init()`：严格校验关节、接口、`can_interface` 和阻尼参数；
- `on_configure()`：创建 libpiper 对象并连接总线，不发送运动命令；
- `on_activate()`：先读取当前位置，再把 command 同步到 state，避免使能跳变；
- `read()`：读取非阻塞 feedback snapshot，检查新鲜度和故障状态；
- `write()`：发送已限幅命令，禁止无界阻塞和动态分配；
- `on_deactivate()` / `on_error()`：先发送安全保持/停止，再释放总线；
- 析构函数：作为异常退出的最后安全网，但不能替代硬件 watchdog 和急停。

板端 `ros2_control 4.35.0` 已支持 Jazzy 4.x 的自动接口导出。现有 Piper 插件
仍使用兼容但已弃用的 `export_state_interfaces()` / `export_command_interfaces()`。
首次移植以保持行为为主，版本跑通后再单独迁移到 `on_export_*` 或自动接口 API，
不要把 ABI 迁移与真机 bringup 混在一个改动里。

RT 路径要求：

- `read()` / `write()` 不执行文件 I/O、参数查询或普通日志刷屏；
- 固定容量容器预分配，避免 `new`、`malloc` 和 vector 扩容；
- CAN I/O 非阻塞并有每周期预算；
- 非 RT 诊断通过预分配缓冲区或 `realtime_tools` 输出；
- 连续丢帧、feedback 超时、CAN bus-off 必须进入明确安全状态；
- 夹爪宽度和单指行程的 `2×` 换算边界必须保留。

## 9. ROS 2 overlay 的启动方式

板端 `/bin/run` 当前不会自动加载用户 workspace。建议最终入口使用 POSIX shell：

```sh
#!/system/bin/sh
set -eu

export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export CYCLONEDDS_URI=file:///data/local/robot/config/cyclonedds_rt.xml
export ROS_LOG_DIR=/data/local/robot/log

exec run sh -c '
  . /data/local/release/usr/setup.sh
  . /data/local/ros2_ws/install/setup.sh
  exec ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
    use_fake_hardware:=false \
    left_can_interface:=piper_left \
    right_can_interface:=piper_right \
    use_rviz:=false \
    cpu_affinity:=none
'
```

这只是目标模板。以下条件完成前必须保持 `use_fake_hardware:=true`：

- 依赖版本与 ABI 已锁定；
- 两条 CAN 已稳定命名和正确映射；
- 急停、上电状态、关节限制与 watchdog 已验证；
- 实时性和丢帧测试通过。

### 9.1 板端 CycloneDDS 配置

workstation 的 `.config/cyclonedds_default.xml` 绑定 `192.168.1.18`，不能原样
复制到 RK3588S。板端配置应绑定自己的地址并把 workstation 作为 peer，例如：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="192.168.1.101"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer address="192.168.1.18"/>
      </Peers>
    </Discovery>
    <Internal>
      <SocketReceiveBufferSize min="10MiB" max="10MiB"/>
      <SocketSendBufferSize min="10MiB" max="10MiB"/>
    </Internal>
  </Domain>
</CycloneDDS>
```

只有地址实际配置为上述值时才能使用此示例。

## 10. CAN 部署约定

硬件插件只打开指定 SocketCAN；它不负责设置 bitrate、重启策略或接口别名。
这些属于 OpenHarmony deployment/init 层。

Piper 当前目标总线参数来自旧 RT host：

```text
bitrate:    1,000,000 bit/s
txqueuelen: 先按经验证的现有部署值，再通过丢帧/延迟测试调整
```

建议稳定别名：

```text
piper_left  -> 经过序列号或物理端口确认的左臂 CAN
piper_right -> 经过序列号或物理端口确认的右臂 CAN
```

每次启动前检查：

```bash
run ip -details link show piper_left
run ip -details link show piper_right
```

真机 CAN bringup 必须在急停可用、机械臂周围清空、夹爪和机械臂均断使能的
条件下进行。首次测试只监听 feedback，不发送运动命令。

## 11. 实时性现状与放行标准

当前内核实测为：

```text
CONFIG_PREEMPT_VOLUNTARY=y
CONFIG_HZ_300=y
CONFIG_PREEMPT_RT 未启用
/sys/kernel/realtime 不存在
```

所以目前只能证明“ROS 2 能运行”，不能证明“该板可替代 RT host”。现有配置的
`update_rate: 500`、`thread_priority: 98` 和 Ruckig `0.002 s` 周期不能仅凭平均
频率判定合格。

`piper_hardware_interface`/libpiper 在检测到非 RT 内核时不会强制 SCHED_FIFO，
因此它可能启动，但不具备确定性保证。

建议放行门槛：

1. 为正确的 RK3588S 产品目标构建 PREEMPT_RT；不要直接套用文档中的
   RK3588A product name；
2. 验证 `CONFIG_PREEMPT_RT=y`、`CONFIG_HIGH_RES_TIMERS=y`、合适的 HZ；
3. 为 controller manager/libpiper 配置 RT priority、memlock 和 SELinux 权限；
4. 根据 RK3588S 核拓扑选择 A76 核并配置 CPU/IRQ affinity；
5. 用 `cyclictest` 建立空载和压力负载的 baseline；
6. 记录 controller manager 500 Hz 的平均、p99、p99.9、最大周期和 deadline miss；
7. 同时压测两条 1 Mbps CAN，统计 RX/TX 丢帧、bus-off、feedback age；
8. workstation 断网、进程崩溃和 heartbeat 丢失时，两臂必须进入已定义安全状态。

不要沿用旧 x86 RT host 的 `RT_CM_CPU_AFFINITY=14,15`。RK3588S 只有 CPU 0–7；
在完成 IRQ 和核型分析前，fake hardware 阶段使用 `cpu_affinity:=none`。

## 12. OpenHarmony 自启动

OpenHarmony 使用 init `.cfg`，不是 systemd。最终可由 `/etc/init/*.cfg` 启动
`/data/local/robot/bin/start_piper_rt.sh`，但应满足：

- 代码和 overlay 位于 `/data`，不放进系统只读分区；
- 服务等待网络、时间同步和两条 CAN ready；
- 使用有限重启策略，避免硬件故障时无限反复使能；
- `SIGTERM/SIGINT` 能触发 controller manager 正常停机；
- 服务 uid/gid、capability 与 SELinux domain 只授予 CAN、RT scheduling 和
  memlock 所需权限；
- 不以永久 `setenforce 0` 作为部署方案。

在 fake hardware、真机手动启动和故障注入全部通过前，不配置上电自启。

## 13. 分阶段迁移与验证计划

### 阶段 A：工具链和包集

- [ ] 获取与当前镜像匹配的 `aarch64-linux-ohos` 工具链和 sysroot；
- [ ] 获取 `libpiper 0.5.x` 源码；
- [ ] 获取 `manipulation_position_controllers 0.3.1` 与
      `joint_trajectory_controller_guard 0.1.0` 的准确源码；
- [ ] 确定 ros2_control 4.35/4.30 回移或 4.45/4.40 整体升级策略；
- [ ] 输出可重复的版本 manifest、toolchain file 和交叉构建命令；
- [ ] 使用 `readelf`、`file`、`ldd` 验证所有产物为 aarch64/musl 且无缺库。

### 阶段 B：板端 fake hardware

```bash
ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
run sh -c '. /data/local/ros2_ws/install/setup.sh && \
  ros2 launch piper_manipulation_rt_launch rt_stack.launch.py \
  use_fake_hardware:=true use_rviz:=false cpu_affinity:=none'
```

- [ ] launch 不依赖 Bash；
- [ ] `joint_state_broadcaster` 为 active；
- [ ] 左右 JSPC/TSKPC/JTC/gripper controllers 为 inactive；
- [ ] joint names、TF、controller interfaces 与 workstation 一致；
- [ ] workstation 能发现 RK3588S 节点；
- [ ] Execution Manager 可完成 acquire/activate/release/switch；
- [ ] heartbeat 丢失时 JTC goal 被本地取消；
- [ ] launch 退出后无 daemon、controller 或硬件进程残留。

### 阶段 C：单臂只读

- [ ] 确认左/右 CAN 物理映射；
- [ ] 仅启动一侧硬件并保持所有 command controller inactive；
- [ ] 检查 encoder、velocity、effort、fault 和 feedback freshness；
- [ ] 人工转动允许被动运动的关节，验证方向、单位和 joint name；
- [ ] 验证断线、bus-off 和错误帧进入安全状态。

### 阶段 D：单臂低速闭环

- [ ] 急停在手边，使用小角度、小速度、无遮挡姿态；
- [ ] 激活一个路由，禁止 JSPC/TSKPC/JTC 同时 claim 同一 command interface；
- [ ] 检查使能瞬间 command-state 同步，无跳变；
- [ ] 验证限位、stale timeout、cancel 和 shutdown；
- [ ] 完成至少一次断网和进程退出故障注入。

### 阶段 E：双臂与实时验收

- [ ] 两条 CAN 同时 500 Hz 工作；
- [ ] 两臂 controller switching 无资源冲突；
- [ ] 双臂 joint state 和 TF 连续；
- [ ] workstation 的双臂策略/teleop/trajectory 三类路由分别验收；
- [ ] 压力负载下记录 p99.9 和最大控制周期；
- [ ] 通过急停、heartbeat、CAN 断线、workstation 掉电、RK3588S 重启测试；
- [ ] 通过后再启用 OpenHarmony init 自启动。

## 14. 常用验收命令

### 板端基础检查

```bash
run ros2 doctor
run ros2 pkg prefix controller_manager
run ros2 pkg prefix piper_hardware_interface
run ros2 pkg prefix piper_manipulation_rt_launch
run ros2 pkg prefix manipulation_position_controllers
run ros2 pkg prefix joint_trajectory_controller_guard
```

### 控制器和接口

```bash
run ros2 control list_controllers
run ros2 control list_hardware_components
run ros2 control list_hardware_interfaces
run ros2 topic hz /joint_states --window 200
```

期望启动状态：

```text
joint_state_broadcaster  active
left_arm_jspc            inactive
left_arm_tskpc           inactive
left_arm_jtc             inactive
left_gripper_fwd         inactive
right_arm_jspc           inactive
right_arm_tskpc          inactive
right_arm_jtc            inactive
right_gripper_fwd        inactive
```

### 跨机 DDS

两端必须同时满足：

```text
ROS_DOMAIN_ID=1
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
两端 CycloneDDS XML 绑定各自正确网卡/IP
系统时间已同步
```

检查：

```bash
ros2 node list
ros2 topic info /joint_states -v
ros2 action info /execution/left_arm/follow_joint_trajectory
ros2 action info /execution/right_arm/follow_joint_trajectory
```

若更改 Domain ID 或 RMW，先停止旧 CLI daemon，避免缓存旧域：

```bash
ros2 daemon stop
```

## 15. 当前结论

RK3588S 已通过两只 candleLight USB-CAN 同时驱动双 Piper：稳定别名、双臂反馈、
DDS 命令、RViz、标准 `apps/teleop.py` 的 claim/接管/释放均已实测通过。当前使用
Python bridge，并由工作站 `/rk3588_piper/controller_manager` 过渡兼容层提供
Execution Manager 所需的服务契约；NUC 原生 `/controller_manager` 与其隔离。

在以下事项完成前，它还不是正式的 ros2_control RT 替代机：

1. 现有 Piper/custom controller 二进制不是 aarch64/OHOS ABI；
2. ros2_control 版本与现有 workstation RT 栈不一致；
3. 当前 launch 依赖板端不存在的 Bash；
4. 板端仍缺少 physical runtime 默认使用的 CycloneDDS RMW；
5. 当前内核不是 PREEMPT_RT，500 Hz 确定性尚未验证；
6. IPv4、HDC 端口和时间同步尚未做成开机持久服务。

推荐下一项工作是进入正式 C++ `ros2_control` overlay 开发，并补齐网络地址的
开机持久化、时间同步及生产 DDS 配置。

## 16. 参考文件

- M-Robots HDC：`/home/alpha/mrobot/docs/0-安装/02-HDC工具安装及使用.md`
- M-Robots ROS 2：`/home/alpha/mrobot/docs/2-ROS开发/ROS2快速入门.md`
- M-Robots RT 内核构建：`/home/alpha/mrobot/docs/0-安装/03-安装M-Robots环境.md`
- OpenHarmony 自启动：`/home/alpha/mrobot/docs/1-开发板和工具使用/03-自启服务配置.md`
- Piper RT bringup：`/home/alpha/physical_ai_runtime/src/bringup/piper_manipulation/rt_launch`
- Piper hardware interface：`/home/alpha/physical_ai_runtime/src/embodiments/robots/piper/piper_hardware_interface`
- Piper description：`/home/alpha/physical_ai_runtime/src/embodiments/robots/piper/piper_description`
- Workstation/RT profile：`/home/alpha/physical_ai_runtime/apps/profiles/piper_bimanual.yaml`
- CPU RT 配置：`/home/alpha/physical_ai_runtime/docs/CPU_HOST_SETUP.md`
- CAN udev 参考：`/home/alpha/physical_ai_runtime/docs/UDEV_HOST_SETUP.md`
