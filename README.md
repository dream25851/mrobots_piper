# mrobots_piper

让 RK3588S 开发板通过两路 USB-CAN 驱动两台 Piper follower，并接入
`physical_ai_runtime` 的 Execution Manager、`apps/teleop.py` 和 RViz2。

## 当前状态

| 侧别 | 开发板逻辑 CAN | USB-CAN 序列号 | 本机 Leader |
|---|---|---|---|
| left | `piper1` | `004400424148570D20343133` | `can0` |
| right | `piper0` | `004500424148570C20343133` | `can1` |

- RK3588S：`192.168.1.19/24`，工作站：`192.168.1.18/24`。
- ROS 2 Jazzy，`ROS_DOMAIN_ID=1`。
- 两路 CAN 均为 1 Mbps，已完成双臂 DDS 遥操和 RViz 验证。
- `apps/teleop.py` 的双臂 claim、0-G 接管、释放和 Ctrl+C 清理已实机通过。

## 架构

```text
工作站
  Piper Leader(can0/can1) -> apps/teleop.py -> Execution Manager
                                      -> /execution/*/joint_reference
                                      -> controller-manager 过渡兼容层
                                                       |
                                                       | DDS
                                                       v
RK3588S
  piper_bridge(left/piper1)  -> USB-CAN -> left follower
  piper_bridge(right/piper0) -> USB-CAN -> right follower
  /joint_states、/piper1/status、/piper0/status -> 工作站/RViz
```

过渡兼容层只实现 Execution Manager 需要的 `list_controllers` 和
`switch_controller`，并以板端新鲜、无 fault 的 ready 状态作为激活条件。它不是
完整 ros2_control；正式 C++ ros2_control 上板后应删除该层。

## 启动

开发板和工作站均接好后，在工作站执行：

```bash
/home/alpha/mrobot/piper_dual_bringup/host_start_board_dual.sh
```

该命令会：

1. 通过网络 HDC 连接 `192.168.1.19:8710`；
2. 按 USB 序列号绑定并检查 `piper1/piper0`；
3. 启动或复用板端两个 bridge；
4. 启动本机 controller-manager 过渡兼容层。

然后启动 `physical_ai_runtime` 工作站栈：

```bash
cd /home/alpha/physical_ai_runtime
pixi run -e runtime bash -c \
  'source install/setup.bash && ros2 launch piper_manipulation_workstation_launch workstation_stack.launch.py'
```

另开终端启动标准遥操客户端：

```bash
cd /home/alpha/physical_ai_runtime
pixi run -e runtime python apps/teleop.py
```

按 Enter 接管，再按 Enter 释放；ACTIVE 时按 Ctrl+C 也会先释放两台 Leader。

首次接入本仓库时，在 `physical_ai_runtime` 根目录应用现场映射与 Ctrl+C 修复：

```bash
git apply /home/alpha/mrobot/patches/physical_ai_runtime.patch
pixi run -e runtime bash -c \
  'source install/setup.bash && colcon build --symlink-install --packages-select piper_manipulation_workstation_launch'
```

## 检查

```bash
# 板端双臂和本机兼容服务（可重复执行）
/home/alpha/mrobot/piper_dual_bringup/host_start_board_dual.sh

# 兼容控制器状态
cd /home/alpha/physical_ai_runtime
pixi run -e runtime ros2 control list_controllers \
  --controller-manager /controller_manager

# 远端反馈
pixi run -e runtime ros2 topic echo /joint_states --once
pixi run -e runtime ros2 topic echo /piper1/status --once
pixi run -e runtime ros2 topic echo /piper0/status --once
```

## 安全边界

- 目标必须处于 Piper 硬关节限位内；不再使用旧的单次相对反馈步长拒绝条件。
- 反馈超过 200 ms、机械臂 fault 或 bridge 未 ready 时停止写命令。
- 命令 300 ms 超时后保持最新反馈姿态。
- CAN 映射只依据 USB 序列号，不依据临时 `can1/can2` 枚举顺序。
- 停止 bridge 不自动失能，避免机械臂突然下坠。

标准 `apps/teleop.py` 按绝对关节目标工作。Leader 与 follower 初始姿态差异较大时，
接管瞬间会向 Leader 姿态运动；调试零偏移相对遥操时可使用
`piper_dual_bringup/host_start_leader_remote.sh`。

## 已知限制

- 当前是 Python 200 Hz bridge，不是硬实时 ros2_control。
- 板端尚无原生 C++ `controller_manager`、Piper hardware plugin 和 route controllers。
- 当前 bridge 只驱动六轴机械臂，夹爪 controller 可被 Execution Manager 仲裁，但板端尚未执行夹爪命令。
- 开发板 IPv4/HDC 8710 尚未安装持久化开机服务。

## 测试

```bash
cd /home/alpha/mrobot/piper_dual_bringup
python3 -m unittest -v test_safety.py test_dual_contract.py
```

更多实现与移植背景见 [RK3588S 双 Piper 开发指南](RK3588S双Piper机械臂RT开发指南.md)。
