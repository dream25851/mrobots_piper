# RK3588S 双 Piper bring-up

主使用说明见仓库根目录 [README](../README.md)。本目录只保存板端 bridge、CAN
准备脚本、本机启动/兼容入口和测试。

## 固定映射

```text
left  -> piper1 -> USB serial 004400424148570D20343133
right -> piper0 -> USB serial 004500424148570C20343133
本机 Leader: left=can0, right=can1
```

板载 SoC `can0` 保持 DOWN，不用于 Piper。两只 USB-CAN 均使用 1 Mbps；插在
USB 2.0 或 USB 3.0 物理口不会改变 CAN 带宽，主要关注 Hub 供电和线缆稳定性。

## 工作站入口

```bash
/home/alpha/mrobot/piper_dual_bringup/host_start_board_dual.sh
```

它通过 `192.168.1.19:8710` 执行严格双 CAN 检查、启动板端双 bridge，并启动
本机 `controller_manager_compat.py`。重复运行是幂等的，只会复用健康进程。

`/data/local/piper_dual_bringup/board_start_dual.sh` 是板端路径，不能直接在
`alpha@alpha` 的本机 shell 中运行。

## 板端入口

仅在 HDC shell/板端终端执行：

```sh
REQUIRE_BOTH=1 /data/local/piper_dual_bringup/board_prepare_piper_can.sh
PIPER_ENABLE_ACTUATION=1 /data/local/piper_dual_bringup/board_start_dual.sh
```

固定 ROS 接口：

```text
/execution/left_arm/joint_reference  -> piper1
/execution/right_arm/joint_reference -> piper0
/joint_states                         -> 工作站/RViz
/piper1/status、/piper0/status         -> ready/fault/feedback
```

## 过渡 controller-manager 兼容层

`controller_manager_compat.py` 在工作站运行，只提供 Execution Manager claim 所需的：

```text
/controller_manager/list_controllers
/controller_manager/switch_controller
```

只有对应 bridge 的 status 在 1 秒内更新且满足 `actuate=true、ready=true、fault=""`
时才接受激活。它不拥有 CAN、不绕过板端保护，也不是完整 ros2_control。

## 安全与验证

- 保留硬关节限位、200 ms 反馈超时、fault 检查和 300 ms 命令 watchdog。
- 已取消绝对目标相对反馈的单步角度差拒绝条件。
- 左右 J1 均完成超过旧门槛的 `0.08 rad` 往返，之后 CAN 错误计数仍为 0。
- 标准 `apps/teleop.py` 已验证双臂 claim、ACTIVE、RELEASE 和 Ctrl+C 清理。

```bash
python3 -m unittest -v test_safety.py test_dual_contract.py
```
