from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parent


class DualContractTests(unittest.TestCase):
    def test_physical_runtime_site_mapping_is_fixed(self) -> None:
        start = (ROOT / "board_start_piper.sh").read_text()
        self.assertIn("left:piper1|right:piper0", start)
        self.assertIn('--can-interface "${CAN_INTERFACE}"', start)

    def test_dual_start_is_staggered_and_fails_closed(self) -> None:
        start = (ROOT / "board_start_dual.sh").read_text()
        self.assertIn("STARTUP_STAGGER_S", start)
        self.assertIn('kill -0 "${left_pid}"', start)
        self.assertIn('kill -0 "${right_pid}"', start)
        self.assertIn("stopping piper0", start)
        self.assertIn("stopping piper1", start)

    def test_remote_leader_wrapper_separates_service_and_data_rmw(self) -> None:
        wrapper = (ROOT / "host_start_leader_remote.sh").read_text()
        self.assertIn("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", wrapper)
        self.assertIn("RMW_IMPLEMENTATION=rmw_fastrtps_cpp", wrapper)
        self.assertIn("set_leader_preempt false", wrapper)
        self.assertIn("leader_remote_teleop.py", wrapper)
        self.assertIn("leader_preempt_control.py", wrapper)
        self.assertIn("engaged=1\nset_leader_preempt true", wrapper)
        self.assertIn('sleep "${duration}"', wrapper)

    def test_host_board_wrapper_uses_network_hdc_and_strict_dual_can(self) -> None:
        wrapper = (ROOT / "host_start_board_dual.sh").read_text()
        self.assertIn("192.168.1.19:8710", wrapper)
        self.assertIn("/data/local/piper_dual_bringup", wrapper)
        self.assertIn("REQUIRE_BOTH=1", wrapper)
        self.assertIn("PIPER_ENABLE_ACTUATION", wrapper)
        self.assertIn("expected two piper bridges", wrapper)
        self.assertIn("controller_manager_compat.py", wrapper)
        self.assertIn("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", wrapper)
        self.assertIn("/rk3588_piper/controller_manager", wrapper)

        stop = (ROOT / "host_stop_board_dual.sh").read_text()
        self.assertIn("[c]ontroller_manager_compat.py", stop)
        self.assertIn("kill -INT", stop)
        self.assertIn("safe to start the NUC RT backend", stop)

    def test_controller_compat_is_ready_gated_and_explicitly_transitional(self) -> None:
        compat = (ROOT / "controller_manager_compat.py").read_text()
        self.assertIn('/list_controllers', compat)
        self.assertIn('/switch_controller', compat)
        self.assertIn("STATUS_TIMEOUT_S", compat)
        self.assertIn("_side_ready", compat)
        self.assertIn("this is not a ros2_control controller_manager", compat)
        self.assertIn("gripper_ready", compat)

    def test_rk3588_execution_manager_config_is_namespaced(self) -> None:
        config = (ROOT / "config" / "execution_manager_rk3588.yaml").read_text()
        self.assertEqual(config.count("controller_manager: /rk3588_piper/controller_manager"), 4)
        self.assertNotIn("controller_manager: /controller_manager\n", config)

    def test_board_bridge_implements_gripper_contract(self) -> None:
        bridge = (ROOT / "piper_bridge.py").read_text()
        self.assertIn("Float64MultiArray", bridge)
        self.assertIn('/execution/{side}_gripper/joint_reference', bridge)
        self.assertIn("GetArmGripperMsgs", bridge)
        self.assertIn("GripperCtrl", bridge)
        self.assertIn("gripper_joint1", bridge)
        demo = (ROOT / "send_gripper_motion.py").read_text()
        self.assertIn("Float64MultiArray", demo)
        self.assertIn("gripper feedback moved only", demo)

    def test_can_mapping_is_serial_based(self) -> None:
        prepare = (ROOT / "board_prepare_piper_can.sh").read_text()
        self.assertIn("find_interface_by_serial", prepare)
        self.assertIn("report_usb_can_inventory", prepare)
        self.assertIn("/sys/bus/usb/devices", prepare)
        self.assertIn("kernel/artifacts/${KERNEL_RELEASE}", prepare)
        self.assertIn("PIPER1_USB_SERIAL", prepare)
        self.assertIn("PIPER0_USB_SERIAL", prepare)
        self.assertNotIn("prepare_one can1", prepare)

    def test_current_piper1_serial_is_recorded(self) -> None:
        config = (ROOT / "config" / "piper_can_serials.env").read_text()
        self.assertIn("PIPER1_USB_SERIAL=004400424148570D20343133", config)

    def test_current_piper0_serial_is_recorded(self) -> None:
        config = (ROOT / "config" / "piper_can_serials.env").read_text()
        self.assertIn("PIPER0_USB_SERIAL=004500424148570C20343133", config)

    def test_remote_rviz_has_no_fake_joint_state_publisher(self) -> None:
        launch = (ROOT / "remote_bimanual_rviz.launch.py").read_text()
        self.assertNotIn('package="joint_state_publisher"', launch)
        self.assertNotIn('package="joint_state_publisher_gui"', launch)
        self.assertIn('package="robot_state_publisher"', launch)


if __name__ == "__main__":
    unittest.main()
