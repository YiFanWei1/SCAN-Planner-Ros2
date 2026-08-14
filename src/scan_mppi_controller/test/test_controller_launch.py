import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    controller = launch_ros.actions.Node(
        package="scan_mppi_controller",
        executable="scan_mppi_controller_node",
        name="scan_mppi_controller_test",
        parameters=[{
            "controller_frequency": 20.0,
            "batch_size": 10,
            "min_batch_size": 10,
            "time_steps": 4,
            "visualization_rate": 0.0,
        }],
        remappings=[("cmd_vel_raw", "/scan_planner/cmd_vel_raw")],
        output="screen")
    gate = launch_ros.actions.Node(
        package="scan_planner",
        executable="cmd_vel_safety_gate",
        name="cmd_vel_safety_gate_test",
        parameters=[{
            "enable_motion_on_start": False,
            "require_planner_heartbeat": False,
            "publish_rate": 20.0,
        }],
        remappings=[
            ("cmd_vel_raw", "/scan_planner/cmd_vel_raw"),
            ("cmd_vel_safe", "/scan_planner/cmd_vel_safe_test"),
        ],
        output="screen")
    return launch.LaunchDescription([
        controller,
        gate,
        launch_testing.actions.ReadyToTest(),
    ]), {"controller": controller, "gate": gate}


class TestControllerWiring(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("scan_mppi_wiring_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail("ROS graph did not reach the expected state")

    def test_single_raw_command_source_and_safety_gate(self):
        raw = "/scan_planner/cmd_vel_raw"
        safe = "/scan_planner/cmd_vel_safe_test"
        controller_publishers = lambda: [
            endpoint for endpoint in self.node.get_publishers_info_by_topic(raw)
            if endpoint.node_name == "scan_mppi_controller_test"]
        gate_subscriptions = lambda: [
            endpoint for endpoint in self.node.get_subscriptions_info_by_topic(raw)
            if endpoint.node_name == "cmd_vel_safety_gate_test"]
        gate_publishers = lambda: [
            endpoint for endpoint in self.node.get_publishers_info_by_topic(safe)
            if endpoint.node_name == "cmd_vel_safety_gate_test"]
        self._wait_for(lambda: len(controller_publishers()) == 1)
        self._wait_for(lambda: len(gate_subscriptions()) == 1)
        self._wait_for(lambda: len(gate_publishers()) == 1)
        self.assertEqual(len(controller_publishers()), 1)
        self.assertEqual(len(gate_subscriptions()), 1)
        self.assertEqual(len(gate_publishers()), 1)


@launch_testing.post_shutdown_test()
class TestProcessesExit(unittest.TestCase):

    def test_exit_codes(self, proc_info, controller, gate):
        launch_testing.asserts.assertExitCodes(proc_info, process=controller)
        launch_testing.asserts.assertExitCodes(proc_info, process=gate)
