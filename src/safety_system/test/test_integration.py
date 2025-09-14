import rclpy
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import launch_testing.actions
from std_msgs.msg import String
import unittest
import time

def generate_test_description():
    node_speed_limiter = Node(
            package='speed_limiter',
            executable='node_speed_limiter',
            parameters=[
                {'stop_boundary': 400.0},
                {'slow_boundary': 800.0},
                {'hysteresis': 10.0}
            ]
        ),

    timer_speed_limiter = TimerAction(
        period = 3.0,
        actions = node_speed_limiter,
    )

    node_proximity_sensor = Node(
            package='proximity_sensor',
            executable='node_proximity_sensor',
            parameters=[
                {'sample_time_ms': 10},
                {'distance_series': [
                    810.0, # FULL_SPEED
                    809.0,
                    400.0, # STOP
                    401.0,
                    800.0, # SLOW
                    801.0,
                    400.0, # STOP
                    850.0, # FULL_SPEED
                    450.0, # SLOW
                    200.0, # STOP
                    1000.0 # FULL_SPEED
                ]}
            ]
        ),

    timer_proximity_sensor = TimerAction(
        period = 6.0,
        actions = node_proximity_sensor 
    )

    return LaunchDescription([
        launch_testing.actions.ReadyToTest(),
        Node(
            package='estop_monitor',
            executable='node_estop_monitor',
            parameters=[
                {'trigger_time_ms': 8000}
            ]
        ),
        timer_speed_limiter,
        timer_proximity_sensor,
    ])

class TestSimulatedSafetySystem(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_integration')

    def tearDown(self):
        self.node.destroy_node()

    def test_correct_speed_state_order(self, proc_output):
        """Check whether speed state changes in the expected order"""
        msgs_rx = []

        def append_and_log(msg):
            msgs_rx.append(msg)
            self.node.get_logger().info(f'Got message: {msg}')

        sub = self.node.create_subscription(
            String, 'analog/speed_state', append_and_log, 10)
        try:
            # Listen to the analog/speed_state topic for few seconds
            end_time = time.time() + 11
            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)
                
            assert msgs_rx[0].data == "ESTOPPED"
            assert msgs_rx[1].data == "STOP"
            assert msgs_rx[2].data == "FULL_SPEED"
            assert msgs_rx[3].data == "STOP"
            assert msgs_rx[4].data == "SLOW"
            assert msgs_rx[5].data == "STOP"
            assert msgs_rx[6].data == "FULL_SPEED"
            assert msgs_rx[7].data == "SLOW"
            assert msgs_rx[8].data == "STOP"
            assert msgs_rx[9].data == "FULL_SPEED"
            assert msgs_rx[10].data == "ESTOPPED"

            assert len(msgs_rx) == 11
        finally:
            self.node.destroy_subscription(sub)
