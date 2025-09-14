import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MVSimController(Node):
    SLOW_SPEED = 0.5
    FULL_SPEED = 1.0
    CMD_PUBLISH_PERIOD = 0.5

    def __init__(self):
        super().__init__('mvsim_controller')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = self.CMD_PUBLISH_PERIOD

        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.present_speed_ = 0.0
        self.estopped_ = False

        self.subscription_ = self.create_subscription(
            String,
            '/analog/speed_state',
            self.speed_state_callback,
            10)

    def timer_callback(self):
        if self.estopped_:
            return

        msg = Twist()
        msg.linear.x = self.present_speed_
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def speed_state_callback(self, msg):
        if (msg.data == "ESTOPPED"):
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.present_speed_ = 0.0
            self.estopped_ = True
            return

        self.estopped_ = False
        if (msg.data == "STOP"):
            self.present_speed_ = 0.0
        elif (msg.data == "SLOW"):
            self.present_speed_ = self.SLOW_SPEED
        elif (msg.data == "FULL_SPEED"):
            self.present_speed_ = self.FULL_SPEED


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MVSimController()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
