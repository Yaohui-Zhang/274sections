#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import Int64, Bool, String
from geometry_msgs.msg import Twist


class Heartbeat(Node):
    def __init__(self) -> None:
        super().__init__("heartbeat")

        self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.hb_timer = self.create_timer(0.2, self.hb_callback)
        #self.hb_counter = 0

        self.motor_sub = self.create_subscription(Bool, "/kill",
                                                  self.health_callback, 10)
        # self.imu_sub = self.create_subscription(Bool, "/health/imu",
        #                                         self.health_callback, 10)
        # self.lidar_sub = self.create_subscription(Bool, "/health/lidar",
        #                                           self.health_callback, 10)

    def hb_callback(self) -> None:
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.75
        # msg.data = 'sending constant control...'
        # print('sending constant control...')
        self.hb_pub.publish(msg)
        

    def health_callback(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().fatal("Heartbeat stopped")
            self.hb_timer.cancel()
            zero_msg = Twist()
            zero_msg.linear.x = 0.0
            zero_msg.angular.z = 0.0
            self.hb_pub.publish(zero_msg)


if __name__ == "__main__":
    rclpy.init()
    node = Heartbeat()
    rclpy.spin(node)
    rclpy.shutdown()
