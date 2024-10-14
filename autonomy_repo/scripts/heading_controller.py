#!/usr/bin/env python3

import rclpy
import numpy as np

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl,TurtleBotState


class HeadingController(BaseHeadingController):
    def __init__(self) -> None:
        super().__init__("HeadingController")
        self.kp=2.0
	
    def compute_control_with_goal(self,state: TurtleBotState,goal: TurtleBotState) -> TurtleBotControl:
    	print("here")
    	heading_err=wrap_angle(goal.theta-state.theta)
    	angular_velocity=self.kp*heading_err
    	control_msg = TurtleBotControl()
    	control_msg.omega=angular_velocity
    	return control_msg


if __name__ == "__main__":
    rclpy.init()
    node = HeadingController()
    rclpy.spin(node)
    rclpy.shutdown()
