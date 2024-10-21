#!/usr/bin/env python3

import numpy 
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

class HeadingController(BaseHeadingController):
    def __init__(self):
        super(HeadingController, self).__init__()
        self.kp = 2.0
        
    def compute_control_with_goal(
        self, 
        current_state: TurtleBotState, 
        desired_state: TurtleBotState
    ) -> TurtleBotControl:
        """
        Args:
            current_state (TurtleBotState)
            desired_state (TurtleBotState)

        Returns:
            control_message (TurtleBotControl)
        """
        
        heading_error = wrap_angle(desired_state.theta - current_state.theta)
        omega = self.kp * heading_error
        control_message = TurtleBotControl()
        control_message.omega = omega
        return control_message
    
if __name__ == "__main__":
    rclpy.init()
    node = HeadingController()
    rclpy.spin(node)
    rclpy.shutdown()
        