#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Float32
import logging

class CommandReciever(Node): 
    '''
    /motor_command
    /motor_command/dpad_lr
    /motor_command/left_shoulder
    /motor_command/left_trigger
    /motor_command/right_shoulder
    /motor_command/right_trigger
    '''
    def __init__(self):
        super().__init__("CommandReciever")
        self.manual_control = True
        self.get_logger().set_level(logging.INFO)
        self.get_logger().info('I have initialized up successfully.')


        self.lt_subscriber = self.create_subscription(
                Float32,
                'motor_command_exposed/left_trigger',
                self.callback,
                10,
                callback_args = ["motor_command_exposed/left_trigger"]
                )
        
        self.rt_subscriber = self.create_subscription(
                Float32,
                'motor_command_exposed/right_trigger',
                self.callback,
                10,
                callback_args = ["motor_command_exposed/right_trigger"]
                )
        
        self.lb_subscriber = self.create_subscription(
                Float32,
                'motor_command_exposed/left_shoulder',
                self.callback,
                10,
                callback_args = ["motor_command_exposed/left_shoulder"]
                )
        
        self.rb_subscriber = self.create_subscription(
                Float32,
                'motor_command_exposed/right_shoulder',
                self.callback,
                10,
                callback_args = ["motor_command_exposed/right_shoulder"]
                )
        
        self.dpad_lr_subscriber = self.create_subscription(
                Float32,
                'motor_command_exposed/dpad_lr',
                self.callback,
                10,
                callback_args = ["motor_command_exposed/dpad_lr"]
                )
    
        # self.pause_subscriber = self.create_subscription(
        #         Float32,
        #         'motor_command_exposed/pause',
        #         self.callback,
        #         10
        #         )






    def callback(self, msg, topic=None):
        if self.manual_control:
            self.get_logger().info(str(topic) if topic is not None else "None")

        # self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = CommandReciever()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
