#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Float32MultiArray
import logging

class CommandReciever(Node): 
    def __init__(self):
        self.get_logger().setLevel(logging.INFO)
        super().__init__("CommandReciever") 


        self.command_subscriber = self.create_subscription(
                Float32MultiArray,
                'motor_command_exposed',
                self.callback,
                QoSPresetProfiles.ACTION_STATUS_DEFAULT
                )


    def callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = CommandReciever() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
