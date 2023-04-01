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
    /motor_command/pause
    /motor_command/right_shoulder
    /motor_command/right_trigger
    '''
    def __init__(self):
        super().__init__("CommandReciever")
        self.manual_control = True
        self.get_logger().set_level(logging.INFO)
        self.get_logger().info('I have initialized up successfully.')

        self.callback_dict = {
                'motor_command_exposed/left_trigger': self.lt_callback,
                'motor_command_exposed/right_trigger': self.rt_callback,
                'motor_command_exposed/left_shoulder': self.lb_callback,
                'motor_command_exposed/right_shoulder': self.rb_callback,
                'motor_command_exposed/dpad_lr': self.dpad_lr_callback,
                'motor_command_exposed/pause': self.pause_callback
                }

        topic = 'motor_command_exposed/left_trigger'
        self.lt_topic = topic
        self.lt_subscriber =self.get_xbox_subscriber(topic)
        
        topic = 'motor_command_exposed/right_trigger'
        self.rt_topic = topic
        self.rt_subscriber = self.get_xbox_subscriber(topic)
        
        topic = 'motor_command_exposed/left_shoulder'
        self.lb_topic = topic
        self.lb_subscriber = self.get_xbox_subscriber(topic)
        
        topic = 'motor_command_exposed/right_shoulder'
        self.rb_topic = topic
        self.rb_subscriber = self.get_xbox_subscriber(topic)
        
        topic = 'motor_command_exposed/dpad_lr'
        self.dpad_lr_topic = topic
        self.dpad_lr_subscriber = self.get_xbox_subscriber(topic) 
    
        topic = 'motor_command_exposed/pause'
        self.pause_topic = topic
        self.pause_subscriber = self.get_xbox_subscriber(topic)

        self.publishers_dict = {
                'motor_command_exposed/left_trigger': self.create_publisher(Float32, 'motor_control/left_trigger', 10),
                'motor_command_exposed/right_trigger': self.create_publisher(Float32, 'motor_control/right_trigger', 10),
                'motor_command_exposed/left_shoulder': self.create_publisher(Float32, 'motor_control/left_shoulder', 10),
                'motor_command_exposed/right_shoulder': self.create_publisher(Float32, 'motor_control/right_shoulder', 10),
                'motor_command_exposed/dpad_lr': self.create_publisher(Float32, 'motor_control/dpad_lr_publisher', 10),
                'motor_command_exposed/pause': self.create_publisher(Float32, 'motor_control/pause', 10)
                }


    def get_xbox_subscriber(self, topic):
        return self.create_subscription(
                    Float32,
                    topic,
                    self.callback_dict[topic],
                    10
                    )

    def lt_callback(self, msg):
        self.callback(msg, self.lt_topic)

    def rt_callback(self, msg):
        self.callback(msg, self.rt_topic)

    def lb_callback(self, msg):
        self.callback(msg, self.lb_topic)

    def rb_callback(self, msg):
        self.callback(msg, self.rb_topic)

    def dpad_lr_callback(self, msg):
        self.callback(msg, self.dpad_lr_topic)

    def paue_callback(self, msg):
        self.callback(msg, self.pause_topic)

    def callback(self, msg, topic):
        if self.manual_control:
            if topic is not None:
                self.publishers_dict[topic].publish(msg)
                self.get_logger().info("Publishing:'%s'" % msg.data)
            else:
                self.get_logger().info(str("None"))
                self.get_logger().info(str(msg.data))

        # self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = CommandReciever()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
