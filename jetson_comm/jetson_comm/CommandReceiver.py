#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
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
        self.get_logger().set_level(logging.INFO)
        self.get_logger().info('I have initialized up successfully.')

        self.declare_parameter('manual_control', True)
        self.manual_control = self.get_parameter('manual_control').value
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.lt_topic = 'motor_command_exposed/left_trigger'
        self.rt_topic = 'motor_command_exposed/right_trigger'
        self.lb_topic = 'motor_command_exposed/left_shoulder'
        self.rb_topic = 'motor_command_exposed/right_shoulder'
        self.dpad_lr_topic = 'motor_command_exposed/dpad_lr'
        self.pause_topic = 'motor_command_exposed/pause'

        self.lt_subscriber =self.get_xbox_subscriber(self.lt_topic)
        self.rt_subscriber = self.get_xbox_subscriber(self.rt_topic)
        self.lb_subscriber = self.get_xbox_subscriber(self.lb_topic)
        self.rb_subscriber = self.get_xbox_subscriber(self.rb_topic)
        self.dpad_lr_subscriber = self.get_xbox_subscriber(self.dpad_lr_topic) 
        self.pause_subscriber = self.get_xbox_subscriber(self.pause_topic)

        self.publishers_dict = {
                'motor_command_exposed/left_trigger': self.create_publisher(Float32, 'motor_control/left_trigger', 10),
                'motor_command_exposed/right_trigger': self.create_publisher(Float32, 'motor_control/right_trigger', 10),
                'motor_command_exposed/left_shoulder': self.create_publisher(Float32, 'motor_control/left_shoulder', 10),
                'motor_command_exposed/right_shoulder': self.create_publisher(Float32, 'motor_control/right_shoulder', 10),
                'motor_command_exposed/dpad_lr': self.create_publisher(Float32, 'motor_control/dpad_lr', 10),
                'motor_command_exposed/pause': self.create_publisher(Float32, 'motor_control/pause', 10)
                }

    def parameters_callback(self, params):
        self.manual_control = self.get_parameter('manual_control').value
        return SetParametersResult(successful=True)
        

    def get_xbox_subscriber(self, topic):
        topic_callback_lambda = lambda x: self.callback(x, topic)
        return self.create_subscription(
                    Float32,
                    topic,
                    topic_callback_lambda,
                    10
                    )

    def callback(self, msg, topic):
        self.get_logger().info(topic)
        self.get_logger().info(type(msg.data))
        if topic == 'motor_command_exposed/pause' and msg.data == 1.:
            self.get_logger().info("toggle pause?")
            manual_ctrl_current = self.get_parameter('manual_control').value
            man_ctrl = Parameter('manual_control', Parameter.Type.BOOLEAN, not manual_ctrl_current)
            self.set_parameters([man_ctrl])

        if self.manual_control:
            if topic is not None:
                self.publishers_dict[topic].publish(msg)
                self.get_logger().info("Publishing:'%s'" % msg.data)
                self.get_logger().info("manual control state: '%s'" % self.manual_control)
            else:
                self.get_logger().info(str("None"))
                self.get_logger().info(str(msg.data))
        # else:
        #     self.get_logger().info(str("You are not in manual control mode"))

        # self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = CommandReciever()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
