import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_sub')
        self.subscription = self.create_subscription(
            String,
            'gps_coords',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard from GPS: "%s"' % msg.data)

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_sub')
        self.subscription = self.create_subscription(
            String,
            'imu_vals',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard from IMU: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    gps_node = GPSNode()
    imu_node = IMUNode()

    try:
        while True:
         rclpy.spin_once(gps_node, timeout_sec=0.01)
         rclpy.spin_once(imu_node, timeout_sec=0.01)
         time.sleep(1)
    except KeyboardInterrupt:
        pass

    gps_node.destroy_node()
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
