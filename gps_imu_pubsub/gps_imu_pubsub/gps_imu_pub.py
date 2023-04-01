import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(String, 'gps_coords', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(String, 'imu_vals', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

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
