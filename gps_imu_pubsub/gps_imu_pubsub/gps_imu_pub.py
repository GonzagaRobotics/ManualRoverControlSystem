import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
import time

class GPSIMUPub(Node):

    def __init__(self):
        super().__init__('gps_imu_pub')
        self.gps_lat_topic = "gps/lat"
        self.gps_lng_topic = "gps/lng"
        self.imu_pitch_topic = "imu/pitch"
        self.imu_roll_topic = "imu/roll"
        self.imu_yaw_topic = "imu/yaw"

        self.gps_lat_pub = self.get_publisher(self.gps_lat_topic)
        self.gps_lng_pub = self.get_publisher(self.gps_lng_topic)
        self.imu_pitch_pub = self.get_publisher(self.imu_pitch_topic)
        self.imu_roll_pub = self.get_publisher(self.imu_roll_topic)
        self.imu_yaw_pub = self.get_publisher(self.imu_yaw_topic)

        '''
        self.publisher_ = self.create_publisher(String, gps_lat, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        '''
    def get_publisher(self, topic):
        topic_callback_lambda = lambda x : self.callback(x, topic)
        return self.create_publisher(Float32, topic, 10)

    def timer_callback(self):
        msg = Float32()
        msg.data = 3.14
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = GPSIMUPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
