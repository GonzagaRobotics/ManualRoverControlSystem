import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from std_msgs.msg import Float32

class GPSIMUSub(Node):

    def __init__(self):
        super().__init__('gps_imu_sub')
        self.gps_lat_topic = "gps/lat"
        self.gps_lng_topic = "gps/lng"
        self.imu_pitch_topic = "imu/pitch"
        self.imu_roll_topic = "imu/roll"
        self.imu_yaw_topic = "imu/yaw"

        self.gps_lat_sub = self.get_subscriber(self.gps_lat_topic)
        self.gps_lng_sub = self.get_subscriber(self.gps_lng_topic)
        self.imu_pitch_sub = self.get_subscriber(self.imu_pitch_topic)
        self.imu_roll_sub = self.get_subscriber(self.imu_roll_topic)
        self.imu_yaw_sub = self.get_subscriber(self.imu_yaw_topic)

    def get_subscriber(self, topic):
        topic_callback_lambda = lambda x : self.callback(x, topic) # x is the message
        return self.create_subscription(
            Float32,
            topic,
            topic_callback_lambda,
            10)

    def callback(self, msg, topic):
        self.get_logger().info(topic+': "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = GPSIMUSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
