import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import ParameterEvent
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

class AlignedDepthToColorCameraInfoSubscriber(Node):
    def __init__(self):
        super().__init__('aligned_depth_to_color_camera_info_subscriber')
        self.create_subscription(CameraInfo, '/camera/aligned_depth_to_color/camera_info', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Aligned Depth to Color Camera Info')

class AlignedDepthToColorImageRawSubscriber(Node):
    def __init__(self):
        super().__init__('aligned_depth_to_color_image_raw_subscriber')
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Aligned Depth to Color Image Raw')

class ColorCameraInfoSubscriber(Node):
    def __init__(self):
        super().__init__('color_camera_info_subscriber')
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Color Camera Info')

class ColorImageRawSubscriber(Node):
    def __init__(self):
        super().__init__('color_image_raw_subscriber')
        self.create_subscription(Image, '/camera/color/image_raw', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Color Image Raw')

class ColorMetadataSubscriber(Node):
    def __init__(self):
        super().__init__('color_metadata_subscriber')
        self.create_subscription(String, '/camera/color/metadata', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Color Metadata')

class DepthCameraInfoSubscriber(Node):
    def __init__(self):
        super().__init__('depth_camera_info_subscriber')
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Depth Camera Info')

class DepthColorPointsSubscriber(Node):
    def __init__(self):
        super().__init__('depth_color_points_subscriber')
        self.create_subscription(PointCloud2, '/camera/depth/color/points', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Depth Color Points')

class DepthImageRectRawSubscriber(Node):
    def __init__(self):
        super().__init__('depth_image_rect_raw_subscriber')
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Depth Image Rect Raw')

class DepthMetadataSubscriber(Node):
    def __init__(self):
        super().__init__('depth_metadata_subscriber')
        self.create_subscription(String, '/camera/depth/metadata', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Depth Metadata')

class ExtrinsicsDepthToColorSubscriber(Node):
    def __init__(self):
        super().__init__('extrinsics_depth_to_color_subscriber')
        self.create_subscription(String, '/camera/extrinsics/depth_to_color', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Extrinsics Depth To Color')

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.create_subscription(String, '/camera/imu', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received IMU Data')

class DiagnosticsSubscriber(Node):
    def __init__(self):
        super().__init__('diagnostics_subscriber')
        self.create_subscription(DiagnosticArray, '/diagnostics', self.listener_callback, 10)

    def listener_callback(self,msg):
        self.get_logger().info('Received Diagnostics Data')

def main(args=None):
    rclpy.init(args=args)
    aligned_depth_to_color_camera_info_subscriber = AlignedDepthToColorCameraInfoSubscriber()
    aligned_depth_to_color_image_raw_subscriber = AlignedDepthToColorImageRawSubscriber()
    color_camera_info_subscriber = ColorCameraInfoSubscriber()
    color_image_raw_subscriber = ColorImageRawSubscriber()
    color_metadata_subscriber = ColorMetadataSubscriber()
    depth_camera_info_subscriber = DepthCameraInfoSubscriber()
    depth_color_points_subscriber = DepthColorPointsSubscriber()
    depth_image_rect_raw_subscriber = DepthImageRectRawSubscriber()
    depth_metadata_subscriber = DepthMetadataSubscriber()
    extrinsics_depth_to_color_subscriber = ExtrinsicsDepthToColorSubscriber()
    imu_subscriber = ImuSubscriber()
    diagnostics_subscriber = DiagnosticsSubscriber()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(aligned_depth_to_color_camera_info_subscriber)
    executor.add_node(aligned_depth_to_color_image_raw_subscriber)
    executor.add_node(color_camera_info_subscriber)
    executor.add_node(color_image_raw_subscriber)
    executor.add_node(color_metadata_subscriber)
    executor.add_node(depth_camera_info_subscriber)
    executor.add_node(depth_color_points_subscriber)
    executor.add_node(depth_image_rect_raw_subscriber)
    executor.add_node(depth_metadata_subscriber)
    executor.add_node(extrinsics_depth_to_color_subscriber)
    executor.add_node(imu_subscriber)
    executor.add_node(diagnostics_subscriber)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        aligned_depth_to_color_camera_info_subscriber.destroy_node()
        aligned_depth_to_color_image_raw_subscriber.destroy_node()
        color_camera_info_subscriber.destroy_node()
        color_image_raw_subscriber.destroy_node()
        color_metadata_subscriber.destroy_node()
        depth_camera_info_subscriber.destroy_node()
        depth_color_points_subscriber.destroy_node()
        depth_image_rect_raw_subscriber.destroy_node()
        depth_metadata_subscriber.destroy_node()
        extrinsics_depth_to_color_subscriber.destroy_node()
        imu_subscriber.destroy_node()
        diagnostics_subscriber.destroy_node()
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()