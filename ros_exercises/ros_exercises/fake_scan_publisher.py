import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
import random
import math


class ScanPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_topic', "fake_scan"),
                ('publish_rate', 20),
                ('angle_min', -2/3 *math.pi),
                ('angle_max', 2/3 * math.pi),
                ('range_min', 1.0),
                ('range_max', 10.0),
                ('angle_increment', 1/300 * math.pi),
            ]
        )

        self.publisher_ = self.create_publisher(LaserScan, self.get_parameter('publish_topic').value, 10)
        self.timer_period = 1/ self.get_parameter('publish_rate').value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min =self.get_parameter('angle_min').value
        scan_msg.angle_max = self.get_parameter('angle_max').value
        scan_msg.angle_increment = self.get_parameter('angle_increment').value
        scan_msg.range_min = self.get_parameter('range_min').value
        scan_msg.range_max = self.get_parameter('range_max').value
        scan_msg.scan_time = self.timer_period
        
        scan_msg.ranges = [random.uniform(scan_msg.range_min, scan_msg.range_max) for _ in range(int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)+1)]

        self.publisher_.publish(scan_msg)
        self.get_logger().info('Published scan data.')


def main(args=None):
    rclpy.init(args=args)

    scan_publisher = ScanPublisher()

    rclpy.spin(scan_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()