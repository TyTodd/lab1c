import rclpy
from rclpy.node import Node

from custom_msgs.msg import OpenSpace
from sensor_msgs.msg import LaserScan

import numpy as np



class SpacePublisher(Node):

    def __init__(self):
        super().__init__('open_space_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_topic', "open_space"),
                ('subscribe_topic', "fake_scan"),
            ]
        )

        self.publisher_ = self.create_publisher(OpenSpace, self.get_parameter('publish_topic').value, 10)
        # self.distance_publisher = self.create_publisher(Float32, 'open_space/distance', 10)
        # self.angle_publisher = self.create_publisher(Float32, 'open_space/angle', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            self.get_parameter('subscribe_topic').value,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        timer_period = 1/20 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.distance = 0.0
        self.angle = 0.0

    def listener_callback(self, msg):
        self.distance = max(msg.ranges)
        self.angle = np.argmax(msg.ranges) * msg.angle_increment + msg.angle_min

        self.get_logger().info('received msg: "%.2f"' % self.distance)
    
    def timer_callback(self):
        space_msg = OpenSpace()
        space_msg.distance = self.distance
        space_msg.angle = self.angle 
        self.publisher_.publish(space_msg)
        self.get_logger().info('Published msg: "%.2f"' % self.distance)

        



def main(args=None):
    rclpy.init(args=args)

    space_publisher = SpacePublisher()

    rclpy.spin(space_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    space_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()