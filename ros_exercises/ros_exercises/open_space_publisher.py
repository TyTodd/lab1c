import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

import numpy as np



class SpacePublisher(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        self.distance_publisher = self.create_publisher(Float32, 'open_space/distance', 10)
        self.angle_publisher = self.create_publisher(Float32, 'open_space/angle', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
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
        dist_msg = Float32() 
        angle_msg = Float32()
        dist_msg.data = self.distance
        angle_msg.data = self.angle 
        self.distance_publisher.publish(dist_msg)
        self.angle_publisher.publish(angle_msg)
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