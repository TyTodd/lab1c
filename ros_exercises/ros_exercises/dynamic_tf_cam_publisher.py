import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

class CamPublisher(Node):

    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # callback function on each message
        # self.subscription = self.create_subscription(
        #     TransformBroadcaster,
        #     'tf',
        #     self.listener_callback,
        #     1)
        # self.subscription  # prevent unused variable warning
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(1/20, self.callback)
    
    def tf_to_se3(self, transform):
        q = transform.rotation
        q = [q.x, q.y, q.z, q.w]
        t = transform.translation
        mat = tf_transformations.quaternion_matrix(q)
        mat[0, 3] = t.x
        mat[1, 3] = t.y
        mat[2, 3] = t.z
        return mat
    
    def se3_to_tf(self, mat, time, parent, child):
        obj = TransformStamped()
        obj.header.stamp = time.to_msg()
        obj.header.frame_id = parent
        obj.child_frame_id = child
        obj.transform.translation.x = mat[0, 3]
        obj.transform.translation.y = mat[1, 3]
        obj.transform.translation.z = mat[2, 3]
        q = tf_transformations.quaternion_from_matrix(mat)
        obj.transform.rotation.x = q[0]
        obj.transform.rotation.y = q[1]
        obj.transform.rotation.z = q[2]
        obj.transform.rotation.w = q[3]
        return obj

    def callback(self):
        try:
            t = self.tf_buffer.lookup_transform('world','base_link_gt',rclpy.time.Time())
            robot_to_world = self.tf_to_se3(t.transform)
            left_to_robot = np.array([[1, 0, 0, -0.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            right_to_robot = np.array([[1, 0, 0, 0.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            left_to_world = robot_to_world @ left_to_robot
            right_to_left = np.linalg.inv(left_to_world) @ right_to_robot

            now = self.get_clock().now()
            left_cam_tf = self.se3_to_tf(left_to_world, now, parent='world', child='left_cam')
            right_cam_tf = self.se3_to_tf(right_to_left, now, parent='left_cam', child='right_cam')
            self.br.sendTransform([left_cam_tf, right_cam_tf])

            self.get_logger().info('published transforms')

        except tf2_ros.TransformException:
            self.get_logger().info('No transform found')
            return


def main():
    rclpy.init()
    node = CamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
