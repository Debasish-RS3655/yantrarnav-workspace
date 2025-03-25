#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class LocalPoseRepublisher(Node):
    def __init__(self):
        super().__init__('local_pose_republisher')
        # Subscribe to RTAB-Map odometry
        self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        # Publisher to republish as PoseStamped on /mavros/local_position/pose
        self.local_pose_pub = self.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)

    def odom_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.local_pose_pub.publish(pose_msg)
        self.get_logger().info('Republished local pose.')

def main(args=None):
    rclpy.init(args=args)
    node = LocalPoseRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
