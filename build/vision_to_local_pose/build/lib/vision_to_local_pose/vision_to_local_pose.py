#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from mavros_msgs.msg import HomePosition

class VisionToLocalPose(Node):
    def __init__(self):
        super().__init__('vision_to_local_pose')

        # Subscribers
        self.subscription_vision = self.create_subscription(
            PoseStamped, 'mavros/vision_pose/pose', self.vision_pose_callback, 10)
        self.subscription_home = self.create_subscription(
            HomePosition, '/mavros/home_position/set', self.home_position_callback, 10)

        # Publisher
        self.local_pose_pub = self.create_publisher(
            PoseStamped, '/mavros/local_position/pose', 10)

        # Timer to publish at 0.1-second intervals
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.home_pose = None
        self.current_vision_pose = None
        self.get_logger().info('Node initialized, waiting for data...')

    def home_position_callback(self, msg: HomePosition):
        # Store the home position
        self.home_pose = Pose()
        self.home_pose.position.x = msg.position.x
        self.home_pose.position.y = msg.position.y
        self.home_pose.position.z = msg.position.z
        self.home_pose.orientation = msg.orientation
        self.get_logger().info('Home position set as origin.')

    def vision_pose_callback(self, msg: PoseStamped):
        self.current_vision_pose = msg

    def timer_callback(self):
        if self.home_pose is None or self.current_vision_pose is None:
            self.get_logger().warn('Waiting for both home and vision pose data...')
            return

        # Calculate local position as vision_pose - home_pose
        local_pose_msg = PoseStamped()
        local_pose_msg.header = self.current_vision_pose.header
        local_pose_msg.pose.position.x = self.current_vision_pose.pose.position.x - self.home_pose.position.x
        local_pose_msg.pose.position.y = self.current_vision_pose.pose.position.y - self.home_pose.position.y
        local_pose_msg.pose.position.z = self.current_vision_pose.pose.position.z - self.home_pose.position.z
        local_pose_msg.pose.orientation = self.current_vision_pose.pose.orientation  # Copy orientation directly

        self.local_pose_pub.publish(local_pose_msg)
        self.get_logger().info(f'Published local position: {local_pose_msg.pose}')

def main():
    rclpy.init()
    node = VisionToLocalPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
