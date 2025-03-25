#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point
from mavros_msgs.msg import HomePosition

class HomeSetter(Node):
    def __init__(self):
        super().__init__('home_setter')
        self.get_logger().info('Node initialized')

        # Subscribe to /vision_pose_temp
        self.subscription = self.create_subscription(
            PoseStamped,
            'mavros/vision_pose/pose',
            self.pose_callback,
            10)
        self.get_logger().info('Subscribed to mavros/vision_pose/pose')

        # Publisher to /mavros/home_position/set using HomePosition type
        self.home_position_pub = self.create_publisher(
            HomePosition,
            '/mavros/home_position/set',
            10)
        self.get_logger().info('Publishing to /mavros/home_position/set')

        # Timer for continuous publishing (e.g., every 1 second)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.current_pose = None  # To store the first (home) pose

    def pose_callback(self, msg: PoseStamped):
        # For this application, we only need the first received pose.
        if self.current_pose is None:
            self.current_pose = msg.pose
            self.get_logger().info(f'Home pose received: {msg.pose}')

    def timer_callback(self):
        if self.current_pose is None:
            self.get_logger().info("No home position received yet.")
            return

        # Create a HomePosition message and populate it using the current pose.
        home_msg = HomePosition()
        home_msg.header.stamp = self.get_clock().now().to_msg()
        home_msg.header.frame_id = "odom"  # Adjust if your coordinate frame is different

        # Map the stored PoseStamped position to the HomePosition's nested position field.
        home_msg.position.x = self.current_pose.position.x
        home_msg.position.y = self.current_pose.position.y
        home_msg.position.z = self.current_pose.position.z

        # Optionally, set the orientation if HomePosition supports it.
        try:
            home_msg.orientation = self.current_pose.orientation
        except AttributeError:
            self.get_logger().warn("HomePosition message type does not support orientation.")

        self.home_position_pub.publish(home_msg)
        self.get_logger().info('Continuously publishing home position to /mavros/home_position/set')

def main():
    rclpy.init()
    node = HomeSetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
