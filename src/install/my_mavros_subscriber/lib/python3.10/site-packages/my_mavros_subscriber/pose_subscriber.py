#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.callback,
            qos
        )

    def callback(self, msg):
        self.get_logger().info(
            f'Received pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()