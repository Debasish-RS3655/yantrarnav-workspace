#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VisionRelay(Node):
    def __init__(self):
        super().__init__('vision_relay')
        self.subscriber = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.vision_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, '/mavros/fake_gps/vision', 10)
        self.get_logger().info("Vision relay node started.")

    def vision_callback(self, msg: PoseStamped):
        # You can add any necessary transformations here if needed
        self.publisher.publish(msg)
        self.get_logger().debug("Relayed vision pose data.")

def main(args=None):
    rclpy.init(args=args)
    node = VisionRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down vision relay node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
