import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Quaternion

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.publisher = self.create_publisher(AttitudeTarget, '/mavros/setpoint_attitude/attitude', 10)
        self.timer = self.create_timer(0.05, self.publish_attitude)  # 20 Hz (1/0.05)

    def publish_attitude(self):
        att_target = AttitudeTarget()
        att_target.header.stamp = self.get_clock().now().to_msg()
        att_target.type_mask = 0b00000111  # Use attitude (roll, pitch, yaw) and thrust
        att_target.thrust = 0.6  # Thrust value (0-1); adjust as needed for takeoff
        att_target.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)  # Level attitude

        self.publisher.publish(att_target)
        self.get_logger().info('Publishing attitude target: thrust = 0.6')

def main():
    rclpy.init()
    node = TakeoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
