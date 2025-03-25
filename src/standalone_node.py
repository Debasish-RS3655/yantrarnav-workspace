import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Quaternion

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_attitude/attitude', 10)
        self.timer = self.create_timer(0.05, self.publish_attitude)  # 20 Hz

        self.att_target = AttitudeTarget()
        self.att_target.type_mask = 0b00000111  # Use attitude and thrust
        self.att_target.thrust = 0.6  # Thrust > 0.5 to lift off
        self.att_target.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)  # Level

    def publish_attitude(self):
        self.pub.publish(self.att_target)

def main():
    rclpy.init()
    node = TakeoffNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()