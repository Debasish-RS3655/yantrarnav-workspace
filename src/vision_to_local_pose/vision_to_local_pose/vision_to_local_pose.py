#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from geometry_msgs.msg import Quaternion
import math
import tf_transformations

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        
        # Publishers
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        
        # Services
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandLong, '/mavros/cmd/takeoff')
        
        # Target attitude (radians)
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        
        # Current state
        self.current_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.current_state = State()
        self.takeoff_initiated = False
        self.takeoff_command_sent = False
        
    def state_callback(self, msg):
        self.current_state = msg
        self.get_logger().info(f"State: Connected={msg.connected}, Armed={msg.armed}, Mode={msg.mode}")
        if not self.takeoff_initiated and self.current_state.connected:
            self.prepare_for_takeoff()
    
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_attitude['roll'] = euler[0]
        self.current_attitude['pitch'] = euler[1]
        self.current_attitude['yaw'] = euler[2]

        self.get_logger().info(f"Current Attitude: Roll={self.current_attitude['roll']:.2f}, "
                              f"Pitch={self.current_attitude['pitch']:.2f}, "
                              f"Yaw={self.current_attitude['yaw']:.2f}")

    def prepare_for_takeoff(self):
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetMode service not available")
            return
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arming service not available")
            return

        mode_req = SetMode.Request()
        mode_req.custom_mode = "GUIDED"
        mode_future = self.mode_client.call_async(mode_req)
        mode_future.add_done_callback(self.mode_response)

    def mode_response(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("GUIDED mode set successfully")
                arm_req = CommandBool.Request()
                arm_req.value = True
                arm_future = self.arming_client.call_async(arm_req)
                arm_future.add_done_callback(self.arm_response)
            else:
                self.get_logger().error("Failed to set GUIDED mode")
        except Exception as e:
            self.get_logger().error(f"Mode service call failed: {e}")

    def arm_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle armed successfully")
                self.takeoff_initiated = True
            else:
                self.get_logger().error("Failed to arm vehicle")
        except Exception as e:
            self.get_logger().error(f"Arming service call failed: {e}")

    def publish_attitude_target(self):
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type_mask = 7  # Ignore rates, use attitude
        q = tf_transformations.quaternion_from_euler(self.target_roll, self.target_pitch, self.target_yaw)
        msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        msg.thrust = 0.7  # Thrust for initial lift
        self.attitude_pub.publish(msg)
        self.get_logger().info(f"Sent attitude target: Pitch={self.target_pitch:.2f}, Thrust={msg.thrust}")

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
