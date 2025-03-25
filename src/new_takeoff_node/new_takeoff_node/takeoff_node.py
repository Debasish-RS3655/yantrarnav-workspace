#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from geometry_msgs.msg import Quaternion
import tf_transformations

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')

        # Publishers
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.altitude_sub = self.create_subscription(Odometry, '/mavros/local_position/pose', self.altitude_callback, 10)

        # Services
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')

        # Flight parameters
        self.default_takeoff_thrust = 0.6  # Initial thrust for takeoff
        self.smooth_takeoff_thrust = 0.5  # Thrust when nearing target altitude
        self.hover_thrust = 0.5  # Thrust to maintain hover
        self.target_altitude = 1.0  # Target altitude in meters
        self.hover_duration = 3.0  # Hover duration in seconds

        # Current state
        self.current_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.current_altitude = 0.0
        self.current_state = State()
        self.flight_state = "IDLE"  # States: IDLE, TAKEOFF, HOVER, LAND
        self.hover_start_time = None

        # Timer for flight control (5 Hz)
        self.flight_timer = self.create_timer(0.2, self.flight_control_callback)

    def state_callback(self, msg):
        self.current_state = msg
        self.get_logger().info(f"State: Connected={msg.connected}, Armed={msg.armed}, Mode={msg.mode}")
        if self.flight_state == "IDLE" and self.current_state.connected:
            self.prepare_for_takeoff()

    def altitude_callback(self, msg):
        self.current_altitude = msg.pose.pose.position.z
        self.get_logger().info(f"Altitude: {self.current_altitude:.2f} meters")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_attitude['roll'] = euler[0]
        self.current_attitude['pitch'] = euler[1]
        self.current_attitude['yaw'] = euler[2]

    def prepare_for_takeoff(self):
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetMode service not available")
            return
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arming service not available")
            return

        mode_req = SetMode.Request()
        mode_req.custom_mode = "GUIDED_NOGPS"
        mode_future = self.mode_client.call_async(mode_req)
        mode_future.add_done_callback(self.mode_response)

    def mode_response(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("GUIDED_NOGPS mode set successfully")
                arm_req = CommandBool.Request()
                arm_req.value = True
                arm_future = self.arming_client.call_async(arm_req)
                arm_future.add_done_callback(self.arm_response)
            else:
                self.get_logger().error("Failed to set GUIDED_NOGPS mode")
        except Exception as e:
            self.get_logger().error(f"Mode service call failed: {e}")

    def arm_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle armed successfully")
                self.flight_state = "TAKEOFF"
            else:
                self.get_logger().error("Failed to arm vehicle")
        except Exception as e:
            self.get_logger().error(f"Arming service call failed: {e}")

    def flight_control_callback(self):
        if self.flight_state == "TAKEOFF":
            self.perform_takeoff()
        elif self.flight_state == "HOVER":
            self.perform_hover()
        elif self.flight_state == "LAND":
            pass  # Landing handled by land command

    def perform_takeoff(self):
        if self.current_altitude >= self.target_altitude * 0.95:
            self.get_logger().info("Reached target altitude, starting hover")
            self.flight_state = "HOVER"
            self.hover_start_time = self.get_clock().now()
            self.set_attitude(thrust=self.hover_thrust)
        else:
            if self.current_altitude >= self.target_altitude * 0.6:
                thrust = self.smooth_takeoff_thrust
            else:
                thrust = self.default_takeoff_thrust
            self.set_attitude(thrust=thrust)

    def perform_hover(self):
        hover_elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
        if hover_elapsed >= self.hover_duration:
            self.get_logger().info("Hover time complete, initiating landing")
            self.flight_state = "LAND"
            self.send_land_command()
        else:
            self.set_attitude(thrust=self.hover_thrust)

    def set_attitude(self, thrust=0.5):
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type_mask = 0b00000111  # Ignore rates, use attitude and thrust
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.current_attitude['yaw'])
        msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        msg.thrust = thrust
        self.attitude_pub.publish(msg)
        self.get_logger().info(f"Set attitude: Thrust={thrust:.2f}")

    def send_land_command(self):
        if not self.command_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Command service not available")
            return

        land_req = CommandLong.Request()
        land_req.command = 21  # MAV_CMD_NAV_LAND
        land_future = self.command_client.call_async(land_req)
        land_future.add_done_callback(self.land_response)

    def land_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Land command successful")
            else:
                self.get_logger().error(f"Land command failed: {response.result}")
        except Exception as e:
            self.get_logger().error(f"Land service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()