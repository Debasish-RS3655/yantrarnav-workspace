#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from geometry_msgs.msg import Quaternion, PoseStamped
import math
import tf_transformations

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        
        # Publishers
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.position_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.altitude_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.altitude_callback, 10)
        
        # Services
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandLong, '/mavros/cmd/takeoff')
        
        # Target parameters
        self.target_roll = 0.0
        self.target_pitch = 0.0  # ~5.7 degrees
        self.target_yaw = 0.0
        self.target_altitude = 0.1  # meters
        self.target_climb_rate = 0.07  # meters per second
        
        # Current state
        self.current_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.current_altitude = 0.0
        self.initial_altitude = None
        self.current_state = State()
        self.takeoff_initiated = False
        self.takeoff_command_sent = False
        self.takeoff_time = None
        
        # Timer for altitude check
        self.timer = self.create_timer(1.0, self.check_altitude_timeout)

    def state_callback(self, msg):
        self.current_state = msg
        self.get_logger().info(f"State: Connected={msg.connected}, Armed={msg.armed}, Mode={msg.mode}")
        if not self.takeoff_initiated and self.current_state.connected:
            self.prepare_for_takeoff()

    def altitude_callback(self, msg):
        self.current_altitude = msg.pose.position.z
        if self.initial_altitude is None and self.current_state.armed:
            self.initial_altitude = self.current_altitude
        self.get_logger().info(f"Altitude: {self.current_altitude:.2f} meters")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_attitude['roll'] = euler[0]
        self.current_attitude['pitch'] = euler[1]
        self.current_attitude['yaw'] = euler[2]

        self.get_logger().info(f"Current Attitude: Roll={self.current_attitude['roll']:.2f}, "
                              f"Pitch={self.current_attitude['pitch']:.2f}, "
                              f"Yaw={self.current_attitude['yaw']:.2f}")

        if (abs(self.current_attitude['pitch'] - self.target_pitch) > 0.02 or
            abs(self.current_attitude['roll'] - self.target_roll) > 0.02):
            self.publish_attitude_target()
        elif self.current_state.mode == "GUIDED" and self.current_state.armed:
            if not self.takeoff_command_sent:
                self.get_logger().info("Attitude aligned, initiating takeoff")
                self.send_takeoff_command()
            else:
                self.maintain_velocity()

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
        msg.thrust = 0.7
        self.attitude_pub.publish(msg)
        self.get_logger().info(f"Sent attitude target: Pitch={self.target_pitch:.2f}, Thrust={msg.thrust}")

    def send_takeoff_command(self):
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Takeoff service not available")
            return

        takeoff_req = CommandLong.Request()
        takeoff_req.command = 22  # MAV_CMD_NAV_TAKEOFF
        takeoff_req.param5 = self.target_climb_rate  # Climb rate in m/s
        takeoff_req.param7 = self.target_altitude    # Target altitude in meters
        takeoff_future = self.takeoff_client.call_async(takeoff_req)
        takeoff_future.add_done_callback(self.takeoff_response)
        self.takeoff_time = self.get_clock().now()

    def takeoff_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Takeoff command successful")
                self.takeoff_command_sent = True
            else:
                self.get_logger().error(f"Takeoff command failed: {response.result}")
        except Exception as e:
            self.get_logger().error(f"Takeoff service call failed: {e}")

    def maintain_velocity(self):
        if abs(self.current_altitude - (self.initial_altitude or 0.0) - self.target_altitude) > 0.02:
            msg = PositionTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | 
                            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | 
                            PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW | 
                            PositionTarget.IGNORE_YAW_RATE)
            msg.velocity.z = self.target_climb_rate  # Positive for ascent
            self.position_pub.publish(msg)
            self.get_logger().info(f"Maintaining velocity: {self.target_climb_rate} m/s")
        else:
            self.get_logger().info("Target altitude reached, holding position")

    def check_altitude_timeout(self):
        if self.takeoff_command_sent and self.takeoff_time is not None:
            elapsed_time = (self.get_clock().now() - self.takeoff_time).nanoseconds / 1e9
            altitude_increase = self.current_altitude - (self.initial_altitude or 0.0)
            self.get_logger().info(f"Elapsed time: {elapsed_time:.1f}s, Altitude increase: {altitude_increase:.2f}m")
            
            if elapsed_time > 30.0 and altitude_increase < 0.05:
                self.get_logger().warn("Altitude not increasing after 30s, disarming")
                self.disarm_vehicle()

    def disarm_vehicle(self):
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arming service not available for disarming")
            return

        disarm_req = CommandBool.Request()
        disarm_req.value = False
        disarm_future = self.arming_client.call_async(disarm_req)
        disarm_future.add_done_callback(self.disarm_response)

    def disarm_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle disarmed successfully")
                self.takeoff_initiated = False
                self.takeoff_command_sent = False
                self.takeoff_time = None
            else:
                self.get_logger().error("Failed to disarm vehicle")
        except Exception as e:
            self.get_logger().error(f"Disarm service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()