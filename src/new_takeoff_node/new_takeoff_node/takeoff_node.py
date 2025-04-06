#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
import tf_transformations
from geometry_msgs.msg import TwistStamped # For raw optical flow data

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')

        # Fixed flight mode to GUIDED_NOGPS
        self.flight_mode = "GUIDED_NOGPS"
        self.get_logger().info(f"Flight mode set to: {self.flight_mode}")

        # **Publishers**
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.altitude_pub = self.create_publisher(Float32, '/drone/altitude', 10)

        # **Subscribers**
        self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.range_sub = self.create_subscription(Range, '/mavros/rangefinder/rangefinder', self.range_callback, 10)
        self.flow_ground_sub = self.create_subscription(Float32, '/mavros/optical_flow/ground_distance', self.flow_ground_callback, 10)
        self.flow_raw_sub = self.create_subscription(TwistStamped, '/mavros/optical_flow/raw/optical_flow', self.flow_raw_callback, 10)

        # **Services**
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # **Flight Parameters**
        self.default_takeoff_thrust = 0.505
        self.hover_thrust = 0.5
        self.target_altitude = 0.4
        self.hover_duration = 7.0
        self.Kp_altitude = 0.25 # Proportional gain for altitude
        self.Kp_flow = 0.2 # Gain for flow position hold
        self.Ki_flow = 0.02 # Integral gain for flow position
        self.Kp_attitude_flow = 0.2 # Proportional gain for attitude (roll/pitch) from flow
        self.max_roll_pitch = 0.3 # Limit roll/pitch adjustments (radians, ~5.7 degrees)
        self.flow_deadband = 0.01 # Deadband threshold for flow correction

        # **Initial Pitch Offset Parameters**
        self.initial_pitch_offset = 0.1 # initial positive pitch offset in radians
        self.pitch_damping_time = 8.0 # seconds over which the offset decays to 0
        self.takeoff_start_time = None # to record the time when takeoff starts

        # **Current State Variables**
        self.current_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.current_altitude = 0.0
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_flow_vel = {'x': 0.0, 'y': 0.0} # Flow velocities in X and Y
        self.flow_integral = {'x': 0.0, 'y': 0.0} # Integrated flow for drift correction
        self.current_state = State()
        self.flight_state = "IDLE"
        self.hover_start_time = None
        # Instead of using the measured yaw, we force desired yaw to 0 later.
        self.initial_yaw = None 
        self.initial_position = None
        self.current_thrust = 0.0
        self.initial_range = None
        self.initial_flow_ground = None
        self.last_flow_time = None # For integrating flow velocities

        # **Flow Calibration Variables**
        self.flow_bias = {'x': 0.0, 'y': 0.0}
        self.flow_calibrated = False
        self.flow_sample_count = 0
        self.flow_bias_sum = {'x': 0.0, 'y': 0.0}
        self.flow_calibration_samples = 10

        # **Odometry Calibration Variable**
        self.odom_offset = None # Will store the initial (offset) attitude

        # **Timers**
        self.flight_timer = self.create_timer(0.2, self.flight_control_callback)
        self.control_timer = self.create_timer(0.02, self.publish_control)

    def state_callback(self, msg):
        self.current_state = msg
        if self.flight_state == "IDLE" and self.current_state.connected and not self.current_state.armed:
            self.prepare_for_takeoff()

    def range_callback(self, msg):
        if self.initial_range is None and msg.range != 0.0:
            self.initial_range = msg.range
            self.get_logger().info(f"Initial range set to {self.initial_range:.2f} meters")

        if self.initial_range is not None:
            self.current_altitude = msg.range - self.initial_range
            self.current_altitude = max(self.current_altitude, 0.0)
            altitude_msg = Float32()
            altitude_msg.data = self.current_altitude
            self.altitude_pub.publish(altitude_msg)

    def flow_ground_callback(self, msg):
        """Update altitude from optical flow ground distance."""
        if self.initial_flow_ground is None and msg.data != 0.0:
            self.initial_flow_ground = msg.data
            self.get_logger().info(f"Initial flow ground distance set to {self.initial_flow_ground:.2f} meters")

        if self.initial_flow_ground is not None:
            self.current_altitude = msg.data - self.initial_flow_ground
            self.current_altitude = max(self.current_altitude, 0.0)
            altitude_msg = Float32()
            altitude_msg.data = self.current_altitude
            self.altitude_pub.publish(altitude_msg)
            self.get_logger().info(f"Ground distance: {msg.data:.2f}m, Altitude: {self.current_altitude:.2f}m")

    def flow_raw_callback(self, msg):
        """Update flow velocities, calibrate sensor, and integrate for position correction."""
        current_time = self.get_clock().now()
        dt = 0.02
        if self.last_flow_time is not None:
            dt = (current_time - self.last_flow_time).nanoseconds / 1e9

        # Raw flow measurements
        raw_x = msg.twist.linear.x
        raw_y = msg.twist.linear.y

        # Calibrate using initial samples
        if not self.flow_calibrated:
            self.flow_bias_sum['x'] += raw_x
            self.flow_bias_sum['y'] += raw_y
            self.flow_sample_count += 1
            if self.flow_sample_count >= self.flow_calibration_samples:
                self.flow_bias['x'] = self.flow_bias_sum['x'] / self.flow_sample_count
                self.flow_bias['y'] = self.flow_bias_sum['y'] / self.flow_sample_count
                self.flow_calibrated = True
                self.get_logger().info(f"Flow calibrated: bias_x={self.flow_bias['x']:.2f}, bias_y={self.flow_bias['y']:.2f}")

        # Subtract bias
        calibrated_x = raw_x - self.flow_bias['x']
        calibrated_y = raw_y - self.flow_bias['y']
        self.current_flow_vel['x'] = calibrated_x
        self.current_flow_vel['y'] = calibrated_y

        if self.last_flow_time is not None:
            self.flow_integral['x'] += calibrated_x * dt
            self.flow_integral['y'] += calibrated_y * dt
        self.last_flow_time = current_time

        self.get_logger().info(f"Flow velocities: vx={self.current_flow_vel['x']:.2f}, vy={self.current_flow_vel['y']:.2f}, "
                             f"Integral: ix={self.flow_integral['x']:.2f}, iy={self.flow_integral['y']:.2f}")

    def odom_callback(self, msg):
        # Extract orientation and convert to Euler angles
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # Use an offset if available so that the first reading is treated as zero
        if self.odom_offset is None:
            self.odom_offset = {'roll': euler[0], 'pitch': euler[1], 'yaw': euler[2]}
            self.get_logger().info(f"Odometry offset set: roll={self.odom_offset['roll']:.2f}, "
                                 f"pitch={self.odom_offset['pitch']:.2f}, yaw={self.odom_offset['yaw']:.2f}")

        # Correct the current readings by subtracting the offset
        self.current_attitude['roll'] = euler[0] - self.odom_offset['roll']
        self.current_attitude['pitch'] = euler[1] - self.odom_offset['pitch']
        self.current_attitude['yaw'] = euler[2] - self.odom_offset['yaw']
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y
        self.current_position['z'] = msg.pose.pose.position.z

        # Save the initial position if not set
        if self.initial_position is None and self.flight_state == "IDLE":
            self.initial_position = {'x': self.current_position['x'], 'y': self.current_position['y']}

        # Log the corrected attitude
        self.get_logger().info(f"Corrected Attitude: roll={self.current_attitude['roll']:.2f}, "
                             f"pitch={self.current_attitude['pitch']:.2f}, yaw={self.current_attitude['yaw']:.2f}")

    def prepare_for_takeoff(self):
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetMode service not available")
            return
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arming service not available")
            return

        self.get_logger().info("Setting GUIDED_NOGPS mode...")
        mode_req = SetMode.Request()
        mode_req.custom_mode = "GUIDED_NOGPS"
        mode_future = self.mode_client.call_async(mode_req)
        mode_future.add_done_callback(self.mode_response)

    def mode_response(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.arm_vehicle()
            else:
                self.get_logger().warn("Failed to set GUIDED_NOGPS mode")
        except Exception as e:
            self.get_logger().error(f"Mode service call failed: {e}")

    def arm_vehicle(self, arm=True):
        arm_req = CommandBool.Request()
        arm_req.value = arm
        arm_future = self.arming_client.call_async(arm_req)
        arm_future.add_done_callback(self.arm_response)

    def arm_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle armed successfully")
                # Instead of using the measured yaw, force the commanded yaw to be zero.
                self.initial_yaw = 0.0 
                if self.initial_position is None:
                    self.initial_position = {'x': self.current_position['x'], 'y': self.current_position['y']}
                self.flight_state = "TAKEOFF"
                self.current_thrust = self.hover_thrust + self.Kp_altitude * self.target_altitude
                self.takeoff_start_time = self.get_clock().now() # Record takeoff start time for pitch damping
                self.last_flow_time = self.get_clock().now() # Initialize flow time
            else:
                self.get_logger().warn("Arming failed")
        except Exception as e:
            self.get_logger().error(f"Arming service call failed: {e}")

    def flight_control_callback(self):
        if self.flight_state == "TAKEOFF":
            self.perform_takeoff()
        elif self.flight_state == "HOVER":
            self.perform_hover()
        elif self.flight_state == "LAND":
            self.perform_landing()

    def perform_takeoff(self):
        if self.current_altitude >= self.target_altitude * 0.95:
            self.get_logger().info("Reached target altitude, starting hover")
            self.flight_state = "HOVER"
            self.hover_start_time = self.get_clock().now()
            # Ensure that during hover thrust is exactly the hover thrust (0.5)
            self.current_thrust = self.hover_thrust
        else:
            altitude_error = self.target_altitude - self.current_altitude
            self.current_thrust = self.hover_thrust + self.Kp_altitude * altitude_error
            self.current_thrust = min(self.current_thrust, 0.57)

    def perform_hover(self):
        hover_elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
        if hover_elapsed >= self.hover_duration:
            self.get_logger().info("Hover time complete, initiating landing")
            self.flight_state = "LAND"
            self.initiate_landing()
        else:
            # Clamp thrust in hover state to not exceed 0.5
            self.current_thrust = self.hover_thrust

    def perform_landing(self):
        if self.current_altitude > 0.05:
            self.current_thrust -= 0.01
            self.current_thrust = max(self.current_thrust, 0.3)
        else:
            self.get_logger().info("Landing complete, disarming...")
            self.flight_state = "DISARM"
            self.arm_vehicle(False)

    def publish_control(self):
        if self.flight_state in ["TAKEOFF", "HOVER", "LAND"] and self.current_state.armed:
            self.set_attitude(thrust=self.current_thrust)
            self.set_position_with_flow()

    def set_attitude(self, thrust=0.5):
        """Set attitude with flow-based roll and pitch correction, adding an initial positive pitch offset that decays."""
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type_mask = 0b00000111 # Ignore rates, use attitude and thrust

        # Compute corrective roll and pitch from flow velocities
        roll_cmd = -self.Kp_attitude_flow * self.current_flow_vel['x'] # Negative to counteract drift
        pitch_cmd = -self.Kp_attitude_flow * self.current_flow_vel['y'] # Negative to counteract drift

        # If in TAKEOFF state, add an initial positive pitch offset that decays over time.
        if self.flight_state == "TAKEOFF" and self.takeoff_start_time is not None:
            elapsed = (self.get_clock().now() - self.takeoff_start_time).nanoseconds / 1e9
            if elapsed < self.pitch_damping_time:
                # Compute a decaying offset (linearly decaying)
                pitch_offset = self.initial_pitch_offset * (1 - (elapsed / self.pitch_damping_time))
                pitch_cmd += pitch_offset

        # Apply deadband to ignore small corrections
        if abs(roll_cmd) < self.flow_deadband:
            roll_cmd = 0.0
        if abs(pitch_cmd) < self.flow_deadband:
            pitch_cmd = 0.0

        # Limit roll and pitch to avoid excessive tilting
        roll_cmd = max(min(roll_cmd, self.max_roll_pitch), -self.max_roll_pitch)
        pitch_cmd = max(min(pitch_cmd, self.max_roll_pitch), -self.max_roll_pitch)

        # Use the forced yaw (0.0) for heading hold
        yaw_cmd = self.initial_yaw if self.initial_yaw is not None else 0.0

        # Convert to quaternion
        q = tf_transformations.quaternion_from_euler(roll_cmd, pitch_cmd, yaw_cmd)
        msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        # If hovering, ensure the thrust does not exceed hover thrust (0.5)
        if self.flight_state == "HOVER":
            thrust = min(thrust, self.hover_thrust)
        msg.thrust = thrust

        self.attitude_pub.publish(msg)
        self.get_logger().info(f"Published attitude: roll={roll_cmd:.2f}, pitch={pitch_cmd:.2f}, yaw={yaw_cmd:.2f}, thrust={thrust:.2f}")

    def set_position_with_flow(self):
        """Set position setpoint with enhanced flow-based drift correction."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Proportional and integral control for flow-based position
        flow_x_error = -self.current_flow_vel['x']
        flow_y_error = -self.current_flow_vel['y']
        x_adjust_p = self.Kp_flow * flow_x_error
        y_adjust_p = self.Kp_flow * flow_y_error
        x_adjust_i = self.Ki_flow * self.flow_integral['x']
        y_adjust_i = self.Ki_flow * self.flow_integral['y']

        # Combine P and I terms for position corrections
        msg.pose.position.x = (self.initial_position['x'] if self.initial_position is not None else 0.0) - (x_adjust_p + x_adjust_i)
        msg.pose.position.y = (self.initial_position['y'] if self.initial_position is not None else 0.0) - (y_adjust_p + y_adjust_i)

        # Control Z based on flight state
        if self.flight_state in ["TAKEOFF", "HOVER"]:
            msg.pose.position.z = self.target_altitude
        elif self.flight_state == "LAND":
            msg.pose.position.z = 0.0

        # Use flow-adjusted attitude for consistency (although we force yaw=0 here)
        q = tf_transformations.quaternion_from_euler(
            -self.Kp_attitude_flow * self.current_flow_vel['x'], # Roll
            -self.Kp_attitude_flow * self.current_flow_vel['y'], # Pitch
            self.initial_yaw if self.initial_yaw is not None else 0.0 # Yaw
        )
        msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.position_pub.publish(msg)
        self.get_logger().debug(f"Published position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}")

    def initiate_landing(self):
        self.get_logger().info("Setting LAND mode...")
        mode_req = SetMode.Request()
        mode_req.custom_mode = "LAND"
        mode_future = self.mode_client.call_async(mode_req)
        mode_future.add_done_callback(self.mode_response)

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()