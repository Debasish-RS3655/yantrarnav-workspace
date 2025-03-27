#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from sensor_msgs.msg import Range
from mavros_msgs.srv import CommandBool, SetMode

class GuidedNoGpsTakeoffOptFlowNode(Node):
    def __init__(self):
        super().__init__('guided_nogps_takeoff_optflow_node')

        # Parameters
        self.declare_parameter("target_altitude", 0.4)
        self.target_altitude = self.get_parameter("target_altitude").value

        # State variables
        self.initial_odom_received = False
        self.initial_position = None
        self.initial_range = None
        self.current_altitude = 0.0
        self.current_state = State()
        self.mode_set = False
        self.armed = False

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        self.range_sub = self.create_subscription(Range, '/mavros/rangefinder/rangefinder', self.range_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # Publisher (ArduPilot requires PositionTarget)
        self.setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        # Services
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Timer (10Hz setpoint publishing)
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoint)

        self.get_logger().info("Node initialized. Waiting for initialization...")

    def odom_callback(self, msg):
        if not self.initial_odom_received:
            self.initial_odom_received = True
            self.initial_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.get_logger().info(f"Initial position set: X={self.initial_position[0]:.2f}, Y={self.initial_position[1]:.2f}")

    def range_callback(self, msg):
        if self.initial_range is None and msg.range > 0.0:
            self.initial_range = msg.range
            self.get_logger().info(f"Initial range: {self.initial_range:.2f}m")
        
        if self.initial_range is not None:
            self.current_altitude = msg.range - self.initial_range

    def state_callback(self, msg):
        self.current_state = msg
        if not self.mode_set and msg.connected:
            self.set_guided_nogps_mode()
        if not self.armed and msg.connected:
            self.arm_vehicle()

    def set_guided_nogps_mode(self):
        req = SetMode.Request()
        req.custom_mode = "GUIDED_NOGPS"
        future = self.mode_client.call_async(req)
        future.add_done_callback(self.mode_callback)

    def mode_callback(self, future):
        try:
            resp = future.result()
            if resp.mode_sent:
                self.mode_set = True
                self.get_logger().info("GUIDED_NOGPS mode set")
            else:
                self.get_logger().error("Failed to set mode")
        except Exception as e:
            self.get_logger().error(f"Mode service error: {str(e)}")

    def arm_vehicle(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.armed = True
                self.get_logger().info("Vehicle armed")
            else:
                self.get_logger().error("Arming failed")
        except Exception as e:
            self.get_logger().error(f"Arming error: {str(e)}")

    def publish_setpoint(self):
        if not all([self.initial_odom_received, self.initial_range is not None]):
            return

        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # Correct type mask (2040 = position control only)
        sp.type_mask = 2040  # Alternative: 0b0000011111111000
        
        # Position control (NED frame)
        if self.initial_position:
            sp.position.x = self.initial_position[0]
            sp.position.y = self.initial_position[1]
        sp.position.z = self.target_altitude  # Negative for climb in NED

        self.setpoint_pub.publish(sp)
        self.get_logger().debug(f"Setpoint: Z={sp.position.z:.2f}m (NED)", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = GuidedNoGpsTakeoffOptFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()