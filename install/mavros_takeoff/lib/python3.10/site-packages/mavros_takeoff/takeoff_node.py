#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
import logging
import time

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')

        # Initialize logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('takeoff_node.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)

        # Subscriber to local position (external navigation source)
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            10
        )
        self.local_pose = None

        # Subscriber to MAVROS state
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        self.current_state = None

        # Service clients
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services
        for client, name in [
            (self.takeoff_client, '/mavros/cmd/takeoff'),
            (self.arming_client, '/mavros/cmd/arming'),
            (self.set_mode_client, '/mavros/set_mode')
        ]:
            if not client.wait_for_service(timeout_sec=10.0):
                self.logger.error(f"Service {name} not available after waiting")
                return
            self.logger.info(f"Service {name} is available")

        # Takeoff parameters
        self.takeoff_altitude = 3.5
        self.min_pitch = 0.0
        self.yaw = 0.0
        self.latitude = 0.0
        self.longitude = 0.0

        # Start takeoff sequence
        self.execute_takeoff()

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.logger.info("External navigation pose received: x=%.2f, y=%.2f, z=%.2f",
                         msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def state_callback(self, msg):
        self.current_state = msg
        self.logger.info("Current state - Mode: %s, Armed: %s",
                         msg.mode, str(msg.armed))

    def arm_vehicle(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.logger.info("Vehicle armed successfully")
            return True
        else:
            self.logger.error("Failed to arm vehicle: %s", future.result())
            return False

    def set_guided_mode(self):
        req = SetMode.Request()
        req.custom_mode = "GUIDED"
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().mode_sent:
            self.logger.info("GUIDED mode set successfully for external navigation")
            return True
        else:
            self.logger.error("Failed to set GUIDED mode: %s", future.result())
            return False

    def send_takeoff_command(self):
        req = CommandTOL.Request()
        req.min_pitch = self.min_pitch
        req.yaw = self.yaw
        req.latitude = self.latitude
        req.longitude = self.longitude
        req.altitude = self.takeoff_altitude

        self.logger.info("Sending takeoff command with altitude=%.2f using external pose", self.takeoff_altitude)
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.logger.info("Takeoff command accepted")
                return True
            else:
                self.logger.warning("Takeoff command rejected with result=%d", future.result().result)
                return False
        else:
            self.logger.error("Takeoff service call failed: %s", future.exception())
            return False

    def execute_takeoff(self):
        self.logger.info("Waiting for MAVROS state...")
        while self.current_state is None and not self.is_shutdown():
            rclpy.spin_once(self, timeout_sec=1.0)
            self.logger.warning("Still waiting for /mavros/state data")
        if self.is_shutdown():
            self.logger.error("Node shutdown while waiting for state")
            return

        self.logger.info("Waiting for external navigation data from /mavros/local_position/pose...")
        while self.local_pose is None and not self.is_shutdown():
            rclpy.spin_once(self, timeout_sec=1.0)
            self.logger.warning("Still waiting for /mavros/local_position/pose data")
        if self.is_shutdown():
            self.logger.error("Node shutdown while waiting for position data")
            return

        if not self.current_state.armed:
            if not self.arm_vehicle():
                self.logger.error("Aborting takeoff due to arming failure")
                return
            time.sleep(1)
        else:
            self.logger.info("Vehicle is already armed")

        if self.current_state.mode != "GUIDED":
            if not self.set_guided_mode():
                self.logger.error("Aborting takeoff due to mode setting failure")
                return
            time.sleep(1)
        else:
            self.logger.info("Vehicle is already in GUIDED mode")

        if self.send_takeoff_command():
            self.logger.info("Takeoff sequence completed successfully using external navigation")
        else:
            self.logger.error("Takeoff sequence failed")

def main():
    rclpy.init()
    node = TakeoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()