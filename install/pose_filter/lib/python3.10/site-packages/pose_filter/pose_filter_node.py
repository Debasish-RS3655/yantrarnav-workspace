#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class SimpleEKF:
    def __init__(self):
        # State vector: [x, y, z, vx, vy, vz] (position and velocity)
        self.x = np.zeros((6, 1))
        # Initial state covariance
        self.P = np.eye(6) * 1.0
        
        # Measurement model: we only measure position [x, y, z]
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        # Process noise covariance
        self.Q = np.eye(6) * 0.1

        # Measurement noise covariance (tune this based on your sensor noise)
        self.R = np.eye(3) * 0.5

    def predict(self, dt):
        # Constant velocity model state transition matrix
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        # Predict state and covariance
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        # z is a 3x1 measurement vector (position)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # Update state estimate and covariance
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

class PoseFilterNode(Node):
    def __init__(self):
        super().__init__('pose_filter_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.pose_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, '/filtered/vision_pose', 10)
        self.ekf = SimpleEKF()
        self.last_time = None
        self.get_logger().info('EKF Pose Filter Node has been started.')

    def pose_callback(self, msg):
        # Get current time in seconds from the message header
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            # First message: initialize time and skip update
            self.last_time = current_time
            dt = 0.0
        else:
            dt = current_time - self.last_time
            self.last_time = current_time
        if dt <= 0:
            dt = 1e-3  # Use a small default dt if needed

        # Prediction step with dt from the last message
        self.ekf.predict(dt)

        # Update step with the measured position
        z = np.array([[msg.pose.position.x],
                      [msg.pose.position.y],
                      [msg.pose.position.z]])
        self.ekf.update(z)

        # Create and publish a new PoseStamped message with filtered position
        filtered_msg = PoseStamped()
        filtered_msg.header = msg.header
        filtered_msg.pose.position.x = float(self.ekf.x[0])
        filtered_msg.pose.position.y = float(self.ekf.x[1])
        filtered_msg.pose.position.z = float(self.ekf.x[2])
        # Optionally, copy the original orientation (or set to a default)
        filtered_msg.pose.orientation = msg.pose.orientation

        self.publisher.publish(filtered_msg)
        self.get_logger().debug('Published filtered pose.')

def main(args=None):
    rclpy.init(args=args)
    node = PoseFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
