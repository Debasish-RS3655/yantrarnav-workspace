#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class AdaptiveEKF:
    def __init__(self):
        # **State vector: [x, y, z, vx, vy, vz]**
        self.x = np.zeros((6, 1))  
        self.P = np.eye(6) * 0.1  

        # Measurement model: Only position is measured
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        # **Process Noise (Q) - Prevents Drift**
        self.Q = np.eye(6) * 0.001  
        self.Q[3:, 3:] *= 0.005  

        # **Measurement Noise (R) - Adjusted Dynamically**
        self.R_base = np.eye(3) * 1.0  
        self.R = np.copy(self.R_base)

        # **EMA Smoothing**
        self.alpha = 0.1  # 90% old data, 10% new

        # **Outlier Threshold**
        self.outlier_threshold = 1.0  

    def predict(self, dt):
        """Predict the next state"""
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        """Update filter with new measurement and adaptive noise tuning"""
        y = z - self.H @ self.x  
        error_magnitude = np.linalg.norm(y)

        # **Outlier Detection**: If jump is too large, ignore measurement
        if error_magnitude > self.outlier_threshold:
            return  

        # **Adaptive Measurement Noise Tuning**
        self.R = self.R_base * (1 + error_magnitude * 0.5)  

        # Kalman Gain
        S = self.H @ self.P @ self.H.T + self.R  
        K = self.P @ self.H.T @ np.linalg.inv(S)  

        # **Update state**
        self.x = self.x + K @ y  
        self.P = (np.eye(6) - K @ self.H) @ self.P  

        # **Apply EMA for Additional Smoothing**
        self.x[:3] = self.alpha * self.x[:3] + (1 - self.alpha) * z  

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
        self.ekf = AdaptiveEKF()
        self.last_time = None
        self.get_logger().info('Adaptive EKF Pose Filter Started.')

    def pose_callback(self, msg):
        """Process noisy pose data and apply filtering"""
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return  

        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0 or dt > 1.0:
            dt = 0.05  

        # Extract measurement
        z = np.array([[msg.pose.position.x], 
                      [msg.pose.position.y], 
                      [msg.pose.position.z]])

        # Apply filtering
        self.ekf.predict(dt)
        self.ekf.update(z)

        # Publish filtered pose
        filtered_msg = PoseStamped()
        filtered_msg.header = msg.header
        filtered_msg.pose.position.x = float(self.ekf.x[0, 0])
        filtered_msg.pose.position.y = float(self.ekf.x[1, 0])
        filtered_msg.pose.position.z = float(self.ekf.x[2, 0])
        filtered_msg.pose.orientation = msg.pose.orientation  

        self.publisher.publish(filtered_msg)
        self.get_logger().debug('Published highly stable filtered pose.')

def main(args=None):
    rclpy.init(args=args)
    node = PoseFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
