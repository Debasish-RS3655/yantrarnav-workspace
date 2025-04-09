#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np
import tf_transformations
import threading
import time

class OdomRateBooster(Node):
    """
    A ROS 2 node that increases the publishing rate of an odometry topic.
    This fixed version ensures reliable publishing at the target rate.
    """
    
    def __init__(self):
        super().__init__('odom_rate_booster')
        
        # Declare parameters
        self.declare_parameter(
            'source_topic', 
            '/rtabmap/odom', 
            ParameterDescriptor(description='Source odometry topic to subscribe to')
        )
        self.declare_parameter(
            'target_rate', 
            30.0, 
            ParameterDescriptor(description='Target publishing rate in Hz')
        )
        
        # Get parameters
        self.source_topic = self.get_parameter('source_topic').value
        self.target_rate = self.get_parameter('target_rate').value
        
        # Create publishers with explicit QoS settings
        self.odom_pub = self.create_publisher(
            Odometry, 
            '/mavros/odometry/in', 
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/vision_pose/pose', 
            10
        )
        
        self.twist_pub = self.create_publisher(
            TwistStamped, 
            '/mavros/local_position/velocity_local', 
            10
        )
        
        # Log publisher creation
        self.get_logger().info(f"Created publishers:")
        self.get_logger().info(f"  - Odometry: {self.odom_pub.topic_name}")
        self.get_logger().info(f"  - Pose: {self.pose_pub.topic_name}")
        self.get_logger().info(f"  - Twist: {self.twist_pub.topic_name}")
        
        # Subscribe to source odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            self.source_topic,
            self.odom_callback,
            10
        )
        self.get_logger().info(f"Subscribed to source topic: {self.source_topic}")
        
        # State variables
        self.latest_odom = None
        self.previous_odom = None
        self.lock = threading.Lock()
        self.msg_count = 0
        self.last_log_time = self.get_clock().now()
        self.publish_count = 0
        
        # Create timer for republishing at target rate with higher priority
        period_ns = int(1e9/self.target_rate)  # Convert to nanoseconds
        self.timer = self.create_timer(1.0/self.target_rate, self.timer_callback)
        self.get_logger().info(f"Created timer with period: {1.0/self.target_rate} seconds")
        
        # Create diagnostic timer to print status every 5 seconds
        self.diagnostic_timer = self.create_timer(5.0, self.diagnostic_callback)
        
        self.get_logger().info(f"Initialized OdomRateBooster node")
    
    def diagnostic_callback(self):
        """Print diagnostic information periodically"""
        now = self.get_clock().now()
        duration = (now - self.last_log_time).nanoseconds / 1e9
        
        if self.latest_odom is None:
            self.get_logger().warn(f"No messages received on {self.source_topic} in the last {duration:.2f} seconds")
        else:
            self.get_logger().info(f"Received {self.msg_count} messages in the last {duration:.2f} seconds")
            self.get_logger().info(f"Published {self.publish_count} messages in the last {duration:.2f} seconds")
            if self.previous_odom is not None:
                self.get_logger().info(f"Interpolating between messages")
            self.get_logger().info(f"Latest position: x={self.latest_odom.pose.pose.position.x:.2f}, y={self.latest_odom.pose.pose.position.y:.2f}, z={self.latest_odom.pose.pose.position.z:.2f}")
        
        # Reset counters
        self.msg_count = 0
        self.publish_count = 0
        self.last_log_time = now
    
    def odom_callback(self, msg):
        """Callback for the odometry subscription"""
        with self.lock:
            self.previous_odom = self.latest_odom
            self.latest_odom = msg
            self.msg_count += 1
        
        # Log first message received
        if self.msg_count == 1:
            self.get_logger().info("First odometry message received!")
            
    def interpolate_odometry(self, t):
        """
        Interpolate between previous and latest odometry messages
        t is a value between 0.0 and 1.0 representing the interpolation factor
        """
        if self.latest_odom is None:
            return None
            
        if self.previous_odom is None:
            # If we only have one message, just use it
            return self.latest_odom
            
        # Create a new message
        interpolated = Odometry()
        interpolated.header.stamp = self.get_clock().now().to_msg()
        interpolated.header.frame_id = self.latest_odom.header.frame_id
        interpolated.child_frame_id = self.latest_odom.child_frame_id
        
        # Linear interpolation for position
        p0 = self.previous_odom.pose.pose.position
        p1 = self.latest_odom.pose.pose.position
        interpolated.pose.pose.position.x = p0.x + t * (p1.x - p0.x)
        interpolated.pose.pose.position.y = p0.y + t * (p1.y - p0.y)
        interpolated.pose.pose.position.z = p0.z + t * (p1.z - p0.z)
        
        # SLERP for orientation
        q0 = [
            self.previous_odom.pose.pose.orientation.x,
            self.previous_odom.pose.pose.orientation.y,
            self.previous_odom.pose.pose.orientation.z,
            self.previous_odom.pose.pose.orientation.w
        ]
        q1 = [
            self.latest_odom.pose.pose.orientation.x,
            self.latest_odom.pose.pose.orientation.y,
            self.latest_odom.pose.pose.orientation.z,
            self.latest_odom.pose.pose.orientation.w
        ]
        
        # Ensure shortest path interpolation
        dot = sum(x * y for x, y in zip(q0, q1))
        if dot < 0.0:
            q1 = [-x for x in q1]
            
        # Perform SLERP
        interp_q = tf_transformations.quaternion_slerp(q0, q1, t)
        
        interpolated.pose.pose.orientation.x = interp_q[0]
        interpolated.pose.pose.orientation.y = interp_q[1]
        interpolated.pose.pose.orientation.z = interp_q[2]
        interpolated.pose.pose.orientation.w = interp_q[3]
        
        # Linear interpolation for twist
        v0 = self.previous_odom.twist.twist.linear
        v1 = self.latest_odom.twist.twist.linear
        interpolated.twist.twist.linear.x = v0.x + t * (v1.x - v0.x)
        interpolated.twist.twist.linear.y = v0.y + t * (v1.y - v0.y)
        interpolated.twist.twist.linear.z = v0.z + t * (v1.z - v0.z)
        
        w0 = self.previous_odom.twist.twist.angular
        w1 = self.latest_odom.twist.twist.angular
        interpolated.twist.twist.angular.x = w0.x + t * (w1.x - w0.x)
        interpolated.twist.twist.angular.y = w0.y + t * (w1.y - w0.y)
        interpolated.twist.twist.angular.z = w0.z + t * (w1.z - w0.z)
        
        # Copy covariance matrices
        interpolated.pose.covariance = self.latest_odom.pose.covariance
        interpolated.twist.covariance = self.latest_odom.twist.covariance
        
        return interpolated
        
    def create_pose_stamped(self, odom_msg):
        """Convert Odometry to PoseStamped for MAVROS"""
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose
        return pose_msg
        
    def create_twist_stamped(self, odom_msg):
        """Convert Odometry to TwistStamped for MAVROS"""
        twist_msg = TwistStamped()
        twist_msg.header = odom_msg.header
        twist_msg.twist = odom_msg.twist.twist
        return twist_msg
    
    def timer_callback(self):
        """Timer callback to republish at target rate"""
        # Always try to publish, even with just the latest message
        odom_msg = None
        
        with self.lock:
            if self.latest_odom is None:
                return  # Nothing to publish yet
            
            # When we only have one message, use it directly
            if self.previous_odom is None:
                odom_msg = self.latest_odom  # Use the latest message directly
            else:
                # Calculate interpolation factor - use middle point for stability
                odom_msg = self.interpolate_odometry(0.5)
        
        if odom_msg is not None:
            # Update timestamp to current time
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Create derived messages
            pose_msg = self.create_pose_stamped(odom_msg)
            twist_msg = self.create_twist_stamped(odom_msg)
            
            # Publish all message types
            self.odom_pub.publish(odom_msg)
            self.pose_pub.publish(pose_msg)
            self.twist_pub.publish(twist_msg)
            
            with self.lock:
                self.publish_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = OdomRateBooster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()