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
    It subscribes to a source odometry topic, interpolates between messages,
    and republishes at a higher frequency. Also provides compatible outputs
    for MAVROS integration.
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
        self.declare_parameter(
            'republish_odom', 
            True, 
            ParameterDescriptor(description='Whether to republish as Odometry message')
        )
        self.declare_parameter(
            'publish_pose', 
            True, 
            ParameterDescriptor(description='Whether to publish PoseStamped for MAVROS')
        )
        self.declare_parameter(
            'publish_twist', 
            True, 
            ParameterDescriptor(description='Whether to publish TwistStamped for MAVROS')
        )
        
        # Get parameters
        self.source_topic = self.get_parameter('source_topic').value
        self.target_rate = self.get_parameter('target_rate').value
        self.republish_odom = self.get_parameter('republish_odom').value
        self.publish_pose = self.get_parameter('publish_pose').value
        self.publish_twist = self.get_parameter('publish_twist').value
        
        # Create publishers
        if self.republish_odom:
            self.odom_pub = self.create_publisher(
                Odometry, 
                '~/odom', 
                10
            )
            
        if self.publish_pose:
            self.pose_pub = self.create_publisher(
                PoseStamped, 
                '~/pose', 
                10
            )
            
        if self.publish_twist:
            self.twist_pub = self.create_publisher(
                TwistStamped, 
                '~/twist', 
                10
            )
        
        # Subscribe to source odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            self.source_topic,
            self.odom_callback,
            10
        )
        
        # State variables
        self.latest_odom = None
        self.previous_odom = None
        self.lock = threading.Lock()
        
        # Create timer for republishing at target rate
        self.timer = self.create_timer(1.0/self.target_rate, self.timer_callback)
        
        self.get_logger().info(f"Initialized OdomRateBooster node")
        self.get_logger().info(f"Subscribing to {self.source_topic}")
        self.get_logger().info(f"Target publishing rate: {self.target_rate} Hz")
    
    def odom_callback(self, msg):
        """Callback for the odometry subscription"""
        with self.lock:
            self.previous_odom = self.latest_odom
            self.latest_odom = msg
            
    def interpolate_odometry(self, t):
        """
        Interpolate between previous and latest odometry messages
        t is a value between 0.0 and 1.0 representing the interpolation factor
        """
        if self.previous_odom is None or self.latest_odom is None:
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
        with self.lock:
            if self.latest_odom is None:
                return
            
            # When we only have one message, we can't interpolate
            if self.previous_odom is None:
                interpolated = self.latest_odom
            else:
                # Calculate interpolation factor
                # For simplicity, using a factor of 0.5 (midpoint)
                # In a more advanced implementation, we could use actual timestamps
                interpolated = self.interpolate_odometry(0.5)
            
            # Publish all requested message types
            if self.republish_odom:
                self.odom_pub.publish(interpolated)
                
            if self.publish_pose:
                pose_msg = self.create_pose_stamped(interpolated)
                self.pose_pub.publish(pose_msg)
                
            if self.publish_twist:
                twist_msg = self.create_twist_stamped(interpolated)
                self.twist_pub.publish(twist_msg)

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