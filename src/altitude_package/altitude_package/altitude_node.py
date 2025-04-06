#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32
from collections import deque
import math
import numpy as np  # For median filter

# Constants for barometric altitude calculation
P0 = 101325  # Sea level standard pressure in Pascals
T0 = 288.15  # Standard temperature in Kelvin
L = 0.0065   # Temperature lapse rate in K/m
R = 287.05   # Specific gas constant for dry air in J/(kg·K)
g0 = 9.80665  # Acceleration due to gravity in m/s²

# Noise filtering parameters
MEDIAN_WINDOW = 7     # Larger window for median filter (better spike removal)
MOVING_AVG_WINDOW = 20  # Longer moving average for better smoothing
EMA_ALPHA = 0.2       # Exponential moving average weight (0.1-0.3 for slow response)
TAKEOFF_ALTITUDE = 0.06  # Initial altitude offset (m)

def pressure_to_altitude(P):
    """Convert static pressure (P) in Pascals to altitude in meters."""
    return (T0 / L) * (1 - (P / P0) ** (R * L / g0))

class AltitudeEstimator(Node):
    def __init__(self):
        super().__init__('altitude_estimator')

        # Best-effort QoS for compatibility with MAVROS sensor topics
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            FluidPressure,
            '/mavros/imu/static_pressure',
            self.pressure_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Float32, '/mavros/altitude', 10)

        self.initial_altitude = None
        self.pressure_values = deque(maxlen=MOVING_AVG_WINDOW)  # Moving average buffer
        self.median_values = deque(maxlen=MEDIAN_WINDOW)  # Median filter buffer
        self.ema_filtered_value = None  # Exponential moving average state

        self.get_logger().info("Altitude estimation node started.")

    def pressure_callback(self, msg):
        P = msg.fluid_pressure  # Static pressure in Pascals

        # 1️⃣ Median Filter - Remove sudden spikes
        self.median_values.append(P)
        if len(self.median_values) == MEDIAN_WINDOW:
            P_median = np.median(self.median_values)
        else:
            P_median = P  # Not enough data, use raw value

        # 2️⃣ Moving Average Filter - Smooth fluctuations
        self.pressure_values.append(P_median)
        P_smooth = sum(self.pressure_values) / len(self.pressure_values)

        # 3️⃣ Exponential Moving Average (EMA) - Further stabilize readings
        if self.ema_filtered_value is None:
            self.ema_filtered_value = P_smooth  # Initialize EMA with first smoothed value
        else:
            self.ema_filtered_value = (EMA_ALPHA * P_smooth) + ((1 - EMA_ALPHA) * self.ema_filtered_value)

        # Convert pressure to altitude
        current_altitude = pressure_to_altitude(self.ema_filtered_value)

        # Set initial altitude as reference (takeoff altitude = 0.06m)
        if self.initial_altitude is None:
            self.initial_altitude = current_altitude
            self.get_logger().info(f"Initial altitude set to {self.initial_altitude:.2f} m")

        relative_altitude = (current_altitude - self.initial_altitude) + TAKEOFF_ALTITUDE
        self.get_logger().info(f"Filtered Pressure: {self.ema_filtered_value:.2f} Pa -> Relative Altitude: {relative_altitude:.2f} m")

        # Publish relative altitude
        self.publisher.publish(Float32(data=relative_altitude))

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
