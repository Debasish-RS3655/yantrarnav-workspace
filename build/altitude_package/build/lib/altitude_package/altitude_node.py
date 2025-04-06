#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32
import math

# Constants for barometric altitude calculation
P0 = 101325  # Sea level standard pressure in Pascals
T0 = 288.15  # Standard temperature in Kelvin
L = 0.0065   # Temperature lapse rate in K/m
R = 287.05   # Specific gas constant for dry air in J/(kg·K)
g0 = 9.80665  # Acceleration due to gravity in m/s²

def pressure_to_altitude(P):
    """Convert static pressure (P) in Pascals to altitude in meters."""
    return (T0 / L) * (1 - (P / P0) ** (R * L / g0))

class AltitudeEstimator(Node):
    def __init__(self):
        super().__init__('altitude_estimator')
        self.subscription = self.create_subscription(
            FluidPressure,
            '/mavros/imu/static_pressure',
            self.pressure_callback,
            10
        )
        self.publisher = self.create_publisher(Float32, '/mavros/altitude', 10)
        self.get_logger().info("Altitude estimation node started.")

    def pressure_callback(self, msg):
        P = msg.fluid_pressure  # Static pressure in Pascals
        altitude = pressure_to_altitude(P)
        self.get_logger().info(f"Pressure: {P:.2f} Pa -> Altitude: {altitude:.2f} m")
        self.publisher.publish(Float32(data=altitude))

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
