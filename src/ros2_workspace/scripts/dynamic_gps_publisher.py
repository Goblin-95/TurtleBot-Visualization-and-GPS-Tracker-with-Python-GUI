#!/usr/bin/env python3
"""
Dynamic GPS Publisher for TurtleBot3
Converts robot odometry to GPS coordinates that change with movement
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math

class DynamicGPSPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_gps_publisher')
        
        # GPS Publisher
        self.gps_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Odometry Subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Base GPS coordinates (Chicago area - like your original)
        # These are realistic coordinates near Chicago
        self.base_latitude = 42.0608  # Chicago area
        self.base_longitude = -87.6755  # Chicago area
        self.base_altitude = 200.0
        
        # Conversion factors (meters to GPS degrees)
        # Making GPS changes MORE VISIBLE for testing
        self.meters_to_lat = 1 / 1000.0  # Much larger changes - 1 meter = 0.001 degrees
        self.meters_to_lon = 1 / 1000.0  # This will make GPS changes very obvious
        
        # Timer to publish GPS at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_gps)
        
        # Current robot position
        self.current_x = 0.0
        self.current_y = 0.0
        
        self.get_logger().info('🛰️  Dynamic GPS Publisher started!')
        self.get_logger().info(f'📍 Base GPS: {self.base_latitude:.6f}, {self.base_longitude:.6f}')
        self.get_logger().info('🤖 Move robot with teleop to see GPS coordinates change!')
        
    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
    def publish_gps(self):
        """Convert robot position to GPS and publish"""
        # Convert robot position (meters) to GPS offset
        lat_offset = self.current_x * self.meters_to_lat
        lon_offset = self.current_y * self.meters_to_lon
        
        # Calculate actual GPS coordinates
        current_lat = self.base_latitude + lat_offset
        current_lon = self.base_longitude + lon_offset
        
        # Create GPS message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps_link'
        
        # GPS status (0 = no fix, 1 = fix, 2 = DGPS fix)
        gps_msg.status.status = 1  # Good GPS fix
        gps_msg.status.service = 1  # GPS service
        
        # GPS coordinates
        gps_msg.latitude = current_lat
        gps_msg.longitude = current_lon
        gps_msg.altitude = self.base_altitude
        
        # Position covariance (uncertainty)
        gps_msg.position_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        gps_msg.position_covariance_type = 1  # Diagonal known
        
        # Publish GPS message
        self.gps_publisher.publish(gps_msg)
        
        # Log more frequently to see changes
        if int(self.get_clock().now().nanoseconds / 1e9) % 5 == 0:
            self.get_logger().info(
                f'📡 GPS: Pos({self.current_x:.2f}, {self.current_y:.2f}) → '
                f'GPS({current_lat:.6f}, {current_lon:.6f})'
            )

def main(args=None):
    rclpy.init(args=args)
    
    node = DynamicGPSPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 GPS Publisher stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
