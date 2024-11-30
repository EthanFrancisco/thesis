#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from gpiozero import LED
import serial
import time
import smbus2

# Define GPIO pin
GPIO_PIN = 18
led = LED(GPIO_PIN)
ser = serial.Serial(
    "/dev/ttyACM0", 
    57600, 
    timeout=0.250
    )
ser_GPS = serial.Serial(
    "/dev/ttyS0", 
    9600, 
    timeout=0.250
    )

# Initialize I2C bus
bus = smbus2.SMBus(1)

class MyNode(Node):
    def __init__(self):
        super().__init__('lidar_data')
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.timer_callback,
             1)

    def timer_callback(self, msg: LaserScan):
        polar_data = msg.ranges
        
        cartesian_data = self.polar_to_cartesian(polar_data, msg.angle_min, msg.angle_increment)
        
        # Determine the region of the points
        points_center, points_left, points_right = self.determine_region(cartesian_data)


        L_detected = len(points_left) > 2
        C_detected = len(points_center) > 2
        R_detected = len(points_right) > 2

        # if not L_detected and not  C_detected and not R_detected:
        #     ser.write(b'A\n')
        #     self.get_logger().info(f"SENT: A")
        # elif not L_detected and not C_detected and R_detected:
        #     ser.write(b'B\n')
        #     self.get_logger().info(f"SENT: B")
        # elif not L_detected and C_detected and not R_detected:
        #     ser.write(b'C\n')
        #     self.get_logger().info(f"SENT: C")
        # elif not L_detected and C_detected and R_detected:
        #     ser.write(b'D\n')
        #     self.get_logger().info(f"SENT: D")
        # elif L_detected and not C_detected and not R_detected:
        #     ser.write(b'E\n')
        #     self.get_logger().info(f"SENT: E")
        # elif L_detected and not C_detected and R_detected:
        #     ser.write(b'F\n')
        #     self.get_logger().info(f"SENT: F")
        # elif L_detected and C_detected and not R_detected:
        #     ser.write(b'G\n')
        #     self.get_logger().info(f"SENT: G")
        # elif L_detected and C_detected and R_detected:
        #     ser.write(b'H\n')
        #     self.get_logger().info(f"SENT: H")                                                                                                                                                                                                                                                      

        # Read data from the HMC5883L

        # # Calculate heading
        # heading = self.calculate_heading(x, y) - 65
        # self.get_logger().info(f"Heading: {heading} degrees")

        # if ser_GPS.in_waiting > 0:
        #     line = ser_GPS.readline()
        #     self.get_logger().info(f"hello {line}")

        # Example of converting Cartesian to Polar coordinates
        # polar_coords = self.cartesian_to_polar([(x,y)])
        # for r, theta in polar_coords:
        #     self.get_logger().info(f"Polar Coord - r: {r}, theta: {theta}")

 
    def polar_to_cartesian(self, ranges, angle_min, angle_increment):
        cartesian_coords = []
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            cartesian_coords.append((x, y))
        return cartesian_coords

    def cartesian_to_polar(self, cartesian_coords):
        polar_coords = []
        for x, y in cartesian_coords:
            r = math.sqrt(x**2 + y**2)
            theta = math.atan2(y, x)
            polar_coords.append((r, theta))
        return polar_coords

    def point_in_boundary(self, point, x_bounds, y_bounds):
        x, y = point
        x_min, x_max = x_bounds
        y_min, y_max = y_bounds
        return x_min <= x <= x_max and y_min <= y <= y_max

    def collect_points_in_boundary(self, cartesian_data, x_bounds, y_bounds):
        points_in_boundary = []
        for point in cartesian_data:
            if self.point_in_boundary(point, x_bounds, y_bounds):
                points_in_boundary.append(point)
        return points_in_boundary

    def determine_region(self, cartesian_data):
        center_bounds = ((-0.0254*10, 0.0254*10), (0.0254*3, 0.0254*20))
        left_bounds = ((-0.0254*20, -0.0254*10), (0.0254*3, 0.0254*20))
        right_bounds = (( 0.0254*10, 0.0254*20), (0.0254*3, 0.0254*20))
        
        points_right = self.collect_points_in_boundary(cartesian_data, *right_bounds)
        points_center = self.collect_points_in_boundary(cartesian_data, *center_bounds)
        points_left = self.collect_points_in_boundary(cartesian_data, *left_bounds)

      
        return points_center, points_left, points_right

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()