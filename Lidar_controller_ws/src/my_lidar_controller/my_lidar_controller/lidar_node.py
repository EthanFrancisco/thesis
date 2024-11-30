#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from gpiozero import LED
import serial
import time
import smbus

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


class MyNode(Node):
    def __init__(self):
        super().__init__('lidar_data')
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.timer_callback,
              1)
        
        # Initialize I2C bus
        self.bus = smbus.SMBus(1)
        self.address = 0x1e

        # Initialize HMC5883L
        self.bus.write_byte_data(self.address, 0x00, 0x70)  # Configuration Register A
        self.bus.write_byte_data(self.address, 0x01, 0xA0)  # Configuration Register B
        self.bus.write_byte_data(self.address, 0x02, 0x00)  # Mode Register

    def timer_callback(self, msg: LaserScan):
        polar_data = msg.ranges
        
        cartesian_data = self.polar_to_cartesian(polar_data, msg.angle_min, msg.angle_increment)
        
        # Determine the region of the points
        points_center, points_left, points_right = self.determine_region(cartesian_data)

        L_detected = len(points_left) > 2
        C_detected = len(points_center) > 2
        R_detected = len(points_right) > 2


        if not L_detected and not C_detected and not R_detected:
            detection_data = "A"
        elif not L_detected and not C_detected and R_detected:
            detection_data = "B"
        elif not L_detected and C_detected and not R_detected:
            detection_data = "C"
        elif not L_detected and C_detected and R_detected:
            detection_data = "D"
        elif L_detected and not C_detected and not R_detected:
            detection_data = "E"
        elif L_detected and not C_detected and R_detected:
            detection_data = "F"
        elif L_detected and C_detected and not R_detected:
            detection_data = "G"
        elif L_detected and C_detected and R_detected:
            detection_data = "H"
        else:
            detection_data = "Unknown"                                                                                                                                                                                                                                         

        # Read data from the HMC5883L
        x, y, z = self.read_hmc5883l()

        # Calculate heading
        heading = self.calculate_heading(x, y)
        self.get_logger().info(f"Heading: {heading} degrees")
        # time.sleep(0.5)

        if ser_GPS.in_waiting > 0:
            line = ser_GPS.readline()
            self.get_logger().info(f"hello {line}")
                # time.sleep(0.5)

        # Example of converting Cartesian to Polar coordinates
        # polar_coords = self.cartesian_to_polar([(x,y)])
        # for r, theta in polar_coords:
        #     self.get_logger().info(f"Polar Coord - r: {r}, theta: {theta}")
        data_to_send = f"{detection_data}|{int(heading)}\n".encode()
        ser.write(data_to_send)
        self.get_logger().info(f"sent: {data_to_send}")


    def read_hmc5883l(self):
        data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
        x = data[0] * 256 + data[1]
        if x > 32767:
            x -= 65536
        y = data[4] * 256 + data[5]
        if y > 32767:
            y -= 65536
        z = data[2] * 256 + data[3]
        if z > 32767:
            z -= 65536
        return x, y, z

    def calculate_heading(self, x, y):
        heading = math.atan2(y, x)
        if heading < 0:
            heading += 2 * math.pi
        heading = math.degrees(heading)
        heading = heading-14-67-8+21-18
        if heading < 0:
            heading += 360
        return heading

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