#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from gpiozero import LED
import serial


# Define GPIO pin
GPIO_PIN = 18
led = LED(GPIO_PIN)
ser = serial.Serial("/dev/ttyACM0", 57600, timeout=0.250)


class MyNode(Node):

    def __init__(self):
        super().__init__('lidar_data')
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.timer_callback,
            10)

    def timer_callback(self, msg: LaserScan):
        polar_data = msg.ranges
        cartesian_data = self.polar_to_cartesian(polar_data, msg.angle_min, msg.angle_increment)
        
        # Example usage of the point_in_boundary function
        for point in cartesian_data:
            if self.point_in_boundary(point, (0, 0.0254*2), (0, 0.0254*2)):
                self.get_logger().info(f"Point {point} is within the boundary")
                led.on()
                ser.write(f"A\n".encode("ascii"))
                print(f"Data sent: {point}\n")
            else:
                ser.write(f"B\n".encode("ascii"))
                led.off()

         # Write data to the serial port
        
        # Read data from the serial port


    def polar_to_cartesian(self, ranges, angle_min, angle_increment):
        cartesian_coords = []
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            cartesian_coords.append((x, y))
        return cartesian_coords
    
    def point_in_boundary(self, point, x_bounds, y_bounds):
        x, y = point
        x_min, x_max = x_bounds
        y_min, y_max = y_bounds
        return x_min <= x <= x_max and y_min <= y <= y_max

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()