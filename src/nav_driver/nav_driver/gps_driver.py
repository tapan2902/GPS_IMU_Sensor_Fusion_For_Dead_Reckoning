#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from gps_msgs.msg import GPSmsg

from builtin_interfaces.msg import Time

import time
import serial
import utm
import sys

def parse_gps_data(data):
    if data.startswith('$GPGGA'):
        parts = data.split(',')
        return parts


class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_node')
        port = "/dev/ttyUSB0"

        args = sys.argv[0:]
        self.get_logger().info(f'GPS array {args}')
        for i in range(len(args)):
            if args[i].startswith("gps_port"):
                port = args[i+1]
                break

        self.gps_port = port
        self.get_logger().info(f'GPS port value is: {self.gps_port}')

        self.ser = self.initialize_serial_port()

        self.publisher_ = self.create_publisher(GPSmsg, '/gps', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def initialize_serial_port(self):
        max_tries = 0
        while max_tries < 10:
            
            try:
                ser = serial.Serial(self.gps_port, baudrate=4800, timeout=1)
                self.get_logger().info(f'Successfully opened serial port: {self.gps_port}')
                return ser
            except serial.SerialException as e:
                self.get_logger().warn(f'Failed to open serial port {self.gps_port}: {e}')
                time.sleep(1)
            finally:
                max_tries += 1    
        self.get_logger().error(f'Failed to open serial port {self.gps_port}')
        exit(0)        

    def parse_time(self, hhmmss):
        hhmmss = str(hhmmss)
        # self.get_logger().info(f'{hhmmss} received string time')
        hours = int(hhmmss[:2])
        minutes = int(hhmmss[2:4])
        seconds = int(hhmmss[4:6])
        nanoseconds = int(hhmmss[7:])
        # self.get_logger().info(f'{hours}, {minutes}, {seconds}, {nanoseconds}')

        # Convert everything to seconds
        total_seconds = hours * 3600 + minutes * 60 + seconds
        return total_seconds, nanoseconds
    
    def convert_to_ros_time(self, seconds, nanoseconds):
        # self.get_logger().info(f'System time is : {seconds}, {nanoseconds}')
        ros_time = Time()
        ros_time.sec = seconds
        ros_time.nanosec = nanoseconds
        return ros_time
    
    def timer_callback(self):
        
        line = self.ser.readline().decode('ascii', errors='replace').strip()
        gps_coords = parse_gps_data(line)
        
        if gps_coords:
            gps_req = [gps_coords[2], gps_coords[3], gps_coords[4], gps_coords[5], gps_coords[9]]
            if next((val for val in gps_req if val is None or val == ''), None) is None:
                msg = GPSmsg()
                current_time = self.get_clock().now()

                seconds = current_time.seconds_nanoseconds()[0]
                nanoseconds = current_time.seconds_nanoseconds()[1]

                # Header
                self.header = Header()
                ros_time = self.convert_to_ros_time(seconds, nanoseconds)
                self.header.stamp = ros_time
                self.header.frame_id = "GPS1_Frame"

                #                   degree                    minutes          / 60
                latitude =  int(gps_coords[2][0:2]) + float(gps_coords[2][2:]) / 60 # latitude index 2
                
                longitude = int(gps_coords[4][0:3]) + float(gps_coords[4][3:]) / 60 # longitude index 3
                
                if(gps_coords[3] == 'S'):
                    latitude *= -1
                if(gps_coords[5] == 'W'):
                    longitude *= -1

                altitude = float(gps_coords[9])
                
                # Convert to UTM coordinates
                utm_coords = utm.from_latlon(latitude=latitude, longitude=longitude)
                msg.header = self.header
                msg.latitude = latitude
                msg.longitude = longitude
                msg.altitude = altitude
                msg.utm_easting = float(utm_coords[0])
                msg.utm_northing = float(utm_coords[1])
                msg.zone = int(utm_coords[2])
                msg.letter = utm_coords[3]
                self.publisher_.publish(msg)
                
                #self.get_logger().info(f'Publishing: {msg}')
            else:
                self.get_logger().info(f'Waiting..')
        

def main(args=None):
    rclpy.init(args=args)
    my_publisher = GpsPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
