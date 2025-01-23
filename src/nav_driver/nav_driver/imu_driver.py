import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import MagneticField, Imu

from imu_msgs.msg import IMUmsg

from builtin_interfaces.msg import Time

import time
import serial
import math
import sys

def parse_gps_data(data):
    if data.startswith('$VNYMR'):
        parts = data.split(',')
        return parts

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('imu_node')
        port = "/dev/ttyUSB0"

        args = sys.argv[0:]
        self.get_logger().info(f'IMU array {args}')
        for i in range(len(args)):
            if args[i].startswith("imu_port"):
                port = args[i+1]
                break

        self.imu_port = port
        self.get_logger().info(f'IMU port value is: {self.imu_port}')

        self.ser = self.initialize_serial_port()
        cmd = "VNWRG,07,40"
        checksum = 0
        for char in cmd:
            checksum ^= ord(char)
        cmd_with_cksum = f"${cmd}*{checksum:02X}\r\n"
        self.ser.write(cmd_with_cksum.encode('utf-8'))
        self.get_logger().info(cmd_with_cksum)
        self.get_logger().info(f'Command Sent to IMU: {cmd_with_cksum.strip()}')
        
        self.publisher_ = self.create_publisher(IMUmsg, '/imu', 40)
        timer_period = 0.025
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        def create_command_with_checksum(command_str):
            checksum = 0
            for char in command_str:
                checksum ^= ord(char)
    
            full_command = f"${command_str}*{checksum:02X}\r\n"
            return full_command
    
    def initialize_serial_port(self):
        max_tries = 0
        while max_tries < 10:
            try:
                ser = serial.Serial(self.imu_port, baudrate=115200, timeout=1)
                self.get_logger().info(f'Successfully opened serial port: {self.imu_port}')
                return ser
            except serial.SerialException as e:
                self.get_logger().warn(f'Failed to open serial port {self.imu_port}: {e}')
                time.sleep(1)
            finally:
                max_tries += 1    
        self.get_logger().error(f'Failed to open serial port {self.imu_port}')
        exit(0)        
    
    def convert_to_ros_time(self, seconds, nanoseconds):
        # self.get_logger().info(f'System time is : {seconds}, {nanoseconds}')
        ros_time = Time()
        ros_time.sec = seconds
        ros_time.nanosec = nanoseconds
        return ros_time
    
    def euler_to_quaternion(self, yaw, pitch, roll):
        # Since vector nav imu gives out in degrees, we need radians
        yaw = (math.pi * yaw) / 180
        pitch = (math.pi * pitch) / 180
        roll = (math.pi * roll) / 180

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return qx, qy, qz, qw

    def timer_callback(self):
        
        line = self.ser.readline().decode('ascii', errors='replace').strip()
        imu_data = parse_gps_data(line)
        
        if imu_data:
            if next((val for val in imu_data if val is None or val == ''), None) is None:
                msg = IMUmsg()

                current_time = self.get_clock().now()

                seconds = current_time.seconds_nanoseconds()[0]
                nanoseconds = current_time.seconds_nanoseconds()[1]
                
                # Header
                self.header = Header()
                ros_time = self.convert_to_ros_time(seconds, nanoseconds)
                self.header.stamp = ros_time
                self.header.frame_id = "IMU1_Frame"
                msg.header = self.header

                # Convert to Quaternion Orientation Calculations
                orientation = Quaternion()
                yaw = float(imu_data[1])
                pitch = float(imu_data[2])
                roll = float(imu_data[3])  
                qx, qy, qz, qw = self.euler_to_quaternion(yaw, pitch, roll)
                orientation.z, orientation.y, orientation.x, orientation.w = qz, qy, qx, qw

                # Magnetic Field
                mag_field = Vector3()
                mag_field.x = float(imu_data[4]) * 1e-4  # in Tesla
                mag_field.y = float(imu_data[5]) * 1e-4
                mag_field.z = float(imu_data[6]) * 1e-4

                # Linear Acceleration
                linear_acceleration = Vector3()
                linear_acceleration.x = float(imu_data[7])
                linear_acceleration.y = float(imu_data[8])
                linear_acceleration.z = float(imu_data[9])     

                # Angular Velocity
                angular_velocity = Vector3()
                angular_velocity.x = float(imu_data[10])
                angular_velocity.y = float(imu_data[11])
                angular_velocity.z = float(imu_data[12][:-3])
                
                # Imu message
                msg.imu.orientation = orientation
                msg.imu.linear_acceleration = linear_acceleration
                msg.imu.angular_velocity = angular_velocity

                # Mag Field message
                msg.mag_field.magnetic_field = mag_field

                # Raw Imu data
                msg.raw_imu = str(imu_data)

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


 # sensor_msgs/IMU imu
                # sensor_msgs/MagneticField mag_field
                # string raw_imu

                # geometry_msgs/Quaternion orientation
                # float64[9] orientation_covariance # Row major about x, y, z axes

                # geometry_msgs/Vector3 angular_velocity
                # float64[9] angular_velocity_covariance # Row major about x, y, z axes
                # UNIT rad/sec
                                
                # geometry_msgs/Vector3 linear_acceleration
                # float64[9] linear_acceleration_covariance # Row major x, y z
                # UNIT m/s^2

                # geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
                # If your sensor does not output 3 axes,
                # UNIT Tesla

                # float64[9] magnetic_field_covariance # Row major about x, y, z axes
                # 0 is interpreted as variance unknown