from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions.execute_local import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gps = LaunchConfiguration('gps_port', default='/dev/ttyUSB0')
    imu = LaunchConfiguration('imu_port', default='/dev/ttyUSB0')
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'gps_port',
            default_value=gps,
            description='Connected USB port for GPS'
        ),

        DeclareLaunchArgument(
            'imu_port',
            default_value=imu,
            description='Connected USB port for IMU'
        ),

        Node(
            package='nav_driver', 
            executable='gps_node', 
            name='gps_node', 
            output='screen',
            emulate_tty=True,
            arguments=['gps_port', gps],
        ),

        Node(
            package='nav_driver', 
            executable='imu_node', 
            name='imu_node', 
            output='screen',
            emulate_tty=True,
            arguments=['imu_port', imu],
        ),

    ])