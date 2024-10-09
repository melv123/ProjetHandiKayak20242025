import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Nom du package
    #package_name = 'imu_package'

    command_path = os.path.join(get_package_share_directory('command_package'))
    robot_localization_file_path = os.path.join(command_path, 'config/ekf.yaml')

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[robot_localization_file_path]
        )

    #Launch Navsat
    navsat_transform_node = Node(
    package='robot_localization',
    executable='navsat_transform_node',
    name='navsat_transform_node',
    remappings=[
        ('gps/fix','Gps_readings'),
        #('odometry/filtered','telemetry/navsat_transform_odometry_output'),
        ("gps/filtered","telemetry/gnss/filtered"),
        ('imu','Imu_readings'),
    ],
    parameters=[{
        "publish_filtered_gps": True,
        "yaw_offset": 1.5707963,
        "zero_altitude": True,
        "use_odometry_yaw": False,
        "magnetic_declination_radians": 0.0383972435, # https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
        "datum": [52.00000, 4.00000,0.0]
    }],
        )

    # Launch the imu node
    imu_treatement = Node(
        package='imu_package', executable='imu_main',
        )

    # Launch the imu node
    imu_data = Node(
        package='imu_package', executable='imu_listener',
        )  
    
    # Launch the ekf_listener node
    ekf_listener = Node(
        package = 'command_package', executable = 'ekf_listener'
    )

    # Launch the gps_listener node
    gps = Node(
        package = 'gps_package', executable = 'gps_listener'
    )

    # Launch the gps_listener node
    buzzer = Node(
        package = 'buzzer_package', executable = 'buzzer_main'
    )

    return LaunchDescription([
        imu_data,
        imu_treatement,
        #gps, 
        #navsat_transform_node,
        start_robot_localization_cmd,
        ekf_listener,
        #buzzer,
        

    ])
