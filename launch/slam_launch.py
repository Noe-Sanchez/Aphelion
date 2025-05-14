# ros2_ws/src/aphelion/launch/slam_launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

from launch_ros.actions import Node

def generate_launch_description():
    # ————————————————————————— SLAM Toolbox arguments —————————————————————————
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('aphelion'),
            'config',
            'mapper_params_online_async.yaml'
        ),
        description='Path to SLAM Toolbox parameters file'
    )
    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulated /clock'
    )
    declare_map_frame = DeclareLaunchArgument(
        'map_frame', default_value='map',
        description='Name of the map frame'
    )
    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame', default_value='odom',
        description='Name of the odom frame'
    )
    declare_base_frame = DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Name of the robot base frame'
    )
    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='Topic for LaserScan messages'
    )

    # Include SLAM Toolbox's own launch
    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_frame': LaunchConfiguration('map_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'scan_topic': LaunchConfiguration('scan_topic'),
        }.items()
    )

    # ————————————————————————— Your existing nodes —————————————————————————
    # RPLIDAR
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_link',
            'angle_compensate': True
        }],
    )

    # Static TF: base_link → laser_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_broadcaster',
        output='screen',
        arguments=[
            '0.0','0.0','0.15',
            '0','0','0',
            'base_link','laser_link'
        ],
    )

    # Robot state publisher (URDF)
    urdf_path = os.path.join(
        get_package_share_directory('aphelion'),
        'urdf',
        'puzzlebot.urdf'
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # Kinematics & ASMC
    kinematics_node = Node(
        package='aphelion',
        executable='simple_kinematics_node',
        name='simple_kinematics_node',
        output='screen',
        parameters=[{'use_prefix': False}],
    )
    asmc_node = Node(
        package='aphelion',
        executable='asmc_node',
        name='asmc_node',
        output='screen',
        parameters=[{'use_prefix': False}],
    )

    # RViz
    rviz_cfg = os.path.join(
        get_package_share_directory('aphelion'),
        'rviz',
        'view_laser.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
    )

    # ——————————————————————— Assemble and return ———————————————————————
    return LaunchDescription([
        # SLAM Toolbox args
        declare_slam_params,
        declare_use_sim,
        declare_map_frame,
        declare_odom_frame,
        declare_base_frame,
        declare_scan_topic,

        # Your core nodes
        rplidar_node,
        static_tf,
        robot_state_pub,
        kinematics_node,
        asmc_node,

        # SLAM Toolbox include
        slam_include,

        # RViz
        rviz_node,
    ])
