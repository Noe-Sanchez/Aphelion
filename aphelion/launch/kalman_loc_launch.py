#IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os
from ament_index_python.packages import get_package_share_directory

#IMPORTS REQUIRED FOR Launching Nodes
from launch import LaunchDescription
from launch_ros.actions import Node

#IMPORTS REQUIRED FOR EVENTS AND ACTIONS
from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
 
    urdf_file_name = 'puzzlebot.urdf'
    urdf_default_path = os.path.join(
                        get_package_share_directory('aphelion'),
                        'urdf',
                        urdf_file_name)
   
    with open(urdf_default_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
                            )
    
    image_node = Node(
                  package='aruco_loc',
                  executable='image_generator_node',
                  name='image_gen_node',
                  output='screen',
                  parameters=[{"capture_width": 640,
                               "capture_height": 480,
                               "display_width": 640,
                               "display_height": 480,
                               "framerate": 30,
                               "flip_method": 0,
                               "camera_info_url": "file:///home/puzzlebot/camera_info/ost.yaml"
                               }]
                  )
    
    marker_node = Node(
                  package='aruco_loc',
                  executable='markers_publisher_node',
                  name='marker_node',
                  output='screen',
                  parameters=[{"markerLength": 0.11,
                               "use_prefix": False,
                               }]
                  )

    odom_node = Node(
                  package='aphelion',
                  executable='odom_node',
                  name='odom_node',
                  output='screen',
                  parameters=[{'use_prefix': False }],
                  )

    l_d = LaunchDescription([robot_state_pub_node, image_node, marker_node, odom_node])

    return l_d
