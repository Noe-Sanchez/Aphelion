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

    l_d = LaunchDescription([robot_state_pub_node, kinematics_node, asmc_node])

    return l_d
