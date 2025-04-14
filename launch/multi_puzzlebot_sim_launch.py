#IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os
from ament_index_python.packages import get_package_share_directory
import sys

#IMPORTS REQUIRED FOR Launching Nodes
from launch import LaunchDescription
from launch_ros.actions import Node

#IMPORTS REQUIRED FOR EVENTS AND ACTIONS
from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    num_puzzlebots = int(sys.argv[4:][0].split('=')[1])
 
    urdf_file_name = 'multi_puzzlebot.urdf'
    urdf_default_path = os.path.join(
                        get_package_share_directory('aphelion'),
                        'urdf',
                        urdf_file_name)
   
    with open(urdf_default_path, 'r') as infp:
        robot_desc = infp.read()

    nodes = []

    for i in range(num_puzzlebots):
      robot_name = 'puzzlebot_' + str(i+1)
      robot_description = robot_desc.replace('pname', robot_name)
      robot_state_pub_node = Node(
                              package='robot_state_publisher',
                              executable='robot_state_publisher',
                              name='robot_state_publisher',
                              output='screen',
                              parameters=[{'robot_description': robot_description}],
                              remappings=[('/robot_description', '/' + robot_name + '/robot_description')],
                              )
      nodes.append(robot_state_pub_node)

      simple_kinematics_node = Node(
                                package='aphelion',
                                executable='simple_kinematics_node',
                                name=robot_name + '_simple_kinematics_node',
                                output='screen',
                                parameters=[{'puzzlebot_id': i+1,
                                             'use_prefix': True }],
                                )

      nodes.append(simple_kinematics_node)

      asmc_node = Node(
                   package='aphelion',
                   executable='asmc_node',
                   name=robot_name + '_asmc_node',
                   output='screen',
                   parameters=[{'puzzlebot_id': i+1,
                                'use_prefix': True }],
                   )

      nodes.append(asmc_node)

    l_d = LaunchDescription(nodes)

    return l_d
