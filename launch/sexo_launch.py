#IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os, sys
from ament_index_python.packages import get_package_share_directory

#IMPORTS REQUIRED FOR Launching Nodes
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode

#IMPORTS REQUIRED FOR EVENTS AND ACTIONS
from launch.actions import  EmitEvent, LogInfo, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
import launch.events
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
 
    urdf_file_name = 'puzzlebot_real.urdf'
    #urdf_file_name = 'multi_puzzlebot.urdf'
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

    #asmc_node = Node(
    #             package='aphelion',
    #             executable='asmc_node',
    #             name='asmc_node',
    #             output='screen',
    #             )
    asmc_node = LifecycleNode( 
                              package='aphelion',
                              executable='asmc_node',
                              name='asmc_node',
                              output='screen',
                              namespace='',
                             )

    asmc_node_configure = EmitEvent(event=ChangeState( 
                              lifecycle_node_matcher=launch.events.matches_action(asmc_node),
                              transition_id=1,  # Configure transition
                              )) 

    ros_gz_bridge_node = Node(package='ros_gz_bridge',
                              executable='parameter_bridge',
                              name='ros_gz_bridge',
                              output='screen',
                              parameters=[{"config_file": os.path.join(get_package_share_directory('aphelion'), 'config', 'sim_bridge.yaml')}],
                             )

    rviz_node = Node(package='rviz2',
                              executable='rviz2',
                              name='rviz2',
                              output='screen',
                             )
    
    static_transform_node = Node(package='tf2_ros',
                              executable='static_transform_publisher',
                              name='map_to_odom',
                              output='screen',
                              arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
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
    
    odom_node = Node(package='aphelion',
                              executable='odom_node',
                              name='odom_node',
                              output='screen',
                              parameters=[{"use_prefix": False, 'sim_time': False}],
                             )

    marker_publisher_node = Node(package='aruco_loc',
                              executable='markers_publisher_node',
                              name='markers_publisher_node',
                              output='screen',
                              parameters=[{"use_prefix": False, "show_window": False, "markerLength": 0.11}],
                             )

    #l_d = LaunchDescription([robot_state_pub_node, asmc_node, ros_gz_bridge_node])
    #l_d = LaunchDescription([robot_state_pub_node, ros_gz_bridge_node, rviz_node, restamper_node, static_transform_node, odom_node, marker_publisher_node])
    # l_d = LaunchDescription([robot_state_pub_node, asmc_node, asmc_node_configure, odom_node, marker_publisher_node])
    l_d = LaunchDescription([robot_state_pub_node, image_node, asmc_node, asmc_node_configure, odom_node, marker_publisher_node])
    #l_d = LaunchDescription([robot_state_pub_node, image_node, asmc_node, asmc_node_configure, odom_node])

    return l_d
