import os

# from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent, RegisterEventHandler
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.conditions import IfCondition, UnlessCondition
# from launch.event_handlers import OnProcessExit
# from launch.events import Shutdown
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node





def generate_launch_description():
    # delare any path variable
    # my_pkg_path = get_package_share_directory('move_to_goal')
    
    move_to_goal_node = Node(
        package='move_to_goal',
        executable='move_to_goal',
        output='screen'
    )
    
    stop_motion_node = Node(
        package='move_to_goal',
        executable='stop_motion',
        output='screen'
    )






     # Create the launch description and populate
    ld = LaunchDescription()
    

    # Add the nodes to the launch description
    ld.add_action(move_to_goal_node)
    ld.add_action(stop_motion_node)

    
    return ld      # return (i.e send) the launch description for excecution

