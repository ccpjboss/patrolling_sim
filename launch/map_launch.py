import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    package_path = get_package_prefix('patrolling_sim_ros2')
    default_map_dir = os.path.join(package_path,'../../src/patrolling_sim_ros2/maps/grid/grid.world')
    map_path_arg = DeclareLaunchArgument("map_path", 
                    default_value=TextSubstitution(text=default_map_dir),
                    description="Full path to .world file")

    stage_node = Node(package='stage_ros2',
                        executable='stage_ros2',
                        name='stage',
                        parameters=[{"world":LaunchConfiguration('map_path')}])

    return LaunchDescription([
        map_path_arg,
        stage_node,
    ])  

generate_launch_description()