import os
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction

package_path = get_package_share_directory('patrolling_sim_ros2')

def launch_setup(context, *args, **kwargs):

    n_robots_str = LaunchConfiguration('n_robots').perform(context)
    default_rviz_dir = os.path.join(package_path, 'rviz')
    rviz_config = os.path.join(default_rviz_dir,'rviz_config_'+n_robots_str+'.rviz')
    print(rviz_config)

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     arguments=['-d', rviz_config],
                     )

    return [rviz_node]

def generate_launch_description():
    return LaunchDescription([
       DeclareLaunchArgument('n_robots'),
        OpaqueFunction(function=launch_setup)
    ])

generate_launch_description()
