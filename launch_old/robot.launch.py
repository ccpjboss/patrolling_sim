import os
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch.actions import OpaqueFunction
import configparser
from math import radians

initPoses = {}

package_path = get_package_share_directory('patrolling_sim_ros2')

def loadInitPoses():
    try:
        ConfigIP = configparser.ConfigParser()
        ConfigIP.read(package_path+"/params/initial_poses.txt")
        for option in ConfigIP.options("InitialPoses"):
            print(option)
            initPoses[option] = ConfigIP.get("InitialPoses", option)
    except:
        print("Could not load initial poses file")


def launch_setup(context, *args, **kwargs):
    loadInitPoses()

    robot_name_str = LaunchConfiguration('robotname').perform(context)
    #(name, i) = robot_name_str.split(sep='_')
    i = robot_name_str.rsplit('robot')
    map_name_str = LaunchConfiguration('map_path').perform(context)
    n_robots_str = LaunchConfiguration('n_robots').perform(context)

    print(robot_name_str + ' ' + map_name_str + '_' + n_robots_str)
    scenario = map_name_str+'_'+n_robots_str
    iposes = initPoses[scenario.lower()]
    print(scenario, '   ', iposes)
    iposes = iposes.replace('[','')
    iposes = iposes.replace(']','')
    iposes = iposes.split(',')
    print(iposes)
    initial_x = float(iposes[int(i[1])*2])
    print(initial_x)
    initial_y = float(iposes[int(i[1])*2+1])
    print(initial_y)
    initial_t = radians(90)

    default_map_dir = os.path.join(package_path, 'maps/1r5/1r5.yaml')
    default_params_dir = os.path.join('.', 'params/nav2_params.yaml')

    lifecycle_nodes_localization = ['map_server', 'amcl']

    lifecycle_nodes_navigation = ['controller_server',
                                  'planner_server',
                                  'recoveries_server',
                                  'bt_navigator',
                                  'waypoint_follower']

    remappings = [(robot_name_str + '/tf', 'tf'),
                  (robot_name_str + '/tf_static', 'tf_static')]

    use_amcl_arg = DeclareLaunchArgument(
        "use_amcl",
        default_value="true")

    use_nav2_arg = DeclareLaunchArgument(
        "use_nav2",
        default_value="true")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true")

    param_files_arg = DeclareLaunchArgument(
        "params_file",
        default_value=TextSubstitution(text=default_params_dir))

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    print(robot_name_str)

    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'map_server.ros__parameters.yaml_filename': (package_path + '/maps/'+map_name_str+'/'+map_name_str+'.yaml'),
        'amcl.ros__parameters.base_frame_id': (robot_name_str + '/base_footprint'),
        'amcl.ros__parameters.odom_frame_id': (robot_name_str + '/odom'),
        'amcl.ros__parameters.scan_topic': ('/'+robot_name_str + '/ranger_0'),
        'amcl.ros__parameters.initial_pose.x': str(initial_x),
        'amcl.ros__parameters.initial_pose.y': str(initial_y),
        'bt_navigator.ros__parameters.robot_base_frame': (robot_name_str + '/base_link'),
        'bt_navigator.ros__parameters.odom_topic': ('/'+robot_name_str + '/odom'),
        'local_costmap.local_costmap.ros__parameters.global_frame': (robot_name_str + '/odom'),
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': (robot_name_str + '/base_link'),
        'local_costmap.local_costmap.ros__parameters.map_topic': ('/'+robot_name_str + '/map'),
        'local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic': ('/'+robot_name_str + '/ranger_0'),
        'global_costmap.global_costmap.ros__parameters.robot_base_frame': (robot_name_str + '/base_link'),
        'global_costmap.global_costmap.ros__parameters.map_topic': ('/'+robot_name_str + '/map'),
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': ('/'+robot_name_str + '/ranger_0'),
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.sensor_frame': (robot_name_str + '/base_scan'),
        'recoveries_server.ros__parameters.global_frame': (robot_name_str + '/odom'),
        'recoveries_server.ros__parameters.robot_base_frame': (robot_name_str + '/base_link')
        }

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=LaunchConfiguration('robotname'),
        param_rewrites=param_substitutions,
        convert_types=True),

    robot_group = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                namespace = robot_name_str,
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
                arguments=['--ros-args','--log-level','DEBUG']
            ),

            Node(
                package='nav2_amcl',
                executable='amcl',
                namespace = robot_name_str,
                name='amcl',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
                arguments=['--ros-args','--log-level','DEBUG']),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                namespace = robot_name_str,
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                            {'autostart': LaunchConfiguration('autostart')},
                            {'node_names': lifecycle_nodes_localization}],
                arguments=['--ros-args','--log-level','DEBUG']),

            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
                namespace = robot_name_str,
                arguments=['--ros-args','--log-level','DEBUG']),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                namespace = robot_name_str,
                remappings=remappings,
                arguments=['--ros-args','--log-level','DEBUG']),

            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                output='screen',
                namespace = robot_name_str,
                parameters=[configured_params],
                remappings=remappings,
                arguments=['--ros-args','--log-level','DEBUG']),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace = robot_name_str,
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
                arguments=['--ros-args','--log-level','DEBUG']),

            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
                namespace = robot_name_str,
                arguments=['--ros-args','--log-level','DEBUG']),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                namespace = robot_name_str,
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                            {'autostart': LaunchConfiguration('autostart')},
                            {'node_names': lifecycle_nodes_navigation}],
                arguments=['--ros-args','--log-level','DEBUG'])
        ]
    )
    return [
        # map_path_arg,
        use_amcl_arg,
        use_sim_time_arg,
        use_nav2_arg,
        param_files_arg,
        autostart_arg,
        robot_group
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robotname'),
        DeclareLaunchArgument('map_path'),
        DeclareLaunchArgument('n_robots'),
        OpaqueFunction(function=launch_setup)
    ])

generate_launch_description()
