
# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import configparser

initPoses = {}
package_path = get_package_share_directory('patrolling_sim_ros2')

def loadInitPoses():
    try:
        ConfigIP = configparser.ConfigParser()
        ConfigIP.read(package_path+"/params/initial_poses.txt")
        for option in ConfigIP.options("InitialPoses"):
            initPoses[option] = ConfigIP.get("InitialPoses", option)
    except:
        print("Could not load initial poses file")

def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    launch_dir = os.path.join(package_path, 'launch')

    # Names and poses of the robots
    loadInitPoses()

    map_name_str = LaunchConfiguration('map_name').perform(context)
    n_robots_str = LaunchConfiguration('n_robots').perform(context)
    n_robots = int(n_robots_str)

    robot_id_str = LaunchConfiguration('robot_id').perform(context)
    robot_id = int(robot_id_str)

    algorithm = LaunchConfiguration('algo').perform(context)

    scenario = map_name_str+'_'+n_robots_str
    iposes = initPoses[scenario.lower()]
    iposes = iposes.replace('[','')
    iposes = iposes.replace(']','')
    iposes = iposes.split(',')

    robots = []
    for i in range(n_robots):
        robots.append({'name': ('robot_'+str(i)), 'x_pose': iposes[i*2], 'y_pose': iposes[i*2+1],
        'z_pose': 0.01, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0})
    print(robots)

    print(robots[robot_id])

    # Simulation settings
    simulator = LaunchConfiguration('simulator')

    # On this example all robots are launched with the same settings
    # map_yaml_file = LaunchConfiguration('map_path')

    # Declare the launch arguments
    world_str = package_path +'/worlds/' + map_name_str + '.world'

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(package_path, 'maps','1r5', '1r5.yaml'),
        description='Full path to map file to load')

    # Define commands for launching the navigation instances
    # params_file = LaunchConfiguration(f"{robot['name']}_params_file")
    params_file = package_path + '/params' + '/nav2_multirobot_params_'+robot_id_str+'.yaml' 
    print(params_file)

    robot = robots[robot_id]

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_path,
                                                   'launch',
                                                   'stop_simulation_launch.py')),
        launch_arguments={'namespace': robot['name'],
                          'use_namespace': 'True',
                          'map': package_path+'/maps/'+map_name_str+'/'+map_name_str+'.yaml',
                          'use_sim_time': 'True',
                          'params_file': params_file,
                          'autostart': 'True',
                          'use_rviz': 'False',
                          'use_simulator': 'False',
                          'headless': 'False',
                          'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                          'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                          'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                          'roll': TextSubstitution(text=str(robot['roll'])),
                          'pitch': TextSubstitution(text=str(robot['pitch'])),
                          'yaw': TextSubstitution(text=str(robot['yaw'])),
                          'robot_name': TextSubstitution(text=robot['name']), }.items())

    patrol_node = Node(
        package='patrolling_sim_ros2',
        executable=algorithm,
        name='patrol_robot_'+robot_id_str,
        namespace=robot['name'],
        arguments=['0',map_name_str,robot_id_str],
        parameters=[{"initial_pos": [float(robot['x_pose']), float(robot['y_pose'])]}])

    nav2_timer = TimerAction(period=3.0, actions=[nav2_launch_cmd])
    patrol_timer = TimerAction(period=10.0, actions=[patrol_node])

    return [nav2_timer, patrol_timer]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_name',description="Name of the map to use"),
        DeclareLaunchArgument('n_robots', description="Number of robots composing the team"),
        DeclareLaunchArgument('robot_id', description="ID of the robot"),
        DeclareLaunchArgument('algo',default_value='Random', description="Algorithm to run Random or Cyclic"),
        OpaqueFunction(function=launch_setup)
    ])

generate_launch_description()
