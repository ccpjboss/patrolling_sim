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

"""
This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks
"""

import os

from ament_index_python.packages import get_package_share_directory
import configparser
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

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
    loadInitPoses()

    map_name_str = LaunchConfiguration('map_path').perform(context)
    n_robots_str = LaunchConfiguration('n_robots').perform(context)
    n_robots = int(n_robots_str)
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    scenario = map_name_str+'_'+n_robots_str
    iposes = initPoses[scenario.lower()]
    iposes = iposes.replace('[','')
    iposes = iposes.replace(']','')
    iposes = iposes.split(',')

    map_file = os.path.join(package_path, 'maps',map_name_str,map_name_str+'.yaml')
    #params_file = os.path.join(package_path, 'params/nav2_params.yaml')
    launch_dir = os.path.join(package_path, 'launch')
    rviz_config_file = os.path.join(package_path,'rviz','rviz_config_2.rviz')
    print(rviz_config_file)

    # Names and poses of the robots
    robots = []
    for i in range(n_robots):
        robots.append({'name': ('robot'+str(i)), 'x_pose': iposes[i*2], 'y_pose': iposes[i*2+1]})
    print(robots)

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(f"{robot['name']}_params_file")

        group = GroupAction([

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_path,
                                                           'launch',
                                                           'patrol_robot_simulation.launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'map': map_file,
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                                  'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                  'autostart': 'True',
                                  'headless': 'False'}.items()),

            Node(
                package='tf_demux',
                executable='tf_demux',
                namespace=robot['name'],
                parameters=[{"robot_prefix": robot['name']}])
        ])

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    #ld = LaunchDescription()

    #for simulation_instance_cmd in nav_instances_cmds:
    #    ld.add_action(simulation_instance_cmd)

    #return ld
    return nav_instances_cmds

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot0_params_file', 
                              default_value = os.path.join(package_path,'params','nav2_multirobot_params_0.yaml'),
                              description='Robot0 params file'),
        DeclareLaunchArgument('robot1_params_file', 
                              default_value = os.path.join(package_path,'params','nav2_multirobot_params_1.yaml'),
                              description='Robot1 params file'),
        DeclareLaunchArgument('robot2_params_file', 
                              default_value = os.path.join(package_path,'params','nav2_multirobot_params_2.yaml'),
                              description='Robot2 params file'),
        DeclareLaunchArgument('robot3_params_file', 
                              default_value = os.path.join(package_path,'params','nav2_multirobot_params_3.yaml'),
                              description='Robot3 params file'),
        DeclareLaunchArgument('robot4_params_file', 
                              default_value = os.path.join(package_path,'params','nav2_multirobot_params_4.yaml'),
                              description='Robot4 params file'),
        DeclareLaunchArgument('robot5_params_file', 
                              default_value = os.path.join(package_path,'params','nav2_multirobot_params_5.yaml'),
                              description='Robot5 params file'),
        DeclareLaunchArgument('robot6_params_file', 
                              default_value = os.path.join(package_path,'params','nav2_multirobot_params_6.yaml'),
                              description='Robot6 params file'),
        DeclareLaunchArgument('robot7_params_file', 
                              default_value = os.path.join(package_path,'params','nav2_multirobot_params_7.yaml'),
                              description='Robot7 params file'),
        DeclareLaunchArgument('map_path'),
        DeclareLaunchArgument('n_robots'),
        OpaqueFunction(function=launch_setup)
    ])

generate_launch_description()
