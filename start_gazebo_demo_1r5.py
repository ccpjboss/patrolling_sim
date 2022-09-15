import os

monitor_cmd = "\"ros2 run patrolling_sim_ros2 monitor 1r5 MSP 2\""
os.system("gnome-terminal --tab -e " + monitor_cmd)
os.system("sleep 1")

param_cmd = "\"ros2 param set /monitor /initial_pos \"[4.0, 9.6, 2.7, 4.05]\"\""
os.system("gnome-terminal --tab -e " + param_cmd)
os.system("sleep 1")

gazebo_cmd = "\"ros2 launch stop_bot_description multi_stop_simulation_launch.py\""
os.system("gnome-terminal --tab -e " + gazebo_cmd)
os.system("sleep 1")

robot1_cmd = "\"ros2 run patrolling_sim_ros2 MSP 0 1r5 0 MSP/1r5/1r5_2_0 --ros-args --remap __name:=patrol_robot_0 -p initial_pos:=\"[4.0,9.6,2.7,4.05]\" -r __ns:=/robot0 -r /tf:=tf -r /tf_static:=tf_static\""
os.system("gnome-terminal --tab -e " + robot1_cmd)
os.system("sleep 1")

robot2_cmd = "\"ros2 run patrolling_sim_ros2 MSP 0 1r5 1 MSP/1r5/1r5_2_0 --ros-args --remap __name:=patrol_robot_1 -p initial_pos:=\"[4.0,9.6,2.7,4.05]\" -r __ns:=/robot1 -r /tf:=tf -r /tf_static:=tf_static\""
os.system("gnome-terminal --tab -e " + robot2_cmd)
os.system("sleep 1")
