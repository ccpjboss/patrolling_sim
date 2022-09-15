import os

monitor_cmd = "\"ros2 run patrolling_sim_ros2 monitor ctcv CBLS 3\""
os.system("gnome-terminal --tab -e " + monitor_cmd)
os.system("sleep 1")

param_cmd = "\"ros2 param set /monitor /initial_pos \"[24.0, 0.5, -0.5, 2.0, -28.0, 3.0]\"\""
os.system("gnome-terminal --tab -e " + param_cmd)
os.system("sleep 1")

gazebo_cmd = "\"ros2 launch stop_bot_description multi_stop_simulation_launch.py\""
os.system("gnome-terminal --tab -e " + gazebo_cmd)
os.system("sleep 1")

robot1_cmd = "\"ros2 run patrolling_sim_ros2 CBLS 0 ctcv 0 3 --ros-args --remap __name:=patrol_robot_0 -p initial_pos:=\"[24.0,0.5,-0.5,2.0,-28.0,3.0]\" -r __ns:=/robot0 -r /tf:=tf -r /tf_static:=tf_static\""
os.system("gnome-terminal --tab -e " + robot1_cmd)
os.system("sleep 1")

robot2_cmd = "\"ros2 run patrolling_sim_ros2 CBLS 0 ctcv 1 3 --ros-args --remap __name:=patrol_robot_1 -p initial_pos:=\"[24.0,0.5,-0.5,2.0,-28.0,3.0]\" -r __ns:=/robot1 -r /tf:=tf -r /tf_static:=tf_static\""
os.system("gnome-terminal --tab -e " + robot2_cmd)
os.system("sleep 1")

robot3_cmd = "\"ros2 run patrolling_sim_ros2 CBLS 0 ctcv 2 3 --ros-args --remap __name:=patrol_robot_2 -p initial_pos:=\"[24.0,0.5,-0.5,2.0,-28.0,3.0]\" -r __ns:=/robot2 -r /tf:=tf -r /tf_static:=tf_static\""
os.system("gnome-terminal --tab -e " + robot3_cmd)
os.system("sleep 1")
