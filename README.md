# UAV_UGV

Simulation project

Pure parsuite
PRM
A*


Test

 ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
 ros2 run prm prm
 ros2 run prm follower
 ros2 topic pub /drone_goal geometry_msgs/msg/Point "{x: 10.0, y: -3.6, z: 1.0}"