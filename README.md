# Particle Filter

This repository contains a ROS2 package that executes a particle filter to accurately localize a robot within a given map.

This project is intended to be used in correspondence with the autonomous_diff_drive repository.

To properly run, import this package into the same workspace containing the autonomous_diff_drive package.

Next, in separate terminals, run the following commands in this specific order. Please make sure to install all necessary packages and dependencies:

`ros2 launch autonomous_diffdrive_robot launch_sim.launch.py world:=./src/autonomous_diffdrive_robot/worlds/pf_room_final.world`

`rviz2 -d ./src/autonomous_diffdrive_robot/config/main.rviz `

`ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=pf_room_final.yaml -p use_sim_time:=true`

`ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=pf_room_final.yaml -p use_sim_time:=true`

`ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/cmd_vel_teleop`


The system is now set up to run the particle filter algorithm. To run the particle filter, execute the following command in a separate terminal:

`ros2 launch particle_filter pf`


The algorithm places the pose (position and orientation) for each of the particles as a PoseArray within RViz, while updating the average pose of the particle (which represents the estimated pose of the robot) using a Pose marker.

