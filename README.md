# Particle Filter

This repository contains a ROS2 package that executes a particle filter to accurately localize a robot within a given map.

This project is intended to be used in correspondence with the autonomous_diff_drive repository.

To properly run, import this package into the same workspace containing the autonomous_diff_drive package.

Next, in separate terminals, run the following commands in this specific order. Please make sure to install all necessary packages and dependencies:

```
`ros2 launch autonomous_diffdrive_robot launch_sim.launch.py world:=./src/autonomous_diffdrive_robot/worlds/pf_room_final.world`

`rviz2 -d ./src/autonomous_diffdrive_robot/config/main.rviz `

`ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=pf_room_final.yaml -p use_sim_time:=true`

`ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/cmd_vel_teleop`
```

In this case, the imported map is named "pf_room_final.yaml". This map must be imported within the same directory level as the 'src' folder in your ROS2 workspace. Other, custom map files may be used in substitution; however, the particle filter parameters may need to be adjusted accordingly (more details on that below).


The system is now set up to run the particle filter algorithm. To run the particle filter, execute the following command in a separate terminal:

`ros2 launch particle_filter pf`


The algorithm places the pose (position and orientation) for each of the particles as a PoseArray within RViz, while updating the average pose of the particle (which represents the estimated pose of the robot) using a Pose marker.




### Particle Filter Parameters

The available particle filter parameters for tuning and customization is found in the pf_params.yaml file in the 'config' folder. The parameters are as follows:


`num_particles` - Number of particles

`lin_thresh` - Threshold value for a linear movement before a particle filter update occurs

`ang_thresh` - Threshold value for angular movement before a particel filter update occurs

`motion_pos_noise_wt` - Weight of noise function (in meters) added to the x- and y- positions in the particle motion model

`z_max` - Maximum scan range of LiDAR sensor in meters

`sigma_hit` - Gaussian noise (standarad deviation in meters) of the measurements

`z_hit` - Weight of measurement noise distribution

`resampling_pos_noise` - Gaussian noise (standard deviation in meters) added to the x- and y- positions when resampling new particles





### Demonstration of Particle Filter


The below GIF showcases the execution of the particle filter using the map pf_room_final and the parameters found in the 'config' folder:

![Particle Filter GIF](https://github.com/praveenrav/particle_filter/blob/main/Particle_Filter.gif)













