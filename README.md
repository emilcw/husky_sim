# Husky Simulation Repository

# FIRST TIME SETUP SIMULATOR and RVIZ

1. Setup docker on your computer
2. Clone the husky_sim to root
3. Go to husky_sim, in my case `cd /home/emil/husky_sim`
4. Create a ros2 workspace
```
mkdir -p husky_ws/src
cd husky_ws/src
```
5. Install these packages in `src` (Note: daep_husky, daep_husky_msgs, lrs_turtle4_husky, yolo_gnn_refiner are private repos, you need to be added to these to clone them). You need SSH or Tokens for this.
```
git clone https://github.com/emilcw/daep_husky.git
git clone https://github.com/emilcw/daep_msgs_husky.git
git clone https://github.com/emilcw/lrs_turtle4_husky.git
git clone https://gitlab.liu.se/lrs2/lrs_util.git
git clone https://gitlab.liu.se/lrs2/lrs_exec.git
git clone https://gitlab.liu.se/lrs2/lrs_msgs_common.git
git clone https://gitlab.liu.se/lrs2/lrs_msgs_tst.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_exec.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_ra.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_tst.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_wdb.git
git clone https://github.com/PRBonn/kiss-icp.git
git clone https://github.com/emilcw/yolo_gnn_refiner.git
```
You should now have the repository structure as visualized below:
```
husky_sim/
├── husky_ws/src/                      # ROS 2 workspace source packages
│   ├── daep_husky/                    # 🎯 Main DAEP exploration package (Emil)
│   ├── daep_msgs_husky/               # 📦 Custom message definitions
|   ├── kiss-icp/                      # Waqas code
│   ├── lrs_exec/                      # 📦 LRS execution package
│   ├── lrs_msgs_common/               # 📦 LRS common messages
│   ├── lrs_msgs_tst/                  # 📦 LRS test messages
│   ├── lrs_srvs_exec/                 # 📦 LRS execution services
│   ├── lrs_srvs_ra/                   # 📦 LRS RA services
│   ├── lrs_srvs_tst/                  # 📦 LRS test services
│   ├── lrs_srvs_wdb/                  # 📦 LRS WDB services
│   ├── lrs_turtle4_husky/             # 📦 LRS Turtlebot4/Husky integration
│   ├── lrs_util/                      # 📦 LRS utilities
|   └── yolo_gnn_refiner/              # Vahab code
├── clearpath/                         # Clearpath Robotics configuration
│   └── robot.yaml                     # Main robot configuration
├── Dockerfile                         # Docker configuration
├── run_jazzy.sh                       # Docker run script
└── README.md                          # This file
```


6. Go back to husky_sim and build docker image
```
./run_jazzy.sh build
```
7. Start docker environment
```
./run_jazzy.sh run
```
8. Build the packages with colcon (in the docker env you just started)
```
colcon build --symlink-install
```
9. In a new terminal, start the simulator
```
./run_jazzy.sh bash
ros2 launch clearpath_gz simulation_daep.launch.py world:=warehouse_actor
```
For more info on the simulator: https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/overview

10. Go to `/husky_sim/husky_ws/src/kiss-icp/ros/launch/odometry.launch.py`
In the launch file:
- Change `lidar_odom_frame` default value to `odom.`
- Remove code related to `rviz_node` to avoid duplicate rviz windows.

11. In a new terminal, start rviz (with octomap fix)
```
./run_jazzy.sh bash
ros2run
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/liboctomap.so ros2 launch daep_husky view_robot.launch.py namespace:=a201_0000
```
11. In a new terminal start tmux. To start exploration, run `rpl_exploration`-tab.

#### Pose from simulation
```
./run_jazzy.sh bash
ros2run
~/husky_ws/src/daep_husky/tmux/daep.tmux --sim --husky --ns /husky0 --package daep_husky --config warehouse_exploration.yaml
```
#### Dynamic (Simple obstacles)
```
./run_jazzy.sh bash
ros2run
~/husky_ws/src/daep_husky/tmux/daep.tmux --sim --husky --ns /husky0 --package daep_husky --config warehouse_exploration.yaml --dynamic-objects
```
#### SLAM
```
./run_jazzy.sh bash
ros2run
~/husky_ws/src/daep_husky/tmux/daep.tmux --sim --husky --slam --ns /husky0 --package daep_husky --config warehouse_exploration.yaml
```
## Clearpath Worlds

Example:
```
ros2 launch clearpath_gz simulation.launch.py world:=office
```
```
Worlds:
construction
office
orchard
pipeline
solar_farm
warehouse
```
### Our Worlds

```
ros2 launch clearpath_gz simulation_daep.launch.py world:=warehouse_actor
```

```
Worlds:
warehouse_actor
granso_22_medium_500k_32
```

# For more info on how to configure robot.yaml and worlds
* https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview/
* https://github.com/clearpathrobotics/clearpath_config/tree/jazzy/clearpath_config/sample
* https://gazebosim.org/docs/citadel/actors/


# Other useful commands (must be run in docker)

### Launch vanilla simulator (change cmd_vel -> /a201_000/cmd_vel to control robot with keyboard)
```
ros2 launch clearpath_gz simulation.launch.py
```
### Launch vanilla rviz
```
ros2 launch clearpath_viz view_robot.launch.py namespace:=a201_0000
```
### Launch vanilla rviz with octomap fix
```
./run_jazzy.sh bash
ros2run
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/liboctomap.so ros2 launch clearpath_viz view_robot.launch.py namespace:=a201_0000
```
#### Add these topics to Rviz
```
/husky0/octomap_full
/husky0/filtered_octomap
/a201_0000/sensors/lidar3d_0/points (heavy to run for gpu)
/husky0/daep_marker_array_visualization (Local planner)
/husky0/rrt_star_marker_array (global planner)
/husky0/bounds_visualization (bounds, path and frustum)
/husky0/rrt_path (chosen global frontier)
/husky0/frontier_marker_array (frontiers)
/husky0/sampled_node (collision checking)
/husky0/daep_marker_visualization (simulated, simple dynamic obstacle)
```
