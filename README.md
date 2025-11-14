# Husky Simulation Repository

# FIRST TIME SETUP SIMULATOR and RVIZ

1. Setup docker on your computer
2. Clone the repo to root
3. Go to repo, in my case `cd /home/emil/husky_sim`
4. Build docker image
```
./run_jazzy.sh build
```
5. Start docker environment
```
./run_jazzy.sh run
```
6. Start the Simulator: For more info: https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/overview
```
ros2 launch clearpath_gz simulation.launch.py
```
7. In the simulator window, change cmd_vel to /a201_0000/cmd_vel
8. Switch to keyboard, now you should be able to drive around the husky with (AWSD)
9. Start a new terminal (Ctrl + Shift + T)

```
./run_jazzy.sh bash
```

```
ros2 launch clearpath_viz view_robot.launch.py namespace:=a201_0000
```
10. Add in pointcloud topic to visualize in Rviz

## Clearpath Worlds
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
## Our Worlds

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

# DAEP - INSTALL AND TEST
To setup DAEP we need some additional packages. First, standing in `husky_sim`, create a ros2 workspace called `husky_ws`.

```
mkdir -p husky_ws/src
```

In this src-folder, we are going to install all packages (modules of code) that we need. The structure of the repo should be as below after you are done. Note that you will add in your code as packages later.

## Repository Structure

```
husky_sim/
├── husky_ws/src/                      # ROS 2 workspace source packages
│   ├── daep_husky/                    # 🎯 Main DAEP exploration package (Emil)
│   ├── daep_msgs_husky/               # 📦 Custom message definitions
│   ├── lrs_exec/                      # 📦 LRS execution package
│   ├── lrs_msgs_common/               # 📦 LRS common messages
│   ├── lrs_msgs_tst/                  # 📦 LRS test messages
│   ├── lrs_srvs_exec/                 # 📦 LRS execution services
│   ├── lrs_srvs_ra/                   # 📦 LRS RA services
│   ├── lrs_srvs_tst/                  # 📦 LRS test services
│   ├── lrs_srvs_wdb/                  # 📦 LRS WDB services
│   ├── lrs_turtle4_husky/             # 📦 LRS Turtlebot4/Husky integration
│   ├── lrs_util/                      # 📦 LRS utilities
|   ├── kiss-icp/                      # Waqas code
|   └── yolo_gnn_refiner/              # Vahab code
├── clearpath/                         # Clearpath Robotics configuration
│   └── robot.yaml                     # Main robot configuration
├── Dockerfile                         # Docker configuration
├── run_jazzy.sh                       # Docker run script
└── README.md                          # This file
```

### Please install these in src (so you get the structure above):
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

### Then you shoule be able to build all packages

```
./run_jazzy run (or bash)

# Make sure you are standing in husky_ws, then run

colcon build --symlink-install
```

### Start the simulation
```
ros2 launch clearpath_gz simulation_daep.launch.py world:=warehouse_actor
```

### Rviz - DAEP
```
./run_jazzy.sh bash
ros2run
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/liboctomap.so ros2 launch daep_husky view_robot.launch.py namespace:=a201_0000
```

### Rviz - Vanilla
```
./run_jazzy.sh bash
ros2run
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/liboctomap.so ros2 launch clearpath_viz view_robot.launch.py namespace:=a201_0000
```
#### Add these topics to Rviz - Vanilla
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

### TMUX
```
./run_jazzy.sh bash
```

#### Vanilla
```
ros2run
~/husky_ws/src/daep_husky/tmux/daep.tmux --sim --husky --ns /husky0 --package daep_husky --config warehouse_exploration.yaml
```
#### Dynamic (Simple)
```
ros2run
~/husky_ws/src/daep_husky/tmux/daep.tmux --sim --husky --ns /husky0 --package daep_husky --config warehouse_exploration.yaml --dynamic-objects
```
#### SLAM
```
ros2run
~/husky_ws/src/daep_husky/tmux/daep.tmux --sim --husky --slam --ns /husky0 --package daep_husky --config warehouse_exploration.yaml
```
