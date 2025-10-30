# Husky Simulation Repository

## Repository Structure

```
husky_sim/
├── husky_ws/src/                      # ROS 2 workspace source packages
│   ├── daep/                          # 🎯 Main DAEP exploration package (Emil)
│   ├── daep_msgs/                     # 📦 Custom message definitions
│   ├── lrs_exec/                      # 📦 LRS execution package
│   ├── lrs_msgs_common/               # 📦 LRS common messages
│   ├── lrs_msgs_tst/                  # 📦 LRS test messages
│   ├── lrs_srvs_exec/                 # 📦 LRS execution services
│   ├── lrs_srvs_ra/                   # 📦 LRS RA services
│   ├── lrs_srvs_tst/                  # 📦 LRS test services
│   ├── lrs_srvs_wdb/                  # 📦 LRS WDB services
│   ├── lrs_turtle4/                   # 📦 LRS Turtlebot4/Husky integration
│   ├── lrs_util/                      # 📦 LRS utilities
|   ├── **WAQAS PACKAGE**              # Waqas code
|   └── **VAHAB PACKAGE**              # Vahab code
├── clearpath/                         # Clearpath Robotics configuration
│   └── robot.yaml                     # Main robot configuration
├── Dockerfile                         # Docker configuration
├── run_jazzy.sh                       # Docker run script
└── README.md                          # This file
```

# SETUP

1. Setup docker on your computer
2. Clone the repo to root
3. Go to repo, in my case `cd /home/emil/husky_sim`
4. Follow steps below

```
# Build docker image
./run_jazzy.sh build

# Start docker environment
./run_jazzy.sh run

# Simulator
ros2 launch clearpath_gz simulation.launch.py

For more info: https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/overview

# In simulator window
change cmd_vel to /a201_0000/cmd_vel

# Now you should be able to control Husky
# use keys to control husky

# Start a new terminal (Ctrl + Shift + T)
./run_jazzy.sh bash

# Rviz
ros2 launch clearpath_viz view_robot.launch.py namespace:=a201_0000

# Also, here you can add the pointcloud as a topic to visualize
```

# For more info on how to configure robot.yaml
* https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview/
* https://github.com/clearpathrobotics/clearpath_config/tree/jazzy/clearpath_config/sample


# DAEP
```

# Rviz for DAEP (Weird workaround to make octomap_rviz_plugin work)
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/liboctomap.so ros2 launch clearpath_viz view_robot.launch.py namespace:=a201_0000

# TMUX
./run_jazzy.sh bash
~/husky_ws/src/daep/tmux/daep.tmux --sim --husky --ns /husky0 --config warehouse_exploration.yaml

# Go to command to try manager.py
ros2run
ros2 topic pub /husky0/goal daep_msgs/msg/Goal "{uuid: '123e4567-e89b-12d3-a456-426614174000', x: 1.5, y: 2.0, z: 0.0, yaw: 1.57, linear_velocity: 0.5, angular_velocity: 0.2, is_last: false, initial_motion: false}"

```

![alt text](image.png)

## Packages needed for daep and lrs_turtle4

**SSH (if you have SSH keys set up):**
```
git clone git@gitlab.liu.se:real-lab/daep.git
git clone git@gitlab.liu.se:real-lab/daep_msgs.git
git clone git@gitlab.liu.se:lrs2/lrs_turtle4.git
git clone git@gitlab.liu.se:lrs2/lrs_util.git
git clone git@gitlab.liu.se:lrs2/lrs_exec.git
git clone git@gitlab.liu.se:lrs2/lrs_msgs_common.git
git clone git@gitlab.liu.se:lrs2/lrs_msgs_tst.git
git clone git@gitlab.liu.se:lrs2/lrs_srvs_exec.git
git clone git@gitlab.liu.se:lrs2/lrs_srvs_ra.git
git clone git@gitlab.liu.se:lrs2/lrs_srvs_tst.git
git clone git@gitlab.liu.se:lrs2/lrs_srvs_wdb.git
```

**HTTPS (if you prefer HTTPS or don't have SSH keys or Access):**
```
git clone https://gitlab.liu.se/real-lab/daep.git
git clone https://gitlab.liu.se/real-lab/daep_msgs.git
git clone git@gitlab.liu.se:lrs2/lrs_turtle4.git
git clone https://gitlab.liu.se/lrs2/lrs_util.git
git clone https://gitlab.liu.se/lrs2/lrs_exec.git
git clone https://gitlab.liu.se/lrs2/lrs_msgs_common.git
git clone https://gitlab.liu.se/lrs2/lrs_msgs_tst.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_exec.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_ra.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_tst.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_wdb.git
```