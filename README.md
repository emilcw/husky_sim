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

# In container:
ros2 launch clearpath_gz simulation.launch.py

For more info: https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/overview

# In simulator window
change cmd_vel to /a200_0000/cmd_vel

# Now you should be able to control Husky
# use keys to control husky

# Start a new terminal (Ctrl + Shift + T)
./run_jazzy.sh bash

# In new terminal
ros2 launch clearpath_viz view_robot.launch.py namespace:=a200_0000

# Also, here you can add the pointcloud as a topic to visualize
```

# For more info on how to configure robot.yaml
* https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview/
* https://github.com/clearpathrobotics/clearpath_config/tree/jazzy/clearpath_config/sample


# DAEP
```
# TMUX
./run_jazzy.sh bash
~/husky_ws/src/daep/tmux/daep.tmux --sim --husky --ns /husky0

# Go to command
source install/setup.bash
ros2 topic pub /turtle0/goal daep_msgs/msg/Goal "{uuid: '123e4567-e89b-12d3-a456-426614174000', x: 1.5, y: 2.0, z: 0.0, yaw: 1.57, linear_velocity: 0.5, angular_velocity: 0.2, is_last: false, initial_motion: true}"

```

## Packages needed for daep and lrs_turtle4
```
git clone https://gitlab.liu.se/lrs2/lrs_util.git
git clone https://gitlab.liu.se/lrs2/lrs_exec.git
git clone https://gitlab.liu.se/lrs2/lrs_msgs_common.git
git clone https://gitlab.liu.se/lrs2/lrs_msgs_tst.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_exec.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_ra.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_tst.git
git clone https://gitlab.liu.se/lrs2/lrs_srvs_wdb.git
```