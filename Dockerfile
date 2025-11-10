# Use the official ROS 2 Jazzy base image
FROM osrf/ros:jazzy-desktop-full-noble

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    nano \
    python3-colcon-common-extensions \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN locale-gen en_US.UTF-8

# Add Clearpath Robotics repository and key
RUN wget -qO - https://packages.clearpathrobotics.com/public.key | apt-key add - && \
    sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

# Add Clearpath ROS2 rosdep source
RUN wget -qO /etc/ros/rosdep/sources.list.d/50-clearpath.list \
    https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list && \
    rosdep update

# Update package lists and install Clearpath packages + ros-gz
RUN apt-get update && apt-get install -y \
    ros-jazzy-clearpath-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-clearpath-simulator \
    ros-jazzy-irobot-create-msgs \
    ros-jazzy-octomap-ros \
    ros-jazzy-octomap-server \  
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-rviz-plugins \
    ros-jazzy-tf-transformations \
    ros-jazzy-clearpath-nav2-demos \
    python3-apt \
    tmux \
    nano \
    evince \
    && rm -rf /var/lib/apt/lists/*

# Install pyproj via pip
RUN pip install --no-cache-dir --break-system-packages pyproj rtree matplotlib numpy pandas tabulate

# Install computer vision dependencies for yolo_gnn_refiner
RUN pip install --no-cache-dir --break-system-packages --ignore-installed \
    torch torchvision torchaudio \
    ultralytics \
    torch-geometric \
    opencv-python \
    opencv-contrib-python

# Install compatible setuptools version for colcon editable installs (AFTER other packages)
# setuptools >= 80.0.0 breaks colcon --symlink-install, but we need >= 65.0.0 for --editable support
# This must be last to ensure the correct version is used
RUN pip install --no-cache-dir --break-system-packages --force-reinstall "setuptools>=65.0.0,<80.0.0"

# Source ROS 2 setup script
COPY .bashrc /root/.bashrc
COPY husky_ws/src/daep/tmux/tommy.tmux.conf /root/.tmux.conf
COPY husky_ws/src/daep/worlds/warehouse_actor.sdf /opt/ros/jazzy/share/clearpath_gz/worlds/warehouse_actor.sdf
COPY husky_ws/src/daep/launch/simulation_daep.launch.py /opt/ros/jazzy/share/clearpath_gz/launch/simulation_daep.launch.py
COPY husky_ws/src/daep/models/granso_22_medium_500k_32 /opt/ros/jazzy/share/clearpath_gz/worlds/granso_22_medium_500k_32
COPY husky_ws/src/daep/models/granso_22_medium_500k_32.sdf /opt/ros/jazzy/share/clearpath_gz/worlds/granso_22_medium_500k_32.sdf

# Default command
CMD ["bash"]

