# Use the official ROS 2 Jazzy base image
FROM osrf/ros:jazzy-desktop-full-noble

# Set environment variables
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
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
    python3-apt \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
RUN mkdir -p /ros2_ws/src

# Set the working directory
WORKDIR /ros2_ws

# Source ROS 2 setup script
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]

