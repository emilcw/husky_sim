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
    ros-jazzy-rmw-zenoh-cpp \
    python3-apt \
    tmux \
    nano \
    evince \
    less \
    && rm -rf /var/lib/apt/lists/*

# Install pyproj via pip
# Note: numpy<2.0 is required for cv_bridge compatibility
RUN pip install --no-cache-dir --break-system-packages pyproj rtree matplotlib "numpy<2.0" pandas tabulate

# Install computer vision dependencies for yolo_gnn_refiner
# Force NumPy < 2.0 for cv_bridge compatibility (cv_bridge is compiled with NumPy 1.x)
RUN pip install --no-cache-dir --break-system-packages --ignore-installed \
    "numpy<2.0" \
    torch torchvision torchaudio

# 2. Install everything else
RUN pip install --no-cache-dir --break-system-packages --ignore-installed \
    ultralytics \
    torch-geometric \
    opencv-python \
    opencv-contrib-python \
    filterpy \
    lap \
    open3d

# 3. Install OC-SORT from GitHub source
# Clone the repository and install it manually since it's not on PyPI
# The ocsort module needs to be accessible as "from ocsort.ocsort import OCSort"
RUN git clone https://github.com/noahcao/OC_SORT.git /tmp/OC_SORT && \
    cd /tmp/OC_SORT && \
    # Get Python site-packages directory dynamically
    PYTHON_SITE=$(python3 -c "import site; print(site.getsitepackages()[0])") && \
    # Copy the entire ocsort_tracker directory to ocsort/ to make it importable
    mkdir -p ${PYTHON_SITE}/ocsort && \
    cp -r trackers/ocsort_tracker/* ${PYTHON_SITE}/ocsort/ && \
    # Create __init__.py to make it a proper Python package
    touch ${PYTHON_SITE}/ocsort/__init__.py && \
    # The ocsort.py file contains the OCSort class, which should be importable as ocsort.ocsort
    rm -rf /tmp/OC_SORT

# Install compatible setuptools version for colcon editable installs (AFTER other packages)
# setuptools >= 80.0.0 breaks colcon --symlink-install, but we need >= 65.0.0 for --editable support
# This must be last to ensure the correct version is used
RUN pip install --no-cache-dir --break-system-packages --force-reinstall "setuptools>=65.0.0,<80.0.0"

# Source ROS 2 setup script
COPY .bashrc /root/.bashrc
COPY husky_ws/src/daep_husky/tmux/tommy.tmux.conf /root/.tmux.conf
COPY husky_ws/src/daep_husky/worlds/warehouse_actor.sdf /opt/ros/jazzy/share/clearpath_gz/worlds/warehouse_actor.sdf
COPY husky_ws/src/daep_husky/launch/simulation_daep.launch.py /opt/ros/jazzy/share/clearpath_gz/launch/simulation_daep.launch.py
COPY husky_ws/src/daep_husky/models/granso_22_medium_500k_32 /opt/ros/jazzy/share/clearpath_gz/worlds/granso_22_medium_500k_32
COPY husky_ws/src/daep_husky/models/granso_22_medium_500k_32.sdf /opt/ros/jazzy/share/clearpath_gz/worlds/granso_22_medium_500k_32.sdf

# Default command
CMD ["bash"]

