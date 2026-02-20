# Use NVIDIA CUDA 12.6.3 base image on Ubuntu 24.04 (closest stable to 12.4.1 that supports 24.04)
FROM nvidia/cuda:12.6.3-devel-ubuntu24.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Set up locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_GB en_GB.UTF-8 \
    && update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
ENV LANG=en_GB.UTF-8

# Install general utilities
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    python3-pip \
    python3-venv \
    git \
    wget \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 Jazzy repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy and essential tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-vision-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialise rosdep
RUN rosdep init && rosdep update

# Create ROS2 workspace
WORKDIR /ros2_ws
ENV WORKSPACE=/ros2_ws

# Setup virtual environment for Python dependencies
# We use a virtual environment to avoid conflicts with system packages
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Copy the source code
# Note: The directory structure in the copy should reflect where this Dockerfile is run from
COPY . src/florence2_ros2_wrapper

# Install Python dependencies inside the virtual environment
RUN pip install --no-cache-dir setuptools wheel && \
    pip install --no-cache-dir colcon-common-extensions empy==3.3.4 lark && \
    pip install --no-cache-dir -r src/florence2_ros2_wrapper/florence2_ros2/requirements.txt

# Install dependencies using rosdep (ignoring src/ dependencies as we've installed them via pip)
RUN apt-get update && \
    . /opt/ros/jazzy/setup.sh && \
    rosdep install -i --from-path src --rosdistro jazzy -y --skip-keys="torch torchvision transformers" && \
    rm -rf /var/lib/apt/lists/*

# Build the ROS2 workspace
RUN . /opt/ros/jazzy/setup.sh && \
    . $VIRTUAL_ENV/bin/activate && \
    colcon build --packages-select florence2_interfaces florence2_ros2 --symlink-install

# Copy entrypoint script
COPY docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Set the default command, allowing it to be overridden
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "florence2_ros2", "florence2_launch.py"]
