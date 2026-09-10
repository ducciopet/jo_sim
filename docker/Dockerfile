# ============================================================
# STAGE 1: Build Open3D from source (required by onboard_detector)
# ============================================================
FROM ubuntu:24.04 AS builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-pip \
    libgl1-mesa-dev \
    libx11-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libeigen3-dev \
    libomp-dev \
    xorg-dev \
    libxcb-shm0 \
    libglu1-mesa-dev \
    libc++-dev \
    libc++abi-dev \
    libsdl2-dev \
    libxi-dev \
    libtbb-dev \
    libosmesa6-dev \
    libudev-dev \
    autoconf \
    libtool \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/isl-org/Open3D && cd Open3D && mkdir build && cd build && cmake .. && make -j$(nproc)
RUN cd Open3D/build && make install DESTDIR=/open3d-install

# ============================================================
# STAGE 2: Main image
# ============================================================
FROM osrf/ros:jazzy-desktop-full

# Retry failed apt fetches up to 5 times — handles transient mirror sync issues
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80retries

# Copy Open3D from builder stage
COPY --from=builder /open3d-install/ /

ENV GZ_VERSION=harmonic
ENV ROS_DOMAIN_ID=42
ARG DISTRO=jazzy


RUN apt update -y \
    && apt install -y python3-pip \
    nano \
    libxcb-cursor0 \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN pip install --break-system-packages ruamel.yaml PyQt6

RUN apt update -y \
    && apt install -y ros-${DISTRO}-plotjuggler \
    ros-${DISTRO}-joint-state-publisher \
    ros-${DISTRO}-joint-state-publisher-gui \
    ros-${DISTRO}-gz-ros2-control \
    ros-${DISTRO}-teleop-twist-joy \
    ros-${DISTRO}-teleop-twist-keyboard \
    ros-${DISTRO}-joy \
    ros-${DISTRO}-ros2-control \
    ros-${DISTRO}-ros2-controllers \
    ros-${DISTRO}-xacro \
    ros-${DISTRO}-rosbag2-storage-mcap \
    ros-${DISTRO}-rmw-cyclonedds-cpp \
    ros-${DISTRO}-realsense2-camera \
    ros-${DISTRO}-realsense2-description \
    ros-${DISTRO}-twist-mux \
    ros-${DISTRO}-nav2-common \
    ros-${DISTRO}-nav2-controller \
    ros-${DISTRO}-nav2-smoother \
    ros-${DISTRO}-nav2-planner \
    ros-${DISTRO}-nav2-behaviors \
    ros-${DISTRO}-nav2-bt-navigator \
    ros-${DISTRO}-nav2-waypoint-follower \
    ros-${DISTRO}-nav2-velocity-smoother \
    ros-${DISTRO}-nav2-lifecycle-manager \
    ros-${DISTRO}-nav2-dwb-controller \
    ros-${DISTRO}-nav2-navfn-planner \
    ros-${DISTRO}-nav2-rviz-plugins \
    ros-${DISTRO}-nav2-graceful-controller \
    ros-${DISTRO}-robot-localization \
    ros-${DISTRO}-tf-transformations \
    ros-${DISTRO}-topic-tools \
    ros-${DISTRO}-cv-bridge \
    ros-${DISTRO}-image-transport \
    ros-${DISTRO}-image-transport-plugins \
    ros-${DISTRO}-compressed-image-transport \
    ros-${DISTRO}-nmea-msgs \
    ros-${DISTRO}-velodyne-msgs \
    ros-${DISTRO}-mavros-msgs \
    ros-${DISTRO}-vision-msgs \
    ros-${DISTRO}-pcl-ros \
    ros-${DISTRO}-pcl-conversions \
    ros-${DISTRO}-message-filters \
    python3-rosdep \
    software-properties-common \
    ca-certificates \
    libmetis-dev \
    gnupg \
    wget \
    xterm \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt update
RUN sudo apt install -y gz-harmonic



RUN curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

# --- CUDA repo (Ubuntu 24.04) ---
RUN wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
 && dpkg -i cuda-keyring_1.1-1_all.deb \
 && wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-ubuntu2404.pin \
 && mv cuda-ubuntu2404.pin /etc/apt/preferences.d/cuda-repository-pin-600

ENV CUDAToolkit_ROOT=/usr/local/cuda

RUN apt update -y \
    && apt install -y libiridescence-dev \
    libgtsam-points-cuda13.1-dev \
    cuda-cudart-13-0 \
    cuda-nvcc-13-0 \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# ============================================================
# OPEN3D RUNTIME COMPATIBILITY
# ============================================================
# Install libc++/libc++abi runtime needed by Open3D
RUN apt-get update && apt-get install -y \
    libc++1 \
    libc++abi1 \
    libtbb12 \
    && rm -rf /var/lib/apt/lists/*

# Open3D in /usr/local may reference LLVM-18 libc++ absolute paths.
# Create compatibility symlinks so CMake/Make can resolve them.
RUN mkdir -p /usr/lib/llvm-18/lib && \
    ln -sf /usr/lib/x86_64-linux-gnu/libc++.so.1 /usr/lib/llvm-18/lib/libc++.so.1 && \
    ln -sf /usr/lib/x86_64-linux-gnu/libc++abi.so.1 /usr/lib/llvm-18/lib/libc++abi.so

# Open3D installs a private TBB in /usr/local/lib that conflicts with the
# distro-provided libtbb.so.12 — replace with symlinks to the system library.
RUN rm -f /usr/local/lib/libtbb.so* && \
    ln -sf /usr/lib/x86_64-linux-gnu/libtbb.so.12.11 /usr/local/lib/libtbb.so.12.12 && \
    ln -sf /usr/lib/x86_64-linux-gnu/libtbb.so.12 /usr/local/lib/libtbb.so.12 && \
    ln -sf /usr/lib/x86_64-linux-gnu/libtbb.so /usr/local/lib/libtbb.so

# ============================================================
# PYTHON ML DEPENDENCIES (required by onboard_detector)
# ============================================================
RUN apt-get update && apt-get remove -y \
    python3-sympy \
    python3-mpmath \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages --ignore-installed pip setuptools wheel

# PyTorch with CUDA support (cu118) — needed for YOLO detector
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    --retries 10 \
    --timeout 300 \
    torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cu118 || \
    echo "WARNING: PyTorch installation failed. YOLO detector will not work, but ICP calibration will."

# Computer vision and scientific stack from apt
RUN apt-get update && apt-get install -y \
    python3-opencv \
    python3-scipy \
    python3-sklearn \
    python3-matplotlib \
    python3-pil \
    python3-yaml \
    python3-tqdm \
    python3-requests \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    --trusted-host pypi.org \
    --trusted-host files.pythonhosted.org \
    --trusted-host pypi.python.org \
    "numpy>=2.0" \
    ultralytics


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN if id -u ${USER_UID} ; then userdel `id -un ${USER_UID}` ; fi
RUN groupadd --gid ${USER_GID} ${USERNAME}
RUN useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt update \
    && apt install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN cd /home/ros/ \
    && git clone https://github.com/RobotecAI/RGLGazeboPlugin.git
RUN cd /home/ros/RGLGazeboPlugin \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j \
    && make install \
    && cd ..
    # && export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/home/ros/RGLGazeboPlugin/install/RGLServerPlugin:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH \
    # && export IGN_GUI_PLUGIN_PATH=/home/ros/RGLGazeboPlugin/install/RGLVisualize:$IGN_GUI_PLUGIN_PATH



USER ros
RUN sudo usermod -aG dialout ros
WORKDIR /home/$USERNAME/
RUN mkdir src

RUN cd src && git clone https://github.com/sutharsan-311/nav2_config.git

RUN cd src && git clone https://github.com/introlab/rtabmap.git
RUN cd src && git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git

RUN cd RGLGazeboPlugin && sudo touch COLCON_IGNORE

# Disable packages with broken setup.py that must never be built
RUN touch src/nav2_config/COLCON_IGNORE \
 && touch src/rtabmap_ros/rtabmap_python/COLCON_IGNORE

SHELL ["/bin/bash", "-c"]

# glim and glim_ros2 are built into the image layer (no --symlink-install).
# jo_* packages are excluded here — they are built with --symlink-install at
# container startup via the entrypoint so host edits are always picked up.
COPY --chown=ros:ros glim                  src/glim
COPY --chown=ros:ros glim_ros2             src/glim_ros2

# jo_* go into src/ as a fallback when no volume mount is present (CI / one-shot).
# At runtime the compose volume mounts override them.
COPY --chown=ros:ros jo_description        src/jo_description
COPY --chown=ros:ros jo_msgs               src/jo_msgs
COPY --chown=ros:ros turtlebot_description src/turtlebot_description
COPY --chown=ros:ros jo_sim                src/jo_sim
COPY --chown=ros:ros jo_navigation         src/jo_navigation
COPY --chown=ros:ros onboard_detector      src/onboard_detector

# Build rtabmap, glim, glim_ros, and jo_msgs (proper install, no symlinks).
# jo_description/turtlebot_description/jo_sim/jo_navigation are light and built
# with --symlink-install in the entrypoint so host edits are always picked up.
# onboard_detector is heavy (~65s) so it is built here with --symlink-install;
# at runtime the volume mount replaces src/onboard_detector so Python/launch/config
# changes are reflected immediately — only C++ changes need a manual colcon build.
# After the build, mark rtabmap and rtabmap_ros with COLCON_IGNORE so that any
# subsequent `colcon build` inside the container never triggers a rebuild of those
# heavy packages (they are not volume-mounted, so the marker persists across restarts).
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build \
    --packages-ignore \
        jo_description turtlebot_description jo_sim jo_navigation onboard_detector \
    --install-base /home/ros/local_install \
    --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release \
 && touch src/rtabmap/COLCON_IGNORE \
 && touch src/rtabmap_ros/COLCON_IGNORE

RUN source /opt/ros/jazzy/setup.bash \
 && source /home/ros/local_install/setup.bash \
 && colcon build \
    --packages-select onboard_detector jo_navigation jo_sim jo_description turtlebot_description \
    --symlink-install \
    --install-base /home/ros/local_install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
RUN echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib/" >> ~/.bashrc
RUN cd RGLGazeboPlugin \
    && echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib/" >> ~/.bashrc \
    && echo "export RGL_PATTERNS_DIR=`pwd`/lidar_patterns" >> ~/.bashrc \
    && echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/install/RGLServerPlugin:$GZ_SIM_SYSTEM_PLUGIN_PATH" >> ~/.bashrc \
    && echo "export GZ_GUI_PLUGIN_PATH=`pwd`/install/RGLVisualize:$GZ_GUI_PLUGIN_PATH" >> ~/.bashrc \
    && echo "export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/ros/src/jo_sim/worlds/external/worlds/" >> ~/.bashrc \
    && echo "export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/ros/src/jo_sim/worlds/external/models/" >> ~/.bashrc

COPY docker/utilities.sh /etc/utilities.sh
RUN echo 'source /etc/utilities.sh' >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
