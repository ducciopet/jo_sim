FROM osrf/ros:jazzy-desktop-full

# Retry failed apt fetches up to 5 times — handles transient mirror sync issues
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80retries

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
    ros-${DISTRO}-zstd-image-transport \
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
    libtbb12 \
    gnupg \
    wget \
    xterm \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# archive.ubuntu.com is a geo-load-balanced mirror redirector; a single
# `apt update` can land on a mirror mid-sync and get an index whose size/hash
# doesn't match its own Release file ("Mirror sync in progress?"), which is a
# hard failure for that RUN. A bare retry of the whole `apt update` almost
# always lands on a consistent mirror the second time.
RUN for i in 1 2 3 4 5; do sudo apt update && break || sleep 10; done
RUN sudo apt install -y gz-harmonic



RUN curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

# --- CUDA repo (Ubuntu 24.04) ---
RUN wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
 && dpkg -i cuda-keyring_1.1-1_all.deb \
 && wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-ubuntu2404.pin \
 && mv cuda-ubuntu2404.pin /etc/apt/preferences.d/cuda-repository-pin-600

ENV CUDAToolkit_ROOT=/usr/local/cuda

# NOTE: libgtsam-points-cuda13.1-dev (koide3 PPA) is built against CUDA 13.1 —
# the cudart/nvcc packages installed below MUST match that minor version
# (cuda-cudart-13-0 would install CUDA 13.0, a different ABI/package line).
RUN (for i in 1 2 3 4 5; do apt update -y && break || sleep 10; done) \
    && apt install -y libiridescence-dev \
    libgtsam-points-cuda13.1-dev \
    cuda-cudart-13-1 \
    cuda-nvcc-13-1 \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# cuda-cudart-13-1/cuda-nvcc-13-1 install nvcc under the versioned path
# /usr/local/cuda-13.1 — only the (much heavier) cuda-toolkit-13-1 meta-package
# sets up the conventional /usr/local/cuda symlink via update-alternatives.
# CUDAToolkit_ROOT=/usr/local/cuda (and gtsam_points'/onboard_detector's own
# find_package(CUDAToolkit), pulled in transitively via gtsam_points-config.cmake
# since it was built with CUDA support) needs that symlink to resolve nvcc.
RUN ln -s /usr/local/cuda-13.1 /usr/local/cuda

# ============================================================
# PYTHON ML DEPENDENCIES (required by onboard_detector)
# ============================================================
RUN (for i in 1 2 3 4 5; do apt-get update && break || sleep 10; done) \
    && apt-get remove -y \
    python3-sympy \
    python3-mpmath \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages --ignore-installed pip setuptools wheel

# PyTorch with CUDA support — needed for YOLO detector. cu128 matches the
# CUDA 13.1 toolkit installed above for gtsam_points (PyTorch wheels bundle
# their own CUDA runtime, so only the driver needs to be new enough for it —
# a driver capable of CUDA 13.1 comfortably satisfies cu128). cu118 is no
# longer used upstream past torch 2.7.1, so pinning to it here would silently
# install a stale/unmaintained build.
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    --retries 10 \
    --timeout 300 \
    torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cu128 || \
    echo "WARNING: PyTorch installation failed. YOLO detector will not work, but ICP calibration will."

# Computer vision and scientific stack from apt
RUN (for i in 1 2 3 4 5; do apt-get update && break || sleep 10; done) \
    && apt-get install -y \
    python3-opencv \
    python3-scipy \
    python3-sklearn \
    python3-matplotlib \
    python3-pil \
    python3-yaml \
    python3-tqdm \
    python3-requests \
    && rm -rf /var/lib/apt/lists/*

# lap ("lap>=0.5.12"): linear-assignment solver ultralytics' built-in
# trackers (ByteTrack/BoT-SORT, used by onboard_detector_v2's
# yolo_seg_track_detector.py) need for frame-to-frame box matching.
# Ultralytics normally auto-installs it on first model.track() call, but
# that AutoUpdate path pip-installs at RUNTIME into this
# externally-managed environment, which fails (PEP 668) — install it here
# instead, once, at image build time.
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    --trusted-host pypi.org \
    --trusted-host files.pythonhosted.org \
    --trusted-host pypi.python.org \
    "numpy>=2.0" \
    ultralytics \
    "lap>=0.5.12"


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN if id -u ${USER_UID} ; then userdel `id -un ${USER_UID}` ; fi
RUN groupadd --gid ${USER_GID} ${USERNAME}
RUN useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && (for i in 1 2 3 4 5; do apt update && break || sleep 10; done) \
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

# xsens_mti_ros2_driver — needed to read bags recorded with an Xsens MTi IMU
# (e.g. bags/*_validation_lab), which contain the custom message type
# xsens_mti_ros2_driver/msg/XsStatusWord on /status. This isn't on the ROS
# apt index, so it has to be built from source like RGLGazeboPlugin/rtabmap
# above. The upstream repo also bundles the Xsens SDK it needs (lib/xspublic)
# and an `ntrip` package (RTCM/diagnostics client) — both come along for free
# since colcon discovers packages recursively.
RUN cd src && git clone --branch ros2 https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git xsens_mti_ros2_driver

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
        xsens_mti_ros2_driver ntrip \
    --install-base /home/ros/local_install \
    --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release \
 && touch src/rtabmap/COLCON_IGNORE \
 && touch src/rtabmap_ros/COLCON_IGNORE

# xsens_mti_ros2_driver/ntrip are built in their own colcon invocation, apart
# from the ~19-package combined build above: mixed into that large parallel
# build they compile fine but silently don't get installed under
# /home/ros/local_install (no error, no COLCON_IGNORE — the install step
# just doesn't happen for these two, probably some interaction with being
# nested two directories deep under src/xsens_mti_ros2_driver/src/*, unlike
# every other package here). Building them alone installs correctly.
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build \
    --packages-select xsens_mti_ros2_driver ntrip \
    --install-base /home/ros/local_install

# onboard_detector/CMakeLists.txt declares `cmake_minimum_required(VERSION 3.8)`,
# older than CMake 3.12 which introduced policy CMP0074 ("find_package uses
# <PackageName>_ROOT variables"). With that policy defaulting to OLD, CMake
# never promotes the ENV{CUDAToolkit_ROOT} we set above into the plain
# CUDAToolkit_ROOT variable that FindCUDAToolkit.cmake's own nvcc-search logic
# actually reads — so it searches as if CUDAToolkit_ROOT were never set, even
# though `echo $CUDAToolkit_ROOT` shows it correctly in the same shell.
# Passing it explicitly as a CMake cache variable bypasses that policy gap.
RUN source /opt/ros/jazzy/setup.bash \
 && source /home/ros/local_install/setup.bash \
 && colcon build \
    --packages-select onboard_detector jo_navigation jo_sim jo_description turtlebot_description \
    --symlink-install \
    --install-base /home/ros/local_install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDAToolkit_ROOT=/usr/local/cuda

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
