# UniMT/CMDT + SimpleTrack (He et al. 2025, github TossherO/ros_packages): upstream detection + tracking code and
# weights; versions exactly as the upstream README (Python 3.10, torch 2.1.2 cu121, mmcv 2.1.0, mmdet 3.2.0,
# mmdet3d 1.4.0, spconv-cu120 2.3.6, flash-attn 2.6.3, numpy 1.26.4) next to ROS Noetic (its Python libs are pure
# python and are used from Python 3.10; cv_bridge is not needed).
FROM nvidia/cuda:12.1.1-cudnn8-devel-ubuntu20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg2 lsb-release software-properties-common git \
      ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-noetic-ros-base ros-noetic-visualization-msgs ros-noetic-nav-msgs \
      ros-noetic-sensor-msgs libgl1 libglib2.0-0 && rm -rf /var/lib/apt/lists/*
# Python 3.10 (deadsnakes no longer serves focal): standalone build via uv, as a venv with pip, called through a python3.10 wrapper (a symlink would
# bypass the venv); python3 stays the system one for roscore/rosbag
RUN curl -LsSf https://astral.sh/uv/0.4.30/install.sh | env UV_INSTALL_DIR=/usr/local sh && \
    uv python install 3.10 && uv venv --seed --python 3.10 /opt/py310 && printf '#!/bin/sh\nexec /opt/py310/bin/python3.10 "$@"\n' > /usr/local/bin/python3.10 && chmod +x /usr/local/bin/python3.10 && \
    python3.10 -c "import sys; assert sys.version_info[:2] == (3, 10)"
RUN python3.10 -m pip install --no-cache-dir torch==2.1.2 torchvision==0.16.2 --index-url https://download.pytorch.org/whl/cu121
RUN python3.10 -m pip install --no-cache-dir -U openmim && python3.10 -m mim install mmengine "mmcv==2.1.0" "mmdet==3.2.0" "mmdet3d==1.4.0"
RUN python3.10 -m pip install --no-cache-dir spconv-cu120==2.3.6 \
      https://github.com/Dao-AILab/flash-attention/releases/download/v2.6.3/flash_attn-2.6.3+cu123torch2.1cxx11abiFALSE-cp310-cp310-linux_x86_64.whl && \
    python3.10 -m pip install --no-cache-dir rospkg "empy==3.3.4" filterpy pyquaternion pyyaml "numpy==1.26.4"
COPY ros_packages_src /ws/src
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
WORKDIR /ws
