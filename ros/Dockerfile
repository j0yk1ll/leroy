FROM docker.io/osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Pin upstream DepthART for reproducible builds.
ARG DEPTHART_REF=0384521b3bcb4c64adf03eeb5d55ebdb1cbdd84c

# CPU-only PyTorch runtime for the development image.
# DepthART was validated locally with torch 2.1 / Python 3.9; Jazzy uses
# Python 3.12, so use the closest PyTorch line with Python 3.12 wheels.
ARG TORCH_VERSION=2.2.2
ARG TORCHVISION_VERSION=0.17.2

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    git \
    python3-pip \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-serial \
    ros-jazzy-usb-cam \
    ros-jazzy-image-proc \
    ros-jazzy-rqt-image-view \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-rclcpp-components \
    ros-jazzy-cv-bridge \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# DepthART runtime
# ------------------------------------------------------------

# Fetch the pinned DepthART source snapshot without Git checkout/LFS behavior.
# Model weights are downloaded separately from Hugging Face below.
RUN mkdir -p /opt/DepthART \
    && python3 -c "import urllib.request; urllib.request.urlretrieve('https://github.com/xuefeng-cvr/DepthART/archive/${DEPTHART_REF}.tar.gz', '/tmp/depthart.tar.gz')" \
    && tar -xzf /tmp/depthart.tar.gz --strip-components=1 -C /opt/DepthART \
    && rm /tmp/depthart.tar.gz

# Ubuntu 24.04 marks the system Python environment as externally managed.
# Install into the same interpreter used by rclpy/cv_bridge and the ROS
# ament_python console entry point.
RUN python3 -m pip install \
      --no-cache-dir \
      --break-system-packages \
      "torch==${TORCH_VERSION}" \
      "torchvision==${TORCHVISION_VERSION}" \
      --index-url https://download.pytorch.org/whl/cpu

# Minimal dependencies required by DepthART metric PyTorch inference.
# OpenCV and NumPy are provided by Ubuntu/ROS.
RUN python3 -m pip install \
      --no-cache-dir \
      --break-system-packages \
      "einops==0.8.1" \
      "timm==0.6.13"

# DepthART distributes model weights through Hugging Face rather than Git.
# Only download the checkpoint used by the current ROS configuration:
# Metric-S, indoor, 448.
RUN python3 -m pip install \
      --no-cache-dir \
      --break-system-packages \
      -U huggingface_hub \
    && cd /opt/DepthART \
    && hf download Fengxue93/DepthART \
         "metric/depthart_metric_indoor_s_448.pth" \
         --local-dir checkpoints \
    && grep 'checkpoints/metric/depthart_metric_indoor_s_448.pth$' \
         CHECKSUMS.sha256 | sha256sum -c -

# Validate the Python runtime and fail the build early on incompatibilities.
RUN python3 - <<'PY'
import sys
import cv2
import numpy
import torch
import torchvision
import timm
import einops

print("Python:", sys.version)
print("PyTorch:", torch.__version__)
print("Torchvision:", torchvision.__version__)
print("NumPy:", numpy.__version__)
print("OpenCV:", cv2.__version__)
print("timm:", timm.__version__)
print("einops:", einops.__version__)

assert not torch.cuda.is_available(), "CPU image unexpectedly has CUDA enabled"
PY

# Validate the DepthART metric inference imports themselves.
RUN cd /opt/DepthART/metric && python3 - <<'PY'
from common import make_K, preprocess
from model import load_model
from network import tvimblock

print("DepthART metric imports OK")
PY

# Verify that the checkpoint expected by leroy_depthart exists.
RUN test -f /opt/DepthART/checkpoints/metric/depthart_metric_indoor_s_448.pth

# ------------------------------------------------------------
# ROS workspace
# ------------------------------------------------------------

RUN mkdir -p /leroy_ws/src

# Leroy robot-specific packages.
#
# Docker build context is expected to be ros/, so:
#
#   ros/
#   ├── Dockerfile
#   ├── exec.sh
#   └── src/
#       ├── leroy_bridge/
#       ├── leroy_bringup/
#       ├── leroy_description/
#       ├── leroy_msgs/
#       ├── leroy_depthart/
#       └── xrslam_ros2/
#
COPY src/leroy_bridge /leroy_ws/src/leroy_bridge
COPY src/leroy_bringup /leroy_ws/src/leroy_bringup
COPY src/leroy_description /leroy_ws/src/leroy_description
COPY src/leroy_msgs /leroy_ws/src/leroy_msgs
COPY src/leroy_depthart /leroy_ws/src/leroy_depthart
# xrslam_ros2 is source-controlled here but must be built separately after libxrslam.so is available.

COPY config /config

# Build through the top-level runtime packages so colcon includes transitive
# dependencies alongside the Leroy packages.
RUN . /opt/ros/jazzy/setup.sh \
    && cd /leroy_ws \
    && colcon build \
         --symlink-install \
         --packages-up-to \
           leroy_bringup \
           leroy_depthart

# Basic build-time validation.
RUN . /opt/ros/jazzy/setup.sh \
    && . /leroy_ws/install/setup.sh \
    && python3 - <<'PY'
import serial
from leroy_msgs.msg import EncoderCounts

print("pyserial:", serial.VERSION)
print("leroy_msgs import OK:", EncoderCounts)
PY

# Make both ROS and Leroy workspace available in interactive shells.
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /leroy_ws/install/setup.bash" >> /root/.bashrc

COPY exec.sh /usr/local/bin/leroy-entrypoint
RUN chmod +x /usr/local/bin/leroy-entrypoint

CMD ["/usr/local/bin/leroy-entrypoint"]
