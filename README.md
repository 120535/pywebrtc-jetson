# WebRTC for ROS2
## for Jetson (Orin) Nano/x86, in Docker, in Python (based on aiortc)
## image plus data channel

Dockerfile/installation and ROS2 node, with the goal of robotic teleoperation.

## Overview

Please see the Medium article for a richer discussion.   Basically this leverages [aiortc](https://github.com/aiortc/aiortc) within the ROS2 environment.

## Installation

This assumes you're on the offical NVidia docker path -  [Developer Environment Setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)    More specifically [Isaac ROS Docker Development Environment](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html).    

1. Clone this repository (on the host)
cd ${ISAAC_ROS_WS}/src
git clone git@github.com:pgaston/pywebrtc.git

2. Copy the following two files - this is for customizing the docker build process
```
${ISAAC_ROS_WS}/src/pywebrtc/docker/.isaac_ros_common-config
${ISAAC_ROS_WS}/src/pywebrtc/docker/.isaac_ros_common-config
```
to the folder
```
${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
```
after this, run_dev.sh should work.

3. Build

Build (in the docker)
```
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select pywebrtc
```
4. Run/test
```
source install/setup.bash
ros2 run pywebrtc websrvr
```

You can then browse to the following page with your browser:

http://127.0.0.1:8080

Once you click `Start` the server will send video from its webcam to the
browser.   The initial screen is gray.

If you have a realsense installed, run in another terminal (do the run_dev.sh thing to get into the same docker)
```
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

** voila - WebRTC showing your realsense image **









## blah blah

The [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
repository contains a number of scripts and Dockerfiles to help
streamline development and testing with the Isaac ROS suite.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_common/isaac_ros_common_tools.png/"><img alt="Isaac ROS DevOps tools" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_common/isaac_ros_common_tools.png/" width="auto"/></a></div>

The Docker images included in this package provide pre-compiled binaries
for ROS 2 Humble on Ubuntu 20.04 Focal.

Additionally, on x86_64 platforms, Docker containers allow you to
quickly set up a sensitive set of frameworks and dependencies to ensure
a smooth experience with Isaac ROS packages. The Dockerfiles for this
platform are based on the version 22.03 image from [Deep Learning
Frameworks Containers](https://docs.nvidia.com/deeplearning/frameworks/support-matrix/index.html).
On Jetson platforms, JetPack manages all of these dependencies for you.

Use of Docker images enables CI|CD systems to scale with DevOps work and
run automated testing in cloud native platforms on Kubernetes.

For solutions to known issues, see the [Troubleshooting](https://nvidia-isaac-ros.github.io/troubleshooting/index.html) section.

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html) to learn how to use this repository.

---

## Latest

Update 2023-10-18: Updated for Isaac ROS 2.0.0.
