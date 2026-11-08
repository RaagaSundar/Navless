# Compile only portable ROS/Gazebo bridge code on native ARM64, never on Jetson.
FROM ros:humble-ros-base-jammy@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855 AS bridge_dependencies
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo 'deb [arch=arm64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      git build-essential python3-colcon-common-extensions \
      libgz-msgs10-dev libgz-transport13-dev \
      ros-humble-actuator-msgs ros-humble-gps-msgs ros-humble-vision-msgs \
      ros-humble-yaml-cpp-vendor ros-humble-rclcpp-components \
      ros-humble-rosgraph-msgs ros-humble-sensor-msgs ros-humble-tf2-msgs \
      ros-humble-trajectory-msgs && rm -rf /var/lib/apt/lists/*
ARG ROS_GZ_REF=856e319ee4da713609b3f6e58c74ac8db7662482
RUN mkdir -p /opt/jetson_gazebo_ws/src && \
    git init /opt/jetson_gazebo_ws/src/ros_gz && \
    cd /opt/jetson_gazebo_ws/src/ros_gz && \
    git remote add origin https://github.com/gazebosim/ros_gz.git && \
    git fetch --depth 1 origin ${ROS_GZ_REF} && git checkout --detach FETCH_HEAD
# Compile only through build_bridge_mac.sh in a named, memory-capped container.
# No compiler RUN is left here: cancelling a BuildKit client was insufficient
# proof that its VM worker had stopped during another chat's flight window.
