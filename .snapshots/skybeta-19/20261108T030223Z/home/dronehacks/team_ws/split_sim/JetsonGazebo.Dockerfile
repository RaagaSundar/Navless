# Experimental headless Gazebo alternative; never rebuild the flight image tag.
FROM skybeta:0.3.3@sha256:4b637ceeb521ff5e247577b216e2edb26195be2cf2d7c7b9915ea5f9116cb2aa AS gazebo_dependencies
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
# OSRF does not publish ros-humble-ros-gzharmonic for Jammy arm64. Keep Gazebo
# binary packages, compile only the official Humble/Harmonic ROS bridge below.
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo 'deb [arch=arm64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic libgz-msgs10-dev libgz-transport13-dev mesa-utils libegl1 \
      ros-humble-actuator-msgs ros-humble-gps-msgs ros-humble-vision-msgs \
      ros-humble-yaml-cpp-vendor && \
    rm -rf /var/lib/apt/lists/*
ARG ROS_GZ_REF=856e319ee4da713609b3f6e58c74ac8db7662482
RUN mkdir -p /opt/jetson_gazebo_ws/src && \
    git init /opt/jetson_gazebo_ws/src/ros_gz && \
    cd /opt/jetson_gazebo_ws/src/ros_gz && \
    git remote add origin https://github.com/gazebosim/ros_gz.git && \
    git fetch --depth 1 origin ${ROS_GZ_REF} && git checkout --detach FETCH_HEAD && \
    source /opt/ros/humble/setup.bash && cd /opt/jetson_gazebo_ws && \
    GZ_VERSION=harmonic MAKEFLAGS=-j1 colcon build --executor sequential \
      --packages-select ros_gz_interfaces \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
# Bridge compilation occurs on the Mac in an ARM64 Jammy/Humble builder.
# Only its install tree is copied; the aircraft image tag is unchanged.
FROM gazebo_dependencies AS gazebo_runtime
COPY jetson_gazebo/bridge_install/ /opt/jetson_gazebo_ws/install/
