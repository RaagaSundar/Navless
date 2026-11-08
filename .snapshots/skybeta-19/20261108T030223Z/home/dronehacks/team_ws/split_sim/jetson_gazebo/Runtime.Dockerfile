# Cached Jetson dependencies plus a native ARM64 Jammy/Humble install artifact.
# This file has no compilation step and never modifies the aircraft image tag.
FROM drone19-gazebo-deps:humble-harmonic
USER root
COPY bridge_install/ /opt/jetson_gazebo_ws/install/
