# Built on PR #32's reusable Apple Silicon ROS/Gazebo desktop image.
FROM dronehacks-local:jazzy-harmonic
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-jazzy-rtabmap-odom ros-jazzy-rtabmap-slam \
    ros-jazzy-message-filters && rm -rf /var/lib/apt/lists/*
