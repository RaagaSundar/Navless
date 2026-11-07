FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic ros-jazzy-ros-gz ros-jazzy-cv-bridge \
      ros-jazzy-tf2-ros ros-jazzy-tf-transformations \
      python3-colcon-common-extensions python3-pytest python3-scipy \
      python3-opencv python3-matplotlib python3-pip python3-venv \
      build-essential cmake ninja-build git ccache pkg-config \
      libopencv-dev libssl-dev libxml2-dev libxml2-utils libeigen3-dev \
      libunwind-dev protobuf-compiler cppzmq-dev \
      xvfb x11vnc novnc websockify openbox xterm mesa-utils \
      libgl1-mesa-dri dbus-x11 procps util-linux x11-utils && \
    rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/sim-venv && \
    /opt/sim-venv/bin/pip install --no-cache-dir \
      'numpy<2' 'empy>=3.3,<4' pymavlink pyserial pyulog \
      jinja2 jsonschema kconfiglib packaging pyyaml toml lxml \
      future pyros-genmsg setuptools
ENV PATH="/opt/sim-venv/bin:${PATH}" \
    DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 \
    ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 GZ_IP=127.0.0.1 \
    GZ_PARTITION=dronehacks_local
COPY desktop.sh /usr/local/bin/sim-desktop
COPY course.sh /usr/local/bin/sim-course
RUN chmod +x /usr/local/bin/sim-desktop /usr/local/bin/sim-course && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc
WORKDIR /workspace/Navless
CMD ["/usr/local/bin/sim-desktop"]
