ARG BASE_IMAGE=skybeta:0.3.3
FROM ${BASE_IMAGE}
USER root
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends openssh-server python3 && rm -rf /var/lib/apt/lists/* /etc/ssh/ssh_host_* && useradd -m -s /bin/bash participant && passwd -d participant
COPY sim_sessions /opt/session
COPY split_sim/sensor_link.py split_sim/sim_slam.launch.py /opt/source/
COPY split_sim/viewer /opt/source/viewer
ENTRYPOINT ["python3","/opt/session/inside.py"]
