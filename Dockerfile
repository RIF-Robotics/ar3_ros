ARG ROS_DISTRO="humble"

FROM docker.io/ros:${ROS_DISTRO}

# Support podman and docker when building locally
# See https://github.com/opencontainers/runc/issues/2517
RUN echo 'APT::Sandbox::User "root";' > /etc/apt/apt.conf.d/sandbox-disable

ENV ROS_OVERLAY /opt/ros/ci

WORKDIR $ROS_OVERLAY

COPY ar3 src/ar3
COPY ar3_bringup src/ar3_bringup
COPY ar3_controllers src/ar3_controllers
COPY ar3_description src/ar3_description
COPY ar3_embedded src/ar3_embedded
COPY ar3_hardware src/ar3_hardware
COPY ar3_hardware_driver src/ar3_hardware_driver
COPY ar3_msgs src/ar3_msgs

RUN apt-get update && \
    rosdep install -iy --from-paths src && \
    rm -rf /var/lib/apt/lists/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon test ; \
    colcon test-result --verbose
