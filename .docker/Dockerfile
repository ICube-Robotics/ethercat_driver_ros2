ARG ROS_DISTRO="jazzy"
FROM ros:${ROS_DISTRO}

#COPY . /ros2_dev/ethercat_driver_ros2

# Install dependencies
# --------------------

# Ethercat Master IgH
RUN apt update && \
    apt install autoconf libtool -y
RUN mkdir ec_dev && cd ec_dev && \
    git clone https://gitlab.com/etherlab.org/ethercat.git && \
    cd ethercat && \
    git checkout stable-1.5 && \
    autoupdate && \
    ./bootstrap && \
    ./configure --prefix=/usr/local/etherlab --disable-kernel && \
    make && make install
# Ros2 control interfaces
RUN apt install -y ros-${ROS_DISTRO}-control-msgs ros-${ROS_DISTRO}-control-toolbox ros-${ROS_DISTRO}-hardware-interface ros-${ROS_DISTRO}-controller-interface
# Ros formating tools
RUN apt install -y ros-${ROS_DISTRO}-ament-clang-format
