ARG ROS_DISTRO="galactic"
FROM ros:${ROS_DISTRO}

COPY . /ros2_dev/ethercat_driver_ros2

RUN apt update && \
    apt install autoconf libtool -y
RUN mkdir ec_dev && cd ec_dev && \
    git clone https://gitlab.com/etherlab.org/ethercat.git && \
    cd ethercat && \
    git checkout stable-1.5 && \ 
    ./bootstrap && \
    ./configure --prefix=/usr/local/etherlab --disable-kernel && \
    make && make install
