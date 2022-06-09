ARG ROS_DISTRO="galactic"
FROM ros:${ROS_DISTRO}

RUN apt update && \
    apt install autoconf libtool -y
RUN mkdir ec_dev && cd ec_dev && \
    git clone https://gitlab.com/etherlab.org/ethercat.git && \
    cd ethercat && \
    git checkout stable-1.5 && \ 
    ./bootstrap && \
    ./configure --prefix=/usr/local/etherlab --disable-kernel && \
    make && make install 
RUN mkdir ros2_dev && cd ros2_dev && \
    . /opt/ros/galactic/setup.sh && \
    git clone https://github.com/ICube-Robotics/ethercat_driver_ros2.git src/ethercat_driver_ros2 && \
    rosdep install --ignore-src --from-paths . -y -r && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

