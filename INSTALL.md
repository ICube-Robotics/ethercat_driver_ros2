## Installation
***Required setup : Ubuntu 20.04 LTS***

### 1. Installing EtherLab
The proposed development builds upon the [IgH EtherCAT Master](https://etherlab.org/en/ethercat/). Installation steps are summarized here:
```shell
$ git clone https://gitlab.com/etherlab.org/ethercat.git
$ cd ethercat
$ git checkout stable-1.5
```
```shell
$ cd ethercat
$ ./bootstrap # to create the configure script, if downloaded from the repository

$ ./configure --prefix=/usr/local/etherlab  --disable-8139too --enable-generic # Ethernet driver e1000e not supported for kernels 4.X
$ make all modules
$ sudo make modules_install install
$ sudo depmod
$ sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
$ sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
$ sudo mkdir -p /etc/sysconfig
$ sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
$ sudo echo KERNEL==\"EtherCAT[0-9]*\", MODE=\"0664\", GROUP=\"ecusers\" > /etc/udev/rules.d/99-EtherCAT.rules

$ sudo vi /etc/sysconfig/ethercat
```
In the configuration file specify the mac address of the network card to be used and its driver
```shell
MASTER0_DEVICE="ff:ff:ff:ff:ff:ff"  # mac address
DEVICE_MODULES="generic"
```

Now you can start the EtherCAT master:
```shell
$ sudo /etc/init.d/ethercat start
```
Make sure your current user is member of `ecusers` and check connected slaves:
```shell
$ ethercat slaves
```

### 2. Building `ethercat_driver_ros2`
1.  Install `ros2` packages. The current developpment is based of `ros2 humble`. Installation steps are decribed [here](https://docs.ros.org/en/humble/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
3. Create a new ros2 workspace:
    ```shell
    mkdir ~/ros2_ws/src
    ```
4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```shell
    cd ~/ros2_ws
    git clone https://github.com/ICube-Robotics/ethercat_driver_ros2.git src/ethercat_driver_ros2
    rosdep install --ignore-src --from-paths . -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```
