# Installation
***Required setup : Ubuntu 22.04 LTS***

## 1. Installing EtherLab
The proposed development builds upon the [IgH EtherCAT Master](https://etherlab.org/en/ethercat/). Installation steps are summarized here:
- Install required tools:
  ```shell
  $ sudo apt-get update
  $ sudo apt-get upgrade
  $ sudo apt-get install git autoconf libtool pkg-config make build-essential net-tools
  ```
- Setup sources for the EtherCAT Master:
  ```shell
  $ git clone https://gitlab.com/etherlab.org/ethercat.git
  $ cd ethercat
  $ git checkout stable-1.5
  $ sudo rm /usr/bin/ethercat
  $ sudo rm /etc/init.d/ethercat
  $ ./bootstrap  # to create the configure script
  ```
- Configure, build and install libs and kernel modules:
  ```shell
  $ ./configure --prefix=/usr/local/etherlab  --disable-8139too --disable-eoe --enable-generic

  $ make all modules
  $ sudo make modules_install install
  $ sudo depmod
  ```
  **NOTE**: This step is needed every time the Linux kernel is updated.
- Configure system:
  ```shell
  $ sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
  $ sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
  $ sudo mkdir -p /etc/sysconfig
  $ sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
  ```
- Create a new `udev` rule:
  ```shell
  $ sudo gedit /etc/udev/rules.d/99-EtherCAT.rules
  ```
  containing:
  ```shell
  KERNEL=="EtherCAT[0-9]*", MODE="0664"
  ```

- Configure the network adapter for EtherCAT:

  ```shell
  $ sudo gedit /etc/sysconfig/ethercat
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
it should print
```shell
Starting EtherCAT master 1.5.2  done
```

You can check connected slaves:
```shell
$ ethercat slaves
```
It should print information of connected slave device:
```shell
<id>  <alias>:<position>  <device_state>  +  <device_name>
```
Example:
```shell
0  0:0  PREOP  +  <device_0_name>
0  0:1  PREOP  +  <device_1_name>
```

## 2. Building `ethercat_driver_ros2`
1.  Install `ros2` packages. The current development is based on `ros2 rolling` and compatible with `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/rolling/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/$ROS_DISTRO/setup.bash
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
