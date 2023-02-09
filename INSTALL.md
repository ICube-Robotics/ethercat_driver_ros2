## Installation
***Required setup : Ubuntu 22.04 LTS***

### 1. Installing EtherLab
The proposed development builds upon the [IgH EtherCAT Master](https://etherlab.org/en/ethercat/). Installation steps are summarized here:
```shell
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install git autoconf libtool pkg-config make build-essential net-tools
$ git clone https://gitlab.com/etherlab.org/ethercat.git
$ cd ethercat
$ git checkout stable-1.5
$ sudo rm /usr/bin/ethercat
$ sudo rm /etc/init.d/ethercat
$ ./bootstrap  # to create the configure script, if downloaded from the repository
$ ./configure --prefix=/usr/local/etherlab  --disable-8139too --enable-generic
$ make all modules
$ sudo make modules_install install
$ sudo depmod
$ sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
$ sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
$ sudo mkdir -p /etc/sysconfig
$ sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat
$ sudo groupadd ecusers
$ sudo usermod -a -G ecusers ${USER}
$ sudo bash -c "echo KERNEL==\"EtherCAT[0-9]*\", MODE=\"0664\", GROUP=\"ecusers\" > /etc/udev/rules.d/99-EtherCAT.rules"
```
```shell
$ sudo vi /etc/sysconfig/ethercat
```
or
```shell
$ sudo gedit /etc/sysconfig/ethercat # If using ubuntu desktop
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

Make sure your current user is member of `ecusers` and check connected slaves:
```shell
$ ethercat slaves
```
It should print information of connected slave device. Example,
```shell
0  2:0  PREOP  +  NX-ECC201 EtherCAT coupler V1.2
```

### 2. Building `ethercat_driver_ros2`
1.  Install `ros2` packages. The current development is based of `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).
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
