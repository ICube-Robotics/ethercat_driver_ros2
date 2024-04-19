Installation
===============================

**Required setup : Ubuntu 22.04 LTS**

Installing EtherLab
-------------------
The proposed development builds upon the `IgH EtherCAT Master <https://etherlab.org/en/ethercat/>`_.
Installation steps are summarized here:

* Verify that you can run unsigned kernel modules

Etherlab is a kernel module that is not signed by default.
To allow the kernel to load unsigned modules, you need to disable secure boot.

Verify if secure boot is enabled (you need to install ''mokutil'' first):
  .. code-block:: console

    $ sudo apt-get install mokutil
    $ mokutil --sb-state

it should print:
  .. code-block:: console

    SecureBoot disabled

if it prints:
  .. code-block:: console

    SecureBoot enabled

Then you need to disable secure boot.
To do so:

  1. reboot your computer and enter the BIOS settings.
  2. In the security tab, disable secure boot.
  3. Save and exit.

* Install required tools:

  .. code-block:: console

    $ sudo apt-get update
    $ sudo apt-get upgrade
    $ sudo apt-get install git autoconf libtool pkg-config make build-essential net-tools

* Setup sources for the EtherCAT Master:

  .. code-block:: console

    $ git clone https://gitlab.com/etherlab.org/ethercat.git
    $ cd ethercat
    $ git checkout stable-1.5
    $ sudo rm /usr/bin/ethercat
    $ sudo rm /etc/init.d/ethercat
    $ ./bootstrap  # to create the configure script

* Configure, build and install libs and kernel modules:

  .. code-block:: console

    $ ./configure --prefix=/usr/local/etherlab  --disable-8139too --disable-eoe --enable-generic

    $ make all modules
    $ sudo make modules_install install
    $ sudo depmod

* Configure system:

  .. code-block:: console

    $ sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
    $ sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat
    $ sudo mkdir -p /etc/sysconfig
    $ sudo cp /usr/local/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat

  .. note::

    These 4 steps may be needed every time the Linux kernel is updated.
    Before re-doing the 4 steps, you can try the following lighter steps:

    Go in the folder where the ethercat project was cloned, from step 2 (Setup sources) do:

    .. code-block:: console

      cd ethercat
      sudo rm /usr/bin/ethercat /etc/init.d/ethercat
      ./bootstrap

    Do integrally step 3 (Configure, build and install ...)
    From Step 4 (Configure system)

    .. code-block:: console

     sudo ln -s /usr/local/etherlab/bin/ethercat /usr/bin/
     sudo ln -s /usr/local/etherlab/etc/init.d/ethercat /etc/init.d/ethercat


* Create a new :code:`udev` rule:

  .. code-block:: console

    $ sudo gedit /etc/udev/rules.d/99-EtherCAT.rules

  containing:

  .. code-block:: console

    KERNEL=="EtherCAT[0-9]*", MODE="0666"


* Configure the network adapter for EtherCAT:

  .. code-block:: console

    $ sudo gedit /etc/sysconfig/ethercat

  In the configuration file specify the mac address of the network card to be used and its driver

  .. code-block:: console

    MASTER0_DEVICE="ff:ff:ff:ff:ff:ff"  # mac address
    DEVICE_MODULES="generic"

Now you can start the EtherCAT master:

.. code-block:: console

  $ sudo /etc/init.d/ethercat start

it should print

.. code-block:: console

  Starting EtherCAT master 1.5.2  done


You can check connected slaves:

.. code-block:: console

  $ ethercat slaves

It should print information of connected slave device:

.. code-block:: console

  <id>  <alias>:<position>  <device_state>  +  <device_name>

Example:

.. code-block:: console

  0  0:0  PREOP  +  <device_0_name>
  0  0:1  PREOP  +  <device_1_name>

Building :code:`ethercat_driver_ros2`
-------------------------------------

1.  Install ROS2 packages. The current development is based of :code:`ros2 humble`. Installation steps are described in the `ROS2 Humble Documentation <https://docs.ros.org/en/humble/Installation.html>`_.
2. Source your ROS2` environment:

  .. code-block:: console

    source /opt/ros/humble/setup.bash

  .. note:: The ROS2 environment needs to be sources in every used terminal. If only one distribution of ROS2 is used, it can be added to the :code:`~/.bashrc` file.

3. Install :code:`colcon` and its extensions :

  .. code-block:: console

    sudo apt install python3-colcon-common-extensions

4. Create a new ROS2 workspace:

  .. code-block:: console

    mkdir ~/ros2_ws/src

5. Pull relevant packages, install dependencies, compile, and source the workspace by using:

  .. code-block:: console

    cd ~/ros2_ws
    git clone https://github.com/ICube-Robotics/ethercat_driver_ros2.git src/ethercat_driver_ros2
    rosdep install --ignore-src --from-paths . -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
