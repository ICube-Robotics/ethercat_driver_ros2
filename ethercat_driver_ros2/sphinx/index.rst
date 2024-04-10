EtherCAT Driver ROS2 Stack
==========================

`EtherCAT <https://www.ethercat.org/default.htm>`_ provides applications with the capacity of reliable, real-time communication between systems and is therefore a common industrial standard. In order to simplify the development/deployment of new application using EtherCAT modules, this stack allows to combine them with `ros2_control <https://github.com/ros-controls/ros2_control>`_. This driver proposes a generic ways to parametrize and assemble Hardware Interfaces based on EtherCAT modules that can be defined using parameter files.

**Project GitHub repository:** `ethercat_driver_ros2 <https://github.com/ICube-Robotics/ethercat_driver_ros2>`_

.. toctree::
  :maxdepth: 1
  :caption: Quickstart
  :glob:

  quickstart/installation
  quickstart/configuration
  quickstart/usage

.. toctree::
  :maxdepth: 1
  :caption: User Guide
  :glob:

  user_guide/config_generic_slave
  user_guide/config_cia402_drive
  user_guide/config_use_case_motor_with_gear_box
  user_guide/sdo_async_com

.. toctree::
  :maxdepth: 1
  :caption: Developer Guide
  :glob:

  developer_guide/coe
  developer_guide/cia402_drive
  developer_guide/new_plugin
  API Reference <https://ICube-Robotics.github.io/ethercat_driver_ros2/api/>
