Configuration of the EtherCAT Driver ROS2 Stack and :code:`ros2_control`
========================================================================

Hardware Interfaces and EtherCAT Master
---------------------------------------

The EtherCAT Driver is designed in such a way that each EtherCAT Master is defined in :code:`ros2_control` as a specific Hardware Interface. This is done in the :code:`ros2_control` description file, where the EtherCAT driver is loaded as Hardware Interface and linked to an EtherCAT Master by its ID:

.. code-block:: xml

  <ros2_control name="mySystem" type="system">
    <hardware>
      <plugin>ethercat_driver/EthercatDriver</plugin>
      <param name="master_id">0</param>
      <param name="control_frequency">100</param>
    </hardware>
  </ros2_control>

.. note:: As in the current implementation of :code:`ros2_control` there is no information about the system update frequency, it needs to be passed here as parameter. This is only needed systems that include EtherCAT modules that use the Distributed Clock.

EtherCAT Slave modules as Plugins
---------------------------------

In this driver, the EtherCAT Slave modules are defined as :ref:`Plugins <https://docs.ros.org/en/foxy/Tutorials/Pluginlib.html>` and can be parametrized in the :code:`ros2_control` description file :

.. code-block:: xml

  <ec_module name="ECModule">
    <plugin>ethercat_plugins/ECModule</plugin>
    <param name="alias">0</param>
    <param name="position">1</param>
  </ec_module>

.. note:: All modules have :code:`alias` and :code:`position` parameters that specify their address in the EtherCAT Bus topology. Additional parameters can be specified depending on the purpose of the module. A list of implemented modules and their parameters can be found :ref:`here <ethercat_plugins/available_plugins.md>`.

Interfacing controllers with EtherCAT Slave modules
---------------------------------------------------

In :code:`ros2_control` the access to resources within a system from a controller is done by means of :ref:`Hardware Resources <https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md>`. For this purpose :code:`state_interface` and :code:`command_interface` tags need to be defined and associated with the module functionalities.
Also, for better understanding of the overall system, the purpose of the used modules need to be clearly identified and sorted into the following types:
* :code:`<joint>`: logical component actuated by at least one actuator with read-write capacity.
* :code:`<sensor>`: logical component to read-only states from system.
* :code:`<gpio>`: logical component for general purpose IO systems.

.. note:: These components have the possibility to include parameters that will be used to link particular states and commands to the slave module input/outputs.

Here is an example of a :code:`gpio` resource built using 2 EtherCAT IO modules, where digital commands are mapped on ports 4 and 6 of the digital output module and analog states are read from ports 1 and 4 of the analog input module:

.. code-block:: xml

  <gpio name="myGPIO">
    <command_interface name="dig_output.1"/>
    <command_interface name="dig_output.2"/>
    <state_interface name="dig_output.1"/>
    <state_interface name="dig_output.2"/>
    <state_interface name="ana_input.1"/>
    <state_interface name="ana_input.2"/>
    <ec_module name="EL3104">
      <plugin>ethercat_plugins/Beckhoff_EL3104</plugin>
      <param name="alias">0</param>
      <param name="position">1</param>
      <param name="ai.1">ana_input.1</param>
      <param name="ai.4">ana_input.2</param>
    </ec_module>
    <ec_module name="EL2008">
      <plugin>ethercat_plugins/Beckhoff_EL2008</plugin>
      <param name="alias">0</param>
      <param name="position">2</param>
      <param name="do.4">dig_output.2</param>
      <param name="do.6">dig_output.1</param>
    </ec_module>
  </gpio>

.. note:: To send commands to :code:`gpio` resources, a generic controller was developed and can be found :ref:`here <https://github.com/mcbed/ros2_controllers/tree/gpio_controllers>`.
