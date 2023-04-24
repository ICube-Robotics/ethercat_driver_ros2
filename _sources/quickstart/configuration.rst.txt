Configuration
=============

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

.. note:: As in the current implementation of :code:`ros2_control` there is no information about the system update frequency, it needs to be passed here as parameter. This is only needed by systems that include EtherCAT modules that use the Distributed Clock.

EtherCAT Slave modules as Plugins
---------------------------------

In this driver, the EtherCAT Slave modules are defined as `Plugins <https://docs.ros.org/en/foxy/Tutorials/Pluginlib.html>`_ and can be parametrized in the :code:`ros2_control` description file :

.. code-block:: xml

  <ec_module name="ECModule">
    <plugin>ethercat_plugins/ECModule</plugin>
    <param name="alias">0</param>
    <param name="position">1</param>
  </ec_module>

.. note:: All modules have :code:`alias` and :code:`position` parameters that specify their address in the EtherCAT Bus topology. Additional parameters can be specified depending on the purpose of the module.

EtherCAT Slave module plugins come in two version:

* **Generic plugins** : generic module implementation configured using a configuration file which purpose is to facilitate the use of generally available devices. For most applications, the use of these plugins is encouraged.
* **Specific plugins** : specific implementations for dedicated devices or dedicated functionalities.

Creating components with EtherCAT Slave modules
-----------------------------------------------

In :code:`ros2_control` the access to resources within a system from a controller is done by means of `Hardware Resources <https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md>`_. For this purpose :code:`state_interface` and :code:`command_interface` tags need to be defined and associated with the module functionalities.
Also, for better understanding of the overall system, the purpose of the used modules need to be clearly identified and sorted into the following types:

* :code:`<joint>`: logical component actuated by at least one actuator with read-write capacity.
* :code:`<sensor>`: logical component to read-only states from system.
* :code:`<gpio>`: logical component for general purpose IO systems.

Here is an example of a :code:`gpio` resource built using 2 EtherCAT IO modules:

.. code-block:: xml

  <gpio name="myGPIO">
    <command_interface name="dig_output.1"/>
    <command_interface name="dig_output.2"/>
    <state_interface name="ana_input.1"/>
    <state_interface name="ana_input.2"/>
    <ec_module name="EL3104">
      <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
      <param name="alias">0</param>
      <param name="position">0</param>
      <param name="slave_config">/path/to/EL3104_slave_config.yaml</param>
    </ec_module>
    <ec_module name="EL2008">
      <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
      <param name="alias">0</param>
      <param name="position">1</param>
      <param name="slave_config">/path/to/EL2008_slave_config.yaml</param>
    </ec_module>
  </gpio>

.. note:: To send commands to :code:`gpio` resources, a generic controller was developed and can be found on this `branch <https://github.com/mcbed/ros2_controllers/tree/gpio_controllers_only>`_.
