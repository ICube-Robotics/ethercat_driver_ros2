FailSafe over EtherCAT (FSoE) : Safety and the Ethercat Driver for ROS2
=======================================================================

In order to have safe robotic systems, it is important to have a safety layer that can monitor the system and take action in case of a failure. The FailSafe over EtherCAT (FSoE) protocol is a safety protocol that can be used to monitor the system and take action in case of a failure. 
In order to use the FSoE protocol with ROS2, the Ethercat Driver for ROS2 has been extended to be compatible with EtherCAT communications using the FSoE protocol.

This document describes the integration of FSoE communications in the Ethercat Driver for ROS2 and how to use it to monitor the system and take action in case of a failure.

In addition to the standard EtherCAT slaves, safety is implemented by adding safety slaves and safety masters to the EtherCAT network, all being «viewed» by the EtherCAT master as standard slaves.
One safety master handles a set of safety slaves creating a safety sub-network within the EtherCAT network. 
All the communications between the safety slaves and their respective safety masters are done inside the standard EtherCAT frames and must not be tampered with or an error will be raised.
The ethercat_driver_ros2 package has been extended:

1. to be able to record safety slaves and safety masters in the ros2_control URDF configuration file
2. to extend communication frames to include safety data
3. to handle safety data transfer between safety slaves and safety masters.

Safety module declaration
-------------------------

There are 2 types of safety slaves or masters:

1. the ones that expose safety data to ROS2 and will be either joints, sensors or gpios, we call them «ros2 safety» slaves or masters.
2. the ones that are not visible to ROS2, we call them «safety only» slaves or masters.

The «ros2 safety» slaves or masters are declared in the URDF configuration file as usual inside the gpio, sensor or joint tags.

The «safety only» slaves or masters are declared in the URDF configuration file in a new tag called «safety», inside standard «ec_module» tags.

For each FSoE net, a «net» tag is added as a child of the «safety» tag. The «net» tag has a the following attributes:

1. a «name» attribute,
2. a «safety_master» attribute providing the name of an «ec_module» that will act as the net safety master.

The «net» tag has children «transfer» tags describing the safety data transfers.

Safety data transfer specification
----------------------------------

A safety data transfer is a continuous memory array that has to be copied from one part of the communication frame to another part of the communication frame. The data to transfer is defined by:

  1. a first offset in the communication frame where the data must be read
  2. a second offset in the communication frame where the data must be written to
  3. the length of the data to transfer.

From a user perspective, the question is how to easily specify where the data is located in the communication frame for read and for write.
Our solution is to specify in the URDF configuration file the first data for read and for write by specifying:

1. the name of the module, the data is coming from or going to
2. the channel index and sub-index of the data in the module

The read location is defined in the the <in> tag and the write location is defined in the <out> tag of a <transfer> tag.
The length of the data to transfer is defined the attribute «size» of the <transfer> tag.

The <transfer> tag is a child of a <net> tag inside the <safety> tag.

Configuration of the field bus communication and ROS2 data exchanges
--------------------------------------------------------------------

From the user perspective modules are identified by their name, however in the EtherCAT network, modules are identified by their position (or alias) in the network.


on_init stage
~~~~~~~~~~~~~

At the «on_init» stage, the Ethercat Driver for ROS2 will:

1. first, look for all the EtherCAT modules in the URDF configuration that are exposed to ROS2 and load the corresponding plugins and their configuration. It effectively  called the :code:`addSlave` method of the :code:`EcMaster` class for each of these modules.
2. then, look for all the EtherCAT modules in the URDF that are «safety only» modules and load the corresponding plugins and their configuration. It effectively  called the :code:`addSlave` method of the :code:`EcMaster` class for each of these modules.
3. Compute the map between transfers in terms of module, name, channel index and sub-index and the corresponding offsets in the communication frame.

For each slave, in the add_slave method of the EcMaster class, the domain info are updated to prepare the ec_pdo_entry_reg_t data structures.
EcSafety is a derived class of EcMaster incorporating the safety data transfers.
So after each slave has been added, the safety data transfers are prepared such that all pointers to the offsets necessary for the safety data transfers are stored in the EcSafety class and will be ready when after the «on_activate» stage, the values pointed will contained the correct offset values.



on_activate stage
~~~~~~~~~~~~~~~~~

At the «on_activate» stage, the Ethercat Driver for ROS2 will:

1. Populate the ec_pdo_entry_reg_t data structures with the data with a call to the ecrt_domain_reg_pdo_entry_list function.
