SDO acyclic exchange using ROS2 services
========================================

Service Data Objects (SDO) contain object dictionary entries that can be exchanged acyclically. SDO works as a mailbox sending and buffering received data. This communication is acyclic and is dependent on available bandwidth in the communication cycle. This communication is not deterministic and is best suited for transmitting configuration data.

Services
--------

The EtherCAT Driver ROS2 stack comes with a service server that allows the exchange of data with an EtherCAT slave using ROS2 services through SDO.
The following services are available:

.. list-table::
  :header-rows: 1

  * - Service
    - Interface
    - Description
  * - :code:`ethercat_manager/get_sdo`
    - :code:`ethercat_msgs::srv::GetSdo`
    - Read data from slave SDO register
  * - :code:`ethercat_manager/set_sdo`
    - :code:`ethercat_msgs::srv::SetSdo`
    - Write data to slave SDO register

Interfaces
----------

The interfaces used by the services are defined such that:

+---------------+-------------------------------------+-------------------------------------+
|               | :code:`ethercat_msgs::srv::GetSdo`  | :code:`ethercat_msgs::srv::SetSdo`  |
+---------------+-------------------------------------+-------------------------------------+
| **Request**   | .. code-block:: shell               | .. code-block:: shell               |
|               |                                     |                                     |
|               |   int16 master_id                   |   int16 master_id                   |
|               |   uint16 slave_position             |   int16 slave_position              |
|               |   uint16 sdo_index                  |   uint16 sdo_index                  |
|               |   uint8 sdo_subindex                |   uint8 sdo_subindex                |
|               |   string sdo_data_type              |   string sdo_data_type              |
|               |                                     |   string sdo_value                  |
+---------------+-------------------------------------+-------------------------------------+
| **Response**  | .. code-block:: shell               | .. code-block:: shell               |
|               |                                     |                                     |
|               |   bool success                      |   bool success                      |
|               |   string sdo_return_message         |   string sdo_return_message         |
|               |   string sdo_return_value_string    |                                     |
|               |   float64 sdo_return_value          |                                     |
+---------------+-------------------------------------+-------------------------------------+

Usage
-----

Build and source the :code:`ethercat_manager` package and run:

.. code-block:: shell

  $ ros2 run ethercat_manager ethercat_sdo_srv_server
