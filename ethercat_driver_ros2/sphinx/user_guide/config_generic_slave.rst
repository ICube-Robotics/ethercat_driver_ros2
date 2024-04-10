Generic EtherCAT Slave configuration
====================================

Configuration options
---------------------

The :code:`GenericEcSlave` allows to configure the following options in the :code:`slave_config` file:

.. list-table::
  :widths: 15 35
  :header-rows: 1

  * - Configuration flag
    - Description
  * - :code:`vendor_id`
    - Vendor identification number in hexadecimal format :code:`0x...`.
  * - :code:`product_id`
    - Product identification number in hexadecimal format :code:`0x...`.
  * - :code:`assign_activate`
    - Distributed Clock Synchronization register in hexadecimal format :code:`0x...`. If not used remove or set to :code:`0x00`.
  * - :code:`sdo`
    - SDO data to be transferred at drive startup for configuration purposes.
  * - :code:`tpdo`
    - Transmit PDO mapping configuration.
  * - :code:`rpdo`
    - Receive PDO mapping configuration.
  * - :code:`sm`
    - Sync Manager configuration.

SDO configuration
~~~~~~~~~~~~~~~~~

Service Data Objects (SDO) are used to setup the module at startup. This is done only one during the activation phase.
Each SDO has the following configuration flags:

.. list-table::
  :widths: 15 35
  :header-rows: 1

  * - SDO flag
    - Description
  * - :code:`index`
    - SDO index in hexadecimal format :code:`0x...`.
  * - :code:`sub_index`
    - SDO sub-index in hexadecimal format :code:`0x...`.
  * - :code:`type`
    - SDO data type. Possible types: :code:`bool`, :code:`uint8`, :code:`int8`, :code:`uint16`, :code:`int16`, :code:`uint32`, :code:`uint32`, :code:`uint64`, :code:`uint64`, :code:`bitN` with N the number of bits required.
  * - :code:`value`
    - Value to be transferred to the module.

PDO mapping configuration
~~~~~~~~~~~~~~~~~~~~~~~~~

The :code:`tpdo` and :code:`rpdo` Process Data Object (PDO) mapping configurations can be composed of multiple PDO mappings.
Each PDO mapping requires to specify its register :code:`index` and the configuration of the PDO Channels it is composed of.

PDO channel configuration
~~~~~~~~~~~~~~~~~~~~~~~~~

Each PDO Channel has the following configuration flags:

.. list-table::
  :widths: 15 35
  :header-rows: 1

  * - PDO Channel flag
    - Description
  * - :code:`index`
    - Channel index in hexadecimal format :code:`0x...`.
  * - :code:`sub_index`
    - Channel sub-index in hexadecimal format :code:`0x...`.
  * - :code:`type`
    - Channel data type. Possible types: :code:`bool`, :code:`uint8`, :code:`int8`, :code:`uint16`, :code:`int16`, :code:`uint32`, :code:`uint32`, :code:`uint64`, :code:`uint64`, :code:`bitN` with N the number of bits required.
  * - :code:`command_interface`
    - **Only for** :code:`rpdo`. Name of the command interface to be used inside :code:`ros2_control`.
  * - :code:`state_interface`
    - **Only for** :code:`tpdo`. Name of the state interface to be used inside :code:`ros2_control`.
  * - :code:`default`
    - **Only for** :code:`rpdo`. Default value to be sent if data received on the command interface is :code:`NaN`.
  * - :code:`mask`
    - Data mask, to be used with :code:`type` = :code:`bool`.
  * - :code:`factor`
    - Data conversion factor/scale (:code:`type` : :code:`double`).
  * - :code:`offset`
    - Data offset term (:code:`type` : :code:`double`).


.. warning:: For each channel, tags :code:`index`, :code:`sub_index` and :code:`type` are **mandatory** even if the channel is not used in order to fill the data layout expected by the module. All other tags can remain unset.
.. note:: Data type :code:`bitN` is used for gaps in the config. Refer to module manual if required.
.. note:: Data type :code:`bool` requires the use of the :code:`mask` option as the registers can only be read as a multiple of 8 bits.

.. note::

   Data (d) can be modified using the optional :code:`factor` (f) and :code:`offset` (o) parameters following a linear relation: :math:`d \longrightarrow f\times d + o`. Default value are :math:`f=1` and :math:`o=0`. It is particularly useful for:

  - scaling analog values to physical units,
  - take into account calibration offsets,
  - convert between different units,
  - take into account transmission parameters like gear reduction or screw lead for motor control.

Sync Manager Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~

A SyncManager protects a DPRAM area from simultaneous access and thus ensures data consistency. For more information, refer to the `Synch Manager EtherCAT documentation <https://infosys.beckhoff.com/english.php?content=../content/1033/tc3_io_intro/4981170059.html&id=>`_.
Sync Manager can be configured with the following options:

.. list-table::
  :widths: 15 35
  :header-rows: 1

  * - Sync Manager flag
    - Description
  * - :code:`index`
    - Sync Manager index.
  * - :code:`type`
    - Sync Manager type. Can be either :code:`output` or :code:`input`.
  * - :code:`pdo`
    - PDO to be mapped on the Sync Manager. Can be :code:`rpdo`, :code:`tpdo` or :code:`~` if empty.
  * - :code:`watchdog`
    - Enables Sync Manager Watchdog. Can be :code:`disable` or :code:`enable`.

.. note:: If :code:`sm` is not specified, the default Sync Manager configuration is :

  .. code-block:: yaml

    sm:  # Sync Manager
      - {index: 0, type: output, pdo: ~, watchdog: disable}
      - {index: 1, type: input, pdo: ~, watchdog: disable}
      - {index: 2, type: output, pdo: rpdo, watchdog: enable}
      - {index: 3, type: input, pdo: tpdo, watchdog: disable}

Usage
-----

Example configuration for the Beckhoff EL3104 analog input module:

..  code-block:: yaml

  # Configuration file for Beckhoff EL3104
  vendor_id: 0x00000002
  product_id: 0x0c1e3052
  tpdo:  # TxPDO
    - index: 0x1a00
      channels:
        - {index: 0x3101, sub_index: 1, type: uint8}
        - {
            index: 0x3101,
            sub_index: 2,
            type: int16,
            state_interface: analog_input.1,
            factor: 0.000305185
          }
    - index: 0x1a01
      channels:
        - {index: 0x3102, sub_index: 1, type: uint8}
        - {
            index: 0x3102,
            sub_index: 2,
            type: int16,
            state_interface: analog_input.2,
            factor: 0.000305185
          }
  sm:  # Sync Manager
    - {index: 0, type: output, pdo: ~, watchdog: disable}
    - {index: 1, type: input, pdo: ~, watchdog: disable}
    - {index: 2, type: output, pdo: ~, watchdog: disable}
    - {index: 3, type: input, pdo: tpdo, watchdog: disable}

Example configuration for the Beckhoff EL2008 digital output module using data type :code:`bool` with :code:`mask`:

.. code-block:: yaml

 # Configuration file for Beckhoff EL2008
  vendor_id: 0x00000002
  product_id: 0x07d83052
  rpdo:  # RxPDO
    - index: 0x1a00
      channels:
        - {index: 0x6000, sub_index: 1, type: bool, mask: 1, command_interface: d_output.1}
    - index: 0x1a01
      channels:
        - {index: 0x6010, sub_index: 1, type: bool}
    - index: 0x1a02
      channels:
        - {index: 0x6020, sub_index: 1, type: bool}
    - index: 0x1a03
      channels:
        - {index: 0x6030, sub_index: 1, type: bool, mask: 8, command_interface: d_output.4}
    - index: 0x1a04
      channels:
        - {index: 0x6040, sub_index: 1, type: bool}
    - index: 0x1a05
      channels:
        - {index: 0x6050, sub_index: 1, type: bool}
    - index: 0x1a06
      channels:
        - {index: 0x6060, sub_index: 1, type: bool}
    - index: 0x1a07
      channels:
        - {index: 0x6070, sub_index: 1, type: bool}
  sm:  # Sync Manager
    - {index: 0, type: output, pdo: rpdo, watchdog: enable}

.. note:: In this configuration only digital output 1 and 4 will be used and are therefore configured. The other channels are set up with the mandatory tags :code:`index`, :code:`sub_index` and :code:`type` to fill the data layout expected by the module.

This configuration can be used for controlling a :code:`gpio` component. Here is an example urdf for :code:`ros2_control` using this configuration together with the :code:`GenericEcSlave` plugin:

.. code-block:: xml

  <ros2_control name="ec_single_gpio" type="system">
      <hardware>
        <plugin>ethercat_driver/EthercatDriver</plugin>
        <param name="master_id">0</param>
        <param name="control_frequency">100</param>
      </hardware>

     <gpio name="gpio_0">
        <state_interface name="analog_input.1"/>
        <state_interface name="analog_input.2"/>
        <ec_module name="EL3104">
          <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
          <param name="alias">0</param>
          <param name="position">0</param>
          <param name="slave_config">/path/to/EL3104_slave_config.yaml</param>
        </ec_module>
      </gpio>

      <gpio name="gpio_1">
        <command_interface name="d_output.1"/>
        <command_interface name="d_output.4"/>
        <ec_module name="EL2008">
          <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
          <param name="alias">0</param>
          <param name="position">1</param>
          <param name="slave_config">/path/to/EL2008_slave_config.yaml</param>
        </ec_module>
      </gpio>
    </ros2_control>
