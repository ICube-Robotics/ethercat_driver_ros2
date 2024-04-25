CiA402 EtherCAT Motor Drive configuration
=========================================

The generic plugin :code:`EcCiA402Drive` is a particular case of the :code:`GenericEcSlave` enabling features compliant with the CiA402 (“CANopen device profile for drives and motion control”) norm. Therefore, the configuration of such a drive uses the same formalism for the :code:`slave_config` file as in the case of the :code:`GenericEcSlave`.

Plugin features
---------------

* **Drive State transitions**: Management of the motor drive states and their transitions.
* **Drive Fault reset**: Management of the motor drive fault reset using :code:`command_interface` "reset_fault".
* **Mode of Operation**: Management of multiple cyclic modes of operation : position (8), velocity (9), effort (10) and homing (6) with the possibility of switch between them.
* **Default position**: Management of the target position when not controlled.

Configuration options
---------------------

In addition to the configuration options given by the :code:`GenericEcSlave`, the :code:`EcCiA402Drive` allows to configure the following options in the :code:`slave_config` file:

.. list-table::
  :widths: 15 35
  :header-rows: 1

  * - Configuration flag
    - Description
  * - :code:`auto_fault_reset`
    - if set to :code:`true` the drive performs automatic fault reset; if set to :code:`false`, fault reset is only performed on rising edge (0 -> 1) command on the :code:`command_interface` "reset_fault".

Behavior
--------
Here are some remarks about the implemented motor drive behavior logic.

After launching the well-configured drive, by default and without fault, motor drive module is brought automatically into the state :code:`OPERATION_ENABLED` making it ready for use. Automatic transition is only enabled when the :code:`control_word` command interface is either missing or set to :code:`NaN` making it possible for the user to take control over the motor drive's state machine by sending corresponding state transition values using the :code:`control_word` command interface.

The default mode of operation of the motor drive can be set either in the configuration yaml file as the default value of the corresponding PDO channel or in urdf using the :code:`mode_of_operation` parameter of the the :code:`EcCiA402Drive`. If both are set, the urdf parameter value overrides the default one from the configuration yaml file.

In order to prevent unwanted movements of the motor, if uncontrolled, the default target position that is send to the drive in all modes of operation is always the last read position. That is why, it is important to send :code:`NaN` in the position command interface when not controlling the motor position. This applies especially for cases when switching between modes.

Usage
-----

Example configuration for the Maxon EPOS3 motor dive:

..  code-block:: yaml
  :linenos:

  # Configuration file for Maxon EPOS3 drive
  vendor_id: 0x000000fb
  product_id: 0x64400000
  assign_activate: 0x0300  # DC Synch register
  auto_fault_reset: false  # true = automatic fault reset, false = fault reset on rising edge command interface "reset_fault"
  sdo:  # sdo data to be transferred at drive startup
    - {index: 0x60C2, sub_index: 1, type: int8, value: 10} # Set interpolation time for cyclic modes to 10 ms
    - {index: 0x60C2, sub_index: 2, type: int8, value: -3} # Set base 10-3s
  rpdo:  # RxPDO = receive PDO Mapping
    - index: 0x1603
      channels:
        - {index: 0x6040, sub_index: 0, type: uint16, default: 0}  # Control word
        - {
            index: 0x607a,
            sub_index: 0,
            type: int32,
            command_interface: position,
            default: .nan}  # Target position
        - {index: 0x60ff, sub_index: 0, type: int32, default: 0}  # Target velocity
        - {index: 0x6071, sub_index: 0, type: int16, default: 0}  # Target torque
        - {index: 0x60b0, sub_index: 0, type: int32, default: 0}  # Offset position
        - {index: 0x60b1, sub_index: 0, type: int32, default: 0}  # Offset velocity
        - {index: 0x60b2, sub_index: 0, type: int16, default: 0}  # Offset torque
        - {index: 0x6060, sub_index: 0, type: int8, default: 8}  # Mode of operation
        - {index: 0x2078, sub_index: 1, type: uint16, default: 0}  # Digital Output Functionalities
        - {index: 0x60b8, sub_index: 0, type: uint16, default: 0}  # Touch Probe Function
  tpdo:  # TxPDO = transmit PDO Mapping
    - index: 0x1a03
      channels:
        - {index: 0x6041, sub_index: 0, type: uint16}  # Status word
        - {
            index: 0x6064,
            sub_index: 0,
            type: int32,
            state_interface: position
          }  # Position actual value
        - {
            index: 0x606c,
            sub_index: 0,
            type: int32,
            state_interface: velocity
          }  # Velocity actual value
        - {
            index: 0x6077,
            sub_index: 0,
            type: int16,
            state_interface: effort
          }  # Torque actual value
        - {index: 0x6061, sub_index: 0, type: int8}  # Mode of operation display
        - {index: 0x2071, sub_index: 1, type: int16}  # Digital Input Functionalities State
        - {index: 0x60b9, sub_index: 0, type: int16}  # Touch Probe Status
        - {index: 0x60ba, sub_index: 0, type: int32}  # Touch Probe Position 1 Positive Value
        - {index: 0x60bb, sub_index: 0, type: int32}  # Touch Probe Position 1 Negative Value

This configuration can be used for controlling a :code:`joint` component. Here is an example urdf for :code:`ros2_control` using this configuration together with the :code:`EcCiA402Drive` plugin:

.. code-block:: xml

  <ros2_control name="ec_single_axis" type="system">
      <hardware>
        <plugin>ethercat_driver/EthercatDriver</plugin>
        <param name="master_id">0</param>
        <param name="control_frequency">100</param>
      </hardware>

     <joint name="joint_0">
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <command_interface name="position"/>
        <command_interface name="reset_fault"/>
        <ec_module name="MAXON">
          <plugin>ethercat_generic_plugins/EcCiA402Drive</plugin>
          <param name="alias">0</param>
          <param name="position">0</param>
          <param name="mode_of_operation">8</param>
          <param name="slave_config">/path/to/maxon.yaml</param>
        </ec_module>
      </joint>
    </ros2_control>
