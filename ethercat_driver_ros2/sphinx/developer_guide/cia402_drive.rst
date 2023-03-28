CANopen over EtherCAT for electrical drives
===========================================

CANopen also defines several device profiles by establishing standard behavior and communication objects for similar devices. In the case of electrical drives, the profile is given by the **CiA402** ("CANopen device profile for drives and motion control") norm compliant with the `IEC <https://webstore.iec.ch/publication/23757>`_ standard that defines programming languages for programmable control systems. The CiA402 norm specifies specific objects in OD in range :code:`0x6000` to :code:`0x67fe`, what guarantees, that for a drive compatible with CiA402, typical process values such as position, velocity, or torque set-points and actual values will always be defined in the same objects with the same address regardless of the manufacturer and by such simplifies the commissioning of different drives. CIA402 also defines a state machine that corresponds to the drive’s actual operating state (:code:`Disabled`, :code:`Fault`, :code:`Switch ON`, etc.). This state machine is controlled and monitored by two mandatory PDO entries: :code:`Control Word` and :code:`Status Word`. The drive cannot be started until the state machine has been put into the appropriate state.

The device states and possible control sequences of the drive are described by the CANopen state machine as follows:

.. image:: https://webhelp.kollmorgen.com/kas/Content/Resources/Images/CoE%20status%20machine.png
  :width: 400
  :alt: ecm_lifecycle
  :align: center

The possible drive states are :

.. list-table:: CiA402 Drive States
  :widths: 15 35
  :header-rows: 1

  * - State
    - Description
  * - :code:`NOT_READY_TO_SWITCH_ON`
    - The drive is not ready to switch on. The drive is in boot phase. Drive motion functions are disabled. No communication established.
  * - :code:`SWITCH_ON_DISABLED`
    - The drive cannot be enabled via the EtherCAT interface. Drive motion functions are disabled.
  * - :code:`READY_TO_SWITCH_ON`
    - The drive can be enabled. Parameters can be transferred. Power may be applied. Drive motion functions are disabled.
  * - :code:`SWITCHED_ON`
    - The drive is enabled but idle No fault detected. Power is enabled Drive motion functions are disabled. Parameters can be transferred. No set-points are transferred from the EtherCAT interface.
  * - :code:`OPERATION_ENABLED`
    - Normal operation mode No fault detected. Drive motion functions and power are enabled. Power enabled. Set-points are transferred from the EtherCAT interface.
  * - :code:`QUICK_STOP_ACTIVE`
    - The quick stop process is executed. Drive motion functions are disabled.  Power is enabled.
  * - :code:`FAULT_REACTION_ACTIVE`
    - A fault has occurred, fault reaction processes are executed.
  * - :code:`FAULT`
    - A fault is active, Drive is stopped and its functions are disabled.

State transitions are the result of events that can be either internal or triggers through the :code:`Control Word`. When a command trigger for state change is received, the transition is performed before any other command is processed.

The possible transition states, their trigger events and performed actions are :

.. list-table:: CiA402 Drive State Transitions
  :widths: 1 20 24
  :header-rows: 1

  * - State Transition
    - Event
    - Action
  * - :code:`0`
    - **Automatic transition** after power-on or reset application
    - Drive device self-test and/or self initialization has to be performed
  * - :code:`1`
    - **Automatic transition**
    - Communication has to be activated
  * - :code:`2`
    - Shutdown command from control device or local signal
    - None
  * - :code:`3`
    - Switch on command received from control device or local signal
    - The high-level power has to be switched ON, if possible
  * - :code:`4`
    - Enable operation command received from control device or local signal
    - The drive function has to be enabled. All internal set-points cleared.
  * - :code:`5`
    - Disable operation command received from control device or local signal
    - The drive function has to be disabled.
  * - :code:`6`
    - Shutdown command received from control device or local signal
    - The high-level power has to be switched OFF, if possible.
  * - :code:`7`
    - Quick stop or disable voltage command from control device or local signal
    - None
  * - :code:`8`
    - Shutdown command from control device or local signal
    - The drive function has to be disabled. The high-level power has to be switched OFF, if possible.
  * - :code:`9`
    - Disable voltage command from control device or local signal
    - The drive function has to be disabled. The high-level power has to be switched OFF, if possible.
  * - :code:`10`
    - Disable voltage or quick stop command from control device or local signal
    - The high-level power has to be switched OFF, if possible.
  * - :code:`11`
    - Quick stop command from control device or local signal
    - The quick stop function has to be started.
  * - :code:`12`
    - Either: - **Automatic transition** when the quick stop function is completed and quick stop option code is 1, 2, 3 or 4. - Disable voltage command received from control device (depends on the quick stop option code)
    - The drive function has to be disabled. The high-level power has to be switched OFF, if possible.
  * - :code:`13`
    - Fault signal
    - The configured fault reaction function has to be executed.
  * - :code:`14`
    - **Automatic transition**
    - The drive function has to be disabled. The high-level power has to be switched OFF, if possible.
  * - :code:`15`
    - Fault reset command from control device or local signal.
    - A reset of the fault condition is performed if no fault exists currently on the drive device. After leaving the Fault state, the Fault reset bit in the control word has to be cleared by the control device.
  * - :code:`16`
    - If the quick stop option code is 5, 6, 7, or 8, enable operation command from control device
    - The drive function has to be enabled

CANopen uses 16-bits :code:`Status Word` and :code:`Control Word` to monitor and control the state machine. These PDO entries have the following bit assignment :

.. list-table:: CiA402 Drive Words
  :widths: 1 20 24
  :header-rows: 1

  * - Bit
    - :code:`Status Word` Name
    - :code:`Control Word` Name
  * - :code:`0`
    - Ready to switch on
    - Switch On
  * - :code:`1`
    - Switched on
    - Disable Voltage
  * - :code:`2`
    - Operation Enabled
    - Quick Stop
  * - :code:`3`
    - Fault
    - Enable Operation
  * - :code:`4`
    - Voltage Enabled
    - Operation-mode Specific
  * - :code:`5`
    - Quick Stop
    - Operation-mode Specific
  * - :code:`6`
    - Switch On Disabled
    - Operation-mode Specific
  * - :code:`7`
    - Warning
    - Reset Fault (only effective for faults)
  * - :code:`8`
    - Manufacturer-specific (reserved)
    - Pause/halt
  * - :code:`9`
    - Remote (always 1)
    - Reserved
  * - :code:`10`
    - Target Reached
    - Reserved
  * - :code:`11`
    - Internal Limit Active
    - Reserved
  * - :code:`12`
    - Operation-mode Specific (reserved)
    - Reserved
  * - :code:`13`
    - Operation-mode Specific (reserved)
    - Manufacturer-specific
  * - :code:`14`
    - Manufacturer-specific (reserved)
    - Manufacturer-specific
  * - :code:`15`
    - Manufacturer-specific (reserved)
    - Manufacturer-specific

.. image:: https://infosys.beckhoff.com/content/1033/ax2000-b110/Images/StateMachine04.gif
  :width: 700
  :alt: words

The :code:`Status Word` is only updated and written by the drive in :code:`Safe-Op` and :code:`Operational` states. The current drive state can be decoded from the logical combination of the bits in the :code:`Status Word`:

.. list-table:: CiA402 Drive State form :code:`Status Word`
  :widths: 50 50
  :header-rows: 1

  * - :code:`Status Word`
    - State
  * - :code:`xxxx xxxx x0xx 0000`
    - Not ready to switch on
  * - :code:`xxxx xxxx x1xx 0000`
    - Switch on disabled
  * - :code:`xxxx xxxx x01x 0001`
    - Ready to switch on
  * - :code:`xxxx xxxx x01x 0011`
    - Switched on
  * - :code:`xxxx xxxx x01x 0111`
    - Operation enabled
  * - :code:`xxxx xxxx x00x 0111`
    - Quick stop active
  * - :code:`xxxx xxxx x0xx 1111`
    - Fault reaction active
  * - :code:`xxxx xxxx x0xx 1000`
    - Fault

The control commands allow the manipulation of the state of a drive by setting its control word. Commands are built up from the logical combination of the bits in the :code:`Control Word`:

.. list-table:: CiA402 Drive Commands to :code:`Control Word`
  :widths: 25 25 25
  :header-rows: 1

  * - :code:`Control Word`
    - Command
    - State Transitions
  * - :code:`xxxx 0xxx x110`
    - Shutdown
    - 2, 6, 8
  * - :code:`xxxx 0xxx 0111`
    - Switch On
    - 3
  * - :code:`xxxx 0xxx 1111`
    - Switch On + Enable Operation
    - 3 + 4
  * - :code:`xxxx 0xxx xx0x`
    - Disable
    - 7, 9, 10, 12
  * - :code:`xxxx 0xxx x01x`
    - Quick Stop
    - 7, 10, 11
  * - :code:`xxxx 0xxx 0111`
    - Disable Operation
    - 5
  * - :code:`xxxx 0xxx 1111`
    - Enable Operation
    - 4, 16
  * - :code:`xxxx 1xxx xxxx`
    - Fault Reset
    - 15
