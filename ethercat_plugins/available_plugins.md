# ethercat_plugins
Collection of plugin implementations of EtherCAT modules for the `ethercat_driver`.

## Module list and configuration
The list of currently supported EtherCAT modules and the available parameters. All modules have the `alias` and `position` parameter to specify the bus topology.
### Beckhoff
- **Beckhoff_EK1100**: EtherCAT Coupler.
- **Beckhoff_EL1008**: EtherCAT Terminal, 8-channel digital input, 24 V DC, 3 ms.
    - parameters: `di.1..8` - requested digital input
- **Beckhoff_EL1018**: EtherCAT Terminal, 8-channel digital input, 24 V DC, 10 us.
    - parameters: `di.1..8` - requested digital input
- **Beckhoff_EL2008**: EtherCAT Terminal, 8-channel digital output, 24 V DC, 0.5 A.
    - parameters: `do.1..8` - requested digital output
- **Beckhoff_EL2088**: EtherCAT Terminal, 8-channel digital output, 24 V DC, 0.5 A, ground switching.
    - parameters: `do.1..8` - requested digital output
- **Beckhoff_EL2124**: EtherCAT Terminal, 4-channel digital output, 5 V DC, 20 mA.
    - parameters: `do.1..4` - requested digital output
- **Beckhoff_EL3102**: EtherCAT Terminal, 2-channel analog input, voltage, ±10 V, 16 bit, differential.
    - parameters: `ai.1..2` - requested analog input
- **Beckhoff_EL3104**: EtherCAT Terminal, 4-channel analog input, voltage, ±10 V, 16 bit, differential.
    - parameters: `ai.1..4` - requested analog input
- **Beckhoff_EL4132**: EtherCAT Terminal, 2-channel analog output, voltage, ±10 V, 16 bit.
    - parameters: `ao.1..2` - requested analog output
- **Beckhoff_EL4134**: EtherCAT Terminal, 4-channel analog output, voltage, ±10 V, 16 bit.
    - parameters: `ao.1..4` - requested analog output
- **Beckhoff_EL5101**: EtherCAT Terminal, 1-channel encoder interface, incremental, 5 V DC (DIFF RS422, TTL), 1 MHz.
    - parameters:
        1. `encoder_position` - current encoder position
        2. `convertion_factor` - encoder tic to unit convertion factor
        3. `encoder_reset` - reset encoder counting (reset on 0->1 switch)

## Maxon
- **EPOS3**: EPOS3 70/10 EtherCAT, digital positioning controller, 10 A, 11 - 70 VDC
    - parameters:
        1. `mode_of_operation`: see [EPOS3 documentation](https://maxonjapan.com/wp-content/uploads/manual/epos/EPOS3_EtherCAT_Firmware_Specification_En.pdf) -> Table 8-116
        2. `motor_position`: current motor position
        3. `motor_velocity`: current motor velocity
        4. `motor_torque`: current motor torque
        5. `target_position`: target motor position
        6. `target_velocity`: target motor velocity
        7. `target_torque`: target motor torque

## ATI
- **ATI_FTSensor**: ATI EtherCAT F/T Sensor
    - parameters: 
        1. `force.[x|y|z].state_interface`: `state_interface` name for force on [x|y|z]-axis
        2. `force.[x|y|z].offset`: data offset for force on [x|y|z]-axis
        3. `torque.[x|y|z].state_interface`: `state_interface` name for torque on [x|y|z]-axis
        4. `torque.[x|y|z].offset`: data offset for torque on [x|y|z]-axis
### Advantech
- **AMAX-5074**: EtherCAT Coupler with ID switch
- **AMAX-5051**: Digital Input Module, 8-channel digital input, 24 V DC, 4 ms.
    - parameters: `di.1..8` - requested digital input
- **AMAX-5056**: Sink-type Digital Ouput Module, 8-channel digital output, 24 V DC, 0.3 A.
    - parameters: `do.1..8` - requested digital output
