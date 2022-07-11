# ethercat_driver_ros2
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/ICube-Robotics/ethercat_driver_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/ICube-Robotics/ethercat_driver_ros2/actions/workflows/ci.yml)

Implementation of a `Hardware Interface` for simple Ethercat module integration with [`ros2_control`](https://github.com/ros-controls/ros2_control) and building upon [IgH EtherCAT Master for Linux](https://etherlab.org/en/ethercat/).

## About
[EtherCAT](https://www.ethercat.org/default.htm) provides applications with the capacity of reliable, real-time communication between systems and is therefore a common industrial standard. In order to simplify the development/deployment of new application using EtherCAT modules, the `ethercat_driver` allows to combine them with [`ros2_control`](https://github.com/ros-controls/ros2_control). This driver proposes a generic ways to parametrise and assemble `Hardware Interfaces` based on EtherCAT modules that can be defined using paramater files.

## Usage
### 1. Installation
Installation steps for [IgH EtherCAT Master for Linux](https://etherlab.org/en/ethercat/) and the driver can be found [here](INSTALL.md).

### 2. Description
#### Hardwre Interfaces and EtherCAT Master
The `ethercat_driver` is designed in such a way that each EtherCAT Master is defined in `ros2_control` as a specific `Hardware Interface`. This is done in the `ros2_control` description file, where the EtherCAT driver is loaded as `Hardware Interface` and linked to an EtherCAT Master by its ID:
```xml
<ros2_control name="mySystem" type="system">
    <hardware>
        <plugin>ethercat_driver/EthercatDriver</plugin>
        <param name="master_id">0</param>
        <param name="control_frequency">100</param>
    </hardware>
</ros2_control>
```
**NOTE**: As in the current implementation of `ros2_control` there is no information about the system update frequency, it needs to be passed here as parameter. This is only needed systems that include EtherCAT modules that use the Distributed Clock.

#### EtherCAT Slave modules as Plugins
In this driver, the EtherCAT Slave modules are defined as [Plugins](https://docs.ros.org/en/foxy/Tutorials/Pluginlib.html) and can be parametrized in the `ros2_control` description file :
```xml
<ec_module name="ECModule">
    <plugin>ethercat_plugins/ECModule</plugin>
    <param name="alias">0</param>
    <param name="position">1</param>
</ec_module>
```
All modules have `alias` and `position` parameters that specify their address in the EtherCAT Bus topology. Additional parameters can be specified depending on the purpose of the module. A list of implemented modules and their parameters can be found [here](ethercat_plugins/available_plugins.md).

#### Interfacing controllers with EtherCAT Slave modules
In `ros2_control` the access to ressources within a system from a controller is done by means of [`Hardware Resources`](https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md). For this purpose `state_interface` and `command_interface` tags need to be defined and associated with the module functionalities.
Also, for better understanding of the overall system, the pupropose of the used modules need to be clearly identified and sorted into the following types:
- `<joint>`: logical component actuated by at least one actuator with read-write capacity.
- `<sensor>`: logical component to read-only states from system.
- `<gpio>`: logical component for general purpose IO systems.

**NOTE**: These components have the possibility to include parameters that will be used to link paricular states and commands to the slave module input/outputs.

Here is an example of a `gpio` ressource built using 2 EtherCAT IO modules, where digital commands are mapped on ports 4 and 6 of the digital output module and analog states are read from ports 1 and 4 of the analog input module:
```xml
<gpio name="myGPIO">
    <command_interface name="dig_output.1"/>
    <command_interface name="dig_output.2"/>
    <state_interface name="dig_output.1"/>
    <state_interface name="dig_output.2"/>
    <state_interface name="ana_input.1"/>
    <state_interface name="ana_input.2"/>
    <param name="ec_component">
        [ec_component]
            [ec_module name="EL3104"]
                [plugin]ethercat_plugins/Beckhoff_EL3104[/plugin]
                [param name="alias"]0[/param]
                [param name="position"]1[/param]
                [param name="ai.1"]ana_input.1[/param]
                [param name="ai.4"]ana_input.2[/param]
            [/ec_module]
            [ec_module name="EL2008"]
                [plugin]ethercat_plugins/Beckhoff_EL2088[/plugin]
                [param name="alias"]0[/param]
                [param name="position"]2[/param]
                [param name="do.4"]dig_output.2[/param]
                [param name="do.6"]dig_output.1[/param]
            [/ec_module]
        [/ec_component]
    </param>
</gpio>
```
**NOTE** : At the time of writing of the driver, the `xml` description of the robot was not available in the `Hardware Interface` thus the passing of a xml-like parameter string. This will replaced in the futur with standard `xml` description.

**NOTE** : To send commands to `gpio` ressources, a generic controller was developed and can be found [here](https://github.com/mcbed/ros2_controllers/tree/gpio_controllers).

## Acknowledgments
Parts of the driver are based on the implementation of [`SimplECAT`](https://bitbucket.org/bsoe/simplecat/src/master/).

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://plateforme.icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](mailto:macbednarczyk@gmail.com)
