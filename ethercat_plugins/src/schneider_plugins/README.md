# URDF, exmaple
```xml
<?xml version="1.0" ?>

<robot name="velocity" xmlns:xacro="http://ros.org/wiki/xacro">

  <ros2_control name="mySystem" type="system">
    <hardware>
        <plugin>ethercat_driver/EthercatDriver</plugin>
        <param name="master_id">0</param>
        <param name="control_frequency">100</param>
    </hardware>
    <joint name="atv_320_test">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <ec_module name="ATV320">
          <plugin>ethercat_plugins/Schneider_ATV320</plugin>
          <param name="alias">0</param>
          <param name="position">0</param>
      </ec_module>
    </joint>
  </ros2_control>
</robot>
```

# Controller example
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    forward_velocity_controller:
            type: velocity_controllers/JointGroupVelocityController

forward_velocity_controller:
  ros__parameters:
    joints:
      - atv_320_test
```

# Launch file example
```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    velocity_controller_path = 'controller.yaml'
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            'velocity_ros2.urdf'
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, velocity_controller_path],
        output='screen',
    )

    velocity_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_velocity_controller'],
        output='screen',
    )
  
    # Create the launch description and populate
    ld = LaunchDescription()

    # declared_arguments

    # nodes_to_start
    ld.add_action(ros2_control_node)
    ld.add_action(velocity_controller_node)

    return ld
```

# Setting velocity from command line
```bash
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [ 1500.0 ]"
```