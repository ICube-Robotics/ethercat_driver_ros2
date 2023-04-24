cmake_minimum_required(VERSION 3.5)
project(cia402_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(cia402_controller
  SHARED
  src/cia402_controller.cpp
)

ament_target_dependencies(cia402_controller
  builtin_interfaces
  controller_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  trajectory_msgs
  Eigen3
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(cia402_controller PRIVATE "MPI_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(cia402_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  cia402_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_cia402_controller
    test/test_load_cia402_controller.cpp
  )

  target_include_directories(
    test_load_cia402_controller
    PRIVATE
    include
  )

  ament_target_dependencies(
    test_load_cia402_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_cia402_controller
    test/test_cia402_controller.cpp
  )

  target_include_directories(
    test_cia402_controller
    PRIVATE
    include
  )

  target_link_libraries(
    test_cia402_controller
    cia402_controller
  )

  ament_target_dependencies(
    test_cia402_controller
    controller_interface
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
  )
endif()

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  cia402_controller
)
ament_package()
