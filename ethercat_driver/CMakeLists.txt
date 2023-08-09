cmake_minimum_required(VERSION 3.8)
project(ethercat_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ethercat_interface REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/ethercat_driver.cpp)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ethercat_interface
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ethercat_driver PRIVATE "ETHERCAT_DRIVER_BUILDING_LIBRARY")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface ethercat_driver_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  # find_package(ament_cmake_gmock REQUIRED)
  # find_package(ament_lint_auto REQUIRED)
  # find_package(pluginlib REQUIRED)
  # find_package(ethercat_interface REQUIRED)
  # find_package(ethercat_master_mock REQUIRED)
  # find_package(ros2_control_test_assets REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  # ament_add_gmock(test_ethercat_driver
  #   test/test_ethercat_driver.cpp)
  # target_include_directories(test_ethercat_driver PRIVATE include)
  # target_link_libraries(test_ethercat_driver ethercat_driver)
  # ament_target_dependencies(test_ethercat_driver
  #   pluginlib
  #   ros2_control_test_assets
  #   ethercat_interface
  #   ethercat_master_mock
  # )
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ethercat_interface
)
ament_package()
