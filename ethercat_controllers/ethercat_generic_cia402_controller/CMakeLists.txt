cmake_minimum_required(VERSION 3.8)
project(ethercat_generic_cia402_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  pluginlib
  rclcpp
  realtime_tools
  ethercat_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(ethercat_generic_cia402_controller SHARED
  src/generic_cia402_controller.cpp
)
target_compile_features(ethercat_generic_cia402_controller PUBLIC c_std_99 cxx_std_17)
target_include_directories(ethercat_generic_cia402_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies( ethercat_generic_cia402_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ethercat_generic_cia402_controller
  PRIVATE
  "ETHERCAT_GENERIC_CIA402_CONTROLLER_BUILDING_DLL"
)
pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ethercat_generic_cia402_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(controller_interface REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Test Load controller
  ament_add_gmock(
    test_load_cia402_controller
    test/test_load_cia402_controller.cpp
  )

  target_include_directories(test_load_cia402_controller PRIVATE include)

  ament_target_dependencies(test_load_cia402_controller
    pluginlib
    controller_manager
    controller_interface
    hardware_interface
    ros2_control_test_assets
  )

  # ament_add_gmock(
  #   test_cia402_controller
  #   test/test_cia402_controller.cpp
  # )

  # target_include_directories(
  #   test_cia402_controller
  #   PRIVATE
  #   include
  # )

  # target_link_libraries(
  #   test_cia402_controller
  #   ethercat_generic_cia402_controller
  # )

  # ament_target_dependencies(
  #   test_cia402_controller
  #   controller_interface
  #   hardware_interface
  #   rclcpp
  #   rclcpp_lifecycle
  #   realtime_tools
  #   ethercat_msgs
  # )
endif()

ament_export_include_directories(include)
ament_export_libraries(ethercat_generic_cia402_controller)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
