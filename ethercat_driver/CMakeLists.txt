cmake_minimum_required(VERSION 3.8)
project(ethercat_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)

add_library(${PROJECT_NAME} ${PLUGINS_SRC})

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ethercat_interface
  yaml_cpp_vendor
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ETHERCAT_DRIVER_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface ethercat_driver_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ethercat_interface REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Ensure test data is copied to the build directory
  set(CFG_FILES
    safety.ros2_control.xacro
    safety_estop.ros2_control.template.xacro
    estop_ethercat_safety.yaml
    test_config_ethercat_safety.yaml
    beckhoff_ek1914.yaml
    beckhoff_el1918.yaml
  )

  set(CFG_IN ${CMAKE_CURRENT_SOURCE_DIR}/examples/configurations)
  set(CFG_DST ${CMAKE_CURRENT_BINARY_DIR}/test_configurations)
  list(TRANSFORM CFG_FILES PREPEND "${CFG_IN}" OUTPUT_VARIABLE CFG_SRC)

  # Ensure the destination directory exists
  file(MAKE_DIRECTORY ${CFG_DST})

  # List to keep track of the output files
  set(CFG_OUT)

  foreach(file ${CFG_FILES})
    set(src_file "${CMAKE_CURRENT_SOURCE_DIR}/examples/configurations/${file}")
    set(dst_file "${CFG_DST}/${file}")

    add_custom_command(
        OUTPUT ${dst_file}
        COMMAND ${CMAKE_COMMAND} -E copy ${src_file} ${dst_file}
        DEPENDS ${src_file}
        COMMENT "Copying ${src_file} to ${dst_file}"
    )

    list(APPEND CFG_OUT ${dst_file})
  endforeach()
  # message(WARNING "Copying ${CFG_SRC} to ${CFG_DST}")

  # Create a custom target to group the copy commands
  add_custom_target(copy_safety_config ALL DEPENDS ${CFG_OUT})

  set(UP_PY ${CMAKE_CURRENT_SOURCE_DIR}/examples/configurations/update_slave_config_path.py)
  set(UP_IN ${CFG_DST}/safety_estop.ros2_control.template.xacro)
  set(UP_OUT ${CFG_DST}/safety_estop.ros2_control.xacro)
  set(UP_DEPS ${UP_PY} ${UP_IN})

  # message(WARNING "Running ${UP_PY} ${UP_IN} -p ${CFG_DST} -o ${UP_OUT}")
  add_custom_command(
    OUTPUT ${UP_OUT}
    COMMAND ${CMAKE_COMMAND} -E echo "Running update_slave_config_path.py"
    COMMAND ${CMAKE_COMMAND} -E env python3 ${UP_PY} ${UP_IN} -p ${CFG_DST} -o ${UP_OUT}
    DEPENDS ${UP_DEPS}
    COMMENT "Running update_slave_config_path.py"
  )

  add_custom_target(update_slave_config_path ALL DEPENDS ${UP_OUT})

  ament_add_gmock(test_ethercat_safety_driver test/test_ethercat_safety_driver.cpp)
  target_include_directories(test_ethercat_safety_driver PRIVATE include)

  target_compile_definitions(test_ethercat_safety_driver PRIVATE "TEST_RESOURCES_DIRECTORY=\"${CFG_DST}\"")

  target_link_libraries(test_ethercat_safety_driver
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_ethercat_safety_driver
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    ethercat_interface
    yaml_cpp_vendor
  )
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
  yaml_cpp_vendor
)
ament_package()
