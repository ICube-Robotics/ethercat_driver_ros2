cmake_minimum_required(VERSION 3.8)
project(ethercat_interface)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)

# EtherLab
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/ec_master.cpp
  src/ec_pdo_channel_manager.cpp
  src/ec_pdo_single_interface_channel_manager.cpp
  src/ec_pdo_group_interface_channel_manager.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${ETHERLAB_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  ${ETHERCAT_LIB}
  rclcpp::rclcpp
)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Test PdoChannelManager
  ament_add_gmock(
    test_ec_pdo_channel_manager
    test/test_ec_pdo_channel_manager.cpp
  )
  target_include_directories(test_ec_pdo_channel_manager PRIVATE include ${ETHERLAB_DIR}/include)

  target_link_libraries(test_ec_pdo_channel_manager
    ${PROJECT_NAME}
    yaml-cpp)
endif()

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
)
ament_package()
