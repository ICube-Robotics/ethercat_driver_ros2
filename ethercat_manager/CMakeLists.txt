cmake_minimum_required(VERSION 3.5)
project(ethercat_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ethercat_msgs REQUIRED)

# EtherLab
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

ament_export_include_directories(
  include
  ${ETHERLAB_DIR}/include
)

add_executable(
  ethercat_sdo_srv_server
  src/ethercat_sdo_srv_server.cpp)
target_include_directories(
  ethercat_sdo_srv_server
  PRIVATE
  include
  ${ETHERLAB_DIR}/include
)
target_link_libraries(ethercat_sdo_srv_server ${ETHERCAT_LIB})
ament_target_dependencies(ethercat_sdo_srv_server rclcpp ethercat_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ethercat_sdo_srv_server
  DESTINATION lib/${PROJECT_NAME})

  ## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ethercat_sdo_srv_server
  ${ETHERCAT_LIBRARY}
)
ament_export_dependencies(
  rclcpp
)
ament_package()
