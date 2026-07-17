cmake_minimum_required(VERSION 3.5)
project(ethercat_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ethercat_msgs REQUIRED)

# EtherLab
set(ETHERLAB_LOCAL_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)
find_path(ETHERCAT_INCLUDE_DIR NAMES ecrt.h PATHS ${ETHERLAB_LOCAL_DIR}/include)
find_library(ETHERCAT_LIB NAMES ethercat PATHS ${ETHERLAB_LOCAL_DIR}/lib)

if(NOT ETHERCAT_INCLUDE_DIR OR NOT ETHERCAT_LIB)
  message(FATAL_ERROR
    "Could not find the IgH EtherCAT master (ecrt.h / libethercat). "
    "Looked in standard system paths and in ${ETHERLAB_LOCAL_DIR}.")
endif()

ament_export_include_directories(
  include
  ${ETHERCAT_INCLUDE_DIR}
)

add_executable(
  ethercat_sdo_srv_server
  src/ethercat_sdo_srv_server.cpp)
target_include_directories(
  ethercat_sdo_srv_server
  PRIVATE
  include
  ${ETHERCAT_INCLUDE_DIR}
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
