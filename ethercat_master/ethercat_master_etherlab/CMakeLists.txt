cmake_minimum_required(VERSION 3.8)
project(ethercat_master_etherlab)

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
find_package(ethercat_interface REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(yaml_cpp_vendor REQUIRED)
# find_package(yaml-cpp REQUIRED)

# EtherLab
set(ETHERLAB_LOCAL_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_path(ETHERCAT_INCLUDE_DIR NAMES ecrt.h PATHS ${ETHERLAB_LOCAL_DIR}/include)
find_library(ETHERCAT_LIB NAMES ethercat PATHS ${ETHERLAB_LOCAL_DIR}/lib)

if(NOT ETHERCAT_INCLUDE_DIR OR NOT ETHERCAT_LIB)
  message(FATAL_ERROR
    "Could not find the IgH EtherCAT master (ecrt.h / libethercat). "
    "Looked in standard system paths and in ${ETHERLAB_LOCAL_DIR}.")
endif()


file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)

add_library(${PROJECT_NAME} ${PLUGINS_SRC})

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_export_include_directories(
  include
  ${ETHERCAT_INCLUDE_DIR}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ETHERCAT_INCLUDE_DIR}
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${ETHERCAT_INCLUDE_DIR}
)

ament_target_dependencies( ${PROJECT_NAME}
  ethercat_interface
  pluginlib
  rclcpp
#  yaml_cpp_vendor
#  yaml-cpp
)
target_link_libraries(${PROJECT_NAME}
#  yaml-cpp
  ${ETHERCAT_LIB}
)

pluginlib_export_plugin_description_file(ethercat_interface master_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(ethercat_generic_slave REQUIRED)
  find_package(ethercat_interface REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Test Load EtherCAT modules
  ament_add_gmock(
    test_load_${PROJECT_NAME}
    test/test_load_ec_master_etherlab.cpp
  )
  target_include_directories(test_load_${PROJECT_NAME} PRIVATE include ${ETHERCAT_INCLUDE_DIR})
  ament_target_dependencies(test_load_${PROJECT_NAME}
    pluginlib
    ethercat_interface
    rclcpp
  )
  # Test Etherlab EtherCAT Slave
  ament_add_gmock(
    test_etherlab_slave
    test/test_etherlab_slave.cpp
  )
  target_include_directories(test_etherlab_slave PRIVATE
    include
    ${ETHERCAT_INCLUDE_DIR}
  )
  target_link_libraries(test_etherlab_slave
    ${PROJECT_NAME}
    yaml-cpp
  )
  ament_target_dependencies(test_etherlab_slave
    pluginlib
    ethercat_interface
    ethercat_generic_slave
    yaml_cpp_vendor
  )
endif()


ament_export_include_directories(
  include
  ${ETHERCAT_INCLUDE_DIR}
)
ament_export_libraries(
  ${PROJECT_NAME}
  ${ETHERCAT_LIBRARY}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
