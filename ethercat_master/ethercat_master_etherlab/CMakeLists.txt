cmake_minimum_required(VERSION 3.8)
project(ethercat_master_etherlab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# EtherLab
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${PLUGINS_SRC})
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ETHERLAB_DIR}/include
)
target_link_libraries(${PROJECT_NAME} ${ETHERCAT_LIB})

ament_target_dependencies(
  ${PROJECT_NAME}
  ethercat_interface
  pluginlib
  rclcpp
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
  find_package(ethercat_interface REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Test Load EtherCAT modules
  ament_add_gmock(
    test_load_${PROJECT_NAME}
    test/test_load_ec_master_etherlab.cpp
  )
  target_include_directories(test_load_${PROJECT_NAME} PRIVATE include ${ETHERLAB_DIR}/include)
  ament_target_dependencies(test_load_${PROJECT_NAME}
    pluginlib
    ethercat_interface
    rclcpp
  )
endif()

ament_export_include_directories(
  include
  ${ETHERLAB_DIR}/include
)
ament_export_libraries(
  ${PROJECT_NAME}
  ${ETHERCAT_LIBRARY}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
