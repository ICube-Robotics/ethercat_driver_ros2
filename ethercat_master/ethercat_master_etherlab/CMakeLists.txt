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
#find_package(yaml_cpp_vendor REQUIRED)
#find_package(yaml-cpp REQUIRED)

# EtherLab
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)


file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)

add_library(${PROJECT_NAME} ${PLUGINS_SRC})

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#ament_export_include_directories(
#  include
#  ${ETHERLAB_DIR}/include
#)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ETHERCAT_DIR}/include
)

#target_include_directories(
#  ${PROJECT_NAME}
#  PRIVATE
#  include
#  ${ETHERLAB_DIR}/include
#)

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
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
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
