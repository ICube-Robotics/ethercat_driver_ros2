cmake_minimum_required(VERSION 3.8)
project(ethercat_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# EtherLab
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

ament_export_include_directories(
  include
  ${ETHERLAB_DIR}/include
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/ec_master.cpp)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${ETHERLAB_DIR}/include
)

target_link_libraries(${PROJECT_NAME} ${ETHERCAT_LIB})

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
)

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
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
  ${ETHERCAT_LIBRARY}
)
ament_export_dependencies(
  rclcpp
)
ament_package()
