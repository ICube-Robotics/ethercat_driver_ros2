cmake_minimum_required(VERSION 3.8)
project(ethercat_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cia402DriveStates.msg"
  "srv/SetSdo.srv"
  "srv/GetSdo.srv"
  "srv/SwitchDriveModeOfOperation.srv"
  "srv/ResetDriveFault.srv"
  DEPENDENCIES
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
