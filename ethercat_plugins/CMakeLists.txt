cmake_minimum_required(VERSION 3.8)
project(ethercat_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(pluginlib REQUIRED)

file(GLOB_RECURSE PLUGINS_SRC src/*/*.cpp)
add_library(ethercat_plugins ${PLUGINS_SRC})
target_compile_features(ethercat_plugins PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ethercat_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ethercat_plugins
  "ethercat_interface"
  "pluginlib"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(ethercat_plugins PRIVATE "ETHERCAT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(ethercat_interface ethercat_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ethercat_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(ethercat_interface REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Test ADVANTECH EtherCAT modules
  ament_add_gmock(
    test_load_advantech_plugins
    test/advantech_plugins/test_load_ec_modules.cpp
  )
  target_include_directories(test_load_advantech_plugins PRIVATE include)
  ament_target_dependencies(test_load_advantech_plugins
    pluginlib
    ethercat_interface
  )
  # Test ATI EtherCAT modules
  ament_add_gmock(
    test_load_ati_plugins
    test/ati_plugins/test_load_ec_modules.cpp
  )
  target_include_directories(test_load_ati_plugins PRIVATE include)
  ament_target_dependencies(test_load_ati_plugins
    pluginlib
    ethercat_interface
  )
  # Test BECKHOFF EtherCAT modules
  ament_add_gmock(
    test_load_beckhoff_plugins
    test/beckhoff_plugins/test_load_ec_modules.cpp
  )
  target_include_directories(test_load_beckhoff_plugins PRIVATE include)
  ament_target_dependencies(test_load_beckhoff_plugins
    pluginlib
    ethercat_interface
  )
  # Test MAXON EtherCAT modules
  ament_add_gmock(
    test_load_maxon_plugins
    test/maxon_plugins/test_load_ec_modules.cpp
  )
  target_include_directories(test_load_maxon_plugins PRIVATE include)
  ament_target_dependencies(test_load_maxon_plugins
    pluginlib
    ethercat_interface
  )
  # Test OMRON EtherCAT modules
  ament_add_gmock(
    test_load_omron_plugins
    test/omron_plugins/test_load_ec_modules.cpp
  )
  target_include_directories(test_load_omron_plugins PRIVATE include)
  ament_target_dependencies(test_load_omron_plugins
    pluginlib
    ethercat_interface
  )
  # Test SCHNEIDER EtherCAT modules
  ament_add_gmock(
    test_load_schneider_plugins
    test/schneider_plugins/test_load_ec_modules.cpp
  )
  target_include_directories(test_load_schneider_plugins PRIVATE include)
  ament_target_dependencies(test_load_schneider_plugins
    pluginlib
    ethercat_interface
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ethercat_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
