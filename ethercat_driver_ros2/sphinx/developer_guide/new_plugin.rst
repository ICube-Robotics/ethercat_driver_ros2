Creating a new device driver
============================

Creating a new device driver to work with the :code:`ethercat_driver_ros2` is fairly easy. You should do this if you need to create a driver for a specific device or a specific device profile that is not yet supported. If you create a driver for a device profile we are happy to integrate the package into this repository - simply create a PR.

What you need to do
-------------------

To create a dedicated driver for an EtherCAT module, you need to create a new package containing the module driver plugin and register it so that it can be called by the :code:`ethercat_driver_ros2`.

How to do it
------------

Create a package
~~~~~~~~~~~~~~~~

Create a new package using the standard ros2 pkg commands. Make sure you add the following dependencies:

* :code:`ethercat_interface`
* :code:`pluginlib`

Create your plugin class
~~~~~~~~~~~~~~~~~~~~~~~~

To be loaded by the :code:`ethercat_driver_ros2`, the new module plugin needs to inherit from the :code:`EcSlave` class and implement some of its virtual functions. To do so create in your package :code:`src` folder a new file :code:`my_ec_device_driver.cpp`:

.. code-block:: cpp

  #include "ethercat_interface/ec_slave.hpp"

  namespace ethercat_plugins
  {

  class MyEcDeviceDriver : public ethercat_interface::EcSlave
  {
  public:
    MyEcDeviceDriver()
    : EcSlave(<vendor_id>, <product_id>) {}
    virtual ~MyEcDeviceDriver() {}
    // data processing method that will be called cyclically for every channel
    // the index corresponds to the values registered in domains_
    virtual void processData(size_t index, uint8_t * domain_address)
    {
      // Your process data logic goes here
    }
    virtual const ec_sync_info_t * syncs() {return &syncs_[0];}
    virtual size_t syncSize()
    {
      return sizeof(syncs_) / sizeof(ec_sync_info_t);
    }
    virtual const ec_pdo_entry_info_t * channels()
    {
      return channels_;
    }
    virtual void domains(DomainMap & domains) const
    {
      domains = domains_;
    }
    // configure the slave module with urdf parameters
    // and link ros2_control command and stat interface
    virtual bool setupSlave(
      std::unordered_map<std::string, std::string> slave_parameters,
      std::vector<double> * state_interface,
      std::vector<double> * command_interface)
    {
      state_interface_ptr_ = state_interface;
      command_interface_ptr_ = command_interface;
      parameters_ = slave_parameters;

      // Your module setup logic goes here

      return true;
    }

  private:
    // configure module channels
    ec_pdo_entry_info_t channels_[X] = {
      {<index>, <sub_index>, <type>},
    };
    // configure module pdos
    ec_pdo_info_t pdos_[X] = {
      {<index>, <nbr_channels>, <channel_ptr>},
    };
    // configure module syncs
    ec_sync_info_t syncs_[X] = {
      {<index>, <type>, <nbr_pdos>, <pdos_ptr>, <watchdog>},
      {0xff}
    };
    // configure module domain
    DomainMap domains_ = {
      {0, {0}} // index of channels that should call processData()
    };
  };

  }  // namespace ethercat_plugins

  #include <pluginlib/class_list_macros.hpp>

  PLUGINLIB_EXPORT_CLASS(ethercat_plugins::MyEcDeviceDriver, ethercat_interface::EcSlave)

Export your plugin
~~~~~~~~~~~~~~~~~~

In the package root directory create a plugin description file :code:`ethercat_plugins.xml` :

.. code-block:: xml

  <library path="ethercat_plugins">
    <class name="ethercat_plugins/MyEcDeviceDriver"
           type="ethercat_plugins::MyEcDeviceDriver"
           base_class_type="ethercat_interface::EcSlave">
      <description>Description of the device driver.</description>
    </class>
  </library>

Modify your :code:`CMakeLists.txt` file so that it looks like this:

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.8)
  project(<your_package>)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ethercat_interface REQUIRED)
  find_package(pluginlib REQUIRED)

  file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)
  add_library(${PROJECT_NAME} ${PLUGINS_SRC})
  target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(
    ${PROJECT_NAME}
    "ethercat_interface"
    "pluginlib"
  )
  pluginlib_export_plugin_description_file(ethercat_interface ethercat_plugins.xml)
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
