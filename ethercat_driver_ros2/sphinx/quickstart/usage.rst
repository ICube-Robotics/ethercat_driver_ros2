Usage
=====

Start EtherCAT Master
---------------------

First start the EtherCAT Master

.. code-block:: console

  $ sudo /etc/init.d/ethercat start

it should print

.. code-block:: console

  Starting EtherCAT master 1.5.2  done

You can check connected slaves:

.. code-block:: console

  $ ethercat slaves

It should print information of connected slave device:

.. code-block:: console

  <id>  <alias>:<position>  <device_state>  +  <device_name>

.. note:: If nothing is displayed or some slave modules are missing, it means that your EtherCAT Master is either not well configured, or that the connection on your field-bus is interrupted.

Launch configuration
--------------------

Once the EtherCAT Master is running you can go on and launch your configuration package.

.. code-block:: console

  $ ros2 launch [package] [launchfile]
