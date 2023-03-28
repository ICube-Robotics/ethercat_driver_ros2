CANopen over EtherCAT
=====================

The CANopen over EtherCAT (CoE) protocol allows devices equipped with CANopen to be used on EtherCAT-based Industrial Ethernet networks.

CANopen
-------

CANopen is a communication protocol based on the `CAN (Controller Area Network) <https://www.ni.com/en-us/innovations/white-papers/06/controller-area-network--can--overview.html>`_ physical communication standard. In the `OSI communication model <https://www.motioncontroltips.com/what-is-industrial-ethernet-and-how-does-it-differ-from-standard-ethernet/>`_, CAN specifies the physical and data link layers, while CANopen addresses the higher layers — network, transport, session, presentation, and application.

The CANopen protocol defines how automation devices are configured and accessed, and how messages are exchanged between them. CANopen is object-based, meaning that each node (drives, controllers, encoders, I/O, and other devices) in the network has an Object Dictionary (OD), which contains communication objects. These communication objects cover network management data; special functions; acyclic configuration data, which is handled by Service Data Objects (SDOs); cyclic real-time data, which is handled by Process Data Objects (PDOs):

- **Process Data Objects (PDO)** contain OD entries that are cyclically transferred as process variables. Before cyclic communication is started during the configuration phase, particular OD objects are mapped to this structure. Each PDO entry has a defined offset in the exchanged dataset, encapsulated in the Ethernet frame, so that, during the cyclic phase, the slaves’ hardware can locate the relevant data. After cyclic communication is started, PDO entries are exchanged between master and slaves in every cycle and cannot be changed without reconfiguring the communication configuration of the network.
-  **Service Data Objects (SDO)** contain object dictionary entries that can be exchanged acyclically. SDO works as a mailbox sending and buffering received data. This communication is acyclic and is dependent on available bandwidth in the communication cycle. This communication is not deterministic and is best suited for transmitting configuration data.

The use of object dictionaries, Service Data Objects, and Process Data Objects is a key component of the CANopen protocol, with SDOs being the mechanism for read-write access to the object dictionary.

Every entry of the OD objects is given an index address and sometimes a sub-index sub-address and each OD object consists of 16-bits and data indexes. In this context, addresses between :code:`0x1000` and :code:`0x1fff` contain communication objects, between :code:`0x2000` and :code:`0x5999` manufacturer specific objects and from :code:`0x6000` device profile objects.

CANopen is widely used thanks to its low hardware cost, wide range of device and application profiles, and simple implementation. It’s also extremely reliable and provides real-time communication, making it suitable for industrial applications.

EtherCAT
--------

EtherCAT is an Industrial Ethernet network. It’s based on standard Ethernet hardware but uses a “processing-on-the-fly” method for transporting and routing messages. In addition to being a real-time networking protocol, EtherCAT is deterministic, meaning it guarantees that a message will be transmitted (or an event will occur) in a specified, predictable period of time — not slower or faster. EtherCAT allows up to 100 meters between nodes (devices) and can provide data transmission rates up to 100 Mbps, with cycle times of less than 100 μs and extremely low jitter, thanks to distributed synchronized clocks.

CANopen over EtherCAT (CoE) protocol
------------------------------------

CANopen over EtherCAT (CoE) allows the CANopen communication protocol to be implemented over an EtherCAT network, providing a user-friendly, cost-effective solution that provides deterministic data delivery along with faster transmission speeds over longer network lengths.

The use of CANopen over EtherCAT is possible in significant part because EtherCAT implements the same communication system, including object dictionaries, SDOs (the SDO protocol is implemented directly from CANopen, without changes) and PDOs. And on an EtherCAT network, PDO frames are sent deterministically and without the 8-byte limit imposed by CANopen. CANopen over EtherCAT also supports the CANopen device profiles, which specify the parameters and behavior of the device, as well as the device class-specific state machines.

.. image:: https://b2600047.smushcdn.com/2600047/wp-content/uploads/2021/12/Applied-Motion-CANopen-over-EtherCAT.png
  :width: 400
  :alt: coe
  :align: center
