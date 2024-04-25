Case study: motor coupled to a gearbox and a transmission screw with an encoder
===============================================================================

This case study is a simple example to show how to set up the config of the :code:`EcCiA402Drive` generic plugin to take into account transmission parameters and encoder parameters.

System configuration
--------------------

The system is composed of a motor coupled to a transmission line (a gearbox and a transmission screw) whose rotation is measured by an encoder.


Transmission configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~

The transmission is composed of a motor, a gearbox, and a transmission screw (lead screw with ball bearing for zero friction) with the following characteristics:

- gearbox with gear ratio, reduction absolute value: 57/13 (:math:`r \triangleq \frac{13}{57}`)
- gearbox maximum efficiency: 0.84 (:math:`\eta_g \triangleq 0.84`)
- lead screw with lead: 1 mm (:math:`l \triangleq 1 mm`)

Encoder configuration
~~~~~~~~~~~~~~~~~~~~~

The encoder has the following characteristics:

- Resolution (number of pulses/revolution):	500 CPR
- Encoder type: quadrature encoder
- Digital resolution (number of counts/revolution): :math:`\text{C} \triangleq 500\times 4 = 2000`

Convert state values
--------------------

The drive will transmit the following state values:

- :code:`position` in :math:`\text{counts}` or :math:`c`
- :code:`velocity` in :math:`\theta` (revolutions per minute)
- :code:`torque` (:math:`T_i`) in :math:`\text{mNm}` (torque of the motor at the input of the gearbox)

We would  like to have the following state values:

- :code:`position` (:math:`d`) in :math:`\text{m}`
- :code:`velocity` (:math:`v`) in :math:`\text{m/s}`
- :code:`force` (:math:`F_o`) in :math:`\text{mN}` (Force of the lead screw)

This is the configuration part of the tpdo of the :code:`EcCiA402Drive` plugin and corresponding to the :code:`state_interface` in ROS2 control terminology.


Linear displacement per count of the encoder
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The motor has a reductor so each revolution of the motor does not correspond to a revolution of the system.
It is necessary to first compute the rotation of the shaft at the output of the gearbox.

  * reduction absolute value is :math:`r \triangleq \frac{13}{57}`

So for 57 revolutions of the motor, the shaft will only turn 13 times. So the motor has to turn approximately 4.4 times to make the system turn once.

The lead of the screw is defined as the linear distance traveled per rotation of the screw and in this case is :math:`1mm`.

The encoder reports :math:`C \triangleq 2000` counts per revolution (CPR) so 2000 counts per revolution of the shaft, so an angular resolution of :math:`\frac{360}{2000} = 0.18^{\circ}`.

Let :math:`d` be the linear displacement in meters, :math:`r` the reduction absolute value, :math:`l` the lead of the screw in meters and :math:`C` the counts per revolution of the encoder and :math:`c` the value of the encoder.
Then, a count :math:`c` of the encoder corresponds to a linear displacement :math:`d` of

.. math::

  \begin{eqnarray}
    c ~~\text{(in counts)} \longrightarrow d & = & r \times l \times \frac{c}{C} ~~\text{m}\\
    & = & \frac{13}{57} \times 10^{-3} \frac{c}{2000} ~~\text{m}\\
    & \approx & 0.000000114 \times c ~~\text{m}
  \end{eqnarray}


The conversion factor is more precisely :math:`0.11403508771929824 \times 10^{-6}`.
So the displacement resolution, the displacement corresponding to one count of the encoder, is approximately :math:`0.114 µm`.


The tpdo part of the configuration of the :code:`EcCiA402Drive` plugin will then be set up with the following values:

..  code-block:: yaml

  tpdo: # TxPDO = transmit PDO Mapping, slave (out) to master (in) (MISO), state values transmitted by the drive to ROS2 control
    - index: 0x1a03
      channels:
        - {
            index: 0x6064,
            sub_index: 0,
            type: int32,
            state_interface: position,
            factor: 0.11403508771929824e-6,
            offset: 0
          }  # Position actual value

Convert velocity in :math:`\text{m/s}` from :math:`\text{rpm}`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If the motor moves at a velocity of :math:`\theta` RPM, then the linear velocity :math:`v` in :math:`\text{m/s}` of the system is given by:

.. math::

  \begin{eqnarray}
    \theta ~~\text{(in rpm)} \longrightarrow v & = & \frac{13}{57} \times 10^{-3} \times \frac{1}{60} \times \theta ~~ \text{m/s} \\
    & \approx & 3.8011695906432747 \times 10^{-6} \times \theta ~~\text{m/s}
  \end{eqnarray}

The tpdo part of the configuration of the :code:`EcCiA402Drive` plugin will then be set up with the following values:

..  code-block:: yaml

  tpdo: # TxPDO = transmit PDO Mapping, slave (out) to master (in) (MISO), state values transmitted by the drive to ROS2 control
    - index: 0x1a03
      channels:
        - {
            index: 0x606c,
            sub_index: 0,
            type: int32,
            state_interface: velocity,
            factor: 3.8011695906432747e-6,
            offset: 0
          }  # Velocity actual value

Transform torque
~~~~~~~~~~~~~~~~

The output torque (:math:`T_o`) of the gearbox given the input torque of the motor (:math:`T_i`) is given by the formula:

.. math::

  \begin{eqnarray}
    T_i \longrightarrow T_o & = & \frac{57}{13} \times \eta_g \times T_i \\
    & \approx & 3.683076923076923 \times T_i
  \end{eqnarray}

Torque to Force conversion
~~~~~~~~~~~~~~~~~~~~~~~~~~

The force :math:`F_o` of the lead screw is given by the formula:

.. math::

  \begin{eqnarray}
    T_o \longrightarrow F_o & = & \frac{2\pi}{l}T_o \\
  \end{eqnarray}


Thus the force :math:`F_o` in :math:`\text{mN}` given the input torque of the motor (:math:`T_i` in :math:`\text{mNm}`) is provided by the formula:

.. math::

  \begin{eqnarray}
    T_i \longrightarrow F_o & = & \frac{2\pi}{l} \times \frac{57}{13} \times \eta_g \times T_i \\
    & \approx & 23141.45480828912 \times T_i
  \end{eqnarray}


The tpdo part of the configuration of the :code:`EcCiA402Drive` plugin will then be set up with the following values:

..  code-block:: yaml

  tpdo: # TxPDO = transmit PDO Mapping, slave (out) to master (in) (MISO), state values transmitted by the drive to ROS2 control
    - index: 0x1a03
      channels:
        - {
            index: 0x6077,
            sub_index: 0,
            type: int32,
            state_interface: effort,
            factor: 23141.45480828912,
            offset: 0
          }  # Force actual value in mN


Convert command values
----------------------

This is the configuration part of the rpdo of the :code:`EcCiA402Drive` plugin and corresponding to the :code:`command_interface` in ROS2 control terminology.

The drive will take the following command values:

- :code:`position` in :math:`\text{counts}` or :math:`c`
- :code:`velocity` in :math:`\theta` (revolutions per minute)
- :code:`torque` (:math:`T_i`) in :math:`\text{mNm}` (torque of the motor at the input of the gearbox)

We would  like to send the following command values:

- :code:`position` (:math:`d`) in :math:`\text{m}`
- :code:`velocity` (:math:`v`) in :math:`\text{m/s}`
- :code:`force` (:math:`F_o`) in :math:`\text{mN}` (Force of the lead screw)

Command in counts to move by a certain distance in m
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To command the motor to move by a certain distance :math:`d` in meters, the number of counts :math:`c` to send to the motor is given by the formula:

.. math::

  \begin{equation}
    d \longrightarrow \text{c} = \frac{C}{rl} \times d \approx 8769.23076923077 \times d
  \end{equation}

The 'rpdo' part of the configuration of the :code:`EcCiA402Drive` plugin will then be set up with the following values:


..  code-block:: yaml

  rpdo: # RxPDO = receive PDO Mapping, master (out) to slave (in) (MOSI), command values received by the drive from ROS2 control
    - index: 0x1603
      channels:
        - {
            index: 0x607a,
            sub_index: 0,
            type: int32,
            command_interface: position,
            factor: 8769.23076923077,
            offset: 0
          }  # Target position



Command in rpm to move at a certain velocity given in m/s
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To command the motor to move at a certain velocity in m/s, the number of rpm to send to the motor is given by the formula:

.. math::

  \begin{eqnarray}
    v ~~\text{(in m/s)}\longrightarrow \theta & = & \frac{60}{rl} \times v ~~\text{(in counts)}\\
               & \approx  & 263076.92307692306 \times v ~~\text{(in counts)}
  \end{eqnarray}

The 'rpdo' part of the configuration of the :code:`EcCiA402Drive` plugin will then be set up with the following values:


..  code-block:: yaml

  rpdo: # RxPDO = receive PDO Mapping, master (out) to slave (in) (MOSI), command values received by the drive from ROS2 control
    - index: 0x1603
      channels:
        - {
            index: 0x60ff,
            sub_index: 0,
            type: int32,
            command_interface: velocity,
            factor: 263076.92307692306,
            offset: 0
          }  # Target velocity

Force to torque conversion
~~~~~~~~~~~~~~~~~~~~~~~~~~

The input torque of the motor (:math:`T_i` in :math:`\text{mNm}`) given the target command force :math:`F_o` in :math:`\text{mN}` is provided by the formula:

.. math::

  \begin{eqnarray}
    F_o \longrightarrow T_i & = & \frac{l}{2\pi} \times \frac{13}{57} \times \frac{1}{\eta_g} \times F_o \\
    & \approx & 23141.45480828912 \times F_o
  \end{eqnarray}


The tpdo part of the configuration of the :code:`EcCiA402Drive` plugin will then be set up with the following values:

..  code-block:: yaml

  rpdo: # RxPDO = receive PDO Mapping, master (out) to slave (in) (MOSI), command values received by the drive from ROS2 control
    - index: 0x1603
      channels:
        - {
            index: 0x6071,
            sub_index: 0,
            type: int32,
            state_interface: effort,
            factor: 4.321249499153383e-05,
            offset: 0
          }  # Target force


Complete configuration example for an EPOS4 drive
-------------------------------------------------

Note that we omit the ``offset`` parameter since their default value is equal to zero.

.. literalinclude:: epos4_config_with_gear_box.yaml
   :language: yaml
   :caption: Example of a complete configuration for an EPOS4 drive with a gearbox and a transmission screw
   :linenos:
