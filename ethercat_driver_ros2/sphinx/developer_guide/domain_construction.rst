How domain construction is done
===============================


Interface between the master and the application
------------------------------------------------

The link between the data used by the application and the one that transit on the EtherCAT network as EtherCAT frames is explicitly provided via the definition of domains.
Domains are contiguous memory areas that transit on the EtherCAT network.

This mapping is provided to the application as offsets inside the communication frames in the domains.
For instance, is a 32bit integer is used by the application, the application got a pointer :math:`\text{pd}` to the start of the domain and an offset :math:`o` that give the pointer :math:`p` to the first octet of the integer via pointer arithmetic: :math:`\text{p} = \text{pd} + o`.

The application has access to objects of type :code:`ec_pdo_entry_reg_t` (IgH master data structures) that contains the information to access the data in the domain:

.. code-block:: cpp

    typedef struct {
    uint16_t alias; /**< Slave alias address. */
    uint16_t position; /**< Slave position. */
    uint32_t vendor_id; /**< Slave vendor ID. */
    uint32_t product_code; /**< Slave product code. */
    uint16_t index; /**< PDO entry index. */
    uint8_t subindex; /**< PDO entry subindex. */
    unsigned int *offset; /**< Pointer to a variable to store the PDO entry's
                       (byte-)offset in the process data. */
    unsigned int *bit_position; /**< Pointer to a variable to store a bit
                                  position (0-7) within the \a offset. Can be
                                  NULL, in which case an error is raised if the
                                  PDO entry does not byte-align. */
    } ec_pdo_entry_reg_t;

In this structure the :code:`offset` and :code:`bit_position` are pointers to the offset and bit position of the data in the domain because this structure is given to the master that fill in those values establishing the link for the application with its data.

The application configures its link with the EtherCAT master via separate API calls. 
It first creates domains with:
1. :code:`ec_domain_t * ecrt_master_create_domain(const ec_master_t *master)`
Then it provides the master with:
2. the list of slaves that are used in the domain 

.. code-block:: cpp
    
    ec_slave_config_t *ecrt_master_slave_config(
        ec_master_t *master, /**< EtherCAT master */
        uint16_t alias, /**< Slave alias. */
        uint16_t position, /**< Slave position. */
        uint32_t vendor_id, /**< Expected vendor ID. */
        uint32_t product_code /**< Expected product code. */
        );

3. the list of PDO entries that are used in the domain, via an array of :code:`ec_pdo_entry_reg_t` objects, where offset and bit_position points to the data the application will use and have to be filled by the master: :code:`bool ecrt_domain_reg_pdo_entry_list(ec_domain_t *domain, ec_pdo_entry_reg_t *entries)`
4. It asks for pointer to the start of the domain memory: :code:`uint8_t * ecrt_domain_data(ec_domain_t *domain)`
One domain can be used for read and one for writing data for instance, so the application may make several calls to the above functions to create the domains it needs.

ethercat_driver_ros2 data structure organization
------------------------------------------------

The library keeps a representation of the masters used as :code:`EcMaster` objects.
Each :code:`EcMaster` object keeps a collection of domain configurations as :code:`DomainInfo` objects organised in a map with the domain index as key.
In a :code:`DomainInfo` a vector of :code:`ec_pdo_entry_reg_t` objects is kept that records the PDO entries used by the application with the links/offsets into that domain data.

.. code-block:: cpp

    struct DomainInfo {
        explicit DomainInfo(ec_master_t * master);
        ~DomainInfo();

        ec_domain_t * domain = NULL;
        ec_domain_state_t domain_state = {};
        uint8_t * domain_pd = NULL;

        /** domain pdo registration array.
            *  do not modify after active(), or may invalidate */
        std::vector<ec_pdo_entry_reg_t> domain_regs;

        /** slave's pdo entries in the domain */
        struct Entry
        {
            EcSlave * slave = NULL;
            int num_pdos = 0;
            uint32_t * offset = NULL;
            uint32_t * bit_position = NULL;
        };

        std::vector<Entry> entries;
    };

    struct EcMaster {
        ...
        ec_master_t *master;
        ...
        std::map<int, DomainInfo> domains;
        ...
    };

All the :code:`DomainInfo` data structures are created during the calls to the :code:`addSlave` method of the :code:`EcMaster` object. :code:`ec_pdo_entry_reg_t` objects are effectively created by :code:`addSlave` with a call of the :code:`registerPDOInDomain` method.

The link is made at the :code:`on_activate` stage of the hardware interface life-cycle in ros2 control.
