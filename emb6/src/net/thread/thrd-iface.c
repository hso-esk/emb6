/**
 * \file thrd-iface.c
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/06/15
 * \version 1.0
 *
 * \brief Thread interface
 */

#include "thrd-iface.h"
#include "thrd-dev.h"
#include "uip-ds6.h"
#include "thrd-addr.h"

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"

/** Thread Network Partition. */
thrd_iface_t thrd_iface = {
		.router_id = 63,						// Initialize Router ID to invalid value.
		.rloc16 = THRD_CREATE_RLOC16(63, 0),	// Initialize RLOC16 to invalid value.
};

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

static void remove_rloc_addr();

static void print_all_addr();

/*==============================================================================
                                    LOCAL FUNCTIONS
 =============================================================================*/

/* --------------------------------------------------------------------------- */

/*==============================================================================
                                     API FUNCTIONS
 =============================================================================*/

void
thrd_iface_init()
{
	// Create ML-EID.
	uip_ipaddr_t addr;
	thrd_create_meshlocal_prefix(&addr);
	thrd_create_random_iid(&addr);
	// Check whether the generated ML-EID's IID is valid.
	while ( thrd_ml_eid_iid_is_valid(&addr) == FALSE ) {
		thrd_create_random_iid(&addr);
	}
	thrd_iface.ml_eid = addr;
	uip_ds6_addr_add(&thrd_iface.ml_eid, 0, ADDR_MANUAL);
	// Create LL-IPv6 ADDRESS based on the MAC Extended Address.
	uip_create_linklocal_prefix(&addr);
	thrd_create_eui_64_bit_iid(&addr, mac_phy_config.mac_address);
	thrd_iface.ll_eid = addr;
	uip_ds6_addr_add(&thrd_iface.ll_eid, 0, ADDR_MANUAL);
	thrd_iface_print();
}

/* --------------------------------------------------------------------------- */

void
thrd_iface_set_router_id(uint8_t router_id)
{
	thrd_iface.router_id = router_id;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_iface_get_router_id()
{
	return thrd_iface.router_id;
}

/* --------------------------------------------------------------------------- */

uint16_t
thrd_iface_get_rloc16()
{
	return thrd_iface.rloc16;
}

/* --------------------------------------------------------------------------- */

void
thrd_iface_set_rloc(uint16_t rloc16)
{
	if ( thrd_iface.rloc16 != rloc16 ) {

		LOG_RAW("Creating new RLOC addresses.\n");
		// Remove old RLOC addresses.
		remove_rloc_addr();
		// Creating and adding new RLOC addresses.
		thrd_iface.rloc16 = rloc16;
		uip_ipaddr_t rloc;
		// ML-RLOC.
		thrd_create_meshlocal_prefix(&rloc);
		thrd_create_rloc_iid(&rloc, rloc16);
		thrd_iface.ml_rloc = rloc;
		uip_ds6_addr_add(&thrd_iface.ml_rloc, 0, ADDR_MANUAL);

		// LL-RLOC.
		uip_create_linklocal_prefix(&rloc);
		thrd_create_rloc_iid(&rloc, rloc16);
		thrd_iface.ll_rloc = rloc;
		uip_ds6_addr_add(&thrd_iface.ll_rloc, 0, ADDR_MANUAL);
		// Set new Router ID.
		thrd_iface_set_router_id(THRD_EXTRACT_ROUTER_ID(rloc16));
	}
}

/* --------------------------------------------------------------------------- */

static void
remove_rloc_addr()
{
	uip_ds6_addr_t *rloc_addr = uip_ds6_addr_lookup(&thrd_iface.ml_rloc);
	uip_ds6_addr_rm(rloc_addr);
	rloc_addr = uip_ds6_addr_lookup(&thrd_iface.ll_rloc);
	uip_ds6_addr_rm(rloc_addr);
}

/*==============================================================================
                                    DEBUG FUNCTIONS
 =============================================================================*/

void
thrd_iface_print()
{
	LOG_RAW("|================================== THREAD INTERFACE ===================================|\n\r");
	if ( thrd_iface.router_id == 63 ) {
		LOG_RAW("| Router ID = 63 (invalid)                                                              |\n\r");
	} else {
		LOG_RAW("| Router ID = %2d                                                                        |\n\r", thrd_iface.router_id);
	}
	if ( thrd_iface.rloc16 == 0xfc00 ) {
		LOG_RAW("| RLOC = fc00 (invalid)                                                                 |\n\r");
	} else {
		LOG_RAW("| RLOC = %04x                                                                           |\n\r", thrd_iface.rloc16);
	}
	LOG_RAW("|---------------------------------------------------------------------------------------|\n\r");
	LOG_RAW("| ML-EID = ");
	LOG_IP6ADDR(&thrd_iface.ml_eid);
	LOG_RAW("\n\r");
	LOG_RAW("| ML-RLOC = ");
	LOG_IP6ADDR(&thrd_iface.ml_rloc);
	LOG_RAW("\n\r");
	LOG_RAW("| LL-EUI-64 = ");
	LOG_IP6ADDR(&thrd_iface.ll_eid);
	LOG_RAW("\n\r");
	LOG_RAW("| LL-RLOC = ");
	LOG_IP6ADDR(&thrd_iface.ll_rloc);
	LOG_RAW("\n\r");
	LOG_RAW("|=======================================================================================|\n\r");
}

/* --------------------------------------------------------------------------- */

static void
print_all_addr()
{
	uint8_t state;
	LOG_RAW("Our IPv6 addresses:\n");
	for(uint8_t i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if( uip_ds6_if.addr_list[i].isused && (state == ADDR_TENTATIVE || state == ADDR_PREFERRED) ) {
			LOG_RAW("  ");
			LOG_IP6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
			LOG_RAW("\n\r");
		}
	}
}

