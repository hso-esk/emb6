/*
 * thrd-iface.c
 *
 *  Created on: 15 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Thread Interface.
 */

#include "thrd-iface.h"
#include "uip-ds6.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

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

static uint8_t extract_router_id(uint16_t rloc16);

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

void
thrd_iface_rloc_set(uint16_t rloc16)
{
	if ( thrd_iface.rloc16 != rloc16 ) {
		PRINTF("Creating new RLOC addresses.\n");
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
		thrd_iface_set_router_id(extract_router_id(rloc16));
		thrd_iface_print();
	}
}

static void
remove_rloc_addr()
{
	uip_ds6_addr_t *rloc_addr = uip_ds6_addr_lookup(&thrd_iface.ml_rloc);
	uip_ds6_addr_rm(rloc_addr);
	rloc_addr = uip_ds6_addr_lookup(&thrd_iface.ll_rloc);
	uip_ds6_addr_rm(rloc_addr);
}

static uint8_t
extract_router_id(uint16_t rloc16)
{
	return ((uint8_t) rloc16 >> 10);
}

/*==============================================================================
                                    DEBUG FUNCTIONS
 =============================================================================*/

void
thrd_iface_print()
{
	PRINTF(ANSI_COLOR_CYAN "|================================== THREAD INTERFACE ===================================|" ANSI_COLOR_RESET "\n\r");
	PRINTF("ML-EID = ");
	PRINT6ADDR(&thrd_iface.ml_eid);
	PRINTF("\n");
	PRINTF("ML-RLOC = ");
	PRINT6ADDR(&thrd_iface.ml_rloc);
	PRINTF("\n");
	PRINTF("LL-EID = ");
	PRINT6ADDR(&thrd_iface.ll_eid);
	PRINTF("\n");
	PRINTF("LL-RLOC = ");
	PRINT6ADDR(&thrd_iface.ll_rloc);
	PRINTF("\n");
	PRINTF(ANSI_COLOR_CYAN "=========================================================================================" ANSI_COLOR_RESET "\n\r");
}

/* --------------------------------------------------------------------------- */

void
print_all_addr()
{
	uint8_t state;
	PRINTF("Our IPv6 addresses:\n");
	for(uint8_t i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if(uip_ds6_if.addr_list[i].isused && (state == ADDR_TENTATIVE || state
				== ADDR_PREFERRED)) {
			PRINTF("  ");
			PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
			PRINTF("\n");
			if(state == ADDR_TENTATIVE) {
				uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
			}
		}
	}
}

