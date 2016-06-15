/*
 * thrd-iface.c
 *
 *  Created on: 15 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Thread Interface.
 */

#include "thrd-iface.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.

/** Thread Network Partition. */
thrd_iface_t thrd_iface;

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

/*==============================================================================
                               LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

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
	PRINTF("--------------\n ml_eid = ");
	PRINT6ADDR(&addr);
	PRINTF("\n ----------------\n");
	// Create ML-RLOC.
	thrd_create_rloc_iid(&addr, thrd_iface.rloc16);
	thrd_iface.ml_rloc = addr;
	// Create LL-IPv6 ADDRESS based on the MAX Extended Address.
	uip_create_linklocal_prefix(&addr);
	thrd_create_eui_64_bit_iid(&addr, mac_phy_config.mac_address);
	thrd_iface.ll_eid = addr;
	// Create LL-RLOC
	thrd_create_rloc_iid(&addr, thrd_iface.rloc16);
	thrd_iface.ll_rloc = addr;
	thrd_iface_print();
}

/* --------------------------------------------------------------------------- */

void
thrd_iface_rloc_set(uint16_t *rloc16)
{
	if ( thrd_iface.rloc16 != *rloc16 ) {
		thrd_iface.rloc16 = *rloc16;
		uip_ipaddr_t rloc;
		// ML-RLOC.
		thrd_create_meshlocal_prefix(&rloc);
		thrd_create_rloc_iid(&rloc, *rloc16);
		thrd_iface.ml_rloc = rloc;
		// LL-RLOC.
		uip_create_linklocal_prefix(&rloc);
		thrd_create_rloc_iid(&rloc, *rloc16);
		thrd_iface.ll_rloc = rloc;
	}
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

