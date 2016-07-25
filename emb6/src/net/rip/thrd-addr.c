/*
 * thrd-addr.c
 *
 *  Created on: 25 Jul 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  THREAD IPv6 Addressing Tools.
 */

#include "emb6.h"
#include "thread_conf.h"
#include "uip.h"
#include "thrd-iface.h"
#include "thrd-addr.h"

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/

static uint16_t link_local_fixed[7] = { UIP_HTONS(0xfe80), 0x0000, 0x0000, 0x0000, 0x0000, UIP_HTONS(0x00ff), UIP_HTONS(0xfe00) };
static uint16_t mesh_local_fixed[7] = { UIP_HTONS(THRD_MESH_LOCAL_PREFIX) , 0x0000, 0x0000, 0x0000, 0x0000, UIP_HTONS(0x00ff), UIP_HTONS(0xfe00)};

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

bool
thrd_is_addr_ll_rloc(uip_ipaddr_t *addr)
{
	if ( memcmp(&link_local_fixed[0], addr->u16, 14) == 0 ) {
		return TRUE;
	}
	return FALSE;
}

/* --------------------------------------------------------------------------- */

bool
thrd_is_addr_ml_rloc(uip_ipaddr_t *addr)
{
	if ( memcmp(&mesh_local_fixed, addr, 14) == 0 ) {
		return TRUE;
	}
	return FALSE;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_extract_router_id_from_rloc_addr(uip_ipaddr_t *rloc_addr)
{
	return ((uint8_t) (rloc_addr->u16[7] >> 10));
}

/* --------------------------------------------------------------------------- */

void
thrd_create_next_hop_addr(uip_ipaddr_t *addr, uint8_t rloc16)
{
	thrd_create_meshlocal_prefix(addr);
	thrd_create_rloc_iid(addr, rloc16);
}
