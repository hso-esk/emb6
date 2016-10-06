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

static uint16_t link_local_addr_fixed[7] = { UIP_HTONS(0xfe80), 0x0000, 0x0000, 0x0000, 0x0000, UIP_HTONS(0x00ff), UIP_HTONS(0xfe00) };
static uint16_t mesh_local_addr_fixed[7] = { UIP_HTONS(THRD_MESH_LOCAL_PREFIX) , 0x0000, 0x0000, 0x0000, 0x0000, UIP_HTONS(0x00ff), UIP_HTONS(0xfe00)};
static uint16_t mesh_local_iid_fixed[3] =  { 0x0000, UIP_HTONS(0x00ff), UIP_HTONS(0xfe00) };
static uint16_t rloc_addr_fixed[3] = { 0x0000, UIP_HTONS(0x00ff), UIP_HTONS(0x00fe00) };

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
	if ( memcmp(&link_local_addr_fixed[0], addr->u16, 14) == 0 ) {
		return TRUE;
	}
	return FALSE;
}

/* --------------------------------------------------------------------------- */

bool
thrd_is_addr_ml_rloc(uip_ipaddr_t *addr)
{
	if ( memcmp(&mesh_local_addr_fixed, addr, 14) == 0 ) {
		return TRUE;
	}
	return FALSE;
}

/* --------------------------------------------------------------------------- */

bool
thrd_is_rloc_addr(uip_ipaddr_t *addr) {
	if ( memcmp(&rloc_addr_fixed, &addr->u8[8], 6) == 0 ) {
		return TRUE;
	}
	return FALSE;
}

/* --------------------------------------------------------------------------- */

bool
thrd_is_linkaddr_rloc(linkaddr_t *link_addr)
{
	if ( link_addr != NULL ) {
		if ( memcmp(&mesh_local_iid_fixed, link_addr->u8, 6) == 0 ) {
			return TRUE;
		}
	}
	return FALSE;
}

/* --------------------------------------------------------------------------- */

void
thrd_eui_64_invert_universal_local_bit(linkaddr_t *link_addr) {
	link_addr->u8[0] &= 0xfd;
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_extract_router_id_from_rloc_addr(uip_ipaddr_t *rloc_addr)
{
	return ((uint8_t) (rloc_addr->u16[7] >> 10));
}

/* --------------------------------------------------------------------------- */

uint8_t
thrd_extract_router_id_from_rloc_linkaddr(linkaddr_t *link_addr)
{
	uint16_t rloc16 = thrd_extract_rloc16_from_rloc_linkaddr(link_addr);
	uint8_t router_id = THRD_EXTRACT_ROUTER_ID(rloc16);
	return router_id;
}

/* --------------------------------------------------------------------------- */

uint16_t
thrd_extract_rloc16_from_rloc_address(uip_ipaddr_t *rloc_addr)
{
	return ((uint16_t) (rloc_addr->u16[7]));
}

/* --------------------------------------------------------------------------- */

uint16_t
thrd_extract_rloc16_from_rloc_linkaddr(linkaddr_t *link_addr)
{
	return ((uint16_t) (( (link_addr->u8[6]) << 8 ) | link_addr->u8[7]));
}

/* --------------------------------------------------------------------------- */

bool
thrd_is_rloc16_valid(uint16_t rloc16)
{
	if ( THRD_EXTRACT_ROUTER_ID(rloc16) < 63 ) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/* --------------------------------------------------------------------------- */

bool
thrd_is_rid_valid(uint8_t router_id)
{
	if ( router_id < 63 ) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/* --------------------------------------------------------------------------- */

void
thrd_create_next_hop_addr(uip_ipaddr_t *addr, uint8_t rloc16)
{
	thrd_create_meshlocal_prefix(addr);
	thrd_create_rloc_iid(addr, rloc16);
}

/* --------------------------------------------------------------------------- */

void
thrd_invert_universal_local_bit(uip_lladdr_t *lladdr)
{
	lladdr->addr[0] &= IPV6_UNIVERSAL_LOCAL_BIT_INVERT_MASK;
}

