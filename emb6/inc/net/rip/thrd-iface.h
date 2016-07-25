/*
 * thrd-iface.h
 *
 *  Created on: 15 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Thread Interface.
 */

#ifndef EMB6_INC_NET_RIP_THRD_IFACE_H_
#define EMB6_INC_NET_RIP_THRD_IFACE_H_

#include "bsp.h"
#include "uip.h"

typedef struct {
	uint16_t rloc16;

	uip_ipaddr_t ml_eid;
	uip_ipaddr_t ml_rloc;
	uip_ipaddr_t ll_eid;
	uip_ipaddr_t ll_rloc;

#if ( (THRD_DEV_TYPE == THRD_DEV_TYPE_ROUTER) || (THRD_DEV_TYPE == THRD_DEV_TYPE_REED) )
	uint8_t router_id;	// Router ID.
#endif
} thrd_iface_t;

extern thrd_iface_t thrd_iface;

/**
 * Initialize Thread Interface. Create Link-Local IPv6 address with the IID set to the
 * MAC Extended Address with the universal/local bit inverted. Create Mesh-Local EID
 * with the IID chosen at random.
 */
void thrd_iface_init();

/**
 * Set the Router ID of the Thread Interface.
 * @param router_id A Router ID.
 */
void thrd_iface_set_router_id(uint8_t router_id);

/**
 * Get the Router ID of the Interface.
 * @return
 */
uint8_t thrd_iface_get_router_id();

/**
 * Set the interface's RLOC16 and update the ML-RLOC and LL-RLOC addresses.
 * @param rloc16 The RLOC16.
 */
void thrd_iface_set_rloc(uint16_t rloc16);

/**
 * Print Thread interface information.
 */
void thrd_iface_print();

#endif /* EMB6_INC_NET_RIP_THRD_IFACE_H_ */
