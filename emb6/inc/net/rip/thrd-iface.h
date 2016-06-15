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

/**
 * Create Mesh-Local Prefrix.
 */
#define thrd_create_meshlocal_prefix(addr) do { 	\
		(addr)->u16[0] = UIP_HTONS(0xfdfd);         \
		(addr)->u16[1] = 0x00;                      \
		(addr)->u16[2] = 0x00;                      \
		(addr)->u16[3] = 0x00;                      \
} while(0)
/**
 * Create a random Interface Identifier (IID).
 */
#define thrd_create_random_iid(addr) do {			\
		(addr)->u16[4] = bsp_getrand(0);			\
		(addr)->u16[5] = bsp_getrand(0);			\
		(addr)->u16[6] = bsp_getrand(0);			\
		(addr)->u16[7] = bsp_getrand(0);			\
} while(0)
/**
 * Check whether the given IPv6 address's IID is valid (Not allowed: 0000:00ff:fe00:xxxx).
 */
#define thrd_ml_eid_iid_is_valid(addr) (( (addr)->u16[4] == 0x0000 && (addr)->u16[5] == 0x00ff && (addr)->u16[6] == 0xfe00 ) ? FALSE : TRUE)
/**
 * Create IID of a given RLOC address.
 */
#define thrd_create_rloc_iid(addr, rloc16) do {		\
		(addr)->u16[4] = UIP_HTONS(0x0000);			\
		(addr)->u16[5] = UIP_HTONS(0x00ff);			\
		(addr)->u16[6] = UIP_HTONS(0xfe00);			\
		(addr)->u16[7] = UIP_HTONS(rloc16);			\
} while(0)
/**
 * Create IID of a EUI-64 Bit IPv6 Address.
 */
#define thrd_create_eui_64_bit_iid(addr, mac) do {	\
		(addr)->u8[8] = mac[0];						\
		(addr)->u8[8] ^= 1 << 1;					\
		(addr)->u8[9] = mac[1];						\
		(addr)->u8[10] = mac[2];					\
		(addr)->u8[11] = mac[3];					\
		(addr)->u8[12] = mac[4];					\
		(addr)->u8[13] = mac[5];					\
		(addr)->u8[14] = mac[6];					\
		(addr)->u8[15] = mac[7];					\
} while(0)

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

void thrd_iface_init();

/**
 * Set the interface's RLOC16 and update the ML-RLOC and LL-RLOC addresses.
 * @param rloc16 The RLOC16.
 */
void thrd_iface_rloc_set(uint16_t *rloc16);

void thrd_iface_print();

#endif /* EMB6_INC_NET_RIP_THRD_IFACE_H_ */
