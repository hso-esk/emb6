/*
 * thread_conf.h
 *
 *  Created on: 23 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 */

#ifndef EMB6_THREAD_CONF_H_
#define EMB6_THREAD_CONF_H_

#include "emb6.h"
// #include "thrd-dev.h"

/*=============================================================================
                                    ERROR CODES
===============================================================================*/

typedef enum {
	THRD_ERROR_NONE = 0,
	THRD_ERROR_FAILED = 1,
	THRD_ERROR_DROP = 2,
	THRD_ERROR_NO_BUFS = 3,
	THRD_ERROR_NO_ROUTE = 4,
	THRD_ERROR_BUSY = 5,
	THRD_ERROR_PARSE = 6,
	THRD_ERROR_INVALID_ARGS = 7,
	THRD_ERROR_SECURITY = 8,
	THRD_ERROR_ADDRESS_QUERY = 9,
	THRD_ERROR_NO_ADDRESS = 10,
	THRD_ERROR_NOT_RECEIVING = 11,
	THRD_ERROR_ABORT = 12,
	THRD_ERROR_NOT_IMPLEMENTED = 13,
	THRD_ERROR_INVALID_STATE = 14,
	THRD_ERROR_NO_TASKLETS = 15,
	THRD_ERROR_ERROR = 255
} thrd_error_t;

/*=============================================================================
                                    DEVICE TYPE
===============================================================================*/

#define THRD_DEV_TYPE			THRD_DEV_TYPE_REED			// REED.
#define THRD_DEV_FUNC			THRD_DEV_FUNC_FFD			// FFD.

// -------------------- THREAD RLOC16 ----------------------------
#define THRD_CREATE_RLOC16(router_id, child_id)		((uint16_t) (0x0000 | ((router_id << 10) | (child_id) )))
#define THRD_EXTRACT_ROUTER_ID(rloc16)		((uint8_t) (rloc16 >> 10))
#define THRD_EXTRACT_CHILD_ID(rloc16)		((uint8_t) (rloc16))

// ---------------- THREAD IPv6 ADDRESSES ------------------------
#define THRD_LINK_LOCAL_ALL_NODES_ADDR(ipaddr)	uip_ip6addr(ipaddr, 0xff02, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001)
#define THRD_LINK_LOCAL_ALL_ROUTERS_ADDR(ipaddr)	uip_ip6addr(ipaddr, 0xff02, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0002)
#define THRD_REALM_LOCAL_ALL_NODES_ADDR(ipaddr)	uip_ip6addr(ipaddr, 0xff03, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001)
#define THRD_REALM_LOCAL_ALL_ROUTERS_ADDR(ipaddr)	uip_ip6addr(ipaddr, 0xff03, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0002)
#define THRD_REALM_LOCAL_ALL_MPL_FORWARDERS_ADDR(ipaddr)	uip_ip6addr(ipaddr, 0xff03, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x00fc)

// ------------- THREAD IPv6 ADDRESS PREFIX ----------------------
#define	THRD_MESH_LOCAL_PREFIX		0xfd00

// ----------------- IANA CONSIDERATIONS ------------------------
#define THRD_MGMT_COAP_PORT				19789 // UIP_HTONS(19789)		// Thread Network Management (:MM).

/*=============================================================================
                                NETWORK LAYER SECTION
===============================================================================*/

#ifndef THRD_USE_ROUTING
#define THRD_USE_ROUTING == TRUE                // Remove this!!!
#endif /* THRD_USE_ROUTING */

// #if THRD_USE_ROUTING == TRUE
#define UIP_CONF_IPV6_THRD_ROUTING		TRUE
#define THRD_SICSLOWPAN_ROUTING			TRUE
#define SICSLOWPAN_USE_MESH_HEADER		TRUE
// #endif

// --------------- THREAD ROUTING PROTOCOL ----------------------


/** Routing table size */
#define THRD_CONF_MAX_ROUTES            5 // 10

/** Exponentially weighted moving average. */
#define EXP_WEIGHT_MOV_AVG_1_8			3	// 1/8 (shifting bits).
#define EXP_WEIGHT_MOV_AVG_1_16			4	// 1/16 (shifting bits).
/** Set the weight. */
#define THRD_EXP_WEIGHT_MOV_AVG			EXP_WEIGHT_MOV_AVG_1_8

/** Network Layer defines. */
#define	ADVERTISEMENT_I_MIN				1		// 1 sec.
#define	ADVERTISEMENT_I_MAX				5		// 2⁵ = 32.
#define	ID_REUSE_DELAY					100
#define	ID_SEQUENCE_PERIOD				10
#define	MAX_NEIGHBOR_AGE				100
#define	MAX_ROUTE_COST					16
#define	MAX_ROUTER_ID					62 // 62
#define	MAX_ROUTERS						32 // 32
#define	MIN_DOWNGRADE_NEIGHBORS			7
#define	NETWORK_ID_TIMEOUT				120
#define	PARENT_ROUTE_TO_LEADER_TIMEOUT	20
#define	ROUTER_SELECTION_JITTER			120
#define	ROUTER_DOWNGRADE_TRESHOLD		23
#define	ROUTER_UPGRADE_TRESHOLD			16
#define	INFINITE_COST_TIMEOUT			90
#define	REED_ADVERTISEMENT_INTERVAL		570
#define	REED_ADVERTISEMENT_MAX_JITTER	60

/** Network Layer TLVs. */
#define THRD_NET_TLV_MAX_NUM			10		// Maximum number of network layer TLVs.
#define THRD_NET_TLV_MAX_SIZE			50		// Maximum size of network layer TLVs.

// --------------- THREAD EID-to-RLOC Mapping -------------------

#define THRD_MAX_LOCAL_ADDRESSES		10		// Maximum number of local addresses (Local Address Set).
#define THRD_MAX_RFD_CHILD_ADDRESSES_LL	10		// Maximum number of Link-Local RFD Child Addresses (RFD Child Address Set).
#define THRD_MAX_RFD_CHILD_ADDRESSES_ML	10		// Maximum number of Mesh-Local RFD Child Addresses (RFD Child Address Set).
#define THRD_MAX_RFD_CHILD_ADDRESSES_OS	10		// Maximum number of Other-Scope RFD Child Addresses (RFD Child Address Set).
#define THRD_MAX_ADDRESS_QUERIES		32		// Maximum number of Address Queries (Address Query Set).

// --------------- THREAD EID-to-RLOC Map Cache -----------------

#define THRD_MAX_EID_RLOC_MAP_CACHE_SIZE	10		// Maximum number of EID-to-RLOC Map Cache entries.

// ------------------- THREAD NETWORK DATA ----------------------

#define THRD_MAX_ON_MESH_PREFIX_SET_ENTRIES		5	// Maximum number of On-Mesh Prefix Set entries.
#define THRD_MAX_EXT_ROUTE_SET_ENTRIES			5	// Maximum number of External Router Set entries.
#define THRD_MAX_SICSLOWPAN_CTX_ID_SET_ENTRIES	5	// Maximum number of 6LoWPAN Context ID Set entries.
#define THRD_MAX_SERVER_SET_ENTRIES				5	// Maximum number of Server Set entries.

// ---------- THREAD PROTOCOL PARAMETERS AND CONSTANTS ----------

#define CONTEXT_ID_REUSE_DELAY					172800		// 48 hours in seconds.
#define DATA_RESUBMIT_DELAY						300			// 300 seconds.
#define MAX_NETWORK_DATA_SIZE					255			// 255 bytes.
#define MIN_PREFIX_LIFETIME						3600		// 3600 seconds.
#define MIN_STABLE_LIFETIME						604800		// 168 hours in seconds.
#define LEADER_TIMEOUT							120			// 120 seconds.

// --------------------------------------------------------------

#endif /* EMB6_THREAD_CONF_H_ */
