/*
 * thread_conf.h
 *
 *  Created on: 23 May 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 */

#ifndef EMB6_THREAD_CONF_H_
#define EMB6_THREAD_CONF_H_

#include "emb6.h"

/*=============================================================================
                                    DEVICE TYPE
===============================================================================*/

#define THRD_DEV_TYPE			THRD_DEV_TYPE_ROUTER		// Router.
#define THRD_DEV_FUNC			THRD_DEV_FUNC_FFD			// FFD.

/**
 * The Router ID.
 */
#if ( (THRD_DEV_TYPE == THRD_DEV_TYPE_ROUTER) || (THRD_DEV_TYPE == THRD_DEV_TYPE_REED) )
uint8_t Router_ID;
#endif

/**
 * Thread Device Types.
 */
typedef enum
{
	THRD_DEV_TYPE_END = 0,   //!< DEV_TYPE_END
	THRD_DEV_TYPE_REED = 1,  //!< DEV_TYPE_REED
	THRD_DEV_TYPE_ROUTER = 2,//!< DEV_TYPE_ROUTER
	THRD_DEV_TYPE_LEADER = 3 //!< DEV_TYPE_LEADER
} thrd_dev_type_t;

/**
 * Thread Device Functionalty (RFD or FFD).
 */
typedef enum
{
	THRD_DEV_FUNC_RFD = 0,//!< THRD_DEV_RFD
	THRD_DEV_FUNC_FFD = 1 //!< THRD_DEV_FFD
} thrd_dev_funct_t;

/**
 * Thread Device Types and RFD/FFD.
 */
typedef struct
{
	uint8_t type;	// thrd_dev_type_t.
	uint8_t isFFD;
	uint8_t isRX_off_when_idle;
} thrd_dev_t;

/*! Thread Device Type Configuration. */
extern thrd_dev_t thrd_dev;

/*=============================================================================
                                NETWORK LAYER SECTION
===============================================================================*/

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
#define	ID_SEQUENCE_PERIOD				1
#define	MAX_NEIGHBOR_AGE				100
#define	MAX_ROUTE_COST					16
#define	MAX_ROUTER_ID					5 // 62
#define	MAX_ROUTERS						5 // 32
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

// --------------------------------------------------------------

#endif /* EMB6_THREAD_CONF_H_ */
