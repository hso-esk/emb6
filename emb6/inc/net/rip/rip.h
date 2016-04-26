/*
 * rip.h
 *
 *  Created on: 12 Apr 2016
 *      Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 */

#ifndef EMB6_INC_NET_RIP_RIP_H_
#define EMB6_INC_NET_RIP_RIP_H_

/** Debug print colors. */
#define ANSI_COLOR_RED     	"\x1b[31m"
#define ANSI_COLOR_YELLOW  	"\x1b[33m"
#define	ANSI_COLOR_GREEN	"\x1B[32m"
#define ANSI_COLOR_CYAN    	"\x1b[36m"
#define ANSI_COLOR_MAGENTA 	"\x1b[35m"
#define ANSI_COLOR_RESET  	"\x1b[0m"

/** Network Level Protocol Parameters */

/** Routing table size */
#define THRD_CONF_MAX_ROUTES            5 // 10

/** Exponentially weighted moving average. */
#define EXP_WEIGHT_MOV_AVG_1_8			3	// 1/8.
#define EXP_WEIGHT_MOV_AVG_1_16			4	// 1/16.
/** Set the weight. */
#define THRD_EXP_WEIGHT_MOV_AVG			EXP_WEIGHT_MOV_AVG_1_8

/** Network Layer defines. */
#define	ADVERTISEMENT_I_MIN				1
#define	ADVERTISEMENT_I_MAX				32
#define	ID_REUSE_DELAY					100
#define	ID_SEQUENCE_PERIOD				10
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

/** ------------------------- Structures ------------------------------- */


/** ------------------------- Public Funtions ------------------------------- */


#endif /* EMB6_INC_NET_RIP_RIP_H_ */
