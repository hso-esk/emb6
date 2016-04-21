/*
 * rip.h
 *
 *  Created on: 12 Apr 2016
 *      Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 */

#ifndef EMB6_INC_NET_RIP_RIP_H_
#define EMB6_INC_NET_RIP_RIP_H_

/** Network Level Protocol Parameters */

#define	ADVERTISEMENT_I_MIN				1
#define	ADVERTISEMENT_I_MAX				32
#define	ID_REUSE_DELAY					100
#define	ID_SEQUENCE_PERIOD				10
#define	MAX_NEIGHBOR_AGE				100
#define	MAX_ROUTE_COST					16
#define	MAX_ROUTER_ID					2 // 62
#define	MAX_ROUTERS						2 // 32
#define	MIN_DOWNGRADE_NEIGHBORS			7
#define	NETWORK_ID_TIMEOUT				120
#define	PARENT_ROUTE_TO_LEADER_TIMEOUT	20
#define	ROUTER_SELECTION_JITTER			120
#define	ROUTER_DOWNGRADE_TRESHOLD		23
#define	ROUTER_UPGRADE_TRESHOLD			16
#define	INFINITE_COST_TIMEOUT			90
#define	REED_ADVERTISEMENT_INTERVAL		570
#define	REED_ADVERTISEMENT_MAX_JITTER	60

/** ------------------------- Structures ------------------------------- */


/** ------------------------- Public Funtions ------------------------------- */


#endif /* EMB6_INC_NET_RIP_RIP_H_ */
