/*
 * thrd-network-data-leader.h
 *
 * Created on: 13 Jul 2016
 * Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 * Thread Network Data managed by the Thread Leader.
 */

#ifndef EMB6_INC_NET_RIP_THRD_NETWORK_DATA_LEADER_H_
#define EMB6_INC_NET_RIP_THRD_NETWORK_DATA_LEADER_H_

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

/*
 ********************************************************************************
 *                               GLOBAL VARIABLES
 ********************************************************************************
 */

/*
 ********************************************************************************
 *                           LOCAL FUNCTION DEFINITIONS
 ********************************************************************************
 */

/*
 ********************************************************************************
 *                           API FUNCTION DEFINITIONS
 ********************************************************************************
 */

void thrd_leader_network_data_init();

void thrd_leader_network_data_reset(void);

#endif /* EMB6_INC_NET_RIP_THRD_NETWORK_DATA_LEADER_H_ */
