/**
 * \file thrd-network-data-leader.c
 * \author Institute for reliable Embedded Systems and Communication Electronics
 * \date 2016/07/13
 * \version 1.0
 *
 * \brief Thread network data managed by the Thread leader
 */

/*
 ********************************************************************************
 *                                   INCLUDES
 ********************************************************************************
 */

#include "emb6.h"
#include "stdlib.h"
#include "bsp.h"
#include "clist.h"
#include "memb.h"
#include "rip.h"
#include "thread_conf.h"
#include "er-coap.h"
#include "er-coap-engine.h"
#include "rest-engine.h"
#include "thrd-network-data.h"
#include "thrd-network-data-leader.h"

#define     LOGGER_ENABLE                 LOGGER_THRD_NET
#include    "logger.h"


/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

static void coap_init();

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
 * CoAP Resources to be activated need to be imported through the extern keyword.
 */
extern resource_t thrd_res_n_sd;

/*
 ********************************************************************************
 *                           LOCAL FUNCTION DEFINITIONS
 ********************************************************************************
 */

static void
coap_init()
{
	LOG_RAW("Starting Thread Network Data (CoAP).\n\r");
	// Bind the resources to their Uri-Path.
	rest_activate_resource(&thrd_res_n_sd, "n/sd");
}

/* --------------------------------------------------------------------------- */

/*
 ********************************************************************************
 *                           API FUNCTION DEFINITIONS
 ********************************************************************************
 */

void
thrd_leader_network_data_init()
{
	coap_init();
}

/* --------------------------------------------------------------------------- */

void
thrd_leader_network_data_reset(void)
{

}

/* --------------------------------------------------------------------------- */

/*
 ********************************************************************************
 *                               END OF FILE
 ********************************************************************************
 */
