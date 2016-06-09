/*
 * thrd-res-router.c
 *
 *  Created on: 7 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  CoAP Resource - Address Query /a/aq.
 */

#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

static void res_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

/**
 * Address Query CoAP Resource (/a/aq).
 */
RESOURCE(thrd_res_a_aq,
         "title=\"Address Query: POST\";rt=\"Text\"",
         NULL,
         res_post_handler,
         NULL,
         NULL);

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

static void
res_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
	/* TODO Process incoming payload.
	 * --> Call thrd_addr_ntf_response(uip_ipaddr_t *target_eid, uint16_t *rloc16,
	 * 				uint8_t *ml_eid_tlv, clock_time_t *last_trans_time);
	 */
	printf("res_post_handler: Receiving CoAP packet!\n");

	const uint8_t *chunk;
	if ( !response ) {

	} else {
		int payload_len = coap_get_payload(request, &chunk);
		if ( payload_len == 32 ) {
			// TODO Process payload -> Receipt of Address Query Messages.
		} else if ( payload_len == 38 ) {
			// TODO Process payload -> Receipt of Address Query Messages.
		}
		printf("res_post_handler: payload_len = %d\n", payload_len);

		for ( uint8_t i = 0; i < payload_len; i++ ) {
			PRINTF("chunk[%d] = %02x\n", i, chunk[i]);
		}
	}
}

/* --------------------------------------------------------------------------- */
