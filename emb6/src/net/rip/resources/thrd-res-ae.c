/* --------------------------------------------------------------------------- */
/*
 * thrd-res-ae.c
 *
 *  Created on: 10 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  CoAP Resource - (Proactive) Address Notification /a/ae.
 */

#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"
#include "tlv.h"
#include "net_tlv.h"

#include "thrd-addr-query.h"
#include "thrd-eid-rloc.h"

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

static void res_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);

static void print_ml_eid(net_tlv_ml_eid_t *ml_eid_tlv);

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

static size_t len = 0;						// CoAP payload length.
static tlv_t *tlv;
static net_tlv_target_eid_t *target_eid_tlv;
static net_tlv_ml_eid_t *ml_eid_tlv;

/**
 * (Proactive) Address Notification CoAP Resource (/a/an).
 */
RESOURCE(thrd_res_a_ae,
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
	const uint8_t *chunk;
	PRINTF("========================== CoAP ==========================\n");
	PRINTF("Receiving CoAP packet (Res: a/ae)!\n");

	if ( (len = coap_get_payload(request, &chunk)) >= 32 ) {
		// TODO Process payload -> Receipt of Address Query Request.
		tlv = (tlv_t*) &chunk[0];
		if ( tlv->type == NET_TLV_TARGET_EID && tlv->length == 16 ) {
			target_eid_tlv = (net_tlv_target_eid_t*) tlv->value;
			PRINTF("Target EID = ");
			PRINT6ADDR(&target_eid_tlv->target_eid);
			PRINTF("\n");
		}
		tlv = (tlv_t*) &chunk[18];
		if ( tlv->type == NET_TLV_ML_EID && tlv->length == 8 ) {
			ml_eid_tlv = (net_tlv_ml_eid_t*) tlv->value;
			PRINTF("ML-EID = ");
			print_ml_eid(ml_eid_tlv);
		}
		// Check Target EID.
		thrd_local_addr_t *local_addr;
		// TODO Also lookup RFD Child's addresses.
		local_addr = thrd_local_addr_lookup(target_eid_tlv->target_eid);
		if ( local_addr != NULL ) {
			thrd_local_addr_rm(local_addr);
		}
	}
	PRINTF("==========================================================\n");
}

/* --------------------------------------------------------------------------- */

static void
print_ml_eid(net_tlv_ml_eid_t *ml_eid_tlv)
{
	for (uint8_t i = 0; i < 8; i++) {
		PRINTF("%02x", ml_eid_tlv->ml_eid[i]);
		if ( i < 7 && (i % 2) == 1 )
			PRINTF(":");
		if ( i == 7 )
			PRINTF("\n");
	}
}

