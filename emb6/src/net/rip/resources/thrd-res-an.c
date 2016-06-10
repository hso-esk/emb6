/*
 * thrd-res-an.c
 *
 *  Created on: 7 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  CoAP Resource - (Proactive) Address Notification /a/an.
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
static net_tlv_rloc16_t *rloc16_tlv;
static net_tlv_ml_eid_t *ml_eid_tlv;
static net_tlv_last_transaction_t *last_transaction_tlv;

/**
 * (Proactive) Address Notification CoAP Resource (/a/an).
 */
RESOURCE(thrd_res_a_an,
         "title=\"Address Notification: POST\";rt=\"Text\"",
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
	PRINTF("Receiving CoAP packet! (Res: a/an)\n");

	if ( (len = coap_get_payload(request, &chunk)) >= 32 ) {
		tlv = (tlv_t*) &chunk[0];
		if ( tlv->type == NET_TLV_TARGET_EID && tlv->length == 16 ) {
			target_eid_tlv = (net_tlv_target_eid_t*) tlv->value;
			PRINTF("Target EID = ");
			PRINT6ADDR(&target_eid_tlv->target_eid);
			PRINTF("\n");
		}
		tlv = (tlv_t*) &chunk[18];
		if ( tlv->type == NET_TLV_RLOC16 && tlv->length == 2 ) {
			rloc16_tlv = (net_tlv_rloc16_t*) tlv->value;
			PRINTF("RLOC16 = %04x \n", rloc16_tlv->rloc16);
		}
		tlv = (tlv_t*) &chunk[22];
		if ( tlv->type == NET_TLV_ML_EID && tlv->length == 8 ) {
			ml_eid_tlv = (net_tlv_ml_eid_t*) tlv->value;
			PRINTF("ML-EID = ");
			print_ml_eid(ml_eid_tlv);
		}
		if ( len == 38 ) {
			tlv = (tlv_t*) &chunk[32];
			if ( tlv->type == NET_TLV_LAST_TRANSACTION_TIME && tlv->length == 4 ) {
				last_transaction_tlv = (net_tlv_last_transaction_t*) tlv->value;
				PRINTF("Last Transaction Time = %08x\n", last_transaction_tlv->last_transaction_time);
			}
		}
		// Update EID-to-RLOC Set (Map Cache).
		uip_ipaddr_t rloc_addr;
		THRD_CREATE_RLOC_ADDR(&rloc_addr, rloc16_tlv->rloc16);
		thrd_eid_rloc_cache_update(target_eid_tlv->target_eid, rloc_addr);
		thrd_eid_rloc_cache_print();
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

/* --------------------------------------------------------------------------- */
