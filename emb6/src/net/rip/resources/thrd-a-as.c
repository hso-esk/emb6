/*
 * thrd-a-as.c
 *
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  CoAP Resource - Address Solicit Request /a/as.
 */

#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"
#include "tlv.h"
#include "net_tlv.h"
#include "uip.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"

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
static net_tlv_ml_eid_t *ml_eid_tlv_;
static net_tlv_rloc16_t *rloc16_tlv;

/**
 * Address Query CoAP Resource (/a/aq).
 */
RESOURCE(thrd_res_a_as,
         "title=\"Address Solicit Request: POST\";rt=\"Text\"",
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

	if ( (len = coap_get_payload(request, &chunk)) >= 20 ) {
		tlv = (tlv_t*) &chunk[0];
		if ( tlv->type == NET_TLV_ML_EID && tlv->length == 8 ) {
			ml_eid_tlv_ = (net_tlv_ml_eid_t*) tlv->value;
			PRINTF("ML-EID = ");
			print_ml_eid(ml_eid_tlv_);
		}
		if ( len == 24 ) {
			tlv = (tlv_t*) &chunk[10];
			if ( tlv->type == NET_TLV_RLOC16 && tlv->length == 2 ) {
				rloc16_tlv = (net_tlv_rloc16_t*) tlv->value;
				PRINTF("RLOC16 = %04x \n", rloc16_tlv->rloc16);
			}
		}
		// TODO Assign a Router ID to the REED if one is available.
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
