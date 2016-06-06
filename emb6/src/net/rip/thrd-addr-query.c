/*
 * thrd-addr-query.c
 *
 *  Created on: 6 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  Thread Address Query.
 */

#include "emb6.h"
#include "thread_conf.h"
#include "stdlib.h"
#include "clist.h"
#include "memb.h"
#include "rip.h"

#include "thrd-addr-query.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"

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

/**
 * Local Address Set
 */
LIST(localAddrSet_list);
MEMB(localAddrSet_memb, thrd_local_addr_t, MAX_ROUTERS);

/**
 * Address Set.
 */
LIST(rfdChildAddrSet_list);
MEMB(rfdChildAddrSet_memb, thrd_rfd_addr_t, MAX_ROUTERS);

/**
 * Address Query Set.
 */
LIST(addrQuerySet_list);
MEMB(addrQuerySet_memb, thrd_addr_qr_t, MAX_ROUTERS);

/* Number of currently stored EIDs in the Local Address Set. */
static uint8_t num_eids = 0;
static uint8_t num_rfd_addr = 0;
static uint8_t num_addr_qr = 0;

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

/* --------------------------------------------------------------------------- */

void
thrd_eid_rloc_db_init(void)
{
	memb_init(&localAddrSet_memb);
	list_init(localAddrSet_list);

	memb_init(&rfdChildAddrSet_memb);
	list_init(rfdChildAddrSet_list);

	memb_init(&addrQuerySet_memb);
	list_init(addrQuerySet_list);
}

/* --------------------------------------------------------------------------- */
/* --------------------------- Local Address Set ----------------------------- */
/* --------------------------------------------------------------------------- */

uint8_t
thrd_local_addr_num()
{
	return num_eids;
}

/* --------------------------------------------------------------------------- */

thrd_local_addr_t
*thrd_local_addr_head(void)
{
	return list_head(localAddrSet_list);
}

/* --------------------------------------------------------------------------- */

thrd_local_addr_t
*thrd_local_addr_next(thrd_local_addr_t *i)
{
	if ( i != NULL ) {
		thrd_local_addr_t *n = list_item_next(i);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_local_addr_t
*thrd_local_addr_lookup(uip_ipaddr_t eid)
{
	thrd_local_addr_t *localAddr;
	thrd_local_addr_t *found_localAddr;

	PRINTF("thrd_local_addr_lookup: Looking up EID: ");
	PRINT6ADDR(&eid);
	PRINTF("\n\r");

	found_localAddr = NULL;
	for ( localAddr = thrd_local_addr_head(); localAddr != NULL; localAddr = thrd_local_addr_next(localAddr) ) {
		if ( uip_ipaddr_cmp(&eid, &(localAddr->eid)) ) {
			found_localAddr = localAddr;
			break;
		}
	}

	if ( found_localAddr != NULL ) {
		PRINTF("thrd_local_addr_lookup: Found EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_local_addr_lookup: No EID found.\n\r");
	}
	return found_localAddr;
}

/* --------------------------------------------------------------------------- */

thrd_local_addr_t
*thrd_local_addr_add(uip_ipaddr_t eid)
{
	thrd_local_addr_t *localAddr;

	/* Find the corresponding Router ID entry (Router ID Set). */

	localAddr = thrd_local_addr_lookup(eid);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if ( localAddr == NULL ) {
		PRINTF("thrd_local_addr_add: Unknown EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");

		/* If there is no router id entry, create one. We first need to
		 * check if we have room for this router id. If not, we remove the
		 * least recently used one we have. */

		if ( thrd_local_addr_num() == THRD_MAX_LOCAL_ADDRESSES ) {
			/* Removing the oldest router id entry from the Router ID Set. The
			 * least recently used link is the first router id on the set. */
			thrd_local_addr_t *oldest;

			oldest = list_tail(localAddrSet_list);
			thrd_local_addr_rm(oldest);
		}

		/* Allocate a router id entry and populate it. */
		localAddr = memb_alloc(&localAddrSet_memb);

		if ( localAddr == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_local_addr_add: Could not allocate EID.\n");
			return NULL;
		}

		localAddr->eid = eid;

		/* Add new router id first - assuming that there is a reason to add this
		 * and that there is a packet coming soon. */
		list_push(localAddrSet_list, localAddr);

		PRINTF("thrd_local_addr_add: Added EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");

		num_eids++;

		PRINTF("thrd_local_addr_add: num_eids %d\n\r", num_eids);

	} else {

		PRINTF(ANSI_COLOR_RED "thrd_local_addr_add: EID is already known for ");
		PRINTF("%d\n", eid);
		PRINTF(ANSI_COLOR_RESET "\n\r");

		PRINTF("thrd_local_addr_add: num_eids %d\n\r", num_eids);
		PRINTF("-----------------------------------------------------\n\r");

		return NULL;
	}

	PRINTF("-----------------------------------------------------\n\r");

	return localAddr;
}

/* --------------------------------------------------------------------------- */

void
thrd_local_addr_rm(thrd_local_addr_t *localAddr)
{
	if ( localAddr != NULL ) {
		PRINTF("thrd_local_addr_rm: Removing EID from 'Local Address Set' with EID: ");
		PRINT6ADDR(&localAddr->eid);
		PRINTF("\n\r");

		/* Remove the router id from the Router ID Set. */
		list_remove(localAddrSet_list, localAddr);
		memb_free(&localAddrSet_memb, localAddr);

		num_eids--;

		PRINTF("thrd_local_addr_rm: num_eids %d\n\r", num_eids);
	}
}

/* --------------------------------------------------------------------------- */

void
thrd_local_addr_empty()
{
	thrd_local_addr_t *localAddr;
	thrd_local_addr_t *localAddr_nxt;
	PRINTF("thrd_local_addr_empty: Removing all (%d) assigned EIDs from 'Local Address Set'.", num_eids);
	PRINTF("\n\r");
	localAddr = thrd_local_addr_head();
	localAddr_nxt = localAddr;
	while ( localAddr_nxt != NULL ) {
		localAddr = localAddr_nxt;
		localAddr_nxt = localAddr->next;
		thrd_local_addr_rm(localAddr);
	}
}

/* --------------------------------------------------------------------------- */
/* ------------------------- RFD Child Address Set --------------------------- */
/* --------------------------------------------------------------------------- */

uint8_t
thrd_rfd_child_addr_num()
{
	return num_rfd_addr;
}

/* --------------------------------------------------------------------------- */

thrd_rfd_addr_t
*thrd_rfd_child_addr_head(void)
{
	return list_head(rfdChildAddrSet_list);
}

/* --------------------------------------------------------------------------- */

thrd_rfd_addr_t
*thrd_rfd_child_addr_next(thrd_rfd_addr_t *i)
{
	if ( i != NULL ) {
		thrd_rfd_addr_t *n = list_item_next(i);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_rfd_addr_t
*thrd_rfd_child_addr_lookup(uip_ipaddr_t child_addr)
{
	thrd_rfd_addr_t *childAddr;
	thrd_rfd_addr_t *found_childAddr;

	PRINTF("thrd_rfd_child_addr_lookup: Looking up EID: ");
	PRINT6ADDR(&child_addr);
	PRINTF("\n\r");

	found_childAddr = NULL;
	for ( childAddr = thrd_rfd_child_addr_head(); childAddr != NULL; childAddr = thrd_rfd_child_addr_next(childAddr) ) {
		if ( uip_ipaddr_cmp(&child_addr, &(childAddr->childAddr)) ) {
			found_childAddr = childAddr;
			break;
		}
	}

	if ( found_childAddr != NULL ) {
		PRINTF("thrd_rfd_child_addr_lookup: Found RFD Child Address: ");
		PRINT6ADDR(&child_addr);
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_rfd_child_addr_lookup: No RFD Child Address found.\n\r");
	}
	return found_childAddr;
}

/* --------------------------------------------------------------------------- */

thrd_rfd_addr_t
*thrd_rfd_child_addr_add(uip_ipaddr_t child_addr)
{
	thrd_rfd_addr_t *childAddr;

	/* Find the corresponding Router ID entry (Router ID Set). */

	childAddr = thrd_rfd_child_addr_lookup(child_addr);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if ( childAddr == NULL ) {
		PRINTF("thrd_rfd_child_addr_add: Unknown RFD Child Address: ");
		PRINT6ADDR(&child_addr);
		PRINTF("\n\r");

		/* If there is no router id entry, create one. We first need to
		 * check if we have room for this router id. If not, we remove the
		 * least recently used one we have. */

		if ( thrd_rfd_child_addr_num() == THRD_MAX_RFD_CHILD_ADDRESSES ) {
			/* Removing the oldest router id entry from the Router ID Set. The
			 * least recently used link is the first router id on the set. */
			thrd_rfd_addr_t *oldest;

			oldest = list_tail(rfdChildAddrSet_list);
			thrd_rfd_child_addr_rm(oldest);
		}

		/* Allocate a router id entry and populate it. */
		childAddr = memb_alloc(&rfdChildAddrSet_memb);

		if ( childAddr == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_rfd_child_addr_add: Could not allocate RFD Child Address.\n");
			return NULL;
		}

		childAddr->childAddr = child_addr;

		/* Add new router id first - assuming that there is a reason to add this
		 * and that there is a packet coming soon. */
		list_push(rfdChildAddrSet_list, childAddr);

		PRINTF("thrd_rfd_child_addr_add: Added RFD Child Address ");
		PRINT6ADDR(&child_addr);
		PRINTF("\n\r");

		num_rfd_addr++;

		PRINTF("thrd_rfd_child_addr_add: num_rfd_addr %d\n\r", num_rfd_addr);

	} else {

		PRINTF(ANSI_COLOR_RED "thrd_rfd_child_addr_add: RFD Child Address is already known for ");
		PRINT6ADDR(&child_addr);
		PRINTF(ANSI_COLOR_RESET "\n\r");

		PRINTF("thrd_rfd_child_addr_add: num_rfd_addr %d\n\r", num_rfd_addr);
		PRINTF("-----------------------------------------------------\n\r");

		return NULL;
	}

	PRINTF("-----------------------------------------------------\n\r");

	return childAddr;
}

/* --------------------------------------------------------------------------- */

void
thrd_rfd_child_addr_rm(thrd_rfd_addr_t *child_addr)
{
	if ( child_addr != NULL ) {
		PRINTF("thrd_rfd_child_addr_rm: Removing RFD Child Address from 'RFD Child Address Set' with EID: ");
		PRINT6ADDR(&child_addr->childAddr);
		PRINTF("\n\r");

		/* Remove the router id from the Router ID Set. */
		list_remove(rfdChildAddrSet_list, child_addr);
		memb_free(&rfdChildAddrSet_memb, child_addr);

		num_rfd_addr--;

		PRINTF("thrd_rfd_child_addr_rm: num_rfd_addr %d\n\r", num_rfd_addr);
	}
}

/* --------------------------------------------------------------------------- */

void
thrd_rfd_child_addr_empty()
{
	thrd_rfd_addr_t *childAddr;
	thrd_rfd_addr_t *childAddr_nxt;
	PRINTF("thrd_rfd_child_addr_empty: Removing all (%d) assigned EIDs from 'RFD Child Address Set'.", num_rfd_addr);
	PRINTF("\n\r");
	childAddr = thrd_rfd_child_addr_head();
	childAddr_nxt = childAddr;
	while ( childAddr_nxt != NULL ) {
		childAddr = childAddr_nxt;
		childAddr_nxt = childAddr->next;
		thrd_rfd_child_addr_rm(childAddr);
	}
}

/* --------------------------------------------------------------------------- */
/* --------------------------- Address Query Set ----------------------------- */
/* --------------------------------------------------------------------------- */

uint8_t
thrd_addr_qr_num()
{
	return num_addr_qr;
}

/* --------------------------------------------------------------------------- */

thrd_addr_qr_t
*thrd_addr_qr_head(void)
{
	return list_head(addrQuerySet_list);
}

/* --------------------------------------------------------------------------- */

thrd_addr_qr_t
*thrd_addr_qr_next(thrd_addr_qr_t *i)
{
	if ( i != NULL ) {
		thrd_addr_qr_t *n = list_item_next(i);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_addr_qr_t
*thrd_addr_qr_lookup(uip_ipaddr_t eid)
{
	thrd_addr_qr_t *addrQr;
	thrd_addr_qr_t *found_addrQr;

	PRINTF("thrd_addr_qr_lookup: Looking up Address Query for EID: ");
	PRINT6ADDR(&eid);
	PRINTF("\n\r");

	found_addrQr = NULL;
	for ( addrQr = thrd_addr_qr_head(); addrQr != NULL; addrQr = thrd_addr_qr_next(addrQr) ) {
		if ( uip_ipaddr_cmp(&eid, &(addrQr->EID)) ) {
			found_addrQr = addrQr;
			break;
		}
	}

	if ( found_addrQr != NULL ) {
		PRINTF("thrd_addr_qr_lookup: Found Address Query for EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_addr_qr_lookup: No Address Query for EID found.\n\r");
	}
	return found_addrQr;
}

/* --------------------------------------------------------------------------- */

thrd_addr_qr_t
*thrd_addr_qr_add(uip_ipaddr_t eid, clock_time_t timeout, clock_time_t retry_delay)
{
	thrd_addr_qr_t *addrQr;

	/* Find the corresponding Router ID entry (Router ID Set). */

	addrQr = thrd_addr_qr_lookup(eid);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if ( addrQr == NULL ) {
		PRINTF("thrd_addr_qr_add: Unknown Address Query for EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");

		/* If there is no router id entry, create one. We first need to
		 * check if we have room for this router id. If not, we remove the
		 * least recently used one we have. */

		if ( thrd_addr_qr_num() == THRD_MAX_ADDRESS_QUERIES ) {
			/* Removing the oldest router id entry from the Router ID Set. The
			 * least recently used link is the first router id on the set. */
			thrd_addr_qr_t *oldest;

			oldest = list_tail(addrQuerySet_list);
			thrd_addr_qr_rm(oldest);
		}

		/* Allocate a router id entry and populate it. */
		addrQr = memb_alloc(&addrQuerySet_memb);

		if ( addrQr == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_addr_qr_add: Could not allocate Address Query.\n");
			return NULL;
		}

		addrQr->EID = eid;
		addrQr->AQ_Timeout = timeout;
		addrQr->AQ_Failures = 0;
		addrQr->AQ_Retry_Delay = retry_delay;

		/* Add new router id first - assuming that there is a reason to add this
		 * and that there is a packet coming soon. */
		list_push(addrQuerySet_list, addrQr);

		PRINTF("thrd_addr_qr_add: Added Address Query for EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");

		num_addr_qr++;

		PRINTF("thrd_addr_qr_add: num_addr_qr %d\n\r", num_addr_qr);

	} else {

		PRINTF(ANSI_COLOR_RED "thrd_addr_qr_add: Address Query is already known for EID ");
		PRINTF("%d\n", eid);
		PRINTF(ANSI_COLOR_RESET "\n\r");

		PRINTF("thrd_addr_qr_add: num_addr_qr %d\n\r", num_addr_qr);
		PRINTF("-----------------------------------------------------\n\r");

		return NULL;
	}

	PRINTF("-----------------------------------------------------\n\r");

	return addrQr;
}

/* --------------------------------------------------------------------------- */

void
thrd_addr_qr_rm(thrd_addr_qr_t *addrQr)
{
	if ( addrQr != NULL ) {
		PRINTF("thrd_local_addr_rm: Removing Address Query from 'Address Query Set' with EID: ");
		PRINT6ADDR(&addrQr->EID);
		PRINTF("\n\r");

		/* Remove the router id from the Router ID Set. */
		list_remove(addrQuerySet_list, addrQr);
		memb_free(&addrQuerySet_memb, addrQr);

		num_addr_qr--;

		PRINTF("thrd_addr_qr_rm: num_addr_qr %d\n\r", num_addr_qr);
	}
}

/* --------------------------------------------------------------------------- */

void
thrd_addr_qr_empty()
{
	thrd_addr_qr_t *addrQr;
	thrd_addr_qr_t *addrQr_nxt;
	PRINTF("thrd_addr_qr_empty: Removing all (%d) assigned Address Queries from 'Address Query Set'.", num_addr_qr);
	PRINTF("\n\r");
	addrQr = thrd_addr_qr_head();
	addrQr_nxt = addrQr;
	while ( addrQr_nxt != NULL ) {
		addrQr = addrQr_nxt;
		addrQr_nxt = addrQr->next;
		thrd_addr_qr_rm(addrQr);
	}
}

/* --------------------------------------------------------------------------- */
/* ----------------------------- Address Query ------------------------------- */
/* --------------------------------------------------------------------------- */

// TODO Section 5.4.2 Address Query.

void
thrd_addr_qr_request(tlv_target_eid_t)
{

}

/* --------------------------------------------------------------------------- */
/* --------------------------------- DEBUG ----------------------------------- */
/* --------------------------------------------------------------------------- */

#if RIP_DEBUG
void
thrd_local_addr_set_print(void)
{
	thrd_local_addr_t *i;
	printf(ANSI_COLOR_RED
			"|============================= LOCAL ADDRESS SET ===============================|"
			ANSI_COLOR_RESET "\n\r");
	printf("-------------------------\n");
	printf("|          EID          |\n");
	printf("-------------------------\n\r");
	for (i = thrd_local_addr_head(); i != NULL; i = thrd_local_addr_next(i)) {
		// printf("| " ANSI_COLOR_YELLOW "%21d" ANSI_COLOR_RESET " |\n", PRINT6ADDR(&i->eid));
		printf("| " ANSI_COLOR_YELLOW);
		PRINT6ADDR(&i->eid);
		printf(ANSI_COLOR_RESET " |\n");
	}
	printf("-------------------------\n\r");
	printf(ANSI_COLOR_RED
			"|===============================================================================|"
			ANSI_COLOR_RESET "\n\r");
}

void
thrd_rfd_child_addr_set_print()
{
	thrd_rfd_addr_t *i;
	printf(ANSI_COLOR_RED
			"|=========================== RFD Child Address Set =============================|"
			ANSI_COLOR_RESET "\n\r");
	printf("-------------------------\n");
	printf("|          EID          |\n");
	printf("-------------------------\n\r");
	for (i = thrd_rfd_child_addr_head(); i != NULL; i = thrd_rfd_child_addr_next(i)) {
		printf("| " ANSI_COLOR_YELLOW);
		PRINT6ADDR(&i->childAddr);
		printf(ANSI_COLOR_RESET " |\n");
	}
	printf("-------------------------\n\r");
	printf(ANSI_COLOR_RED
			"|===============================================================================|"
			ANSI_COLOR_RESET "\n\r");
}

void
thrd_addr_qr_set_print()
{
	thrd_addr_qr_t *i;
	printf(ANSI_COLOR_RED
			"|============================= ADDRESS QUERY SET ===============================|"
			ANSI_COLOR_RESET "\n\r");
	printf("---------------------------------------------------------------------------------\n");
	printf("|      EID      | AQ_Timeout | AQ_Failures | AQ_Retry_Delay |\n");
	printf("---------------------------------------------------------------------------------\n\r");
	for (i = thrd_addr_qr_head(); i != NULL; i = thrd_addr_qr_next(i)) {
		printf("| ");
		PRINT6ADDR(&i->EID);
		printf( " | " ANSI_COLOR_YELLOW "%10d" ANSI_COLOR_RESET
				" | " ANSI_COLOR_YELLOW "%11d" ANSI_COLOR_RESET
				" | " ANSI_COLOR_YELLOW "%14d" ANSI_COLOR_RESET
				" |\n", i->AQ_Timeout, i->AQ_Failures, i->AQ_Retry_Delay);
	}
	printf("---------------------------------------------------------------------------------\n\r");
	printf(ANSI_COLOR_RED
			"|===============================================================================|"
			ANSI_COLOR_RESET "\n\r");
}
#endif /* RIP_DEBUG */

/*
 ********************************************************************************
 *                               END OF FILE
 ********************************************************************************
 */
