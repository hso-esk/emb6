/*
 * thrd-eid-rloc.c
 *
 *  Created on: 7 Jun 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *  EID-to-RLOC Map Cache.
 */

#include "emb6.h"
#include "thread_conf.h"
#include "stdlib.h"
#include "clist.h"
#include "memb.h"
#include "rip.h"

#include "thrd-eid-rloc.h"


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
LIST(eidRlocMapCache_list);
MEMB(eidRlocMapCache_memb, thrd_eid_rloc_cache_t, THRD_MAX_EID_RLOC_MAP_CACHE_SIZE);

/* Number of currently stored EID-to-RLOC entries in the EID-to-RLOC Map Cache. */
static uint8_t num_entries = 0;


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
thrd_eid_rloc_cache_init(void)
{
	memb_init(&eidRlocMapCache_memb);
	list_init(eidRlocMapCache_list);
}


uint8_t
thrd_eid_rloc_cache_num()
{
	return num_entries;
}

/* --------------------------------------------------------------------------- */

thrd_eid_rloc_cache_t
*thrd_eid_rloc_cache_head(void)
{
	return list_head(eidRlocMapCache_list);
}

/* --------------------------------------------------------------------------- */

thrd_eid_rloc_cache_t
*thrd_eid_rloc_cache_next(thrd_eid_rloc_cache_t *i)
{
	if ( i != NULL ) {
		thrd_eid_rloc_cache_t *n = list_item_next(i);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_eid_rloc_cache_t
*thrd_eid_rloc_cache_lookup(uip_ipaddr_t eid)
{
	thrd_eid_rloc_cache_t *entry;
	thrd_eid_rloc_cache_t *found_entry;

	PRINTF("thrd_eid_rloc_cache_lookup: Looking up EID-to-RLOC Map Cache for EID: ");
	PRINT6ADDR(&eid);
	PRINTF("\n\r");

	found_entry = NULL;
	for ( entry = thrd_eid_rloc_cache_head(); entry != NULL; entry = thrd_eid_rloc_cache_next(entry) ) {
		if ( uip_ipaddr_cmp(&eid, &(entry->EID)) ) {
			found_entry = entry;
			break;
		}
	}

	if ( found_entry != NULL ) {
		PRINTF("thrd_eid_rloc_cache_lookup: Found EID-to-RLOC entry: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_eid_rloc_cache_lookup: No EID-to-RLOC entry found.\n\r");
	}
	return found_entry;
}

/* --------------------------------------------------------------------------- */

thrd_eid_rloc_cache_t
*thrd_eid_rloc_cache_add(uip_ipaddr_t eid, uip_ipaddr_t rloc, clock_time_t age)
{
	thrd_eid_rloc_cache_t *entry;

	/* Find the corresponding Router ID entry (Router ID Set). */

	entry = thrd_eid_rloc_cache_lookup(eid);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if ( entry == NULL ) {
		PRINTF("thrd_eid_rloc_cache_add: Unknown EID: ");
		PRINT6ADDR(&eid);
		PRINTF("\n\r");

		/* If there is no router id entry, create one. We first need to
		 * check if we have room for this router id. If not, we remove the
		 * least recently used one we have. */

		if ( thrd_eid_rloc_cache_num() == THRD_MAX_EID_RLOC_MAP_CACHE_SIZE ) {
			/* Removing the oldest router id entry from the Router ID Set. The
			 * least recently used link is the first router id on the set. */
			thrd_eid_rloc_cache_t *oldest;

			oldest = list_tail(eidRlocMapCache_list);
			thrd_eid_rloc_cache_rm(oldest);
		}

		/* Allocate a router id entry and populate it. */
		entry = memb_alloc(&eidRlocMapCache_memb);

		if ( entry == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_eid_rloc_cache_add: Could not allocate EID-to-RLOC entry.\n");
			return NULL;
		}

		entry->EID = eid;
		entry->RLOC = rloc;
		entry->Age = age;

		/* Add new router id first - assuming that there is a reason to add this
		 * and that there is a packet coming soon. */
		list_push(eidRlocMapCache_list, entry);

		PRINTF("thrd_eid_rloc_cache_add: Added EID-to-RLOC entry:\n"
				"EID = ");
		PRINT6ADDR(&eid);
		PRINTF("\n"
				"RLOC = ");
		PRINT6ADDR(&rloc);
		PRINTF("\n"
				"AGE = ", age);
		PRINTF("\n\r");

		num_entries++;

		PRINTF("thrd_eid_rloc_cache_add: num_entries %d\n\r", num_entries);

	} else {

		PRINTF(ANSI_COLOR_RED "thrd_eid_rloc_cache_add: EID is already known for ");
		PRINTF("%d\n", eid);
		PRINTF(ANSI_COLOR_RESET "\n\r");

		PRINTF("thrd_eid_rloc_cache_add: num_entries %d\n\r", num_entries);
		PRINTF("-----------------------------------------------------\n\r");

		return NULL;
	}

	PRINTF("-----------------------------------------------------\n\r");

	return entry;
}

/* --------------------------------------------------------------------------- */

void
thrd_eid_rloc_cache_rm(thrd_eid_rloc_cache_t *entry)
{
	if ( entry != NULL ) {
		PRINTF("thrd_eid_rloc_cache_rm: Removing EID-to-RLOC entry from "
				"'EID-to-RLOC Map Cache' with EID: ");
		PRINT6ADDR(&entry->EID);
		PRINTF("\n\r");

		/* Remove the router id from the Router ID Set. */
		list_remove(eidRlocMapCache_list, entry);
		memb_free(&eidRlocMapCache_memb, entry);

		num_entries--;

		PRINTF("thrd_eid_rloc_cache_rm: num_entries %d\n\r", num_entries);
	}
}

/* --------------------------------------------------------------------------- */

void
thrd_eid_rloc_cache_empty()
{
	thrd_eid_rloc_cache_t *entry;
	thrd_eid_rloc_cache_t *entry_nxt;
	PRINTF("thrd_eid_rloc_cache_empty: Removing all (%d) assigned EID-to-RLOC "
			"entries from 'EID-to-RLOC Map Cache'.", num_entries);
	PRINTF("\n\r");
	entry = thrd_eid_rloc_cache_head();
	entry_nxt = entry;
	while ( entry_nxt != NULL ) {
		entry = entry_nxt;
		entry_nxt = entry->next;
		thrd_eid_rloc_cache_rm(entry);
	}
}

/* --------------------------------------------------------------------------- */
/* --------------------------------- DEBUG ----------------------------------- */
/* --------------------------------------------------------------------------- */

#if RIP_DEBUG
void
thrd_eid_rloc_cache_print()
{
	thrd_eid_rloc_cache_t *i;
	printf(ANSI_COLOR_RED
			"|=========================== EID-TO-RLOC MAP CACHE =============================|"
			"|============================== EID-to-RLOC-Set ================================|"
			ANSI_COLOR_RESET "\n\r");
	printf("---------------------------------------------------------------------------------\n");
	printf("|      EID      |     RLOC     |     Age     |\n");
	printf("---------------------------------------------------------------------------------\n\r");
	for (i = thrd_eid_rloc_cache_head(); i != NULL; i = thrd_eid_rloc_cache_next(i)) {
		printf("| ");
		PRINT6ADDR(&i->EID);
		printf(" | ");
		PRINT6ADDR(&i->RLOC);
		printf( " | " ANSI_COLOR_YELLOW "%10d" ANSI_COLOR_RESET
				" |\n", i->Age);
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
