/*
 * thrd-leader-db.c
 *
 *  Created on: 26 Apr 2016
 *  Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 *  Leader database manipulation.
 */

#include "emb6.h"
#include "stdlib.h"
#include "clist.h"
#include "memb.h"
#include "thread_conf.h"

#include "thrd-leader-db.h"

// #define DEBUG DEBUG_PRINT
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

/* Each 'ID Assignment Set' is represented by a thrd_ldb_id_assign structure
 * and memory for each entry is allocated from the idassign_memb memory block.
 * These routes are maintained on the idassign_list. */
LIST(idassign_list);
MEMB(idassign_memb, thrd_ldb_ida_t, MAX_ROUTERS);

/* Number of currently stored id assignments ids in the ID Assignment Set. */
static size_t num_ida = 0;

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
thrd_ldb_init(void)
{
	memb_init(&idassign_memb);
	list_init(idassign_list);
}

/* --------------------------------------------------------------------------- */

size_t
thrd_ldb_num_ida(void)
{
	return num_ida;
}

/* --------------------------------------------------------------------------- */

thrd_ldb_ida_t
*thrd_ldb_ida_head(void)
{
	return list_head(idassign_list);
}

/* --------------------------------------------------------------------------- */

thrd_ldb_ida_t
*thrd_ldb_ida_next(thrd_ldb_ida_t *i)
{
	if (i != NULL) {
		thrd_ldb_ida_t *n = list_item_next(i);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

thrd_ldb_ida_t
*thrd_ldb_ida_lookup(uint8_t router_id)
{
	thrd_ldb_ida_t *ida;
	thrd_ldb_ida_t *found_ida;

	PRINTF("thrd_ldb_ida_lookup: Looking up ID Assignment Set for router id ");
	PRINTF("%d\n", router_id);
	PRINTF("\n\r");

	found_ida = NULL;
	for (ida = thrd_ldb_ida_head(); ida != NULL; ida = thrd_ldb_ida_next(ida)) {
		PRINTF("%d\n", ida->ID_id);
		PRINTF("\n\r");
		if (ida->ID_id == router_id) {
			found_ida = ida;
			break;
		}
	}

	if (found_ida != NULL) {
		PRINTF("thrd_ldb_ida_lookup: Found router id: ");
		PRINTF("%d\n", router_id);
		PRINTF("\n\r");
	} else {
		PRINTF("thrd_ldb_ida_lookup: No router id found\n\r");
	}

	if (found_ida != NULL && found_ida != list_head(idassign_list)) {
		/* If we found the router id, we put it at the start of the idassign_list
		 list. The list is ordered by how recently we looked them up:
		 the least recently used router id will be at the end of the
		 list - for fast lookups (assuming multiple packets to the same node). */
		list_remove(idassign_list, found_ida);
		list_push(idassign_list, found_ida);
	}

	return found_ida;
}

/* --------------------------------------------------------------------------- */

thrd_ldb_ida_t
*thrd_ldb_ida_add(uint8_t router_id, uint8_t *owner, clock_time_t reuse_time)
{
	thrd_ldb_ida_t *ida;

	/* Find the corresponding Router ID entry (Router ID Set). */

	ida = thrd_ldb_ida_lookup(router_id);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if (ida == NULL) {
		PRINTF("thrd_ldb_ida_add: router id unknown for ");
		PRINTF("%d\n", router_id);
		PRINTF("\n\r");

		/* If there is no router id entry, create one. We first need to
		 * check if we have room for this router id. If not, we remove the
		 * least recently used one we have. */

		if (thrd_ldb_num_ida() == MAX_ROUTER_ID) {
			/* Removing the oldest router id entry from the Router ID Set. The
			 * least recently used link is the first router id on the set. */
			thrd_ldb_ida_t *oldest;

			oldest = list_tail(idassign_list);
			thrd_ldb_ida_rm(oldest);
		}

		/* Allocate a router id entry and populate it. */
		ida = memb_alloc(&idassign_memb);

		if (ida == NULL) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_ldb_ida_add: could not allocate router id\n");
			return NULL;
		}

		ida->ID_id = router_id;
		memcpy(&ida->ID_owner, owner, 8);

		/* Add new router id first - assuming that there is a reason to add this
		 * and that there is a packet coming soon. */
		list_push(idassign_list, ida);

		PRINTF("thrd_ldb_ida_add: Added router id %d\n\r", router_id);

		num_ida++;

		PRINTF("thrd_ldb_ida_add: num_ida %d\n\r", num_ida);

	} else {

		PRINTF(
				ANSI_COLOR_RED "thrd_ldb_ida_add: router id is already known for ");
		PRINTF("%d\n", router_id);
		PRINTF(ANSI_COLOR_RESET "\n\r");

		PRINTF("thrd_ldb_idassign_add: num_ida %d\n\r", num_ida);
		PRINTF("-----------------------------------------------------\n\r");

		return NULL;
	}

	PRINTF("-----------------------------------------------------\n\r");

	return ida;
}

/* --------------------------------------------------------------------------- */

void
thrd_ldb_ida_rm(thrd_ldb_ida_t *ida)
{
	if (ida != NULL) {
		PRINTF("thrd_ldb_idassign_rm: removing router id from 'Router ID Set' with router id: ");
		PRINTF("%d\n", ida->ID_id);
		PRINTF("\n\r");

		/* Remove the router id from the Router ID Set. */
		list_remove(idassign_list, ida);
		memb_free(&idassign_memb, ida);

		num_ida--;

		PRINTF("thrd_rdb_rid_rm: num_ida %d\n\r", num_ida);
	}
}

/* --------------------------------------------------------------------------- */

void
thrd_ldb_ida_empty()
{
	thrd_ldb_ida_t *ida;
	thrd_ldb_ida_t *ida_nxt;
	PRINTF("thrd_rdb_ida_empty: removing all (%d) assigned router ids from 'ID Assignment Set'.", num_ida);
	PRINTF("\n\r");
	ida = thrd_ldb_ida_head();
	ida_nxt = ida;
	while ( ida_nxt != NULL ) {
		ida = ida_nxt;
		ida_nxt = ida->next;
		thrd_ldb_ida_rm(ida);
	}
}

/* --------------------------------------------------------------------------- */

#if RIP_DEBUG
void
thrd_ldb_print_leader_database(void)
{
	thrd_ldb_ida_t *i;
	printf(ANSI_COLOR_RED
			"|============================== LEADER DATABASE ================================|"
			ANSI_COLOR_RESET "\n\r");
	printf("---------------- ID ASSIGNMENT SET ----------------\n");
	printf("| ID_id |        ID_owner         | ID_reuse_time |\n");
	printf("---------------------------------------------------\n\r");
	for (i = thrd_ldb_ida_head(); i != NULL; i = thrd_ldb_ida_next(i)) {
		printf("| " ANSI_COLOR_YELLOW "%5d" ANSI_COLOR_RESET " ", i->ID_id);
		printf("| " ANSI_COLOR_YELLOW "%02x %02x %02x %02x %02x %02x %02x %02x" ANSI_COLOR_RESET " ",
				i->ID_owner[0], i->ID_owner[1], i->ID_owner[2], i->ID_owner[3],
				i->ID_owner[4],i->ID_owner[5], i->ID_owner[6], i->ID_owner[7]);
		printf("| " ANSI_COLOR_YELLOW "%13d" ANSI_COLOR_RESET " |\n", i->ID_reuse_time);
	}
	printf("---------------------------------------------------\n\r");
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
