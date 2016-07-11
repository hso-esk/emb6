/*
 * thrd-network-data.c
 *
 * Created on: 11 Jul 2016
 * Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 *
 * Thread Network Data.
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

#include "thrd-network-data.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"

/*
 ********************************************************************************
 *                          LOCAL FUNCTION DECLARATIONS
 ********************************************************************************
 */

static uint8_t thrd_get_num_on_mesh_prefixes(void);
static uint8_t thrd_get_num_ext_routes(void);
static uint8_t thrd_get_num_sicslowpan_ctx_ids(void);
static uint8_t thrd_get_num_servers(void);

static thrd_on_mesh_prefix_set_t* thrd_on_mesh_prefix_set_head(void);
static thrd_ext_route_set_t* thrd_ext_route_set_head(void);
static thrd_sicslowpan_ctx_id_set_t* thrd_sicslowpan_ctx_id_set_head(void);
static thrd_server_set_t* thrd_server_set_head(void);

static thrd_on_mesh_prefix_set_t* thrd_on_mesh_prefix_set_next(thrd_on_mesh_prefix_set_t *r);
static thrd_ext_route_set_t* thrd_ext_route_set_next(thrd_ext_route_set_t *r);
static thrd_sicslowpan_ctx_id_set_t* thrd_sicslowpan_ctx_id_set_next(thrd_sicslowpan_ctx_id_set_t *r);
static thrd_server_set_t* thrd_server_set_next(thrd_server_set_t *r);

static thrd_on_mesh_prefix_set_t* thrd_on_mesh_prefix_set_lookup(uint8_t *on_mesh_prefix);
static thrd_ext_route_set_t* thrd_ext_route_set_lookup(uint8_t domain_id);
static thrd_sicslowpan_ctx_id_set_t* thrd_sicslowpan_ctx_id_set_lookup(uint8_t sicslowpan_ctx_id);
static thrd_server_set_t* thrd_server_set_lookup(uint16_t enterprise_number);

static thrd_on_mesh_prefix_set_t* thrd_on_mesh_prefix_set_add(uint8_t *prefix, uint8_t domain_id,
		uint16_t border_router_16, bool stable, bool slaac_preferred, bool slaac_valid, bool dhcp,
		bool configure, bool def, bool preference);

static void thrd_on_mesh_prefix_set_rm(thrd_on_mesh_prefix_set_t *pref);

/*
 ********************************************************************************
 *                               LOCAL VARIABLES
 ********************************************************************************
 */

/**
 * On-Mesh Prefix Set memory allocation.
 */
LIST(on_mesh_prefix_list);
MEMB(on_mesh_prefix_memb, thrd_on_mesh_prefix_set_t, THRD_MAX_ON_MESH_PREFIX_SET_ENTRIES);

/**
 * External Route Set memory allocation.
 */
LIST(ext_route_list);
MEMB(ext_route_memb, thrd_ext_route_set_t, THRD_MAX_EXT_ROUTE_SET_ENTRIES);

/**
 * 6LoWPAN Context ID Set memory allocation.
 */
LIST(sicslowpan_ctx_id_list);
MEMB(sicslowpan_ctx_id_memb, thrd_sicslowpan_ctx_id_set_t, THRD_MAX_SICSLOWPAN_CTX_ID_SET_ENTRIES);

/**
 * Server Set memory allocation.
 */
LIST(server_list);
MEMB(server_memb, thrd_server_set_t, THRD_MAX_SERVER_SET_ENTIES);

/* Number of currently stored On-Mesh Prefixes in the On-Mesh Prefix Set. */
static uint8_t num_on_mesh_prefixes = 0;
/* Number of currently stored External Routers in the External Route Set. */
static uint8_t num_ext_routes = 0;
/* Number of currently stored 6LoWPAN Context IDs in the 6LoWPAN Context ID Set. */
static uint8_t num_sicslowpan_ctx_ids = 0;
/* Number of currently stored Servers in the Server Set. */
static uint8_t num_servers = 0;

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

void
thrd_leader_network_data_init(void)
{
	memb_init(&on_mesh_prefix_memb);
	list_init(on_mesh_prefix_list);

	memb_init(&ext_route_memb);
	list_init(ext_route_list);

	memb_init(&sicslowpan_ctx_id_memb);
	list_init(sicslowpan_ctx_id_list);

	memb_init(&server_memb);
	list_init(server_list);
}

/* --------------------------------------------------------------------------- */

static uint8_t
thrd_get_num_on_mesh_prefixes(void)
{
	return num_on_mesh_prefixes;
}

/* --------------------------------------------------------------------------- */

static uint8_t
thrd_get_num_ext_routes(void)
{
	return num_ext_routes;
}

/* --------------------------------------------------------------------------- */

static uint8_t
thrd_get_num_sicslowpan_ctx_ids(void)
{
	return num_sicslowpan_ctx_ids;
}

/* --------------------------------------------------------------------------- */

static uint8_t
thrd_get_num_servers(void)
{
	return num_servers;
}

/* --------------------------------------------------------------------------- */

static thrd_on_mesh_prefix_set_t*
thrd_on_mesh_prefix_set_head(void)
{
	return list_head(on_mesh_prefix_list);
}

/* --------------------------------------------------------------------------- */

static thrd_ext_route_set_t
*thrd_ext_route_set_head(void)
{
	return list_head(ext_route_list);
}

/* --------------------------------------------------------------------------- */

static thrd_sicslowpan_ctx_id_set_t*
thrd_sicslowpan_ctx_id_set_head(void)
{
	return list_head(sicslowpan_ctx_id_list);
}

/* --------------------------------------------------------------------------- */

static thrd_server_set_t*
thrd_server_set_head(void)
{
	return list_head(server_list);
}

/* --------------------------------------------------------------------------- */

static thrd_on_mesh_prefix_set_t*
thrd_on_mesh_prefix_set_next(thrd_on_mesh_prefix_set_t *r)
{
	if ( r != NULL ) {
		thrd_on_mesh_prefix_set_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

static thrd_ext_route_set_t*
thrd_ext_route_set_next(thrd_ext_route_set_t *r)
{
	if ( r != NULL ) {
		thrd_ext_route_set_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

static thrd_sicslowpan_ctx_id_set_t*
thrd_sicslowpan_ctx_id_set_next(thrd_sicslowpan_ctx_id_set_t *r)
{
	if ( r != NULL ) {
		thrd_sicslowpan_ctx_id_set_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

static thrd_server_set_t*
thrd_server_set_next(thrd_server_set_t *r)
{
	if ( r != NULL ) {
		thrd_server_set_t *n = list_item_next(r);
		return n;
	}
	return NULL;
}

/* --------------------------------------------------------------------------- */

static thrd_on_mesh_prefix_set_t*
thrd_on_mesh_prefix_set_lookup(uint8_t *on_mesh_prefix)
{
	thrd_on_mesh_prefix_set_t *prefix;
	thrd_on_mesh_prefix_set_t *found_prefix;

	PRINTF("thrd_on_mesh_prefix_set_lookup: Looking up On-Mesh Prefix.\n\r");

	found_prefix = NULL;
	for ( prefix = thrd_on_mesh_prefix_set_head(); prefix != NULL; prefix = thrd_on_mesh_prefix_set_next(prefix) ) {
		if ( memcmp(on_mesh_prefix, prefix->P_prefix, 8) == 0 ) {
			found_prefix = prefix;
			break;
		}
	}
	if ( found_prefix != NULL ) {
		PRINTF("thrd_on_mesh_prefix_set_lookup: Found On-Mesh Prefix.\n\r");
	} else {
		PRINTF("thrd_on_mesh_prefix_set_lookup: No On-Mesh Prefix found.\n\r");
	}
	return found_prefix;
}

/* --------------------------------------------------------------------------- */

static thrd_ext_route_set_t*
thrd_ext_route_set_lookup(uint8_t domain_id)
{
	thrd_ext_route_set_t *ext_route;
	thrd_ext_route_set_t *found_ext_route;

	PRINTF("thrd_ext_route_set_lookup: Looking up External Route Set for Domain ID: ");
	PRINTF("%d.", domain_id);
	PRINTF("\n\r");

	found_ext_route = NULL;
	for ( ext_route = thrd_ext_route_set_head(); ext_route != NULL; ext_route = thrd_ext_route_set_next(ext_route) ) {
		if ( ext_route->R_domain_id == domain_id ) {
			found_ext_route = ext_route;
			break;
		}
	}
	if ( found_ext_route != NULL ) {
		PRINTF("thrd_ext_route_set_lookup: Found Domain ID: %d.\n\r", domain_id);
	} else {
		PRINTF("thrd_ext_route_set_lookup: No Domain ID found.\n\r");
	}
	return found_ext_route;
}

/* --------------------------------------------------------------------------- */

static thrd_sicslowpan_ctx_id_set_t*
thrd_sicslowpan_ctx_id_set_lookup(uint8_t sicslowpan_ctx_id)
{
	thrd_sicslowpan_ctx_id_set_t *ctx_id;
	thrd_sicslowpan_ctx_id_set_t *found_ctx_id;

	PRINTF("thrd_sicslowpan_ctx_id_set_lookup: Looking up 6LoWPAN Context ID: ");
	PRINTF("%d.", sicslowpan_ctx_id);
	PRINTF("\n\r");

	found_ctx_id = NULL;
	for ( ctx_id = thrd_sicslowpan_ctx_id_set_head(); ctx_id != NULL; ctx_id = thrd_sicslowpan_ctx_id_set_next(ctx_id) ) {
		if ( ctx_id->CID_id == sicslowpan_ctx_id ) {
			found_ctx_id = ctx_id;
			break;
		}
	}
	if ( found_ctx_id != NULL ) {
		PRINTF("thrd_sicslowpan_ctx_id_set_lookup: Found 6LoWPAN Context ID: %d.\n\r", sicslowpan_ctx_id);
	} else {
		PRINTF("thrd_sicslowpan_ctx_id_set_lookup: No 6LoWPAN Context ID found.\n\r");
	}
	return found_ctx_id;
}

/* --------------------------------------------------------------------------- */

static thrd_server_set_t*
thrd_server_set_lookup(uint16_t enterprise_number)
{
	thrd_server_set_t *server;
	thrd_server_set_t *found_server;

	PRINTF("thrd_server_set_lookup: Looking up Server Set.\n\r");

	found_server = NULL;
	for ( server = thrd_server_set_head(); server != NULL; server = thrd_server_set_next(server) ) {
		if ( server->S_enterprise_number == enterprise_number ) {
			found_server = server;
			break;
		}
	}
	if ( found_server != NULL ) {
		PRINTF("thrd_server_set_lookup: Found Server.\n\r");
	} else {
		PRINTF("thrd_server_set_lookup: No Server found.\n\r");
	}
	return found_server;
}

/* --------------------------------------------------------------------------- */

static thrd_on_mesh_prefix_set_t*
thrd_on_mesh_prefix_set_add(uint8_t *prefix, uint8_t domain_id,
		uint16_t border_router_16, bool stable, bool slaac_preferred,
		bool slaac_valid, bool dhcp, bool configure, bool def,
		bool preference)
{
	thrd_on_mesh_prefix_set_t *pref;
	pref = thrd_on_mesh_prefix_set_lookup(prefix);

	if ( pref == NULL ) {
		PRINTF("thrd_on_mesh_prefix_set_add: On-Mesh Prefix unknown.\n\r");

		if ( thrd_get_num_on_mesh_prefixes() == THRD_MAX_ON_MESH_PREFIX_SET_ENTRIES ) {
			/* Removing the last entry from the On-Mesh Prefix Set. */
			thrd_on_mesh_prefix_set_t *last;

			last = list_tail(on_mesh_prefix_list);
			thrd_on_mesh_prefix_set_rm(last);
		}

		/* Allocate a router id entry and populate it. */
		pref = memb_alloc(&on_mesh_prefix_memb);

		if ( pref == NULL ) {
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("thrd_on_mesh_prefix_set_add: Could not allocate On-Mesh Prefix.\n\r");
			return NULL;
		}
		list_push(on_mesh_prefix_list, pref);

		memcpy(&(pref->P_prefix[0]), prefix, 8);
		pref->P_domain_id = domain_id;
		pref->P_border_router_16 = border_router_16;
		pref->P_stable = stable;
		pref->P_slaac_preferred = slaac_preferred;
		pref->P_slaac_valid = slaac_valid;
		pref->P_dhcp = dhcp;
		pref->P_configure = configure;
		pref->P_default = def;
		pref->P_preference = preference;
		PRINTF("thrd_on_mesh_prefix_set_add: Added On-Mesh Prefix.\n\r");
		num_on_mesh_prefixes++;
		PRINTF("thrd_on_mesh_prefix_set_add: num_on_mesh_prefixes %d\n\r", num_on_mesh_prefixes);
	} else {

		PRINTF(ANSI_COLOR_RED "thrd_on_mesh_prefix_set_add: On-Mesh Prefix is already known.\n\r");
		PRINTF("thrd_on_mesh_prefix_set_add: num_on_mesh_prefixes %d\n\r", num_on_mesh_prefixes);
		PRINTF("-----------------------------------------------------\n\r");
		return NULL;
	}
	PRINTF("-----------------------------------------------------\n\r");
	return pref;
}

/* --------------------------------------------------------------------------- */

static void
thrd_on_mesh_prefix_set_rm(thrd_on_mesh_prefix_set_t *pref)
{
	if ( pref != NULL ) {
		PRINTF("thrd_on_mesh_prefix_set_rm: Removing On-Mesh Prefix from On-Mesh Prefix Set.\n\r");

		/* Remove the router id from the Router ID Set. */
		list_remove(on_mesh_prefix_list, pref);
		memb_free(&on_mesh_prefix_memb, pref);
		num_on_mesh_prefixes--;
		PRINTF("thrd_on_mesh_prefix_set_rm: num_on_mesh_prefixes %d\n\r", num_on_mesh_prefixes);
	}
}

/* --------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------- */


/*
 ********************************************************************************
 *                               END OF FILE
 ********************************************************************************
 */
