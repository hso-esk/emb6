/*
 * route.c
 *
 *  Created on: 13 Apr 2016
 *      Author: Lukas Zimmermann <lzimmer1@stud.hs-offenburg.de>
 */

#include "emb6.h"
#include "uip.h"
#include "stdlib.h"
#include "string.h"
#include "clist.h"
#include "memb.h"

#include "route.h"

#define DEBUG DEBUG_PRINT
#include "uip-debug.h"

/* --------------------------------------------------------------------------- */


#ifdef RIP_DEBUG
void rt_print_iface_list() {

	struct rt_iface *ptr;

	ptr = rt_db->iface;									// Pointer to the first element of the set.

	PRINTF("\n");
	PRINTF("------------Iface List------------\n");

	while ( ptr != NULL ) {
		PRINTF("[%s]\n", ptr->iface_config->name);
		// PRINTF("Print Pointer (iface): %p\n", ptr);
		// PRINTF("Print Pointer (iface_config): %p\n", (void*)(ptr->iface_config));
		ptr = ptr->next;
	}
	PRINTF("----------------------------------\n");
	PRINTF("\n");
}
#endif



