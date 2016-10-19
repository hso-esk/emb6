/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*============================================================================*/
/*!
    \file   mle_table.c

    \author Nidhal Mars <nidhal.mars@hs-offenburg.de>

    \brief  manipulate child and neighbor routers table

    \date   27 Apr 2016

  	\version  0.1
*/


/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "mle_table.h"
#include "clist.h"
#include "memb.h"


#define		DEBUG		DEBUG_PRINT
#include "uip-debug.h"

/*==============================================================================
                            LOCAL VARIABLES
==============================================================================*/

LIST(nb_router_list);
MEMB(nb_router_memb, mle_neighbor_t, MAX_NB_ROUTER);


LIST(childs_list);
MEMB(childs_memb, mle_neighbor_t, MAX_CHILD);



/*==============================================================================
                             LOCAL FUNCTIONS DECLARATIONS
==============================================================================*/

static mle_neighbor_t * mle_neigbhor_head(list_t* list);
static mle_neighbor_t * mle_find_neigbhor(list_t *list, uint8_t id);
static mle_neighbor_t *  mle_find_neigbhor_by_16Add(list_t *list, uint16_t address);
static mle_neighbor_t * mle_add_neigbhor( struct memb* m , list_t *list , uint8_t id, uint16_t address, uint32_t  MLEFrameCounter,
		uint8_t modeTLV, uint8_t  linkQuality );
static uint8_t mle_rm_neigbhor( struct memb* m , list_t *list, mle_neighbor_t *nb );
static void mle_print_table(list_t *list);

/*==============================================================================
                             LOCAL FUNCTIONS
==============================================================================*/

/**
 * \brief  get pointer to the head of list
 *
 * \param list  pointer to the list
 *
 * \return pointer to the head of the given list if it is found otherwise NULL
 */
static mle_neighbor_t * mle_neigbhor_head(list_t* list)
{
	return list_head(*list);
}


/**
 * \brief  find a neighbor on a given list
 *
 * \param list  pointer to the list
 *
 * \return pointer to the neighbor if it is found otherwise NULL
 */
static mle_neighbor_t * mle_find_neigbhor(list_t *list, uint8_t id)
{
	mle_neighbor_t *nb;

	for (nb = mle_neigbhor_head(list); nb != NULL; nb = list_item_next(nb))
	{
		if (nb->id == id)
			return nb;
	}
	return NULL ;
}


/**
 * \brief find a neighbor router on a given list by ipv6 address
 *
 * \param list        pointer to the list
 * \param address     ipv6 of the neighbor to find
 *
 * \return pointer to the neighbor if it is found otherwise NULL
 */
static mle_neighbor_t *  mle_find_neigbhor_byAdd(list_t *list, uip_ipaddr_t* address)
{
	mle_neighbor_t *nb;

	for (nb = mle_neigbhor_head(list); nb != NULL; nb = list_item_next(nb))
	{
		if (uip_ipaddr_cmp(&nb->tmp , address))
			return nb;
	}
	return NULL ;
}



/**
 * \brief find a neighbor router on a given list by 16-bit short address
 *
 * \param list        pointer to the list
 * \param address     16-bit short address of the neighbor to find
 *
 * \return pointer to the neighbor if it is found otherwise NULL
 */
static mle_neighbor_t *  mle_find_neigbhor_by_16Add(list_t *list, uint16_t address)
{
	mle_neighbor_t *nb;

	for (nb = mle_neigbhor_head(list); nb != NULL; nb = list_item_next(nb))
	{
		if (nb->address16 == address)
			return nb;
	}
	return NULL ;
}


/**
 * \brief  add a neighbor into a given list
 *
 * \param  m                    A memory block previously declared with MEMB()
 * \param  list                 The name of the list
 * \param  id		  			neighbor id
 * \param  address	  			16-bit of the neighbor
 * \param  MLEFrameCounter	    MLE frame counter
 * \param  modeTLV	   	     	Mode TLV
 * \param  linkQuality	  		Two-way link quality
 *
 * \return
 *       -  pointer to the neighbor if success
 *       - NULL if error
 */
static mle_neighbor_t * mle_add_neigbhor(struct memb* m, list_t *list, uint8_t id, uint16_t address, uint32_t MLEFrameCounter,
		uint8_t modeTLV, uint8_t linkQuality)
{
	mle_neighbor_t *nb;

	/* find the neighbor by id */
	nb = mle_find_neigbhor(list, id);

	/* Check whether the given router id already has an entry in the Router ID Set. */
	if (nb == NULL /*don't forget to check the max size */)
	{
		/* Allocate a router id entry and populate it. */
		nb = memb_alloc(m);

		if (nb == NULL) {
			PRINTF("Could not allocate neighbor id \n" ANSI_COLOR_RESET);
			return NULL;
		}

		nb->id = id;
		nb->address16=address;
		nb->MLEFrameCounter = MLEFrameCounter;
		nb->LQ=linkQuality;
		nb->modeTLV = modeTLV;

		/* add the neighbor */
		list_push(*list, nb);
	}
	else
	{
		return NULL;
	}
	return nb;
}



/**
 * \brief  remove a neighbor  from a given list
 *
 * \param  m       A memory block previously declared with MEMB()
 * \param  list    The name of the list
 *
 * \return
 *       -  1 sucess
 *       -  0 error
 */
static uint8_t mle_rm_neigbhor(struct memb* m, list_t *list, mle_neighbor_t *nb)
{
	if (nb != NULL) {
		/* Remove the neighbor */
		list_remove(*list, nb);
		memb_free(m, nb);
	}
	return 0 ;
}


/**
 * \brief  print all neighbors on a given list
 *
 * \param  list    The name of the list
 *
 */
static void mle_print_table(list_t *list)
{
	mle_neighbor_t *i;
	PRINTF("| id | modeTLV | MLEFrameCounter | address \n");
	PRINTF("---------------------------------------------------------\n\r");
	for (i = mle_neigbhor_head(list); i != NULL; i = list_item_next(i)) {
		PRINTF("| " ANSI_COLOR_YELLOW "%2d" ANSI_COLOR_RESET
				" | " ANSI_COLOR_YELLOW "%7d" ANSI_COLOR_RESET
				" | " ANSI_COLOR_YELLOW "%15d" ANSI_COLOR_RESET
				" | " ANSI_COLOR_YELLOW "%7d" ANSI_COLOR_RESET
				" |  ", i->id, i->modeTLV, i->MLEFrameCounter, i->address16);
		PRINTF("\n");
	}
	PRINTF("---------------------------------------------------------\n\r");
}


/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

void mle_nb_device_init(void)
{
	memb_init(&nb_router_memb);
	list_init(nb_router_list);

	memb_init(&childs_memb);
	list_init(childs_list);
}


mle_neighbor_t * mle_childs_head(void)
{
	return mle_neigbhor_head(&childs_list);
}
mle_neighbor_t * mle_nb_router_head(void)
{
	return mle_neigbhor_head(&nb_router_list);
}


mle_neighbor_t * mle_find_nb_router(uint8_t id)
{
	return mle_find_neigbhor(&nb_router_list,  id) ;
}
mle_neighbor_t * mle_find_child( uint8_t id)
{
	return mle_find_neigbhor(&childs_list,  id) ;
}


mle_neighbor_t * mle_find_nb_router_byAdd(uip_ipaddr_t* address)
{
	return mle_find_neigbhor_byAdd(&nb_router_list,  address) ;
}
mle_neighbor_t * mle_find_child_byAdd( uip_ipaddr_t* address)
{
	return mle_find_neigbhor_byAdd(&childs_list,  address) ;
}


mle_neighbor_t * mle_find_nb_router_by_16Add(uint16_t address)
{
	return mle_find_neigbhor_by_16Add(&nb_router_list,  address) ;
}
mle_neighbor_t * mle_find_child_by_16Add( uint16_t address)
{
	return mle_find_neigbhor_by_16Add(&childs_list,  address) ;
}



mle_neighbor_t * mle_add_child( uint8_t id, uint16_t address, uint32_t MLEFrameCounter,
		uint8_t modeTLV, uint8_t  linkQuality )
{
	return mle_add_neigbhor(&childs_memb, &childs_list, id, address, MLEFrameCounter, modeTLV, linkQuality );
}

mle_neighbor_t * mle_add_nb_router( uint8_t id, uint16_t address, uint32_t MLEFrameCounter,
		uint8_t modeTLV, uint8_t  linkQuality )
{
	return mle_add_neigbhor(&nb_router_memb, &nb_router_list, id, address, MLEFrameCounter, modeTLV, linkQuality);
}


uint8_t mle_rm_nb_router(mle_neighbor_t *nb)
{
	return mle_rm_neigbhor(&nb_router_memb ,  &nb_router_list, nb) ;
}

uint8_t mle_rm_child( mle_neighbor_t *nb)
{
	return mle_rm_neigbhor(&childs_memb , &childs_list, nb) ;
}

void mle_rm_all_nb_router(void)
{
	mle_neighbor_t *nb;
	nb = mle_neigbhor_head(&nb_router_list);
	while(nb)
	{
		mle_rm_nb_router(nb);
		nb = mle_neigbhor_head(&nb_router_list);
	}

}

void mle_rm_all_child(void)
{
	mle_neighbor_t *nb;
	nb = mle_neigbhor_head(&childs_list);
	while(nb)
	{
		mle_rm_child(nb);
		nb = mle_neigbhor_head(&childs_list);
	}

}

uint8_t count_neighbor_LQ(uint8_t N )
{
	mle_neighbor_t *nb;
	uint8_t count=0;

	for (nb = mle_neigbhor_head(&nb_router_list); nb != NULL; nb = list_item_next(nb))
	{
		//if(nb!= NULL)
		if(nb->LQ == N)
			count++;
	}
return count ;
}

bool mle_is_child(uint8_t child_id)
{
	mle_neighbor_t *nb;
	nb = mle_find_child(child_id);

	if ( nb != NULL ) {
		return TRUE;
	}
	return FALSE ;
}

void  mle_print_child_table(void)
{
	PRINTFR( "|=========================== MLE CHILD TABLE  ===========================|" ANSI_COLOR_RESET "\n\r");
	mle_print_table(&childs_list);
	PRINTFR( "|========================================================================|" ANSI_COLOR_RESET "\n\r");
}

void  mle_print_nb_router_table(void)
{
	PRINTFR( "|========================= MLE NB ROUTER TABLE  =========================|" ANSI_COLOR_RESET "\n\r");
	mle_print_table(&nb_router_list);
	PRINTFR( "|========================================================================|" ANSI_COLOR_RESET "\n\r");
}

