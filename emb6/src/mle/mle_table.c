/*
 * mle_table.c
 *
 *  Created on: 27 Apr 2016
 *      Author: Nidhal Mars
 *      manipulate child and routers neighbors table
 */

#include "mle_table.h"
#include "clist.h"
#include "memb.h"


#define		DEBUG		DEBUG_PRINT
#include "uip-debug.h"



LIST(nb_router_list);
MEMB(nb_router_memb, mle_neighbor_t, MAX_NB_ROUTER);


LIST(childs_list);
MEMB(childs_memb, mle_neighbor_t, MAX_CHILD);


/******  functions to manipulate the neighbors list   *******/

void mle_nb_device_init(void)
{
	memb_init(&nb_router_memb);
	list_init(nb_router_list);

	memb_init(&childs_memb);
	list_init(childs_list);
}


static mle_neighbor_t * mle_neigbhor_head(list_t* list)
{
	return list_head(*list);
}

mle_neighbor_t * mle_childs_head(void)
{
	return mle_neigbhor_head(&childs_list);
}
mle_neighbor_t * mle_nb_router_head(void)
{
	return mle_neigbhor_head(&nb_router_list);
}




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

mle_neighbor_t * mle_find_nb_router(uint8_t id)
{
	return mle_find_neigbhor(&nb_router_list,  id) ;
}
mle_neighbor_t * mle_find_child( uint8_t id)
{
	return mle_find_neigbhor(&childs_list,  id) ;
}


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

mle_neighbor_t * mle_find_nb_router_byAdd(uip_ipaddr_t* address)
{
	return mle_find_neigbhor_byAdd(&nb_router_list,  address) ;
}
mle_neighbor_t * mle_find_child_byAdd( uip_ipaddr_t* address)
{
	return mle_find_neigbhor_byAdd(&childs_list,  address) ;
}



static mle_neighbor_t * mle_add_neigbhor(struct memb* m , list_t *list , uint8_t id, /*uip_ipaddr_t * */ uint16_t address, uint32_t  MLEFrameCounter ,
		uint8_t modeTLV, uint8_t  linkQuality)
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
			/* This should not happen, as we explicitly deallocated one
			 * link set entry above. */
			PRINTF("Could not allocate neighbor id \n" ANSI_COLOR_RESET);
			return NULL;
		}

		nb->id = id;
		//uip_ip6addr(&nb->address, 0xff02, 0, 0, 0, 0, 0, 0, 0x0001);

	//	uip_ip6addr_copy(&nb->address ,address);
		nb->address16=address;
		nb->MLEFrameCounter = MLEFrameCounter;
		nb->LQ=linkQuality;
		nb->modeTLV = modeTLV;

		/* add the neighbor */
		list_push(*list, nb);

		//PRINTFG("Neighbor added  : id = %d \n\r" ANSI_COLOR_RESET , id);

		// if i will use a counter a should increment it
	}
	else
	{
		//PRINTFR("Neighbor already exist \n" ANSI_COLOR_RESET );
		return NULL;
	}
	return nb;
}

mle_neighbor_t * mle_add_child(uint8_t id, uint16_t address, uint32_t  MLEFrameCounter ,
		uint8_t modeTLV, uint8_t  linkQuality)
{
	return mle_add_neigbhor(&childs_memb , &childs_list ,  id,   address,   MLEFrameCounter , modeTLV,   linkQuality);
}

mle_neighbor_t * mle_add_nb_router(uint8_t id, uint16_t address, uint32_t  MLEFrameCounter ,
		uint8_t modeTLV, uint8_t  linkQuality)
{
	return mle_add_neigbhor(&nb_router_memb ,  &nb_router_list ,  id,   address,   MLEFrameCounter , modeTLV,   linkQuality);
}



static uint8_t mle_rm_neigbhor(struct memb* m , list_t *list ,mle_neighbor_t *nb)
{
	if (nb != NULL) {
		/* Remove the router id from the Router ID Set. */
	//	PRINTFR("removing Neighbor: id = %d \n" ANSI_COLOR_RESET ,nb->id);
		list_remove(*list, nb);
		memb_free(m, nb);
	}
	return 0 ;
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

static void mle_print_table(list_t *list)
{
	mle_neighbor_t *i;
	PRINTF("| id | modeTLV | MLEFrameCounter |  address  \n");
	PRINTF("---------------------------------------------------------\n\r");
	for (i = mle_neigbhor_head(list); i != NULL; i = list_item_next(i)) {
		PRINTF("| " ANSI_COLOR_YELLOW "%2d" ANSI_COLOR_RESET
				" | " ANSI_COLOR_YELLOW "%7d" ANSI_COLOR_RESET
				" | " ANSI_COLOR_YELLOW "%15d" ANSI_COLOR_RESET
				" |  ", i->id, i->modeTLV, i->MLEFrameCounter);
		//PRINT6ADDR(&i->address);
		PRINTF("\n");
	}
	PRINTF("---------------------------------------------------------\n\r");
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

