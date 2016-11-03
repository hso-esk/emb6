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
    \file   mle_table.h

    \author Nidhal Mars <nidhal.mars@hs-offenburg.de>

    \brief  manipulate child and neighbor routers table

    \date   27 Apr 2016

  	\version  0.1
*/


#ifndef EMB6_INC_MLE_MLE_TABLE_H_
#define EMB6_INC_MLE_MLE_TABLE_H_


/*==============================================================================
                                 INCLUDE FILES
 =============================================================================*/

#include "ctimer.h"
#include "tcpip.h"


/*==============================================================================
                                     MACROS
 =============================================================================*/

#define     MAX_NB_ROUTER               9
#define     MAX_CHILD                   5

/*==============================================================================
									TYPEDEFS
 =============================================================================*/

/**
* neighbor state
*/
typedef enum {PENDING, LINKED} nb_state_t;


/**
* neighbors structure
*/
typedef struct nodemle {
	struct nodemle 		*next;
	/* timer for the neigbhor used in different process */
	struct ctimer        			 timer;
	/* neigbhor id */
	uint8_t 			       		 id;
	/* state of the neighbor PENDING or LINKED */
	nb_state_t 			       		 state;
	/* challenge used diring the join process : TODO  check the length for other device !!! */
	uint32_t         			     challenge;
	/* 16-bit short address */
	uint16_t         			     address16;
	/* ipv6 address used once the device still requesting for short address */
	uip_ipaddr_t              		 tmp;
	/* time out  */
	uint32_t  						 time_out ;
	/* MLE frame counter */
	uint32_t            			 MLEFrameCounter;
	/* TLV MODE */
	uint8_t 						 modeTLV;
	/* Two-way link qualtiy */
	uint8_t 						 LQ;
	/* received RSSI  */
	uint8_t 						 rec_rssi;
}mle_neighbor_t ;



/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

/**
 * \brief  initialise neighbors tables
 */
void mle_nb_device_init(void);


/**
 * \brief  get pointer to the head of child's table
 *
 * \return pointer to the head of child's table
 */
mle_neighbor_t * mle_childs_head(void);


/**
 * \brief  get pointer to the head of neighbor's table
 *
 * \return pointer to the head of neighbor's table
 */
mle_neighbor_t * mle_nb_router_head(void);


/**
 * \brief  find a neighbor router in the neighbor table by id
 *
 * \param  id	  router's id to find
 *
 * \return
 *       -  pointer to the router if success
 *       - NULL if it is not existing
 */
mle_neighbor_t * mle_find_nb_router(uint8_t id);


/**
 * \brief  find a child in the child table by id
 *
 * \param  id	  child's id to find
 *
 * \return
 *       -  pointer to the child if success
 *       - NULL if it is not existing
 */
mle_neighbor_t * mle_find_child( uint8_t id);


/**
 * \brief  find a neighbor router in the neighbor table by ipv6 address
 *
 * \param  address	  ipv6 of the router to find
 *
 * \return
 *       -  pointer to the router if success
 *       - NULL if it is not existing
 */
mle_neighbor_t * mle_find_nb_router_byAdd(uip_ipaddr_t* address);


/**
 * \brief  find a child in the child table by ipv6 address
 *
 * \param  address	  ipv6 of the child to find
 *
 * \return
 *       -  pointer to the child if success
 *       - NULL if it is not existing
 */
mle_neighbor_t * mle_find_child_byAdd( uip_ipaddr_t* address);


/**
 * \brief  find a neighbor router in the neighbor table by 16-bit address
 *
 * \param  address	  16-bit of the router to find
 *
 * \return
 *       -  pointer to the router if success
 *       - NULL if it is not existing
 */
mle_neighbor_t * mle_find_nb_router_by_16Add(uint16_t address);


/**
 * \brief  find a child in the child table by 16-bit address
 *
 * \param  address	  16-bit of the child to find
 *
 * \return
 *       -  pointer to the child if success
 *       - NULL if it is not existing
 */
mle_neighbor_t * mle_find_child_by_16Add( uint16_t address);


/**
 * \brief  add a child into the child table
 *
 * \param  id		  			child id
 * \param  address	  			16-bit of the child
 * \param  MLEFrameCounter	    MLE frame counter
 * \param  modeTLV	   	     	Mode TLV
 * \param  linkQuality	  		Two-way link quality
 *
 * \return
 *       -  pointer to the child if success
 *       - NULL if error
 */
mle_neighbor_t * mle_add_child(uint8_t id, uint16_t  address, uint32_t  MLEFrameCounter , uint8_t modeTLV, uint8_t  linkQuality);


/**
 * \brief  add a neighbor router into the neighbor table
 *
 * \param  id		  			router id
 * \param  address	  			16-bit of the router
 * \param  MLEFrameCounter	    MLE frame counter
 * \param  modeTLV	   	     	Mode TLV
 * \param  linkQuality	  		Two-way link quality
 *
 * \return
 *       -  pointer to the child if success
 *       - NULL if error
 */
mle_neighbor_t * mle_add_nb_router(uint8_t id, uint16_t address, uint32_t  MLEFrameCounter , uint8_t modeTLV, uint8_t  linkQuality);


/**
 * \brief  remove a neighbor router from the neighbor table
 *
 * \param  nb	pointer to the neighbor to remove
 *
 * \return
 *       -  1 sucess
 *       -  0 error
 */
uint8_t mle_rm_nb_router(mle_neighbor_t *nb);


/**
 * \brief  remove a child router from the child table
 *
 * \param  nb	pointer to the child to remove
 *
 * \return
 *       -  1 sucess
 *       -  0 error
 */
uint8_t mle_rm_child( mle_neighbor_t *nb);

/**
 * \brief  remove all neighbor routers from neighbor table
 *
 */
void mle_rm_all_nb_router(void);


/**
 * \brief  remove all child from child table
 *
 */
void mle_rm_all_child(void);


/**
 * \brief  count the number of neighbors that have a given link quality
 *
 * \param  N	link quality: could be 1, 2, 3
 *
 * \return  number of neighbors that have the given link quality
 */
uint8_t count_neighbor_LQ(uint8_t N ); // The number of neighboring device with which the sender shares a link of quality N


/**
 * \brief  print child table
 *
 */
void  mle_print_child_table(void);


/**
 * \brief  print neighbor table
 *
 */
void  mle_print_nb_router_table(void);

extern bool mle_is_child(uint8_t rloc16);

#endif /* EMB6_INC_MLE_MLE_TABLE_H_ */
