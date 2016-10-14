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
    \file   tlv.h

    \author Nidhal Mars <nidhal.mars@hs-offenburg.de>

    \brief  Functions to handle TLV parameters used in MLE command

    \version  0.1
*/

#ifndef  __TLV_H_
#define  __TLV_H_

/*==============================================================================
                                 INCLUDES
 =============================================================================*/
#include "emb6.h"
#include "net_tlv.h"

#define		DEBUG		DEBUG_PRINT
#include "uip-debug.h"	// For debugging terminal output.


/*==============================================================================
									TYPEDEFS
 =============================================================================*/

/**
* MLE TLV type enum
*/
typedef enum  {
	TLV_SOURCE_ADDRESS,
	TLV_MODE,
	TLV_TIME_OUT,
	TLV_CHALLENGE,
	TLV_RESPONSE,
	TLV_LINK_LAYER_FRAME_COUNTER,
	TLV_LINK_QUALITY,  // Not used in Thread Network
	TLV_NETWORK_PARAMETER,  // Not used in Thread Network
	TLV_MLE_FRAME_COUNTER,
	TLV_ROUTE64,
	TLV_ADDRESS16,
	TLV_LEADER_DATA,
	TLV_NETWORK_DATA,
	TLV_TLV_REQUEST,
	TLV_SCAN_MASK,
	TLV_CONNECTIVITY,
	TLV_LINK_MARGIN,
	TLV_STATUS,
	TLV_VERSION,
	TLV_ADDRESS_REGISTRATION,
	TLV_NONE,
}  tlv_type_t  ;


/**
* TLV structure
*/
typedef struct __attribute__((__packed__))
{
	/* type */
	uint8_t  type;
	/* length */
	uint8_t	 length;
	 /* pointer to the value */
	uint8_t  value[1];
}  tlv_t;

/**
* TLV ROUTE64 structure
*/
typedef struct __attribute__((__packed__))
{
	/* id sequence number */
	uint8_t  id_sequence_number;
	/* assigned router id mask */
	uint64_t router_id_mask;
	/* link quality and route data */
	uint8_t  lq_rd[1];
}  tlv_route64_t;

/**
* TLV LEADER structure
*/
typedef struct __attribute__((__packed__))
{
	/* thread partition id */
	uint32_t  partition_id;
	/* weight */
	uint8_t	  weight;
	/* data version */
	uint8_t   data_version;
	/* stable data version */
	uint8_t   stable_data_version;
	/* leader router id */
	uint8_t   leader_router_id;
}  tlv_leader_t;

/**
* TLV CONNECTIVITY structure
*/
typedef struct __attribute__((__packed__))
{
	/* maximum  number of child that can be supported */
	uint8_t   maxChild  ;
	/* number of attached child */
	uint8_t	  child_count  ;
	/* number of neighbors with link quality 3  */
	uint8_t   LQ3  ;
	/* number of neighbors with link quality 2  */
	uint8_t   LQ2  ;
	/* number of neighbors with link quality 1  */
	uint8_t   LQ1  ;
	/* the cost to the leader  */
	uint8_t   LeaderCost  ;
	/* sequence id */
	uint8_t   id_seq  ;
}  tlv_connectivity_t   ;

/*==============================================================================
                                     MACROS
 =============================================================================*/

/* useful functions to manipulate bits a=variable, b=bit number */
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_RESET(a,b) ((a) &= ~(1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))


/*==============================================================================
								API FUNCTION
 =============================================================================*/

/**
 * @brief  init the tlv with a buffer
 *
 * @param  tlv	  			pointer to tlv object
 * @param  ptr	  			pointer to the buffer
 *
 * @return
         -  1 success
         -  0 error
 */
uint8_t tlv_init(tlv_t **tlv, uint8_t * ptr );

/**
 * @brief  init tlv leader structure
 *
 * @param  tlv	  			pointer to leader tlv structure
 * @param  ptr	  			pointer to the buffer
 *
 * @return
         -  1 success
         -  0 error
 */
uint8_t tlv_leader_init(tlv_leader_t **tlv, uint8_t * ptr );

/**
 * @brief  init tlv router64 structure
 *
 * @param  tlv	  			pointer to router64 tlv structure
 * @param  ptr	  			pointer to the buffer
 *
 * @return
         -  1 success
         -  0 error
 */
uint8_t tlv_route64_init(tlv_route64_t **tlv, uint8_t * ptr );


/**
 * @brief  init tlv connectivity structure
 *
 * @param  tlv	  			pointer to connectivity tlv structure
 * @param  ptr	  			pointer to the buffer
 *
 * @return
         -  1 success
         -  0 error
 */
uint8_t tlv_connectivity_init(tlv_connectivity_t **tlv, uint8_t * ptr );


/**
 * @brief  write on tlv object
 *
 * @param  tlv	  			pointer to tlv object to write on
 * @param  type	  			type of tlv object
 * @param  length	 		length of the value
 * @param  value	 		pointer to the value
 *
 * @return
         -  1 success
         -  0 error
 */
uint8_t tlv_write(tlv_t *tlv , tlv_type_t type, int8_t length, uint8_t * value );

/**
 * @brief  print a tlv object
 *
 * @param  tlv	  tlv opbject to print

 * @return
         -  1 success
         -  0 error
 */
uint8_t tlv_print(tlv_t * tlv);

/**
 * @brief  find tlv type in buffer
 *
 * @param  buf	          buffer
 * @param  buf_length	  buffer length
 * @param  type	  		  type of tlv to looking for
*
 * @return pointer to the tlv if exist
 *         otherwise NULL
 */
tlv_t* tlv_find(uint8_t * buf, uint8_t buf_length, const tlv_type_t type);


/* Lukas.Z: tlv  functions used on the network layer */

/**
 * @brief  init tlv target eid structure
 *
 * @param  tlv	  			pointer to target eid tlv structure
 * @param  ptr	  			pointer to the buffer
 *
 * @return
         -  1 success
         -  0 error
 */
uint8_t tlv_target_eid_init(net_tlv_target_eid_t **tlv, uint8_t *ptr);

/**
 * @brief  init tlv rloc16 structure
 *
 * @param  tlv	  			pointer to rloc16 tlv structure
 * @param  ptr	  			pointer to the buffer
 *
 * @return
         -  1 success
         -  0 error
 */
uint8_t tlv_rloc16_init(net_tlv_rloc16_t **tlv, uint8_t *ptr);

/**
 * @brief  init tlv ml eid structure
 *
 * @param  tlv	  			pointer to  ml eid tlv structure
 * @param  ptr	  			pointer to the buffer
 *
 * @return
         -  1 success
         -  0 error
 */
uint8_t tlv_ml_eid_init(net_tlv_ml_eid_t **tlv, uint8_t *ptr);


#endif /* __TLV_H_ */
