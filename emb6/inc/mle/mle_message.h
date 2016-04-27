#ifndef  __MLE_MSG_H_
#define  __MLE_MSG_H_

#include "emb6.h"
#include "mle_cmd.h"



typedef struct {
	uint8_t          aux_header;     /**< Aux Header */
	mle_cmd_t          command;        /**< mle command */
	uint8_t            mic ;           /**< MIC */
}mle_msg_t ;

/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/

 uint8_t  add_tlv32_bit_to_cmd(mle_cmd_t* cmd , tlv_type_t type , uint32_t value);

/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

uint16_t  get_16_MAC(void);
uint8_t   add_src_address_to_cmd(mle_cmd_t* cmd);
uint8_t   add_time_out_to_cmd(mle_cmd_t* cmd , uint32_t time);
uint8_t   add_mode_RSDN_to_cmd(mle_cmd_t* cmd , uint8_t R , uint8_t S , uint8_t D, uint8_t N );
uint8_t   add_scan_mask_to_cmd(mle_cmd_t* cmd , uint8_t R , uint8_t E );
uint8_t   add_MLEframe_counter_to_cmd(mle_cmd_t* cmd , uint32_t frame) ;
uint8_t   add_status_to_cmd(mle_cmd_t* cmd);
uint8_t   add_version_to_cmd(mle_cmd_t* cmd);



/*
static uint8_t   get_Mode(void);

static void *    generate_challenge(void);
 */



#endif /* __MLE_MSG_H_ */
