#ifndef  __MLE_MSG_H_
#define  __MLE_MSG_H_

#include "emb6.h"
#include "mle_cmd.h"
#include "framer_802154.h"


typedef struct {
	uint8_t 					   secured ;     // Thread uses an initial byte of ‘0’ indicates that the message is secured (encrypted and authenticated)
	frame802154_aux_hdr_t          aux_hdr;     /**< Aux Header */
	mle_cmd_t           		   cmd;        /**< mle command */
	uint8_t          			   mic ;           /**< MIC */
}mle_msg_t ;



/*==============================================================================
                                 API FUNCTIONS
 =============================================================================*/

uint16_t  get_16_MAC(void);
uint8_t   add_src_address_to_cmd(mle_cmd_t* cmd);
uint8_t   add_time_out_to_cmd(mle_cmd_t* cmd , uint32_t time);
uint8_t   add_mode_RSDN_to_cmd(mle_cmd_t* cmd , uint8_t R , uint8_t S , uint8_t D , uint8_t N );
uint8_t   add_scan_mask_to_cmd(mle_cmd_t* cmd , uint8_t R , uint8_t E );
uint8_t   add_MLEframe_counter_to_cmd(mle_cmd_t* cmd, uint32_t frame) ;
uint8_t   add_status_to_cmd(mle_cmd_t* cmd);
uint8_t   add_version_to_cmd(mle_cmd_t* cmd);
uint32_t   add_rand_challenge_to_cmd(mle_cmd_t* cmd );
uint8_t  add_response_to_cmd(mle_cmd_t* cmd , tlv_t* resp );
uint8_t  add_response32_to_cmd(mle_cmd_t* cmd ,  uint32_t value );
uint8_t  add_Link_margin_to_cmd(mle_cmd_t* cmd, uint8_t lm);
uint8_t  add_Cnnectivity_to_cmd(mle_cmd_t* cmd, uint8_t max_child, uint8_t child_count, uint8_t LQ3, uint8_t LQ2,
									uint8_t LQ1, uint8_t Leader_cost,uint8_t id_sed);
uint8_t  comp_resp_chall(uint32_t challenge , uint8_t * buf);

uint8_t   set_security_flag(mle_msg_t* msg , uint8_t flag);

/*
static uint8_t   get_Mode(void);

static void *    generate_challenge(void);
 */



#endif /* __MLE_MSG_H_ */
