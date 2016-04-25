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
                                 API FUNCTIONS
 =============================================================================*/

//uint8_t mle_build_cmd( mle_cmd_type_t  cmd_type , mle_cmd_t* mle_cmd);


/*==============================================================================
								LOCAL FUNCTION
 =============================================================================*/

static uint16_t  get_16_MAC(void);
uint8_t  add_src_address_tlv_to_cmd(mle_cmd_t* cmd);

/*
static uint8_t mle_build_link_request_cmd(mle_cmd_t* cmd);
static uint8_t mle_build_link_accept_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_link_accept_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_link_reject_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_advertisement_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_data_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_data_response_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_parent_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_parent_response_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_child_id_request_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_child_id_response_cmd(mle_cmd_t* mle_cmd);

static uint8_t mle_build_child_update_cmd(mle_cmd_t* mle_cmd);
static uint8_t mle_build_child_update_response_cmd(mle_cmd_t* mle_cmd);

static uint8_t   get_Mode(void);

static void *    generate_challenge(void);
 */



#endif /* __MLE_MSG_H_ */
