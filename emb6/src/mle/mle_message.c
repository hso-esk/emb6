

#include "mle_message.h"







static uint16_t  get_16_MAC(void)
{
	//printf("mac addres is : %04x" ,a);
	return (mac_phy_config.mac_address[6] | (mac_phy_config.mac_address[7] << 8));
}


uint8_t  add_src_address_tlv_to_cmd(mle_cmd_t* cmd)
{
	if (mle_add_tlv_to_cmd( cmd ,TLV_SOURCE_ADDRESS , 2 ,(uint8_t*) &mac_phy_config.mac_address[6] ))
		return 1 ;
	else
		return 0 ;
}



/*
mac_phy_config.pan_id = 0xabcd;
 */




/**
 * @brief Send multicast MLE link request to all routers
 *
 * @return 1   OK.
 * @return 0   FAIL
 *
 */
/*
uint8_t mle_link_request_to_routers()
{

} */


/**
 * @brief Send multicast MLE parent request
 *
 * @param scan_mask       to indicate the types of devices that should reply for this request
 * 						  0x10 : Only active router
 * 						  0x01 : Only reed
 * 		     			  0x11 : both router and reed
 *
 * @return 1    OK.
 * @return 0   FAIL.
 *
 */
/*
uint8_t mle_parent_request( uint8_t scan_mask)
{

}

 */





