

#include "mle_message.h"







/*static*/ uint16_t  get_16_MAC(void)
{
	//printf("mac addres is : %04x" ,a);
return (mac_phy_config.mac_address[6] | (mac_phy_config.mac_address[7] << 8));
//	return 0xabcd ;
}

/*
memcpy(mac_phy_config.mac_address, App_SrcMACAddr, sizeof(App_SrcMACAddr));
mac_phy_config.pan_id = 0xabcd;
x = mac_address[7] | (mac_address[6] << 8);
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





