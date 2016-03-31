/*
 * board_conf.h
 *
 *  Created on: 06.10.2015
 *      Author: nphuong
 */

#ifndef _BOARD_CONF_H_
#define _BOARD_CONF_H_


/*
********************************************************************************
*                           MAC PHY CONFIGURATIONS
********************************************************************************
*/
#ifndef TX_POWER
#define TX_POWER                    (int8_t  )( 14 )
#endif

#ifndef RX_SENSITIVITY
#define RX_SENSITIVITY              (int8_t  )(  0 )    /* FIXME use proper configuration */
#endif

#ifndef MODULATION
#define MODULATION                  (uint8_t )(  0 )    /* FIXME use proper configuration */
#endif

/*
********************************************************************************
*                           API FUNCTION DECLARATION
********************************************************************************
*/
uint8_t board_conf(s_ns_t* ps_nStack);

#endif /* _BOARD_CONF_H_ */
