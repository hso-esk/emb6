 /**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       board_conf.h
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      Board configuration.
 */

 /*! @defgroup emb6_bsp emb6 board configuration
     @{

     This sections describes how the board is configured. */

#ifndef _BOARD_CONF_H_
#define _BOARD_CONF_H_

/*============================================================================*/
/*                          MAC PHY CONFIGURATIONS                            */
/*============================================================================*/

#ifndef TX_POWER
#define TX_POWER                    		(int8_t  )( 14 )
#endif

#ifndef RX_SENSITIVITY
#define RX_SENSITIVITY              		(int8_t  )(  0 )    /* FIXME use proper configuration */
#endif

#ifndef MODULATION
#define MODULATION                  		MODULATION_2FSK50   /* FIXME use proper configuration */
#endif

/* enable auto-acknowledgment of radio driver */
#ifndef NETSTK_CFG_RF_SW_AUTOACK_EN
#define NETSTK_CFG_RF_SW_AUTOACK_EN         TRUE
#endif

/* radio transceiver does not support standard-specified checksum */
#ifndef NETSTK_CFG_RF_CRC_EN
#define NETSTK_CFG_RF_CRC_EN                TRUE
#endif

#ifndef CC13XX_LCD_ENABLE
#define CC13XX_LCD_ENABLE           		1
#endif

/*============================================================================*/
/*                       API FUNCTION DECLARATION                             */
/*============================================================================*/

 /*!
 * @brief   Configure Board Support Package module.
 *
 * This function initializes the netstack structure and thus sets which drivers are used.
 *
 * @param   p_netstk Pointer to net stack structure.
 * @return  1 if success; otherwise 0.
 */
uint8_t board_conf(s_ns_t* p_netstk);

#endif /* _BOARD_CONF_H_ */

/*! @} end of emb6_bsp */
