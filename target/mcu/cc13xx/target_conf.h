/**
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       hwinit.h
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      Hardware configuration.
 */

 /*! @defgroup emb6_mcu_config mcu configuration
     @{
     @ingroup  emb6_mcu

     This sections describes how the hardware is initialized. */

#ifndef HWINIT_H_
#define HWINIT_H_

/*==============================================================================
                                    INCLUDES
==============================================================================*/
#include "driverlib/ioc.h"
#include "inc/hw_memmap.h"

/* Stack specific include */
#include "packetbuf.h"
#include "evproc.h"
#include "phy_framer_802154.h"
/*==============================================================================
                                     MACROS
==============================================================================*/
/*! Enable tx. */
#define CC13XX_TX_ENABLED 1U
/*! Enable rx. */
#define CC13XX_RX_ENABLED 1U
/*! Enable tx LED. */
#define CC13XX_TX_LED_ENABLED 1U
/*! Enable rx LED. */
#define CC13XX_RX_LED_ENABLED 1U
/*! Enable interrupt based rf handling. */
#define CC13XX_IS_POLLED_RADIO 0U
/*! Define the max telegram length */
#define CC13XX_MAX_TELEGRAM_LEN  (uint16_t)(PHY_PSDU_MAX + PHY_HEADER_LEN)
/*======================= CC1300 DRIVER LIB MACROS ===========================*/
/*! Needed for TI driverlib. */
#define BOARD_SMARTRF06EB
/*! Needed for TI driverlib. */
#define MODULE_CC13XX_7X7
/*! Needed for TI driverlib. */
#define CC1310F128
/*! Needed for TI driverlib. */
#define RFC_INCLUDE_GFSK

/*========================= PLATFROM =========================================*/
/*! Platform interfaces: Definition of RX pin and TX pin. */
#ifndef UART_IOID_RXD
#define UART_IOID_RXD                IOID_2
#endif /* #ifndef UART_IOID_RXD */

/*! Platform interfaces: Definition of TX pin. */
#ifndef UART_IOID_TXD
#define UART_IOID_TXD                IOID_3
#endif /* #ifndef UART_IOID_TXD */

#define CC1310_LED0                     0
#define CC1310_LED1                     1
#define CC1310_LED2                     2
#define CC1310_LED3                     3

/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/

/*==============================================================================
                          GLOBAL VARIABLE DECLARATIONS
==============================================================================*/

/*==============================================================================
                                GLOBAL CONSTANTS
==============================================================================*/

/*! @} 6lowpan_mcu_config */

#endif /* HWINIT_H_ */
