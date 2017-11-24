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
#define TX_POWER                    (int8_t  )( 14 )
#endif

#ifndef RX_SENSITIVITY
#define RX_SENSITIVITY              (int8_t  )(  0 )    /* FIXME use proper configuration */
#endif

#ifndef MODULATION
#define MODULATION                  MODULATION_2FSK50
#endif


/* enable auto-acknowledgment of radio driver */
#ifndef NETSTK_CFG_RF_SW_AUTOACK_EN
#define NETSTK_CFG_RF_SW_AUTOACK_EN         TRUE
#endif

/** transceiver supports standard-specific checksum algorithm */
#define NETSTK_SUPPORT_HW_CRC               TRUE

/* radio transceiver does not support standard-specified checksum */
#ifndef NETSTK_CFG_RF_CRC_EN
#define NETSTK_CFG_RF_CRC_EN                TRUE
#endif

#ifndef CC13XX_LCD_ENABLE
#define CC13XX_LCD_ENABLE           		FALSE
#endif


/** Number of supported LEDs */
#ifndef HAL_SUPPORT_LEDNUM
#define HAL_SUPPORT_LEDNUM                  4
#endif /* #ifndef HAL_SUPPORT_LEDNUM */

/** Enable  HAL SUPPORT SLIPUART_RX */
#ifndef HAL_SUPPORT_PERIPHIRQ_SLIPUART_RX
#define HAL_SUPPORT_PERIPHIRQ_SLIPUART_RX                FALSE
#endif /* #ifndef HAL_SUPPORT_PERIPHIRQ_SLIPUART_RX */

/** Enable LED0 */
#ifndef HAL_SUPPORT_LED0
#define HAL_SUPPORT_LED0                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED0 */

/** Enable LED1 */
#ifndef HAL_SUPPORT_LED1
#define HAL_SUPPORT_LED1                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED1 */

/** Enable LED2 */
#ifndef HAL_SUPPORT_LED2
#define HAL_SUPPORT_LED2                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED2 */

/** Enable LED3 */
#ifndef HAL_SUPPORT_LED3
#define HAL_SUPPORT_LED3                    TRUE
#endif /* #ifndef HAL_SUPPORT_LED3 */

/* Enable the flash memory. */
#ifndef HAL_SUPPORT_NVM_FLASH
#define HAL_SUPPORT_NVM_FLASH               TRUE
#endif /* #ifndef HAL_SUPPORT_NVM_FLASH */

/* Enable the flash memory. */
#ifndef HAL_SUPPORT_NVM_EEPROM
#define HAL_SUPPORT_NVM_EEPROM              FALSE
#endif /* #ifndef HAL_SUPPORT_NVM_EEPROM */

/* Defines the data type of the NVM offset. */
#define MNVM_OFFSET_TYPE                    uint32_t


/* --------------------------------------------------------------------- */
/* TSCH related defines */

#ifndef HAL_SUPPORT_RTIMER
#define HAL_SUPPORT_RTIMER                      TRUE
#endif /* #ifndef SUPPORT_RTIMER */

/* Delay between GO signal and SFD */
#define RADIO_DELAY_BEFORE_TX ((unsigned)bsp_us_to_rtimerTiscks(81))
/* Delay between GO signal and start listening.
 * This value is so small because the radio is constantly on within each timeslot. */
#define RADIO_DELAY_BEFORE_RX ((unsigned)bsp_us_to_rtimerTiscks(15))
/* Delay between the SFD finishes arriving and it is detected in software. */
#define RADIO_DELAY_BEFORE_DETECT ((unsigned)bsp_us_to_rtimerTiscks(352))

/* Timer conversion; radio is running at 4 MHz */
#define RADIO_TIMER_SECOND   4000000u
#if (RTIMER_SECOND % 256) || (RADIO_TIMER_SECOND % 256)
#error RADIO_TO_RTIMER macro must be fixed!
#endif
#define RADIO_TO_RTIMER(X)   ((uint32_t)(((uint64_t)(X) * (RTIMER_SECOND / 256)) / (RADIO_TIMER_SECOND / 256)))
#define USEC_TO_RADIO(X)     ((X) * 4)

/* The PHY header (preamble + SFD, 4+1 bytes) duration is equivalent to 10 symbols */
#define RADIO_IEEE_802154_PHY_HEADER_DURATION_USEC 160

/* Do not turn off TSCH within a timeslot: not enough time */
#define TSCH_CONF_RADIO_ON_DURING_TIMESLOT 1

/* Disable TSCH frame filtering */
#define TSCH_CONF_HW_FRAME_FILTERING	0

/* Use hardware timestamps */
#ifndef TSCH_CONF_RESYNC_WITH_SFD_TIMESTAMPS
#define TSCH_CONF_RESYNC_WITH_SFD_TIMESTAMPS 1
#define TSCH_CONF_TIMESYNC_REMOVE_JITTER 0
#endif

#ifndef TSCH_CONF_BASE_DRIFT_PPM
/* The drift compared to "true" 10ms slots.
 * Enable adaptive sync to enable compensation for this. */
#define TSCH_CONF_BASE_DRIFT_PPM  0 // -977
#endif

/* 10 times per second */
#ifndef TSCH_CONF_CHANNEL_SCAN_DURATION
#define TSCH_CONF_CHANNEL_SCAN_DURATION (bsp_getTRes() / 10)
#endif

/* Slightly reduce the TSCH guard time (from 2200 usec to 1800 usec) to make sure
 * the CC26xx radio has sufficient time to start up. */
#ifndef TSCH_CONF_RX_WAIT
#define TSCH_CONF_RX_WAIT  5800
#endif

#define CCA_ENABLED   0
/*============================================================================*/
/*                       API FUNCTION DECLARATION                             */
/*============================================================================*/

 /*!
 * @brief   Configure Board Support Package module.
 *
 * This function initializes the netstack structure and thus sets which drivers are used.
 *
 * @param   p_netstk Pointer to net stack structure.
 * @return  0 if function succeeded, otherwise function failed.
 */
uint8_t board_conf(s_ns_t* p_netstk);

#endif /* _BOARD_CONF_H_ */

/*! @} end of emb6_bsp */
