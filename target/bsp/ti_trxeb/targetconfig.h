#ifndef __TARGETCONFIG_H__
#define __TARGETCONFIG_H__

/*============================================================================*/
/**
 * \file    targetconfig.h
 *
 * \author  Manuel Schappacher
 *
 * \brief   Target configuration
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <msp430.h>
#include <stdint.h>
#include "hal_types.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** Number of init delays */
#define TARGET_CONFIG_INIT_DELAY_NUM              (3)
/** Number of init delay cycles */
#define TARGET_CONFIG_INIT_DELAY_CYCLES           (2000000)


/*============================================================================*/
/*                           FUCNTIONAL DEFINITIONS                           */
/*============================================================================*/


/* enable PORT1 */
#define TARGET_CONFIG_PORT1                       TRUE
/* enable PORT2 */
#define TARGET_CONFIG_PORT2                       TRUE
/* enable PORT3 */
#define TARGET_CONFIG_PORT3                       TRUE
/* enable PORT4 */
#define TARGET_CONFIG_PORT4                       TRUE
/* enable PORT5 */
#define TARGET_CONFIG_PORT5                       TRUE
/* enable PORT6 */
#define TARGET_CONFIG_PORT6                       TRUE
/* enable PORT7 */
#define TARGET_CONFIG_PORT7                       TRUE
/* enable PORT8 */
#define TARGET_CONFIG_PORT8                       TRUE
/* enable PORT9 */
#define TARGET_CONFIG_PORT9                       TRUE
/* enable PORT10 */
#define TARGET_CONFIG_PORT10                      TRUE

/* enable LED1 */
#define TARGET_CONFIG_LED1                        TRUE
/* enable LED2 */
#define TARGET_CONFIG_LED2                        TRUE
/* enable LED3 */
#define TARGET_CONFIG_LED3                        TRUE
/* enable LED4 */
#define TARGET_CONFIG_LED4                        TRUE

/* high logic enabled */
#define TARGET_CONFIG_LED_HL                      FALSE

/* enable KEY1 */
#define TARGET_CONFIG_KEY1                        TRUE
/* enable KEY2 */
#define TARGET_CONFIG_KEY2                        TRUE
/* enable KEY3 */
#define TARGET_CONFIG_KEY3                        TRUE
/* enable KEY4 */
#define TARGET_CONFIG_KEY4                        TRUE
/* enable KEY5 */
#define TARGET_CONFIG_KEY5                        TRUE

/* enable LCD */
#define TARGET_CONFIG_LCD                         FALSE

/* enable RF */
#define TARGET_CONFIG_RF                          TRUE

/* enable UART0 */
#define TARGET_CONFIG_UART0                       TRUE

/* enable UART1 */
#define TARGET_CONFIG_UART1                       TRUE

/* enable UART3 */
#define TARGET_CONFIG_UART3                       TRUE


/*============================================================================*/
/*                              PIN DEFINITIONS                               */
/*============================================================================*/


#if( TARGET_CONFIG_LED1 == TRUE )
/* LED1 is on P4.0*/
#define TARGETCONFIG_LED1_PORT                    E_IO_PORT_P4
#define TARGETCONFIG_LED1_PIN                     0
#define TARGETCONFIG_LED1_MSK                    (1 << TARGETCONFIG_LED1_PIN)
#endif /* #if( TARGET_CONFIG_LED1 == TRUE ) */

#if( TARGET_CONFIG_LED2 == TRUE )
/* LED1 is on P4.1*/
#define TARGETCONFIG_LED2_PORT                    E_IO_PORT_P4
#define TARGETCONFIG_LED2_PIN                     1
#define TARGETCONFIG_LED2_MSK                     (1 << TARGETCONFIG_LED2_PIN)
#endif /* #if( TARGET_CONFIG_LED2 == TRUE ) */

#if( TARGET_CONFIG_LED3 == TRUE )
/* LED1 is on P4.2*/
#define TARGETCONFIG_LED3_PORT                    E_IO_PORT_P4
#define TARGETCONFIG_LED3_PIN                     2
#define TARGETCONFIG_LED3_MSK                     (1 << TARGETCONFIG_LED3_PIN)
#endif /* #if( TARGET_CONFIG_LED3 == TRUE ) */

#if( TARGET_CONFIG_LED4 == TRUE )
/* LED1 is on P4.3*/
#define TARGETCONFIG_LED4_PORT                    E_IO_PORT_P4
#define TARGETCONFIG_LED4_PIN                     3
#define TARGETCONFIG_LED4_MSK                     (1 << TARGETCONFIG_LED4_PIN)
#endif /* #if( TARGET_CONFIG_LED4 == TRUE ) */


#if( TARGET_CONFIG_KEY1 == TRUE )
/* KEY1 is on P2.0*/
#define TARGETCONFIG_KEY1_PORT                    E_IO_PORT_P2
#define TARGETCONFIG_KEY1_PIN                     1
#define TARGETCONFIG_KEY1_MSK                     (1 << TARGETCONFIG_KEY1_PIN)
#endif /* #if( TARGET_CONFIG_KEY1 == TRUE ) */

#if( TARGET_CONFIG_KEY2 == TRUE )
/* KEY2 is on P2.2*/
#define TARGETCONFIG_KEY2_PORT                    E_IO_PORT_P2
#define TARGETCONFIG_KEY2_PIN                     2
#define TARGETCONFIG_KEY2_MSK                     (1 << TARGETCONFIG_KEY2_PIN)
#endif /* #if( TARGET_CONFIG_KEY2 == TRUE ) */

#if( TARGET_CONFIG_KEY3 == TRUE )
/* KEY3 is on P2.4*/
#define TARGETCONFIG_KEY3_PORT                    E_IO_PORT_P2
#define TARGETCONFIG_KEY3_PIN                     4
#define TARGETCONFIG_KEY3_MSK                     (1 << TARGETCONFIG_KEY3_PIN)
#endif /* #if( TARGET_CONFIG_KEY3 == TRUE ) */

#if( TARGET_CONFIG_KEY4 == TRUE )
/* KEY4 is on P2.5*/
#define TARGETCONFIG_KEY4_PORT                    E_IO_PORT_P2
#define TARGETCONFIG_KEY4_PIN                     5
#define TARGETCONFIG_KEY4_MSK                     (1 << TARGETCONFIG_KEY4_PIN)
#endif /* #if( TARGET_CONFIG_KEY4 == TRUE ) */

#if( TARGET_CONFIG_KEY5 == TRUE )
/* KEY5 is on P2.3*/
#define TARGETCONFIG_KEY5_PORT                    E_IO_PORT_P2
#define TARGETCONFIG_KEY5_PIN                     3
#define TARGETCONFIG_KEY5_MSK                     (1 << TARGETCONFIG_KEY5_PIN)
#endif /* #if( TARGET_CONFIG_KEY5 == TRUE ) */


#if( TARGET_CONFIG_LCD == TRUE )
/* SPI MOSI is on P9.1*/
#define TARGETCONFIG_LCD_SPI_MOSI_PORT            E_IO_PORT_P9
#define TARGETCONFIG_LCD_SPI_MOSI_PIN             1
#define TARGETCONFIG_LCD_SPI_MOSI_MSK            (1 << TARGETCONFIG_LCD_SPI_MOSI_PIN)
/* SPI MISO is on P9.2*/
#define TARGETCONFIG_LCD_SPI_MISO_PORT            E_IO_PORT_P9
#define TARGETCONFIG_LCD_SPI_MISO_PIN             2
#define TARGETCONFIG_LCD_SPI_MISO_MSK             (1 << TARGETCONFIG_LCD_SPI_MISO_PIN)
/* SPI SCLK is on P9.3*/
#define TARGETCONFIG_LCD_SPI_SCLK_PORT            E_IO_PORT_P9
#define TARGETCONFIG_LCD_SPI_SCLK_PIN             3
#define TARGETCONFIG_LCD_SPI_SCLK_MSK             (1 << TARGETCONFIG_LCD_SPI_SCLK_PIN)
/* SPI CSN is on P9.6*/
#define TARGETCONFIG_LCD_SPI_CSN_PORT             E_IO_PORT_P9
#define TARGETCONFIG_LCD_SPI_CSN_PIN              6
#define TARGETCONFIG_LCD_SPI_CSN_MSK              (1 << TARGETCONFIG_LCD_SPI_CSN_PIN)

/* MODE is on P9.7*/
#define TARGETCONFIG_LCD_MODE_PORT                E_IO_PORT_P9
#define TARGETCONFIG_LCD_MODE_PIN                 7
#define TARGETCONFIG_LCD_MODE_MSK                 (1 << TARGETCONFIG_LCD_MODE_PIN)

/* RST is on P7.3*/
#define TARGETCONFIG_LCD_RST_PORT                 E_IO_PORT_P7
#define TARGETCONFIG_LCD_RST_PIN                  3
#define TARGETCONFIG_LCD_RST_MSK                  (1 << TARGETCONFIG_LCD_RST_PIN)

/* PWR is on P7.7*/
#define TARGETCONFIG_LCD_PWR_PORT                 E_IO_PORT_P7
#define TARGETCONFIG_LCD_PWR_PIN                  7
#define TARGETCONFIG_LCD_PWR_MSK                  (1 << TARGETCONFIG_LCD_PWR_PIN)
#endif /* #if( TARGET_CONFIG_LCD == TRUE ) */

#if( TARGET_CONFIG_RF == TRUE )
/* SPI MOSI is on P3.2*/
#define TARGETCONFIG_RF_SPI_MOSI_PORT             E_IO_PORT_P3
#define TARGETCONFIG_RF_SPI_MOSI_PIN              1
#define TARGETCONFIG_RF_SPI_MOSI_MSK              (1 << TARGETCONFIG_RF_SPI_MOSI_PIN)
/* SPI MISO is on P3.1*/
#define TARGETCONFIG_RF_SPI_MISO_PORT             E_IO_PORT_P3
#define TARGETCONFIG_RF_SPI_MISO_PIN              2
#define TARGETCONFIG_RF_SPI_MISO_MSK              (1 << TARGETCONFIG_RF_SPI_MISO_PIN)
/* SPI SCLK is on P3.2*/
#define TARGETCONFIG_RF_SPI_SCLK_PORT             E_IO_PORT_P3
#define TARGETCONFIG_RF_SPI_SCLK_PIN              3
#define TARGETCONFIG_RF_SPI_SCLK_MSK              (1 << TARGETCONFIG_RF_SPI_SCLK_PIN)
/* SPI CSN is on P2.2*/
#define TARGETCONFIG_RF_SPI_CSN_PORT              E_IO_PORT_P3
#define TARGETCONFIG_RF_SPI_CSN_PIN               0
#define TARGETCONFIG_RF_SPI_CSN_MSK               (1 << TARGETCONFIG_RF_SPI_CSN_PIN)

/* MODE is on P1.7*/
#define TARGETCONFIG_RF_GPIO0_PORT                E_IO_PORT_P1
#define TARGETCONFIG_RF_GPIO0_PIN                 7
#define TARGETCONFIG_RF_GPIO0_MSK                 (1 << TARGETCONFIG_RF_GPIO0_PIN)

/* RST is on P1.3*/
#define TARGETCONFIG_RF_GPIO2_PORT                E_IO_PORT_P1
#define TARGETCONFIG_RF_GPIO2_PIN                 3
#define TARGETCONFIG_RF_GPIO2_MSK                 (1 << TARGETCONFIG_RF_GPIO2_PIN)

/* PWR is on P1.2*/
#define TARGETCONFIG_RF_GPIO3_PORT                E_IO_PORT_P1
#define TARGETCONFIG_RF_GPIO3_PIN                 2
#define TARGETCONFIG_RF_GPIO3_MSK                 (1 << TARGETCONFIG_RF_GPIO3_PIN)
#endif /* #if( TARGET_CONFIG_RF == TRUE ) */

#if( TARGET_CONFIG_UART0 == TRUE )
/* UART0 RX is on P5.7*/
#define TARGETCONFIG_UART0_RX_PORT                E_IO_PORT_P3
#define TARGETCONFIG_UART0_RX_PIN                 5
#define TARGETCONFIG_UART0_RX_MSK                 (1 << TARGETCONFIG_UART0_RX_PIN)
/* UART0 TX is on P5.6*/
#define TARGETCONFIG_UART0_TX_PORT                E_IO_PORT_P3
#define TARGETCONFIG_UART0_TX_PIN                 4
#define TARGETCONFIG_UART0_TX_MSK                 (1 << TARGETCONFIG_UART0_TX_PIN)
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */

#if( TARGET_CONFIG_UART1 == TRUE )
/* UART1 RX is on P5.7*/
#define TARGETCONFIG_UART1_RX_PORT                E_IO_PORT_P5
#define TARGETCONFIG_UART1_RX_PIN                 7
#define TARGETCONFIG_UART1_RX_MSK                 (1 << TARGETCONFIG_UART1_RX_PIN)
/* UART1 TX is on P5.6*/
#define TARGETCONFIG_UART1_TX_PORT                E_IO_PORT_P5
#define TARGETCONFIG_UART1_TX_PIN                 6
#define TARGETCONFIG_UART1_TX_MSK                 (1 << TARGETCONFIG_UART1_TX_PIN)
#endif /* #if( TARGET_CONFIG_UART1 == TRUE ) */

#if( TARGET_CONFIG_UART3 == TRUE )
/* UART3 RX is on P10.5*/
#define TARGETCONFIG_UART3_RX_PORT                E_IO_PORT_P10
#define TARGETCONFIG_UART3_RX_PIN                 5
#define TARGETCONFIG_UART3_RX_MSK                 (1 << TARGETCONFIG_UART3_RX_PIN)
/* UART3 TX is on P10.4*/
#define TARGETCONFIG_UART3_TX_PORT                E_IO_PORT_P10
#define TARGETCONFIG_UART3_TX_PIN                 4
#define TARGETCONFIG_UART3_TX_MSK                 (1 << TARGETCONFIG_UART3_TX_PIN)
#endif /* #if( TARGET_CONFIG_UART3 == TRUE ) */


/* XT1-IN is on port 7.0 */
#define TARGETCONFIG_MCU_XT1IN_PORT               E_IO_PORT_P7
#define TARGETCONFIG_MCU_XT1IN_PIN                0
#define TARGETCONFIG_MCU_XT1IN_MSK                (1 << TARGETCONFIG_MCU_XT1IN_PIN)

/* XT1-OUT is on port 7.1 */
#define TARGETCONFIG_MCU_XT1OUT_PORT              E_IO_PORT_P7
#define TARGETCONFIG_MCU_XT1OUT_PIN               1
#define TARGETCONFIG_MCU_XT1OUT_MSK               (1 << TARGETCONFIG_MCU_XT1OUT_PIN)


/*============================================================================*/
/*                            LOW POWER DEFINITIONS                           */
/*============================================================================*/


/** Low power mode to use */
#define TARGET_CONFIG_LPM                         E_MCU_PM_3

/** IO Port1 Pins to exit LPM */
#define TARGET_CONFIG_LPM_EXIT_IOP1               (TARGETCONFIG_RF_GPIO2_MSK)

/** IO Port2 Pins to exit LPM */
#define TARGET_CONFIG_LPM_EXIT_IOP2               (TARGETCONFIG_KEY5_MSK)

/** Shall Timer0 exit LPM */
#define TARGET_CONFIG_LPM_EXIT_TIMER0              FALSE

/** Shall Timer1 exit LPM */
#define TARGET_CONFIG_LPM_EXIT_TIMER1              TRUE

#if( TARGET_CONFIG_UART0 == TRUE )
/** Shall UART0 exit LPM */
#define TARGET_CONFIG_LPM_EXIT_UART0               TRUE
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */

#if( TARGET_CONFIG_UART1 == TRUE )
/** Shall UART1 exit LPM */
#define TARGET_CONFIG_LPM_EXIT_UART1               TRUE
#endif /* #if( TARGET_CONFIG_UART1 == TRUE ) */

#if( TARGET_CONFIG_UART3 == TRUE )
/** Shall UART3 exit LPM */
#define TARGET_CONFIG_LPM_EXIT_UART3               TRUE
#endif /* #if( TARGET_CONFIG_UART3 == TRUE ) */

/** Shall RTC exit LPM */
#define TARGET_CONFIG_LPM_EXIT_RTC                 TRUE

#endif /* #ifndef __TARGETCONFIG_H__ */
