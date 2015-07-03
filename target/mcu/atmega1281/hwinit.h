/**
 * \defgroup mcu MCU HAL library
 *
 * MCU Hardware Abstraction APIs
 * @{
 *
 * \defgroup atmega1281 MCU (ATMega1281) HAL library
 *
 * The atmega1281 is a library for manipulating hardware from bsp.c
 * @{
 *
 *
 */
/*============================================================================*/
/*! \file   atmega1281/hwinit.h

    \author Artem Yushev artem.yushev@hs-offenbrug.de

    \brief  Hardware dependent initialization header file for ATMega1281.

   \version 0.0.1
*/

#ifndef HWINIT_H_
#define HWINIT_H_

/*============================================================================*/

/*==============================================================================
                                 INCLUDE FILES
==============================================================================*/

/*==============================================================================
                                     MACROS
==============================================================================*/

/* The AVR tick interrupt usually is done with an 8 bit counter around 128 Hz.
 * 125 Hz needs slightly more overhead during the interrupt, as does a 32 bit
 * clock_time_t.
 */
/* Clock ticks per second */
#define CLOCK_SECOND                         125
#define CLOCK_LT(a,b)                          ((signed long)((a)-(b)) < 0)
#define INFINITE_TIME                         0xffffffff
#define RIME_CONF_BROADCAST_ANNOUNCEMENT_MAX_TIME \
                                            INFINITE_TIME/CLOCK_CONF_SECOND /* Default uses 600 */
#define COLLECT_CONF_BROADCAST_ANNOUNCEMENT_MAX_TIME \
                                            INFINITE_TIME/CLOCK_CONF_SECOND /* Default uses 600 */

/*==============================================================================
                                     ENUMS
==============================================================================*/

/*==============================================================================
                         STRUCTURES AND OTHER TYPEDEFS
==============================================================================*/
typedef unsigned long                     off_t;
/*==============================================================================
                          GLOBAL VARIABLE DECLARATIONS
==============================================================================*/

/*==============================================================================
                                GLOBAL CONSTANTS
==============================================================================*/
// Put default MAC address in FLASH, defined in $(IF).c
extern const uint8_t mac_address[8];
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 common
 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Determine CPU frequency */
#define F_CPU                        8000000UL
#define     CONF_TICK_SEC                CLOCK_SECOND
/* Determine prescale factor for timer 0 (F_CPU = 8MHz, CONF_TICK_SEC = 1000)*/
#define CONF_TMR0_PRESCALE             256
/* Determine USART port output*/
#define USART                        1

/* Concatenate macro*/
#define CAT(x, y)                      x##y
#define CAT2(x, y, z)                  x##y##z
/* Determine ports, ddrs and pins concatenate macro */
#define DDR(x)                         CAT(DDR,  x)
#define PORT(x)                       CAT(PORT, x)
#define PIN(x)                         CAT(PIN,  x)


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 usart
 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define UCSR(num, let)                 CAT2(UCSR,num,let)
#define RXEN(x)                        CAT(RXEN,x)
#define TXEN(x)                        CAT(TXEN,x)
#define TXC(x)                         CAT(TXC,x)
#define RXC(x)                         CAT(RXC,x)
#define RXCIE(x)                       CAT(RXCIE,x)
#define UCSZ(x,y)                      CAT2(UCSZ,x,y)
#define UBRR(x,y)                      CAT2(UBRR,x,y)
#define UDRE(x)                        CAT(UDRE,x)
#define UDRIE(x)                       CAT(UDRIE,x)
#define UDR(x)                         CAT(UDR,x)
#define TCNT(x)                        CAT(TCNT,x)
#define TIMSK(x)                       CAT(TIMSK,x)
#define TCCR(x,y)                      CAT2(TCCR,x,y)
#define COM(x,y)                       CAT2(COM,x,y)
#define OCR(x,y)                       CAT2(OCR,x,y)
#define CS(x,y)                        CAT2(CS,x,y)
#define WGM(x,y)                       CAT2(WGM,x,y)
#define OCIE(x,y)                      CAT2(OCIE,x,y)

/* Single speed operation (U2X = 0)*/
#define USART_BAUD_2400             207
#define USART_BAUD_4800             103
#define USART_BAUD_9600             51
#define USART_BAUD_14400             34
#define USART_BAUD_19200             25
#define USART_BAUD_28800             16
#define USART_BAUD_38400             12
#define USART_BAUD_57600             8
#define USART_BAUD_76800             6
#define USART_BAUD_115200             3
#define USART_BAUD_230400             1
#define USART_BAUD_250000             1
#define USART_BAUD_500000             0
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 vectors
 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define COMPVECT(x)                    CAT2(TIMER,x,_COMPA_vect)
#define UDREVECT(x)                    CAT2(USART,x,_UDRE_vect)
#define RXVECT(x)                      CAT2(USART,x,_RX_vect)
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 spi
 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define SSPORT                         B
#define SSPIN                          (0x00)
#define SPIPORT                        B
#define MOSIPORT                     SPIPORT
#define MOSIPIN                        (0x02)
#define MISOPORT                     SPIPORT
#define MISOPIN                        (0x03)
#define SCKPORT                     SPIPORT
#define SCKPIN                         (0x01)
#define RSTPORT                        A
#define RSTPIN                         (0x07)
#define IRQPORT                        E
#define IRQPIN                         (0x05)
#define SLPTRPORT                      B
#define SLPTRPIN                       (0x04)
#define TXCWPORT                       B
#define TXCWPIN                        (0x07)



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
#endif /* HWINIT_H_ */
/** @} */
/** @} */
