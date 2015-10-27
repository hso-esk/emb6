/**
 * @file    target.c
 * @author  PN
 * @brief   HAL implementation of TI MSP430x5xx
 */

#include "target.h"


/*
********************************************************************************
*                                  INCLUDES
********************************************************************************
*/

#include <msp430.h>

#include "targetconfig.h"
#include "target.h"

#include "bsp.h"
#include "io.h"
#include "int.h"
#include "lcd.h"
#include "mcu.h"
#include "led.h"
#include "key.h"
#include "spi.h"
#include "tmr.h"
#include "uart.h"
#include "rtc.h"
#include "infoflash.h"

#if 1
#include "lib_tmr.h"
#endif

/*
********************************************************************************
*                               LOCAL DEFINES
********************************************************************************
*/
#define TARGET_CFG_ARG_CHK_EN                   ( 1u )

#define TARGET_CFG_SYSTICK_RESOLUTION           (clock_time_t)( 1000u )
#define TARGET_CFG_SYSTICK_SCALER               (clock_time_t)(    2u )


/*
********************************************************************************
*                               LOCAL DATATYPE
********************************************************************************
*/
typedef uint8_t     spiDesc_t;

/*
********************************************************************************
*                               LOCAL VARIABLES
********************************************************************************
*/
static s_io_pin_desc_t s_target_extIntPin[] =
{
    {&gps_io_port[TARGETCONFIG_RF_GPIO0_PORT], TARGETCONFIG_RF_GPIO0_PIN, TARGETCONFIG_RF_GPIO0_MSK},
    {&gps_io_port[TARGETCONFIG_RF_GPIO2_PORT], TARGETCONFIG_RF_GPIO2_PIN, TARGETCONFIG_RF_GPIO2_MSK},
    {&gps_io_port[TARGETCONFIG_RF_GPIO3_PORT], TARGETCONFIG_RF_GPIO3_PIN, TARGETCONFIG_RF_GPIO3_MSK},
};

static clock_time_t volatile    hal_ticks;
static spiDesc_t                hal_rfspi;

/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
static void _hal_systick(void);
static void _hal_isrSysTick( void *p_arg );

/*
********************************************************************************
*                           LOCAL FUNCTION DEFINITIONS
********************************************************************************
*/
static void _hal_isrSysTick( void *p_arg )
{
    hal_ticks++;

    if ((hal_ticks % TARGET_CFG_SYSTICK_SCALER) == 0) {
        Tmr_Update();
    }
}


static void _hal_systick(void)
{
    tmr_config(E_TMR_0,
               (uint16_t)(TARGET_CFG_SYSTICK_RESOLUTION / TARGET_CFG_SYSTICK_SCALER));

    tmr_start(E_TMR_0, _hal_isrSysTick );
}

static void _hal_uartInit(void)
{
    char *p_line = "\r\n\r\n========== BEGIN ===========\r\n\r\n";

    uart_init();
    uart_config(E_UART_SEL_UART1, 115200, NULL);
    uart_send(E_UART_SEL_UART1, p_line, strlen(p_line));
}


/*
********************************************************************************
*                           GLOBAL FUNCTION DEFINITIONS
********************************************************************************
*/
int putchar(int c)
{
    uart_send(E_UART_SEL_UART1, (char *)&c, 1);
    return c;
}


/*
********************************************************************************
*                            API FUNCTION DEFINITIONS
********************************************************************************
*/
void hal_enterCritical(void)
{
    __disable_interrupt();
}


void hal_exitCritical(void)
{
    __enable_interrupt();
}


/**
 *  \brief      This function should initialize all of the MCU peripherals
 *  \return     status
 */
int8_t hal_init(void)
{
    int i = 0;


    hal_enterCritical();
    {
        /*
         * Initialize peripherals: UART, SPI, LED,...
         * As BSP implementation is common for all platform at the time of writing,
         * platform-specific modules shall be initialized in this function
         */

        /* initialize IO */
        io_init();

        /* initialize system clock */
        mcu_sysClockInit( MCU_SYSCLK_4MHZ);

        /* initialize interrupts */
        int_init();

        /* initialize timer */
        tmr_init();

        /* initialize LEDs */
        led_init();

        /* initialize keys */
        key_init();

        /* initialize RF SPI */
#if (TARGET_CONFIG_RF == TRUE)
        spi_rfInit(1);
#endif

        /* initialize LCD */
#if (TARGET_CONFIG_LCD == TRUE)
        spi_lcdInit(MCU_SYSCLK_8MHZ);
        lcd_init();
#endif

        /* initialize UART */
        _hal_uartInit();

        /* initialize info flash */
        infoflash_init();

        /* initialize RTC */
        rtc_init();

        /* Initialize System tick timer */
        _hal_systick();

        /* Enable global interrupt */
        _BIS_SR(GIE);

        /* delay */
        for( i = 0; i < TARGET_CONFIG_INIT_DELAY_NUM; i++ )
            __delay_cycles( TARGET_CONFIG_INIT_DELAY_CYCLES );
    }
    hal_exitCritical();

    /*
     * Initialize HAL local variables
     */
    hal_rfspi = 0;
    hal_ticks = 0;
    return 1;
}


uint8_t hal_getrand(void)
{
    /*TODO missing implementation */
#if STK_CFG_REFACTOR_EN
    return (uint8_t)TmrCurTick;
#else
    return 0;
#endif
}


void hal_ledOn(uint16_t ui_led)
{
    switch (ui_led) {
#if TARGET_CONFIG_LED1
        case E_BSP_LED_ORANGE:
            led_set(E_LED_1);
            break;
#endif
#if TARGET_CONFIG_LED2
        case E_BSP_LED_YELLOW:
            led_set(E_LED_2);
            break;
#endif
#if TARGET_CONFIG_LED3
        case E_BSP_LED_GREEN:
            led_set(E_LED_3);
            break;
#endif
#if TARGET_CONFIG_LED4
        case E_BSP_LED_RED:
            led_set(E_LED_4);
            break;
#endif
        default:
            break;
    }
}


void hal_ledOff(uint16_t ui_led)
{
    switch (ui_led) {
#if TARGET_CONFIG_LED1
        case E_BSP_LED_ORANGE:
            led_clear(E_LED_1);
            break;
#endif
#if TARGET_CONFIG_LED2
        case E_BSP_LED_YELLOW:
            led_clear(E_LED_2);
            break;
#endif
#if TARGET_CONFIG_LED3
        case E_BSP_LED_GREEN:
            led_clear(E_LED_3);
            break;
#endif
#if TARGET_CONFIG_LED4
        case E_BSP_LED_RED:
            led_clear(E_LED_4);
            break;
#endif
        default:
            break;
    }
}


uint8_t hal_extIntEnable(en_targetExtInt_t      e_pin,
                         en_targetIntEdge_t     e_edge,
                         pfn_intCallb_t         pf_cb)
{
    s_io_pin_desc_t *ps_desc = &s_target_extIntPin[e_pin];



    if ((pf_cb != NULL) &&
        (ps_desc->PORT != NULL))
    {
        /*
         * Enable interrupt
         */
        switch (e_edge) {
            case E_TARGET_INT_EDGE_FALLING:
                io_irqEnable( ps_desc, INT_EDGE_FALLING, pf_cb );
                break;

            case E_TARGET_INT_EDGE_RISING:
                io_irqEnable( ps_desc, INT_EDGE_RISING, pf_cb );
                break;

            default:
                break;
        }
    }

    return 0;
}


uint8_t hal_extIntDisable(en_targetExtInt_t e_pin)
{
    s_io_pin_desc_t *ps_desc = &s_target_extIntPin[e_pin];


    if (ps_desc->PORT != NULL) {
        io_irqDisable(ps_desc);
    }
    return 0;
}


uint8_t hal_extIntClear(en_targetExtInt_t e_pin)
{
    s_io_pin_desc_t *ps_desc = &s_target_extIntPin[e_pin];


    if (ps_desc->PORT != NULL) {
        io_irqClear(ps_desc);
    }
    return 0;
}


void hal_delay_us(uint32_t ul_delay)
{
    /*
     * Note(s)
     *
     * hal_delay_us() is only called by emb6.c to make a delay multiple of 500us,
     * which is equivalent to 1 systick
     */
    uint32_t tick_stop;


    hal_enterCritical();
    tick_stop  = hal_ticks;
    tick_stop += ul_delay / 500;
    hal_exitCritical();
    while (tick_stop > hal_ticks) {
        /* do nothing */
    }
}


uint8_t hal_gpioPinInit(uint8_t c_pin, uint8_t c_dir, uint8_t c_initState)
{
    /*TODO missing implementation */
    return 0;
}


void * hal_ctrlPinInit(en_targetExtPin_t e_pinType)
{
    /*TODO missing implementation */
    return NULL;
}


void hal_pinSet(void * p_pin)
{
    /*TODO missing implementation */
}


void hal_pinClr(void * p_pin)
{
    /*TODO missing implementation */
}


uint8_t hal_pinGet(void * p_pin)
{
    return 0;
}


void *hal_spiInit(void)
{
#if (TARGET_CONFIG_RF == TRUE)
    return &hal_rfspi;
#else
    return NULL;
#endif
}


/**
 * @brief   Select/Deselect a SPI driver
 * @param   p_spi
 * @param   action
 * @return
 */
uint8_t hal_spiSlaveSel(void *p_spi, bool action)
{
#if TARGET_CFG_ARG_CHK_EN
    if (p_spi == NULL) {
        return 0;
    }
#endif


    if (action == TRUE) {
        spi_rfSelect();
    } else {
        spi_rfDeselect();
    }
    return 1;
}


void hal_spiSelect(void *p_spi)
{
    spi_rfSelect();
}

void hal_spiDeSelect(void *p_spi)
{
    spi_rfDeselect();
}


void hal_spiTxRx(uint8_t *p_tx, uint8_t *p_rx, uint16_t len)
{
    spi_rfTxRx(p_tx, p_rx, len);
}

/**
 * @brief   Read a given number of data from a SPI bus
 * @param   p_data  Point to buffer holding data to receive
 * @param   i_len   Length of data to receive
 * @return
 */
uint8_t hal_spiRead(uint8_t *p_data, uint16_t i_len)
{
    uint8_t ret;


#if TARGET_CFG_ARG_CHK_EN
    if ((p_data == NULL) ||
        (i_len == 0)) {
        return 0;
    }
#endif

    ret = spi_rfRead(p_data, i_len);
    return ret;
}


/**
 * @brief   Write a given number of bytes onto a SPI bus
 * @param   p_data  Point to data to send
 * @param   i_len   Length of data to send
 */
void hal_spiWrite(uint8_t *p_data, uint16_t i_len)
{
#if TARGET_CFG_ARG_CHK_EN
    if ((p_data == NULL) ||
        (i_len == 0)) {
        return;
    }
#endif

    spi_rfWrite(p_data, i_len);
}


void hal_watchdogReset(void)
{
    /*TODO missing implementation */
}


void hal_watchdogStart(void)
{
    /*TODO missing implementation */
}


void hal_watchdogStop(void)
{
    /*TODO missing implementation */
}


clock_time_t hal_getTick(void)
{
    return TmrCurTick;
}


clock_time_t hal_getSec(void)
{
    clock_time_t secs = 0;

    secs = TmrCurTick / TARGET_CFG_SYSTICK_RESOLUTION;
    return secs;
}


clock_time_t hal_getTRes(void)
{
    return TARGET_CFG_SYSTICK_RESOLUTION;
}
