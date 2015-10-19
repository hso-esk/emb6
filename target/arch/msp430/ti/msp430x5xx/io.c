/*============================================================================*/
/**
 * \file    io.c
 *
 * \author  Tobias Neff
 *
 * \brief   Initialization of GPIOs
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdlib.h>
#include <stdint.h>
#include <msp430.h>
#include "targetconfig.h"
#include "hal_types.h"

#define __DECL_IO_H__
#include "io.h"

#include "spi.h"
#include "int.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

/** maximum number of PINs per port */
#define IO_PIN_MAX                    8

/*============================================================================*/
/*                                MACROS                                      */
/*============================================================================*/
#ifdef IAR_COMPILER
#define HAL_INT_LOCK(x)    st( (x) = __get_SR_register(); \
                               __disable_interrupt(); )
#endif
#ifdef GCC_COMPILER
#define HAL_INT_LOCK(x)    st( (x) = __get_SR_register(); \
                               __disable_interrupt(); )
#endif
#define HAL_INT_UNLOCK(x)  st( __enable_interrupt(); /*_bis_SR_register(x);*/ )


/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

/* initialize MCU IOs */
static void _io_mcu_init( void );

/* initialize LED IOs */
static void _io_led_init( void );

/* initialize Key IOs */
static void _io_key_init( void );

#if( TARGET_CONFIG_LCD == TRUE )
/* initialize LCD IOs */
static void _io_lcd_init();
#endif /* #if( TARGET_CONFIG_LCD == TRUE ) */

#if( TARGET_CONFIG_RF == TRUE )
/* initialize RF IOs */
static void _io_rf_init();
#endif /* #if( TARGET_CONFIG_RF == TRUE ) */

#if( TARGET_CONFIG_UART0 == TRUE )
/* initialize UART0 */
static void _io_uart0_init();
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */

#if( TARGET_CONFIG_UART1 == TRUE )
/* initialize UART1 */
static void _io_uart1_init();
#endif /* #if( TARGET_CONFIG_UART1 == TRUE ) */

#if( TARGET_CONFIG_UART3 == TRUE )
/* initialize UART3 */
static void _io_uart3_init();
#endif /* #if( TARGET_CONFIG_UART3 == TRUE ) */


/* Port1 interrupt handler */
static void _io_irqP1( void* p_params );

/* Port2 interrupt handler */
static void _io_irqP2( void* p_params );


/*============================================================================*/
/*                            LOCAL VARIABLES                                 */
/*============================================================================*/

/** callbacks for each of the PIN interrupts */
static pf_io_cb gpf_io_irq_cb[E_IO_PORT_MAX][IO_PIN_MAX];

/**
 * Port definitions.
 */
__DECL_IO_H__ s_io_port_desc_t gps_io_port[E_IO_PORT_MAX] = {
#if (TARGET_CONFIG_PORT1 == TRUE)
  /* Port 1 */
  {&PASEL_L, &PADIR_L, &PAIN_L, &PAOUT_L, &PAREN_L, &PADS_L, &PAIFG_L, &PAIE_L, &PAIES_L, E_IO_PORT_P1, E_INT_IRQ_SRC_P1, _io_irqP1},
#endif /* #if (TARGET_CONFIG_PORT1 == TRUE) */
#if (TARGET_CONFIG_PORT2 == TRUE)
  /* Port 2 */
  {&PASEL_H, &PADIR_H, &PAIN_H, &PAOUT_H, &PAREN_H, &PADS_H, &PAIFG_H, &PAIE_H, &PAIES_H, E_IO_PORT_P2, E_INT_IRQ_SRC_P2, _io_irqP2},
#endif /* #if (TARGET_CONFIG_PORT2 == TRUE) */
#if (TARGET_CONFIG_PORT3 == TRUE)
  /* Port 3 */
  {&PBSEL_L, &PBDIR_L, &PBIN_L, &PBOUT_L, &PBREN_L, &PBDS_L, NULL, NULL, NULL, E_IO_PORT_MAX, E_IRQ_SRC_MAX, NULL},
#endif /* #if (TARGET_CONFIG_PORT3 == TRUE) */
#if (TARGET_CONFIG_PORT4 == TRUE)
  /* Port 4 */
  {&PBSEL_H, &PBDIR_H, &PBIN_H, &PBOUT_H, &PBREN_H, &PBDS_H, NULL, NULL, NULL, E_IO_PORT_MAX, E_IRQ_SRC_MAX, NULL},
#endif /* #if (TARGET_CONFIG_PORT4 == TRUE) */
#if (TARGET_CONFIG_PORT5 == TRUE)
  /* Port 5 */
  {&PCSEL_L, &PCDIR_L, &PCIN_L, &PCOUT_L, &PCREN_L, &PCDS_L, NULL, NULL, NULL, E_IO_PORT_MAX, E_IRQ_SRC_MAX, NULL},
#endif /* #if (TARGET_CONFIG_PORT5 == TRUE) */
#if (TARGET_CONFIG_PORT6 == TRUE)
  /* Port 6 */
  {&PCSEL_H, &PCDIR_H, &PCIN_H, &PCOUT_H, &PCREN_H, &PCDS_H, NULL, NULL, NULL, E_IO_PORT_MAX, E_IRQ_SRC_MAX, NULL},
#endif /* #if (TARGET_CONFIG_PORT6 == TRUE) */
#if (TARGET_CONFIG_PORT7 == TRUE)
  /* Port 7 */
  {&PDSEL_L, &PDDIR_L, &PDIN_L, &PDOUT_L, &PDREN_L, &PDDS_L, NULL, NULL, NULL, E_IO_PORT_MAX, E_IRQ_SRC_MAX, NULL},
#endif /* #if (TARGET_CONFIG_PORT7 == TRUE) */
#if (TARGET_CONFIG_PORT8 == TRUE)
  /* Port 8 */
  {&PDSEL_H, &PDDIR_H, &PDIN_H, &PDOUT_H, &PDREN_H, &PDDS_H, NULL, NULL, NULL, E_IO_PORT_MAX, E_IRQ_SRC_MAX, NULL},
#endif /* #if (TARGET_CONFIG_PORT8 == TRUE) */
#if (TARGET_CONFIG_PORT9 == TRUE)
  /* Port 9 */
  {&PESEL_L, &PEDIR_L, &PEIN_L, &PEOUT_L, &PEREN_L, &PEDS_L, NULL, NULL, NULL, E_IO_PORT_MAX, E_IRQ_SRC_MAX, NULL},
#endif /* #if (TARGET_CONFIG_PORT9 == TRUE) */
#if (TARGET_CONFIG_PORT10 == TRUE)
  /* Port 10 */
  {&PESEL_H, &PEDIR_H, &PEIN_H, &PEOUT_H, &PEREN_H, &PEDS_H, NULL, NULL, NULL, E_IO_PORT_MAX, E_IRQ_SRC_MAX, NULL}
#endif /* #if (TARGET_CONFIG_PORT10 == TRUE) */
};


/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief   Initialize MCU IOs.
 */
/*============================================================================*/
static void _io_mcu_init( void )
{

  s_io_pin_desc_t ps_pin_xt1in =
    {&gps_io_port[TARGETCONFIG_MCU_XT1IN_PORT], TARGETCONFIG_MCU_XT1IN_PIN, TARGETCONFIG_MCU_XT1IN_MSK};

  s_io_pin_desc_t ps_pin_xt1out =
    {&gps_io_port[TARGETCONFIG_MCU_XT1OUT_PORT], TARGETCONFIG_MCU_XT1OUT_PIN, TARGETCONFIG_MCU_XT1OUT_MSK};


  /* set the function for the XT1 pins */
  *ps_pin_xt1in.PORT->PSEL |= ps_pin_xt1in.MSK;
  *ps_pin_xt1out.PORT->PSEL |= ps_pin_xt1out.MSK;
}

/*============================================================================*/
/**
 * @brief   Initialize LED IOs.
 */
/*============================================================================*/
static void _io_led_init( void )
{
  int i = 0;
  s_io_pin_desc_t ps_pin[] = {
#if( TARGET_CONFIG_LED1 == TRUE )
    {&gps_io_port[TARGETCONFIG_LED1_PORT], TARGETCONFIG_LED1_PIN, TARGETCONFIG_LED1_MSK},
#endif /* #if( TARGET_CONFIG_LED1 == TRUE ) */
#if( TARGET_CONFIG_LED2 == TRUE )
    {&gps_io_port[TARGETCONFIG_LED2_PORT], TARGETCONFIG_LED2_PIN, TARGETCONFIG_LED2_MSK},
#endif /* #if( TARGET_CONFIG_LED2 == TRUE ) */
#if( TARGET_CONFIG_LED3 == TRUE )
    {&gps_io_port[TARGETCONFIG_LED3_PORT], TARGETCONFIG_LED3_PIN, TARGETCONFIG_LED3_MSK},
#endif /* #if( TARGET_CONFIG_LED3 == TRUE ) */
#if( TARGET_CONFIG_LED4 == TRUE )
    {&gps_io_port[TARGETCONFIG_LED4_PORT], TARGETCONFIG_LED4_PIN, TARGETCONFIG_LED4_MSK},
#endif /* #if( TARGET_CONFIG_LED4 == TRUE ) */
  };

  for( i = 0; i < sizeof(ps_pin)/sizeof(s_io_pin_desc_t); i++ )
  {
    /* initialize the current pin with IO functionality*/
    *ps_pin[i].PORT->PSEL &= ~ps_pin[i].MSK;
    /* set as output */
    *ps_pin[i].PORT->PDIR |= ps_pin[i].MSK;
    /* disable */
#if( TARGET_CONFIG_LED_HL == TRUE )
    *ps_pin[i].PORT->POUT &= ~ps_pin[i].MSK;
#else
    *ps_pin[i].PORT->POUT |= ps_pin[i].MSK;
#endif /* #if( TARGET_CONFIG_LED_HL == TRUE ) */
  }
}


/*============================================================================*/
/**
 * @brief   initialize Key IOs.
 */
/*============================================================================*/
static void _io_key_init( void )
{
  int i = 0;
  s_io_pin_desc_t ps_pin[] = {
#if( TARGET_CONFIG_KEY1 == TRUE )
    {&gps_io_port[TARGETCONFIG_KEY1_PORT], TARGETCONFIG_KEY1_PIN, TARGETCONFIG_KEY1_MSK},
#endif /* #if( TARGET_CONFIG_KEY1 == TRUE ) */
#if( TARGET_CONFIG_KEY == TRUE )
    {&gps_io_port[TARGETCONFIG_KEY2_PORT], TARGETCONFIG_KEY2_PIN, TARGETCONFIG_KEY2_MSK},
#endif /* #if( TARGET_CONFIG_KEY2 == TRUE ) */
#if( TARGET_CONFIG_KEY3 == TRUE )
    {&gps_io_port[TARGETCONFIG_KEY3_PORT], TARGETCONFIG_KEY3_PIN, TARGETCONFIG_KEY3_MSK},
#endif /* #if( TARGET_CONFIG_KEY3 == TRUE ) */
#if( TARGET_CONFIG_KEY4 == TRUE )
    {&gps_io_port[TARGETCONFIG_KEY4_PORT], TARGETCONFIG_KEY4_PIN, TARGETCONFIG_KEY4_MSK},
#endif /* #if( TARGET_CONFIG_KEY4 == TRUE ) */
#if( TARGET_CONFIG_KEY5 == TRUE )
    {&gps_io_port[TARGETCONFIG_KEY5_PORT], TARGETCONFIG_KEY5_PIN, TARGETCONFIG_KEY5_MSK},
#endif /* #if( TARGET_CONFIG_KEY5 == TRUE ) */
  };

  for( i = 0; i < sizeof(ps_pin)/sizeof(s_io_pin_desc_t); i++ )
  {
    /* initialize the current pin with IO functionality*/
    *ps_pin[i].PORT->PSEL &= ~ps_pin[i].MSK;
    /* set as input */
    *ps_pin[i].PORT->PDIR &= ~ps_pin[i].MSK;
    /* set pullup and resistor */
    *ps_pin[i].PORT->POUT |= ps_pin[i].MSK;
    *ps_pin[i].PORT->PREN |= ps_pin[i].MSK;
  }
}


#if( TARGET_CONFIG_LCD == TRUE )
/*============================================================================*/
/**
 * @brief   initialize LCD IOs.
 */
/*============================================================================*/
static void _io_lcd_init()
{
  s_io_pin_desc_t s_pin_spi_mosi = {
    &gps_io_port[TARGETCONFIG_LCD_SPI_MOSI_PORT], TARGETCONFIG_LCD_SPI_MOSI_PIN, TARGETCONFIG_LCD_SPI_MOSI_MSK,
  };
  s_io_pin_desc_t s_pin_spi_miso = {
    &gps_io_port[TARGETCONFIG_LCD_SPI_MISO_PORT], TARGETCONFIG_LCD_SPI_MISO_PIN, TARGETCONFIG_LCD_SPI_MISO_MSK,
  };
  s_io_pin_desc_t s_pin_spi_sclk = {
    &gps_io_port[TARGETCONFIG_LCD_SPI_SCLK_PORT], TARGETCONFIG_LCD_SPI_SCLK_PIN, TARGETCONFIG_LCD_SPI_SCLK_MSK,
  };
  s_io_pin_desc_t s_pin_spi_csn = {
    &gps_io_port[TARGETCONFIG_LCD_SPI_CSN_PORT], TARGETCONFIG_LCD_SPI_CSN_PIN, TARGETCONFIG_LCD_SPI_CSN_MSK,
  };

  s_io_pin_desc_t s_pin_mode = {
    &gps_io_port[TARGETCONFIG_LCD_MODE_PORT], TARGETCONFIG_LCD_MODE_PIN, TARGETCONFIG_LCD_MODE_MSK,
  };
  s_io_pin_desc_t s_pin_rst = {
    &gps_io_port[TARGETCONFIG_LCD_RST_PORT], TARGETCONFIG_LCD_RST_PIN, TARGETCONFIG_LCD_RST_MSK,
  };
  s_io_pin_desc_t s_pin_pwr = {
    &gps_io_port[TARGETCONFIG_LCD_PWR_PORT], TARGETCONFIG_LCD_PWR_PIN, TARGETCONFIG_LCD_PWR_MSK,
  };

  /* Initialize MOSI as alternate function and as output */
  *s_pin_spi_mosi.PORT->PSEL |= s_pin_spi_mosi.MSK;
  *s_pin_spi_mosi.PORT->PDIR |= s_pin_spi_mosi.MSK;

  /* Initialize MISO as alternate function and as input with pullup */
  *s_pin_spi_miso.PORT->PSEL |= s_pin_spi_miso.MSK;
  *s_pin_spi_miso.PORT->PDIR &= ~s_pin_spi_miso.MSK;
  *s_pin_spi_miso.PORT->POUT |= s_pin_spi_miso.MSK;

  /* Initialize SCLK as alternate function and as output */
  *s_pin_spi_sclk.PORT->PSEL |= s_pin_spi_sclk.MSK;
  *s_pin_spi_sclk.PORT->PDIR |= s_pin_spi_sclk.MSK;

  /* Initialize CSN as GPIO and as output. Pull high. */
  *s_pin_spi_csn.PORT->PSEL &= ~s_pin_spi_csn.MSK;
  *s_pin_spi_csn.PORT->PDIR |= s_pin_spi_csn.MSK;
  *s_pin_spi_csn.PORT->POUT |= s_pin_spi_csn.MSK;

  /* Initialize MODE as GPIO and as output. Pull high. */
  *s_pin_mode.PORT->PSEL &= ~s_pin_mode.MSK;
  *s_pin_mode.PORT->PDIR |= s_pin_mode.MSK;
  *s_pin_mode.PORT->POUT |= s_pin_mode.MSK;

  /* Initialize RST as GPIO and as output. Pull low. */
  *s_pin_rst.PORT->PSEL &= ~s_pin_rst.MSK;
  *s_pin_rst.PORT->PDIR |= s_pin_rst.MSK;
  *s_pin_rst.PORT->POUT &= ~s_pin_rst.MSK;

  /* Initialize PWR as GPIO and as output with high drive strength. Pull Low. */
  *s_pin_pwr.PORT->PSEL &= ~s_pin_pwr.MSK;
  *s_pin_pwr.PORT->PDIR |= s_pin_pwr.MSK;
  *s_pin_pwr.PORT->POUT &= ~s_pin_pwr.MSK;
  *s_pin_pwr.PORT->PDS |= s_pin_pwr.MSK;
}
#endif /* #if( TARGET_CONFIG_LCD == TRUE ) */

#if( TARGET_CONFIG_RF == TRUE )
/*============================================================================*/
/**
 * @brief   initialize RF IOs.
 */
/*============================================================================*/
static void _io_rf_init()
{
  s_io_pin_desc_t s_pin_spi_mosi = {
    &gps_io_port[TARGETCONFIG_RF_SPI_MOSI_PORT], TARGETCONFIG_RF_SPI_MOSI_PIN, TARGETCONFIG_RF_SPI_MOSI_MSK,
  };
  s_io_pin_desc_t s_pin_spi_miso = {
    &gps_io_port[TARGETCONFIG_RF_SPI_MISO_PORT], TARGETCONFIG_RF_SPI_MISO_PIN, TARGETCONFIG_RF_SPI_MISO_MSK,
  };
  s_io_pin_desc_t s_pin_spi_sclk = {
    &gps_io_port[TARGETCONFIG_RF_SPI_SCLK_PORT], TARGETCONFIG_RF_SPI_SCLK_PIN, TARGETCONFIG_RF_SPI_SCLK_MSK,
  };
  s_io_pin_desc_t s_pin_spi_csn = {
    &gps_io_port[TARGETCONFIG_RF_SPI_CSN_PORT], TARGETCONFIG_RF_SPI_CSN_PIN, TARGETCONFIG_RF_SPI_CSN_MSK,
  };

  s_io_pin_desc_t s_pin_gpio0 = {
    &gps_io_port[TARGETCONFIG_RF_GPIO0_PORT], TARGETCONFIG_RF_GPIO0_PIN, TARGETCONFIG_RF_GPIO0_MSK,
  };
  s_io_pin_desc_t s_pin_gpio2 = {
    &gps_io_port[TARGETCONFIG_RF_GPIO2_PORT], TARGETCONFIG_RF_GPIO2_PIN, TARGETCONFIG_RF_GPIO2_MSK,
  };
  s_io_pin_desc_t s_pin_gpio3 = {
    &gps_io_port[TARGETCONFIG_RF_GPIO3_PORT], TARGETCONFIG_RF_GPIO3_PIN, TARGETCONFIG_RF_GPIO3_MSK,
  };

  /* Initialize MOSI as alternate function and as output */
  *s_pin_spi_mosi.PORT->PSEL |= s_pin_spi_mosi.MSK;
  *s_pin_spi_mosi.PORT->PDIR |= s_pin_spi_mosi.MSK;

  /* Initialize MISO as alternate function and as input with pullup */
  *s_pin_spi_miso.PORT->PSEL |= s_pin_spi_miso.MSK;
  *s_pin_spi_miso.PORT->PDIR &= ~s_pin_spi_miso.MSK;
  *s_pin_spi_miso.PORT->POUT |= s_pin_spi_miso.MSK;

  /* Initialize SCLK as alternate function and as output */
  *s_pin_spi_sclk.PORT->PSEL |= s_pin_spi_sclk.MSK;
  *s_pin_spi_sclk.PORT->PDIR |= s_pin_spi_sclk.MSK;

  /* Initialize CSN as GPIO and as output. Pull high. */
  *s_pin_spi_csn.PORT->PSEL &= ~s_pin_spi_csn.MSK;
  *s_pin_spi_csn.PORT->PDIR |= s_pin_spi_csn.MSK;
  *s_pin_spi_csn.PORT->POUT |= s_pin_spi_csn.MSK;

  /* Initialize GPIO0 as GPIO and as input. */
  *s_pin_gpio0.PORT->PSEL &= ~s_pin_gpio0.MSK;
  *s_pin_gpio0.PORT->PDIR &= ~s_pin_gpio0.MSK;
  *s_pin_gpio0.PORT->POUT &= ~s_pin_gpio0.MSK;

  /* Initialize GPIO2 as GPIO and as input. */
  *s_pin_gpio2.PORT->PSEL &= ~s_pin_gpio2.MSK;
  *s_pin_gpio2.PORT->PDIR &= ~s_pin_gpio2.MSK;
  *s_pin_gpio2.PORT->POUT &= ~s_pin_gpio2.MSK;

  /* Initialize GPIO3 as GPIO and as input. */
  *s_pin_gpio3.PORT->PSEL &= ~s_pin_gpio3.MSK;
  *s_pin_gpio3.PORT->PDIR &= ~s_pin_gpio3.MSK;
  *s_pin_gpio3.PORT->POUT &= ~s_pin_gpio3.MSK;
}
#endif /* #if( TARGET_CONFIG_RF == TRUE ) */

#if( TARGET_CONFIG_UART0 == TRUE )
/*============================================================================*/
/**
 * @brief   initialize UART0 IOs.
 */
/*============================================================================*/
static void _io_uart0_init()
{
  s_io_pin_desc_t s_pin_rx = {
    &gps_io_port[TARGETCONFIG_UART0_RX_PORT], TARGETCONFIG_UART0_RX_PIN, TARGETCONFIG_UART0_RX_MSK,
  };
  s_io_pin_desc_t s_pin_tx = {
    &gps_io_port[TARGETCONFIG_UART0_TX_PORT], TARGETCONFIG_UART0_TX_PIN, TARGETCONFIG_UART0_TX_MSK,
  };

  /* Set Rx Pin as alternate function and input*/
  *s_pin_rx.PORT->PSEL |= s_pin_rx.MSK;
  *s_pin_rx.PORT->PDIR &= ~s_pin_rx.MSK;

   /* Set Tx Pin as alternate function and output*/
  *s_pin_tx.PORT->PSEL |= s_pin_tx.MSK;
  *s_pin_tx.PORT->PDIR |= s_pin_tx.MSK;
}
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */

#if( TARGET_CONFIG_UART1 == TRUE )
/*============================================================================*/
/**
 * @brief   initialize UART1 IOs.
 */
/*============================================================================*/
static void _io_uart1_init()
{
  s_io_pin_desc_t s_pin_rx = {
    &gps_io_port[TARGETCONFIG_UART1_RX_PORT], TARGETCONFIG_UART1_RX_PIN, TARGETCONFIG_UART1_RX_MSK,
  };
  s_io_pin_desc_t s_pin_tx = {
    &gps_io_port[TARGETCONFIG_UART1_TX_PORT], TARGETCONFIG_UART1_TX_PIN, TARGETCONFIG_UART1_TX_MSK,
  };

  /* Set Rx Pin as alternate function and input*/
  *s_pin_rx.PORT->PSEL |= s_pin_rx.MSK;
  *s_pin_rx.PORT->PDIR &= ~s_pin_rx.MSK;

   /* Set Tx Pin as alternate function and output*/
  *s_pin_tx.PORT->PSEL |= s_pin_tx.MSK;
  *s_pin_tx.PORT->PDIR |= s_pin_tx.MSK;
}
#endif /* #if( TARGET_CONFIG_UART1 == TRUE ) */

#if( TARGET_CONFIG_UART3 == TRUE )
/*============================================================================*/
/**
 * @brief   initialize UART3 IOs.
 */
/*============================================================================*/
static void _io_uart3_init()
{
  s_io_pin_desc_t s_pin_rx = {
    &gps_io_port[TARGETCONFIG_UART3_RX_PORT], TARGETCONFIG_UART3_RX_PIN, TARGETCONFIG_UART3_RX_MSK,
  };
  s_io_pin_desc_t s_pin_tx = {
    &gps_io_port[TARGETCONFIG_UART3_TX_PORT], TARGETCONFIG_UART3_TX_PIN, TARGETCONFIG_UART3_TX_MSK,
  };

  /* Set Rx Pin as alternate function and input*/
  *s_pin_rx.PORT->PSEL |= s_pin_rx.MSK;
  *s_pin_rx.PORT->PDIR &= ~s_pin_rx.MSK;

   /* Set Tx Pin as alternate function and output*/
  *s_pin_tx.PORT->PSEL |= s_pin_tx.MSK;
  *s_pin_tx.PORT->PDIR |= s_pin_tx.MSK;
}
#endif /* #if( TARGET_CONFIG_UART3 == TRUE ) */

/*=============================================================================*/
/**
 * @brief   initialize UART3 IOs.
 */
 /*============================================================================*/
static void _io_irqP1( void* p_params )
{
  int i = 0;
  e_io_port_t e_port = E_IO_PORT_P1;

  /* get port flags */
  uint8_t uc_flags = *((uint8_t*)p_params);

  if( uc_flags )
  {
    /* check all the bits within the interrupt register and call the according
     interrupt if registered */
    for (i = 0; i < IO_PIN_MAX; i++)
    {
      register const uint8_t c_bitMask = (1 << i);
      if((uc_flags & c_bitMask) && (gpf_io_irq_cb[e_port][i] != NULL))
        /* call registered interrupt */
        gpf_io_irq_cb[e_port][i](NULL);
    }
  }
}

/*=============================================================================*/
/**
 * @brief   initialize UART3 IOs.
 */
 /*============================================================================*/
static void _io_irqP2( void* p_params )
{
  int i = 0;
  e_io_port_t e_port = E_IO_PORT_P2;

  /* get port flags */
  uint8_t uc_flags = *((uint8_t*)p_params);

  if( uc_flags )
  {
    /* check all the bits within the interrupt register and call the according
     interrupt if registered */
    for (i = 0; i < IO_PIN_MAX; i++)
    {
      register const uint8_t c_bitMask = (1 << i);
      if((uc_flags & c_bitMask) && (gpf_io_irq_cb[e_port][i] != NULL))
        /* call registered interrupt */
        gpf_io_irq_cb[e_port][i](NULL);
    }
  }
}

/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/


/*=============================================================================
 *  io_init()
 *============================================================================*/
void io_init (void)
{
  /* Initialization of MCU IOs */
  _io_mcu_init();

  /* Initialization of LEDs */
  _io_led_init();

  /* Initialization of Keys */
  _io_key_init();

#if( TARGET_CONFIG_UART0 == TRUE )
  /* Initialization of UART0 */
  _io_uart0_init();
#endif /* #if( TARGET_CONFIG_UART0 == TRUE ) */

#if( TARGET_CONFIG_UART1 == TRUE )
  /* Initialization of UART1 */
  _io_uart1_init();
#endif /* #if( TARGET_CONFIG_UART1 == TRUE ) */

#if( TARGET_CONFIG_UART3 == TRUE )
  /* Initialization of UART3 */
  _io_uart3_init();
#endif /* #if( TARGET_CONFIG_UART3 == TRUE ) */

#if( TARGET_CONFIG_LCD == TRUE )
  /* initialization of LCD */
  _io_lcd_init();
#endif /* #if( TARGET_CONFIG_LCD == TRUE ) */

#if( TARGET_CONFIG_RF == TRUE )
  /* initialization of RF */
  _io_rf_init();
#endif /* #if( TARGET_CONFIG_RF == TRUE ) */

}/* io_init() */

/*=============================================================================
 *  io_set()
 *============================================================================*/
int8_t io_set( s_io_pin_desc_t* ps_pin )
{
  int8_t i_ret = -1;

  /* Pin must be IO output */
  if( (ps_pin != NULL) && (*ps_pin->PORT->PDIR & ps_pin->MSK )
     && ((*ps_pin->PORT->PSEL & ps_pin->MSK) == 0) )
  {
    /* set the Pin */
    *ps_pin->PORT->POUT |= ps_pin->MSK;
    i_ret = 0;
  }

  return i_ret;
}/* io_set() */


/*=============================================================================
 *  io_clear()
 *============================================================================*/
int8_t io_clear( s_io_pin_desc_t* ps_pin )
{
  int8_t i_ret = -1;

  /* Pin must be IO output */
  if( (ps_pin != NULL) && (*ps_pin->PORT->PDIR & ps_pin->MSK )
     && ((*ps_pin->PORT->PSEL & ps_pin->MSK) == 0) )
  {
    /* set the Pin */
    *ps_pin->PORT->POUT &= ~ps_pin->MSK;
    i_ret = 0;
  }

  return i_ret;
}/* io_clear() */


/*=============================================================================
 *  io_get()
 *============================================================================*/
int8_t io_get( s_io_pin_desc_t* ps_pin )
{
  int8_t i_ret = -1;

  /* Pin must be IO input */
  if( (ps_pin != NULL) && ((*ps_pin->PORT->PDIR & ps_pin->MSK ) == 0)
     && ((*ps_pin->PORT->PSEL & ps_pin->MSK) == 0) )
  {
    /* get the Pin */
    i_ret = (*ps_pin->PORT->PIN & ps_pin->MSK) >> ps_pin->PIN;
  }

  return i_ret;
}/* io_get() */


/*=============================================================================
 *  io_irqEnable()
 *============================================================================*/
int8_t io_irqEnable( s_io_pin_desc_t* ps_pin, uint8_t uc_edge, pf_io_cb pf_cb )
{
  uint8_t uc_state = 0U;

  HAL_INT_LOCK( uc_state );

  /* Check for a valid parameters */
  if( (pf_cb != NULL) && (ps_pin != NULL) )
  {
    /* A valid callback was given. Now check if the port is able
     to handle interrupts */
    if( (*ps_pin->PORT->pf_isr != NULL) && (ps_pin->PORT->e_port < E_IO_PORT_MAX)  )
    {
      /* Port is able to handle interrupts. Now activate the interrupt and
       set the according callback function */
      int_irqRegister( ps_pin->PORT->e_irqSrc, ps_pin->PORT->pf_isr );

      /* Set the callback function */
      gpf_io_irq_cb[ps_pin->PORT->e_port][ps_pin->PIN] = pf_cb;
      /* Set edge and interrupt enable */
      if (uc_edge == INT_EDGE_FALLING)
        *ps_pin->PORT->PIES |= ps_pin->MSK;
      else
        *ps_pin->PORT->PIES &= ~ps_pin->MSK;
      *ps_pin->PORT->PIE |= ps_pin->MSK;

      /* clear the interrupt */
      io_irqClear( ps_pin );
    }
  }

  HAL_INT_UNLOCK(uc_state);
  
  (void)&uc_state;      /* Prevent warning of unused variable */
  
  return 0;
}/* io_irqEnable() */

/*=============================================================================
 *  io_irqClear()
 *============================================================================*/
int8_t io_irqClear( s_io_pin_desc_t* ps_pin )
{
  /* Check for a valid parameters */
  if( ps_pin != NULL )
  {
    /* clear the interrupt */
    *ps_pin->PORT->PIFG &= ~ps_pin->MSK;
  }

  return 0;
}/* io_irqClear() */

/*=============================================================================
 *  io_irqDisable()
 *============================================================================*/
int8_t io_irqDisable( s_io_pin_desc_t* ps_pin )
{
  /* Check for a valid parameters */
  if( ps_pin != NULL )
  {
    /* disable the interrupt */
    *ps_pin->PORT->PIE &= ~ps_pin->MSK;
  }

  return 0;
}/* io_irqDisable() */
