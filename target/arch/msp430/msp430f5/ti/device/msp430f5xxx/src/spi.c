/*============================================================================*/
/**
 * \file    spi.c
 *
 * \author  Tobias Neff
 *
 * \brief   SPI functions.
 *
 */
/*============================================================================*/

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include <stdint.h>
#include <stdlib.h>
#include <msp430.h>
#include "targetconfig.h"
#include "hal_types.h"
#include "spi.h"
#include "io.h"
#include "mcu.h"

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/

#define RADIO_BURST_ACCESS   0x40
#define RADIO_SINGLE_ACCESS  0x00
#define RADIO_READ_ACCESS    0x80
#define RADIO_WRITE_ACCESS   0x00

/** Counter timout for register accesses */
#define SPI_REGACCESS_CNTTOT            1000

/*============================================================================*/
/*                                MACROS                                      */
/*============================================================================*/

/* Macro for asserting LCD CSn (set low) */
#define SPI_LCD_BEGIN()         (*ps_spi_LcdPinCsn.PORT->POUT &= ~ps_spi_LcdPinCsn.MSK)

/* Macro for deasserting LCD CSn (set high) */
#define SPI_LCD_END()           (*ps_spi_LcdPinCsn.PORT->POUT |= ps_spi_LcdPinCsn.MSK)

/* Condition for SPI TX-buffer to be ready for new data */
#define SPI_LCD_TX_BUF_READY()      (UCB2IFG & UCTXIFG)

/* Condition for SPI transmission to be completed. */
#define SPI_LCD_TX_BUSY()     (UCB2STAT & UCBUSY)

/* Pull CS_N low before starting SPI use. */
#define SPI_TRX_BEGIN()      st( (*ps_spi_RfPinCsn.PORT->POUT &= ~ps_spi_RfPinCsn.MSK); NOP(); )

#define SPI_TRX_TX(x)        st( UCB0IFG &= ~UCRXIFG; UCB0TXBUF= (x); )

/* Wait until RX flag is 1, which means RXBUF has received 1 byte (CS). */
#define SPI_TRX_WAIT_DONE()    st( while(!(UCB0IFG & UCRXIFG)); )

#define SPI_TRX_RX()     UCB0RXBUF

#define SPI_TRX_WAIT_MISO_LOW(x)   st( uint8 uc_count = 200; \
                                           while(TRX_PORT_IN & TRX_SPI_MISO_PIN) \
                                           { \
                                              __delay_cycles(5000); \
                                              uc_count--; \
                                              if (uc_count == 0) break; \
                                           } \
                                           if(uc_count>0) (x) = 1; \
                                           else (x) = 0; )

#define SPI_TRX_END()                st( NOP(); *ps_spi_RfPinCsn.PORT->POUT |= ps_spi_RfPinCsn.MSK; )

/*============================================================================*/
/*                             ENUMERATIONS                                   */
/*============================================================================*/

/*============================================================================*/
/*                              STRUCTURES                                    */
/*============================================================================*/

/*============================================================================*/
/*                               CONSTANTS                                    */
/*============================================================================*/

/*============================================================================*/
/*                            LOCAL VARIABLES                                 */
/*============================================================================*/

#if( TARGET_CONFIG_LCD == TRUE )
static s_io_pin_desc_t ps_spi_LcdPinCsn = {
    &gps_io_port[TARGETCONFIG_LCD_SPI_CSN_PORT], TARGETCONFIG_LCD_SPI_CSN_PIN, TARGETCONFIG_LCD_SPI_CSN_MSK,
};
#endif /* #if( TARGET_CONFIG_LCD == TRUE ) */

static s_io_pin_desc_t ps_spi_RfPinCsn = {
    &gps_io_port[TARGETCONFIG_RF_SPI_CSN_PORT], TARGETCONFIG_RF_SPI_CSN_PIN, TARGETCONFIG_RF_SPI_CSN_MSK,
};

static s_io_pin_desc_t ps_spi_RfPinMiso = {
    &gps_io_port[TARGETCONFIG_RF_SPI_MISO_PORT], TARGETCONFIG_RF_SPI_MISO_PIN, TARGETCONFIG_RF_SPI_MISO_MSK,
};

/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

/*============================================================================*/
/*                              FUNCTIONS                                     */
/*============================================================================*/


/*=============================================================================
 * spi_spiSend()
 *============================================================================*/
void spi_spiSend(e_spi_port_t e_spiSel, char* pc_array, uint16_t ui_size,
    uint8_t uc_address, uint8_t *pc_data, uint16_t ui_len )
{
  if (e_spiSel == E_SPI_PORT_LCD)
  {
#if( TARGET_CONFIG_LCD == TRUE )
    SPI_LCD_BEGIN(); /* asserting LCD CSn (set low) */

    while (ui_size--)
    {
      /*
       * Wait for TX buffer to be ready, send control data and increment
       * data pointer.
       */
      while (!SPI_LCD_TX_BUF_READY())
        ;
      UCB2TXBUF = *pc_array;
      pc_array++;
    }

    /* Wait for transmission to complete before emptying RX buffer */
    while (SPI_LCD_TX_BUSY())
      ;
    UCB2IFG &= ~UCRXIFG;

    SPI_LCD_END(); /* deasserting LCD CSn (set high) */
#endif /* #if( TARGET_CONFIG_LCD == TRUE ) */
  } else if (e_spiSel == E_SPI_PORT_RF)
  {

    uint16_t ui_counter;

    /* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
    /* Check if read (1) or write(0) [R/W Single/Burst A5 A4 A3 A2 A1 A0] . */
    if (uc_address & RADIO_READ_ACCESS)
    {
      /* Check if burst access. */
      if (uc_address & RADIO_BURST_ACCESS)
      {
        for (ui_counter = 0; ui_counter < ui_len; ui_counter++)
        {
          SPI_TRX_TX(0); /* Possible to combining read and write as one access type */
          SPI_TRX_WAIT_DONE();
          *pc_data = SPI_TRX_RX(); /* Store pc_data from last pData RX */
          pc_data++;
        }
      } else
      {
        SPI_TRX_TX(0);
        SPI_TRX_WAIT_DONE();
        *pc_data = SPI_TRX_RX();
      }
    } else
    {
      if (uc_address & RADIO_BURST_ACCESS)
      {
        /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
        for (ui_counter = 0; ui_counter < ui_len; ui_counter++)
        {
          SPI_TRX_TX(*pc_data);
          SPI_TRX_WAIT_DONE();

          pc_data++;
        }
      } else
      {
        SPI_TRX_TX(*pc_data);
        SPI_TRX_WAIT_DONE();
      }
    }
  } else
  {
    /* unknown SPI port */
  }
}


/*=============================================================================
 * spi_trxInit()
 *============================================================================*/
void spi_trxInit (uint8_t uc_clockDivider)
{
  /* keep peripheral in reset state. */
  UCB0CTL1 |= UCSWRST;

  UCB0CTL0 = UCMST + UCSYNC + UCMODE_0 + UCMSB + UCCKPH;
  UCB0CTL1 |= UCSSEL_2;

  /* Set the clock divider */
  UCB0BR1 = 0x00;
  UCB0BR0 = uc_clockDivider;

  /* Release for operation */
  UCB0CTL1 &= ~UCSWRST;
}

/*=============================================================================
 * spi_lcdInit()
 *============================================================================*/
void spi_lcdInit (uint32_t ul_clockSpeed)
{
#if( TARGET_CONFIG_LCD == TRUE )
  uint16_t ui_div;
  uint32_t ul_sysClk = mcu_sysClockSpeedGet();

  /* Check arguments */
  if (ul_clockSpeed > ul_sysClk)
  {
    ul_clockSpeed = ul_sysClk;
  }
  if (ul_clockSpeed > 20000000)
  {
    /* Clock speed too high. Max 20 MHz for signal integrity. */
    ul_clockSpeed = 20000000;
  }

  /* Calculate divider and store actual clock speed */
  ui_div = ul_sysClk / ul_clockSpeed;
  if (ul_sysClk % ul_clockSpeed)
  {
    /* Choose the closest, low-side rate */
    ui_div++;
  }

  /*
   * Configure USCI B2 for SPI master. Disabling SPI module.
   * Configuration: Synchronous mode, 3-pin PSI, master, 8-bit, MSB
   * first, clock pol=1 (inactive high), clock pha=0 (sampling on second
   * edge), clock source = SMCLK.
   */
  UCB2CTL1 |= UCSWRST;
  UCB2CTL0 = UCSYNC | UCMST | UCCKPL | UCMSB;
  UCB2CTL1 |= UCSSEL1 | UCSSEL0;

  /* Apply divider (low nibble, high nibble) */
  UCB2BR0 = (ui_div & 0xFF);
  UCB2BR1 = ((ui_div >> 8) & 0xFF);

  /* Enable SPI interface */
  UCB2CTL1 &= ~UCSWRST;
#endif /* #if( TARGET_CONFIG_LCD == TRUE ) */
}


/*
********************************************************************************
*                               REFACTORING
********************************************************************************
*/
/**
 * @brief   Initialize RF SPI port
 */
void spi_rfInit(uint8_t divider)
{
    spi_trxInit(divider);
}


/**
 * @brief   Select RF SPI driver
 */
void spi_rfSelect(void)
{
    SPI_TRX_BEGIN();

    /* wait for S0 to go low before communication starts */
    int8_t is_on;
    do {
        is_on = io_get(&ps_spi_RfPinMiso);
    } while (is_on != 0);
}


/**
 * @brief   Deselect RF SPI driver
 */
void spi_rfDeselect(void)
{
    SPI_TRX_END();
}

void spi_rfTxRx(uint8_t *p_tx, uint8_t *p_rx, uint16_t len)
{
    while (len--) {
        SPI_TRX_TX(*p_tx++);
        SPI_TRX_WAIT_DONE();
        *p_rx++ = SPI_TRX_RX();
    }
}


/**
 * @brief   Read a given number of bytes from SPI bus
 */
uint8_t spi_rfRead(uint8_t *p_data, uint16_t len)
{
    uint8_t dummy_data = 0xff;
    uint8_t chip_status;


    while (len--) {
        SPI_TRX_TX(dummy_data);
        SPI_TRX_WAIT_DONE();
        chip_status = SPI_TRX_RX();
        *p_data++ = chip_status;
    }
    return chip_status;
}


/**
 * @brief   Write a given number of bytes onto SPI bus
 */
void spi_rfWrite(uint8_t *p_data, uint16_t len)
{
    while (len--) {
        SPI_TRX_TX(*p_data++);
        SPI_TRX_WAIT_DONE();
    }
}
