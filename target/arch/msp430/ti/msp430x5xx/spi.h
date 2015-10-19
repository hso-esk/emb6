#ifndef __SPI_H__
#define __SPI_H__

/*============================================================================*/
/**
 * \file    spi.h
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
#include <msp430.h>
#include <stdint.h>
#include "targetconfig.h"

/*============================================================================*/
/*                                MACROS                                      */
/*============================================================================*/

/** */
#define st(x)   do { x } while (__LINE__ == -1)

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/**
 * Available SPI ports
 */
typedef enum E_SPI_PORT_T
{
  /** LCD port */
  E_SPI_PORT_LCD,
  /** RF port */
  E_SPI_PORT_RF,

  E_SPI_PORT_MAX

} e_spi_port_t;

/*============================================================================*/
/*                          FUNCTION PROTOTYPES                               */
/*============================================================================*/

/*============================================================================*/
/**
 * @brief              Sends data over SPI to LCD or transceiver.
 *
 *
 * @param e_spiSel     Select SPI module. Valid values: SPI_LCD, SPI_TRX
 * @param pc_array     Data for the LCD.
 * @param ui_size      Size of data for LCD.
 * @param uc_address   Register address of the transceiver.
 * @param pc_data      Data for the transceiver.
 * @param ui_len       Size of data of transceiver.
 *
 */
/*============================================================================*/
void spi_spiSend( e_spi_port_t e_spiSel, char* pc_array, uint16_t ui_size,
    uint8_t uc_address, uint8_t *pc_data, uint16_t ui_len);

/*============================================================================*/
/**
 * @brief               8 bit register access.
 *
 *
 * @param uc_accessType         Access type.
 * @param uc_addressByte        Register address.
 * @param pc_data               Data to send.
 * @param uc_len                Size of data to be sent.
 *
 * @return          Chip Status.
 */
/*============================================================================*/
uint8_t spi_8BitRegAccess( uint8_t uc_accessType, uint8_t uc_addressByte,
    uint8_t *pc_data, uint8_t uc_len );

/*============================================================================*/
/**
 * @brief               16 bit register access.
 *
 *
 * @param uc_accessType         Access type.
 * @param uc_extendedAddr       Extended address.
 * @param uc_regAddr            Register address.
 * @param pc_data               Data to send.
 * @param uc_len                Size of data to be sent.
 *
 * @return          Chip status.
 */
/*============================================================================*/
uint8_t spi_16BitRegAccess( uint8_t uc_accessType, uint8_t uc_extendedAddr,
    uint8_t uc_regAddr, uint8_t *pc_data, uint8_t uc_len );

/*============================================================================*/
/**
 * @brief       Sending command strobes.
 *
 *
 * @param uc_cmd        Command to be sent.
 *
 * @return          Chip status.
 */
/*============================================================================*/
uint8_t spi_cmdStrobe( uint8_t uc_cmd );

/*============================================================================*/
/**
 * @brief Initiates transceiver SPI interfaces. SMCLK is chosen as clock
 *      source.
 *
 * @param uc_clockDivider  SMCLK/c_clockDivider gives SCLK frequency.
 *
 * @return          None
 */
/*============================================================================*/
void spi_trxInit( uint8_t uc_clockDivider );

/*============================================================================*/
/**
 * @brief    Function initializes the SPI interface specified by \e ui8Spi to
 *           the baud rate specified by \e ui32ClockSpeed.
 *
 *           The actual SPI clock speed may differ from \e ui32ClockSpeed if
 *           (SMCLK divided by \e ui32ClockSpeed is not an integer. In this case
 *           the SPI clock speed will be the nearest frequency less than
 *           \e ui32ClockSpeed.
 *
 * @param    ul_clockSpeed   is the SPI baudrate in hertz. The actual SPI clock
 *                           speed configured may be less than the specified.
 *
 * @return   None
 */
/*============================================================================*/
void spi_lcdInit( uint32_t ul_clockSpeed );


/*
********************************************************************************
*                               REFACTORING
********************************************************************************
*/
void spi_rfInit(uint8_t divider);
void spi_rfSelect(void);
void spi_rfDeselect(void);
void spi_rfTxRx(uint8_t *p_tx, uint8_t *p_rx, uint16_t len);
uint8_t spi_rfRead(uint8_t *p_data, uint16_t len);
void spi_rfWrite(uint8_t *p_data, uint16_t len);


#endif /* #ifndef __SPI_H__ */
