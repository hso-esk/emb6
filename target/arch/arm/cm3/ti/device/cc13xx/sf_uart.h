/**
* @code
*  ___ _____ _   ___ _  _____ ___  ___  ___ ___
* / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
* \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
* |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
* embedded.connectivity.solutions.==============
* @endcode
*
* @file       sf_uart.h
* @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
* @author     STACKFORCE
* @brief      API to the UART interface.
*/

#ifndef __SF_UART_H__
#define __SF_UART_H__

#ifndef __DECL_SF_UART_H__
#define __DECL_SF_UART_H__ extern
#else
#define __UART_INIT_VAR__
#endif /* __DECL_SF_UART_H__ */

 /*! @defgroup sf_mcu_uart sf_mcu_uart driver
     @{
     @ingroup  sf_mcu

     This sections describes the STACKFORCE UART driver. */

/*==============================================================================
                            DEFINES
==============================================================================*/

typedef void (*sf_uart_rx_cb)( uint8_t p_data );

/*==============================================================================
                            FUNCTIONS
==============================================================================*/

/*============================================================================*/
/*!
 * @brief Initializes the UART.
 */
/*============================================================================*/
bool sf_uart_init(void);

/*============================================================================*/
/*!
 * @brief Writes data to the UART.
 * @param pc_data     Array of bytes to write.
 * @param i_len       Number of bytes to write.
 */
/*============================================================================*/
uint16_t sf_uart_write(uint8_t *pc_data, uint16_t i_len);

/*============================================================================*/
/*!
 * @brief Reads data from the UART.
 * @param pc_data     Array to write data in.
 * @param i_len       Number of bytes to read.
 * @return            Number of read bytes.
 */
/*============================================================================*/
uint16_t sf_uart_read(uint8_t *pc_data, uint16_t i_len);

/*============================================================================*/
/*!
 * @brief Returns the number of received bytes within the Rx-buffer.
 * @return Number of received bytes.
 */
/*============================================================================*/
uint16_t sf_uart_cntRxBytes(void);

/*============================================================================*/
/*!
 * @brief Returns the number of bytes which could be added to the Tx-buffer.
 * @return Number of possible transmit bytes.
 */
/*============================================================================*/
uint16_t sf_uart_cntTxBytes(void);

/*============================================================================*/
/*!
 * @brief Checks if there was a Rx overflow.
 *        If this function is called the overflow flag is resetted
 *        automatically.
 * @return  @ref true if there was an overflow.
 */
/*============================================================================*/
bool sf_uart_isRxOverflow(void);

/*============================================================================*/
/*!
  @brief  Runs the MCU UART driver.
          This function should be called as often as possible to ensure proper
          handling of the driver.
==============================================================================*/
void sf_uart_run(void);

/*============================================================================*/
/*!
 * @brief set the call back function for UART Rx.
 */
/*============================================================================*/
void sf_uart_setRxCb(sf_uart_rx_cb  rx_cb);

/*! @} sf_mcu_uart */
#endif /* __SF_UART_H__ */
