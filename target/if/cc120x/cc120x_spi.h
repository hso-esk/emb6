/**
 * @file    cc120x_spi.h
 * @date    12.11.2015
 * @author  PN
 * @brief   CC120X SPI HAL
 */

#ifndef CC120X_SPI_PRESENT
#define CC120X_SPI_PRESENT

typedef uint8_t rf_status_t;


rf_status_t cc120x_spiRegRead(uint16_t addr, uint8_t *data, uint8_t len);
rf_status_t cc120x_statusTxGet(void);
rf_status_t cc120x_statusRxGet(void);
rf_status_t cc120x_spiRegWrite(uint16_t addr, uint8_t *data, uint8_t len);
rf_status_t cc120x_spiTxFifoWrite(uint8_t *pWriteData, uint8_t len);
rf_status_t cc120x_spiRxFifoRead(uint8_t *pReadData, uint8_t len);


rf_status_t cc120x_spiCmdStrobe(uint8_t cmd);

#endif /* CC120X_SPI_PRESENT */
