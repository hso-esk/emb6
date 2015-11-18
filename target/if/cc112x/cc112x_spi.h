/**
 * @file    cc112x_spi.h
 * @date    12.11.2015
 * @author  PN
 * @brief   CC112X SPI HAL
 */

#ifndef CC112X_SPI_PRESENT
#define CC112X_SPI_PRESENT

typedef uint8_t rf_status_t;

void cc112x_spiInit(void);

rf_status_t cc112x_spiRegRead(uint16_t addr, uint8_t *data, uint8_t len);
rf_status_t cc112x_statusTxGet(void);
rf_status_t cc112x_statusRxGet(void);
rf_status_t cc112x_spiRegWrite(uint16_t addr, uint8_t *data, uint8_t len);
rf_status_t cc112x_spiTxFifoWrite(uint8_t *pWriteData, uint8_t len);
rf_status_t cc112x_spiRxFifoRead(uint8_t *pReadData, uint8_t len);


rf_status_t cc112x_spiCmdStrobe(uint8_t cmd);

#endif /* CC112X_SPI_PRESENT */
