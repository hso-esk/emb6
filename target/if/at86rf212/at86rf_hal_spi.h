/**
 * @file    at86rf_hal_spi.h
 * @date    18.11.2015
 * @author  PN
 */

#ifndef __AT86RF_HAL_SPI_H__
#define __AT86RF_HAL_SPI_H__

void    at86rf_halSpiInit(void);
uint8_t at86rf_halSpiBitRead(uint8_t c_addr, uint8_t c_mask, uint8_t c_off );
uint8_t at86rf_halSpiRegRead(uint8_t c_addr);
void    at86rf_halSpiRegWrite(uint8_t c_addr, uint8_t  c_data);
void    at86rf_halSpiFrameRead(uint8_t c_addr, uint8_t * pc_data, uint16_t * pi_len);
void    at86rf_halSpiFrameWrite(uint8_t c_addr, uint8_t * pc_data, uint8_t c_len);


#endif /* __AT86RF_HAL_SPI_H__ */
