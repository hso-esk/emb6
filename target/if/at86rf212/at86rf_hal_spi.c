/**
 * @file    at86rf_hal_spi.c
 * @date    18.11.2015
 * @author  PN
 */

#include "emb6.h"
#include "bsp.h"
#include "at86rf_hal_spi.h"

#define     LOGGER_ENABLE        LOGGER_HAL
#include    "logger.h"

static void* p_at86RfSpi;
static void* p_at86RfSpiCs;

void at86rf_halSpiInit(void)
{
    p_at86RfSpiCs = bsp_pinInit( EN_HAL_PIN_RFSPICS );
    p_at86RfSpi = bsp_spiInit( EN_HAL_SPI_RF );
}

uint8_t at86rf_halSpiBitRead(uint8_t c_addr, uint8_t c_mask, uint8_t c_off )
{
    uint8_t c_data;
    if (!bsp_spiSlaveSel(p_at86RfSpi, p_at86RfSpiCs, TRUE, TRUE)) {
        LOG_ERR("%s\n\r","SPI transaction failed - bitread.");
        return 0;
    }

    bsp_spiTx(p_at86RfSpi,&c_addr, 1);
    bsp_spiRx(p_at86RfSpi,&c_data, 1);
    bsp_spiSlaveSel(p_at86RfSpi, p_at86RfSpiCs, FALSE, TRUE);

    c_data &= c_mask;
    c_data >>=  c_off;
    return c_data;
}

uint8_t at86rf_halSpiRegRead(uint8_t c_addr)
{
    uint8_t c_data;
    if (!bsp_spiSlaveSel(p_at86RfSpi, p_at86RfSpiCs, TRUE, TRUE)) {
        LOG_ERR("%s\n\r","SPI transaction failed - regread.");
        return 0;
    }

    bsp_spiTx(p_at86RfSpi,&c_addr, 1);
    bsp_spiRx(p_at86RfSpi,&c_data, 1);
    bsp_spiSlaveSel(p_at86RfSpi, p_at86RfSpiCs, FALSE, TRUE);
    return c_data;
}

void at86rf_halSpiRegWrite(uint8_t c_addr, uint8_t c_data)
{
    if (!bsp_spiSlaveSel(p_at86RfSpi, p_at86RfSpiCs, TRUE, TRUE)) {
        LOG_ERR("%s\n\r","SPI transaction failed - regwrite.");
        return;
    }

    bsp_spiTx(p_at86RfSpi,&c_addr, 1);
    bsp_spiTx(p_at86RfSpi,&c_data, 1);
    bsp_spiSlaveSel(p_at86RfSpi, p_at86RfSpiCs, FALSE, TRUE);
}

void at86rf_halSpiFrameRead(uint8_t c_addr, uint8_t * pc_data, uint16_t * pi_len)
{
    if (!bsp_spiSlaveSel(p_at86RfSpi, p_at86RfSpiCs, TRUE, TRUE)) {
        LOG_ERR("%s\n\r","SPI transaction failed - fread.");
        return;
    }

    bsp_spiTx(p_at86RfSpi,&c_addr, 1);
    bsp_spiRx(p_at86RfSpi,(uint8_t *)pi_len, 1);
    bsp_spiRx(p_at86RfSpi,pc_data, *pi_len + 1);
    bsp_spiSlaveSel(p_at86RfSpi, p_at86RfSpiCs, FALSE, TRUE);
}

void at86rf_halSpiFrameWrite(uint8_t c_addr, uint8_t * pc_data, uint8_t c_len)
{
    if (!bsp_spiSlaveSel(p_at86RfSpi, p_at86RfSpiCs, TRUE, TRUE)) {
        LOG_ERR("%s\n\r","SPI transaction failed - fwrite.");
        return;
    }
    bsp_spiTx(p_at86RfSpi,&c_addr, 1);
    bsp_spiTx(p_at86RfSpi,&c_len, 1);
    bsp_spiTx(p_at86RfSpi,pc_data, (uint16_t)c_len);
    bsp_spiSlaveSel(p_at86RfSpi, p_at86RfSpiCs, FALSE, TRUE);
}


