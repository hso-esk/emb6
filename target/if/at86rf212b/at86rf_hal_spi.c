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

static void *p_at86RfSpi;

void at86rf_halSpiInit(void)
{
    p_at86RfSpi = bsp_spiInit();
}

uint8_t at86rf_halSpiBitRead(uint8_t c_addr, uint8_t c_mask, uint8_t c_off )
{
    uint8_t c_data;
    if (!bsp_spiSlaveSel(p_at86RfSpi,true)) {
        LOG_ERR("%s\n\r","SPI transaction failed - bitread.");
        return 0;
    }

    bsp_spiWrite(&c_addr, 1);
    bsp_spiRead(&c_data, 1);
    bsp_spiSlaveSel(p_at86RfSpi,false);

    c_data &= c_mask;
    c_data >>=  c_off;
    return c_data;
}

uint8_t at86rf_halSpiRegRead(uint8_t c_addr)
{
    uint8_t c_data;
    if (!bsp_spiSlaveSel(p_at86RfSpi,true)) {
        LOG_ERR("%s\n\r","SPI transaction failed - regread.");
        return 0;
    }

    bsp_spiWrite(&c_addr, 1);
    bsp_spiRead(&c_data, 1);
    bsp_spiSlaveSel(p_at86RfSpi,false);
    return c_data;
}

void at86rf_halSpiRegWrite(uint8_t c_addr, uint8_t c_data)
{
    if (!bsp_spiSlaveSel(p_at86RfSpi,true)) {
        LOG_ERR("%s\n\r","SPI transaction failed - regwrite.");
        return;
    }

    bsp_spiWrite(&c_addr, 1);
    bsp_spiWrite(&c_data, 1);
    bsp_spiSlaveSel(p_at86RfSpi,false);
}

void at86rf_halSpiFrameRead(uint8_t c_addr, uint8_t * pc_data, uint16_t * pi_len)
{
    if (!bsp_spiSlaveSel(p_at86RfSpi,true)) {
        LOG_ERR("%s\n\r","SPI transaction failed - fread.");
        return;
    }

    bsp_spiWrite(&c_addr, 1);
    bsp_spiRead((uint8_t *)pi_len, 1);
    bsp_spiRead(pc_data, *pi_len + 1);
    bsp_spiSlaveSel(p_at86RfSpi,false);
}

void at86rf_halSpiFrameWrite(uint8_t c_addr, uint8_t * pc_data, uint8_t c_len)
{
    if (!bsp_spiSlaveSel(p_at86RfSpi,true)) {
        LOG_ERR("%s\n\r","SPI transaction failed - fwrite.");
        return;
    }
    bsp_spiWrite(&c_addr, 1);
    bsp_spiWrite(&c_len, 1);
    bsp_spiWrite(pc_data, (uint16_t)c_len);
    bsp_spiSlaveSel(p_at86RfSpi,false);
}


