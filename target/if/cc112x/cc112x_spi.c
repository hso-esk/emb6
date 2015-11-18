/**
 * @file    cc112x_spi.c
 * @date    12.11.2015
 * @author  PN
 */

/*
********************************************************************************
*                                   INCLUDES
********************************************************************************
*/
#include "emb6.h"


#if NETSTK_CFG_RF_CC112X_EN
#include "bsp.h"
#include "cc112x.h"
#include "cc112x_spi.h"


/*
********************************************************************************
*                                LOCAL DEFINES
********************************************************************************
*/
#define STATUS_CHIP_RDY         (uint8_t)( 0x80 )
#define RADIO_BURST_ACCESS      (uint8_t)( 0x40 )
#define RADIO_SINGLE_ACCESS     (uint8_t)( 0x00 )
#define RADIO_READ_ACCESS       (uint8_t)( 0x80 )
#define RADIO_WRITE_ACCESS      (uint8_t)( 0x00 )

#define CC112X_SPI_ON()         do {bsp_spiSlaveSel(cc112x_spiHandle, TRUE );} while(0)
#define CC112X_SPI_OFF()        do {bsp_spiSlaveSel(cc112x_spiHandle, FALSE);} while(0)


/*
********************************************************************************
*                                LOCAL VARIABLES
********************************************************************************
*/
static void *cc112x_spiHandle;



/*
********************************************************************************
*                           LOCAL FUNCTIONS DECLARATION
********************************************************************************
*/
static uint8_t cc112x_spi8bitRegAccess(uint8_t access, uint8_t addr,
        uint8_t *p_data, uint8_t len);

static uint8_t cc112x_spi16BitRegAccess(uint8_t access, uint8_t ext_addr,
        uint8_t reg_addr, uint8_t *p_data, uint8_t len);


/*
********************************************************************************
*                           LOCAL FUNCTIONS DEFINITIONS
********************************************************************************
*/

/**
 *
 * @param uc_accessType
 * @param uc_addressByte
 * @param puc_data
 * @param uc_len
 * @return
 */
static uint8_t cc112x_spi8bitRegAccess(uint8_t     access,
                                       uint8_t     addr,
                                       uint8_t    *p_data,
                                       uint8_t     len)
{
    uint8_t chip_status;
    uint8_t data;


    CC112X_SPI_ON();
    {
        /* Transmit access command */
        data = access | addr;
        bsp_spiTxRx(&data, &chip_status, 1);

        /* Read/Write data */
        if (data & RADIO_READ_ACCESS) {
            if (data & RADIO_BURST_ACCESS) {        /* Burst  Read Access   */
                bsp_spiRead(p_data, len);
            } else {                                /* Single Read Access   */
                bsp_spiRead(p_data, 1);
            }
        } else {
            if (data & RADIO_BURST_ACCESS) {        /* Burst Write Access   */
                bsp_spiWrite(p_data, len);
            } else {                                /* Single Write Access  */
                bsp_spiWrite(p_data, 1);
            }
        }
    }
    CC112X_SPI_OFF();


    return (chip_status);
}


/**
 *
 * @param uc_accessType
 * @param uc_extendedAddr
 * @param uc_regAddr
 * @param puc_data
 * @param uc_len
 * @return
 */
static uint8_t cc112x_spi16BitRegAccess(uint8_t    access,
                                        uint8_t    ext_addr,
                                        uint8_t    reg_addr,
                                        uint8_t   *p_data,
                                        uint8_t    len)
{
    uint8_t chip_status;
    uint8_t data;


    CC112X_SPI_ON();
    {
        /* Transmit access command */
        data = access | ext_addr;
        bsp_spiTxRx(&data, &chip_status, 1);
        bsp_spiTxRx(&reg_addr, &chip_status, 1);

        /* Read/Write data */
        if (data & RADIO_READ_ACCESS) {
            if (data & RADIO_BURST_ACCESS) {        /* Burst  Read Access   */
                bsp_spiRead(p_data, len);
            } else {                                /* Single Read Access   */
                bsp_spiRead(p_data, 1);
            }
        } else {
            if (data & RADIO_BURST_ACCESS) {        /* Burst Write Access   */
                bsp_spiWrite(p_data, len);
            } else {                                /* Single Write Access  */
                bsp_spiWrite(p_data, 1);
            }
        }
    }
    CC112X_SPI_OFF();

    return chip_status;
}


/*
********************************************************************************
*                             API FUNCTIONS DEFINITIONS
********************************************************************************
*/
rf_status_t cc112x_spiRegRead(uint16_t addr, uint8_t *p_data, uint8_t len)
{
    uint8_t status;
    uint8_t access = RADIO_BURST_ACCESS | RADIO_READ_ACCESS;
    uint8_t temp_ext = (uint8_t) (addr >> 8);
    uint8_t temp_addr = (uint8_t) (addr & 0x00FF);


    /* Checking if this is a FIFO access - returns chip not ready */
    if ((temp_addr >= CC112X_SINGLE_TXFIFO) &&
        (temp_ext  == 0)) {
        return STATUS_CHIP_RDY;
    }

    /* Decide what register space is accessed */
    if (!temp_ext) {
        status = cc112x_spi8bitRegAccess(access, temp_addr, p_data, len);
    } else if (temp_ext == 0x2F) {
        status = cc112x_spi16BitRegAccess(access, temp_ext, temp_addr, p_data, len);
    }

    return status;
}


rf_status_t cc112x_spiRegWrite(uint16_t addr, uint8_t *p_data, uint8_t len)
{
    uint8_t status;
    uint8_t access = RADIO_BURST_ACCESS | RADIO_WRITE_ACCESS;
    uint8_t temp_ext = (uint8_t) (addr >> 8);
    uint8_t temp_addr = (uint8_t) (addr & 0x00FF);


    /* Checking if this is a FIFO access - returns chip not ready */
    if ((temp_addr >= CC112X_SINGLE_TXFIFO) &&
        (temp_ext  == 0)) {
        return STATUS_CHIP_RDY;
    }

    /* Decide what register space is accessed */
    if (!temp_ext) {
        status = cc112x_spi8bitRegAccess(access, temp_addr, p_data, len);
    } else if (temp_ext == 0x2F) {
        status = cc112x_spi16BitRegAccess(access, temp_ext, temp_addr, p_data, len);
    }
    return status;
}


rf_status_t cc112x_spiTxFifoWrite(uint8_t *p_data, uint8_t len)
{
    uint8_t status;

    status = cc112x_spi8bitRegAccess(0x00, CC112X_BURST_TXFIFO, p_data, len);
    return status;
}


rf_status_t cc112x_spiRxFifoRead(uint8_t *p_buf, uint8_t len)
{
    uint8_t status;

    status = cc112x_spi8bitRegAccess(0x00, CC112X_BURST_RXFIFO, p_buf, len);
    return status;
}


rf_status_t cc112x_statusTxGet(void)
{
    uint8_t status;

    status = cc112x_spiCmdStrobe(CC112X_SNOP);
    return status;
}


rf_status_t cc112x_statusRxGet(void)
{
    uint8_t status;

    status = cc112x_spiCmdStrobe(CC112X_SNOP | RADIO_READ_ACCESS);
    return status;
}


void cc112x_spiInit(void)
{
    cc112x_spiHandle = bsp_spiInit();
}


/**
 *
 * @param uc_cmd
 * @return
 */
rf_status_t cc112x_spiCmdStrobe(uint8_t cmd)
{
    rf_status_t chip_status;

    CC112X_SPI_ON();
    bsp_spiTxRx(&cmd, &chip_status, 1);
    CC112X_SPI_OFF();

    return chip_status;
}
#endif /* NETSTK_CFG_RF_CC112X_EN */
