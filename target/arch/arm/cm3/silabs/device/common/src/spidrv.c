/***************************************************************************//**
 * @file spidrv.c
 * @brief SPIDRV API implementation.
 * @version 3.20.5
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include <string.h>

#include "em_device.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_usart.h"

#include "dmactrl.h"
#include "spidrv.h"

/// @cond DO_NOT_INCLUDE_WITH_DOXYGEN

static bool spidrvIsInitialized = false;

static void     BlockingComplete( SPIDRV_Handle_t handle,
                                  Ecode_t transferStatus,
                                  int itemsTransferred );

static Ecode_t  ConfigGPIO(       SPIDRV_Handle_t handle, bool enable );

static void     RxDMAComplete(    unsigned int channel,
                                  bool primary,
                                  void *user );

#if defined( EMDRV_SPIDRV_INCLUDE_SLAVE )
static void     SlaveTimeout(     RTCDRV_TimerID_t id,
                                  void *user );
#endif

static void     StartReceiveDMA(  SPIDRV_Handle_t handle,
                                  void *buffer,
                                  int count,
                                  SPIDRV_Callback_t callback );

static void     StartTransferDMA( SPIDRV_Handle_t handle,
                                  const void *txBuffer,
                                  void *rxBuffer,
                                  int count,
                                  SPIDRV_Callback_t callback );

static void     StartTransmitDMA( SPIDRV_Handle_t handle,
                                  const void *buffer,
                                  int count,
                                  SPIDRV_Callback_t callback );

static Ecode_t  TransferApiPrologue( SPIDRV_Handle_t handle,
                                  void *buffer,
                                  int count );

static Ecode_t  TransferApiBlockingPrologue( SPIDRV_Handle_t handle,
                                  void *buffer,
                                  int count );

#if defined( EMDRV_SPIDRV_INCLUDE_SLAVE )
static Ecode_t  WaitForIdleLine(  SPIDRV_Handle_t handle );
#endif

/// @endcond

/***************************************************************************//**
 * @brief
 *    Initialize a SPI driver instance.
 *
 * @param[out] handle  Pointer to a SPI driver handle, refer to @ref
 *                     SPIDRV_Handle_t.
 *
 * @param[in] initData Pointer to an initialization data structure,
 *                     refer to @ref SPIDRV_Init_t.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate
 *    SPIDRV @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_Init( SPIDRV_Handle_t handle, SPIDRV_Init_t *initData )
{
  Ecode_t retVal;
  DMA_Init_TypeDef dmaInit;
  DMA_CfgChannel_TypeDef dmaChannelCfg;
  uint32_t rxDmaSrcSelect, txDmaSrcSelect;
  USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;

  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  if ( initData == NULL ) {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  memset( handle, 0, sizeof( SPIDRV_HandleData_t ) );

  if ( 0 ) {
  #if defined( USART0 )
  } else if ( initData->port == USART0 ) {
    handle->usartClock = cmuClock_USART0;
    txDmaSrcSelect     = DMAREQ_USART0_TXBL;
    rxDmaSrcSelect     = DMAREQ_USART0_RXDATAV;
  #endif
  #if defined( USART1 )
  } else if ( initData->port == USART1 ) {
    handle->usartClock = cmuClock_USART1;
    txDmaSrcSelect     = DMAREQ_USART1_TXBL;
    rxDmaSrcSelect     = DMAREQ_USART1_RXDATAV;
  #endif
  #if defined( USART2 )
  } else if ( initData->port == USART2 ) {
    handle->usartClock = cmuClock_USART2;
    txDmaSrcSelect     = DMAREQ_USART2_TXBL;
    rxDmaSrcSelect     = DMAREQ_USART2_RXDATAV;
  #endif
  } else {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  handle->initData = *initData;

  if ( initData->bitOrder == spidrvBitOrderMsbFirst ) {
    usartInit.msbf = true;
  }

  if ( initData->clockMode == spidrvClockMode0 ) {
    usartInit.clockMode = usartClockMode0;
  } else if ( initData->clockMode == spidrvClockMode1 ) {
    usartInit.clockMode = usartClockMode1;
  } else if ( initData->clockMode == spidrvClockMode2 ) {
    usartInit.clockMode = usartClockMode2;
  } else if ( initData->clockMode == spidrvClockMode3 ) {
    usartInit.clockMode = usartClockMode3;
  } else {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  if ( initData->type == spidrvSlave ) {
    usartInit.master = false;
    usartInit.baudrate = 1000;      // Dummy value needed by USART_InitSync()
  } else {
    usartInit.baudrate = initData->bitRate;
  }

  CMU_ClockEnable( cmuClock_HFPER, true );
  CMU_ClockEnable( cmuClock_GPIO, true );
  CMU_ClockEnable( cmuClock_DMA, true );
  CMU_ClockEnable( handle->usartClock, true );
  USART_InitSync(  initData->port, &usartInit );

  if ( ( initData->type == spidrvMaster )
       && ( initData->csControl == spidrvCsControlAuto ) ) {
    initData->port->CTRL |= USART_CTRL_AUTOCS;
  }

  if ( ( initData->type == spidrvMaster )
       && ( initData->csControl == spidrvCsControlApplication ) ) {
    initData->port->ROUTE = USART_ROUTE_TXPEN
                            | USART_ROUTE_RXPEN
                            | USART_ROUTE_CLKPEN
                            | (initData->portLocation
                              << _USART_ROUTE_LOCATION_SHIFT );
  } else {
    initData->port->ROUTE = USART_ROUTE_TXPEN
                            | USART_ROUTE_RXPEN
                            | USART_ROUTE_CLKPEN
                            | USART_ROUTE_CSPEN
                            | (initData->portLocation
                              << _USART_ROUTE_LOCATION_SHIFT );
  }

  if ( ( retVal = ConfigGPIO( handle, true ) ) != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  INT_Disable();
  if ( ! spidrvIsInitialized ) {
    spidrvIsInitialized = true;
    INT_Enable();

    // Do one-time-only initialization.

    // Initialize DMA controller.
    NVIC_SetPriority( DMA_IRQn, EMDRV_SPIDRV_DMA_IRQ_PRIORITY );
    dmaInit.hprot        = 0;
    dmaInit.controlBlock = dmaControlBlock;
    DMA_Init( &dmaInit );

#if defined( EMDRV_SPIDRV_INCLUDE_SLAVE )
    RTCDRV_Init();
#endif
  } else {
    INT_Enable();
  }

#if defined( EMDRV_SPIDRV_INCLUDE_SLAVE )
  if ( initData->type == spidrvSlave ) {
    if ( RTCDRV_AllocateTimer( &handle->timer ) != ECODE_EMDRV_RTCDRV_OK ) {
      return ECODE_EMDRV_SPIDRV_TIMER_ALLOC_ERROR;
    }
  }
#endif

  // Initialize DMA channels.
  dmaChannelCfg.highPri       = false;

  dmaChannelCfg.enableInt     = false;
  dmaChannelCfg.select        = txDmaSrcSelect;
  dmaChannelCfg.cb            = NULL;
  DMA_CfgChannel( initData->txDMACh, &dmaChannelCfg );

  dmaChannelCfg.enableInt     = true;
  dmaChannelCfg.select        = rxDmaSrcSelect;
  handle->rxDmaCbData.cbFunc  = RxDMAComplete;
  dmaChannelCfg.cb            = &(handle->rxDmaCbData);
  handle->rxDmaCbData.userPtr = (void*)handle;
  DMA_CfgChannel( initData->rxDMACh, &dmaChannelCfg );

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Deinitialize a SPI driver instance.
 *
 * @param[in] handle Pointer to a SPI driver handle.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate
 *    SPIDRV @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_DeInit( SPIDRV_Handle_t handle )
{
  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  // Stop DMA's.
  DMA_ChannelEnable( handle->initData.rxDMACh, false );
  DMA_ChannelEnable( handle->initData.txDMACh, false );

  ConfigGPIO( handle, false );

#if defined( EMDRV_SPIDRV_INCLUDE_SLAVE )
  if ( handle->initData.type == spidrvSlave ) {
    RTCDRV_StopTimer( handle->timer );
    RTCDRV_FreeTimer( handle->timer );
  }
#endif

  USART_Reset( handle->initData.port );
  CMU_ClockEnable( handle->usartClock, false );

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Abort an ongoing SPI transfer.
 *
 * @param[in] handle Pointer to a SPI driver handle.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success, @ref ECODE_EMDRV_SPIDRV_IDLE if
 *    SPI is idle. On failure an appropriate SPIDRV @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_AbortTransfer( SPIDRV_Handle_t handle )
{
  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  INT_Disable();
  if ( handle->state == spidrvStateIdle ) {
    INT_Enable();
    return ECODE_EMDRV_SPIDRV_IDLE;
  }

#if defined( EMDRV_SPIDRV_INCLUDE_SLAVE )
  if ( handle->initData.type == spidrvSlave ) {
    RTCDRV_StopTimer( handle->timer );
  }
#endif

  // Stop DMA's.
  DMA_ChannelEnable( handle->initData.rxDMACh, false );
  DMA_ChannelEnable( handle->initData.txDMACh, false );
  handle->remaining = 1 + ( ( dmaControlBlock[ handle->initData.rxDMACh ].CTRL
                              & _DMA_CTRL_N_MINUS_1_MASK )
                            >> _DMA_CTRL_N_MINUS_1_SHIFT );

  handle->transferStatus    = ECODE_EMDRV_SPIDRV_ABORTED;
  handle->state             = spidrvStateIdle;
  handle->transferStatus    = ECODE_EMDRV_SPIDRV_ABORTED;
  handle->blockingCompleted = true;

  if ( handle->userCallback != NULL ) {
    handle->userCallback( handle,
                          ECODE_EMDRV_SPIDRV_ABORTED,
                          handle->transferCount - handle->remaining );
  }
  INT_Enable();

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Get current SPI bus bitrate.
 *
 * @param[in] handle Pointer to a SPI driver handle.
 *
 * @param[out] bitRate Current SPI bus bitrate.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_GetBitrate( SPIDRV_Handle_t handle, uint32_t *bitRate )
{
  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  if ( bitRate == NULL ) {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  *bitRate = USART_BaudrateGet( handle->initData.port );

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Get current SPI framelength.
 *
 * @param[in] handle Pointer to a SPI driver handle.
 *
 * @param[out] frameLength Current SPI bus framelength.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_GetFramelength( SPIDRV_Handle_t handle, uint32_t *frameLength )
{
  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  if ( frameLength == NULL ) {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  *frameLength = handle->initData.frameLength;

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Get the status of a SPI transfer.
 *
 * @details
 *    Returns status of an ongoing transfer. If no transfer is in progress
 *    the status of the last transfer is reported.
 *
 * @param[in] handle Pointer to a SPI driver handle.
 *
 * @param[out] itemsTransferred Number of items (frames) transferred.
 *
 * @param[out] itemsRemaining Number of items (frames) remaining.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_GetTransferStatus( SPIDRV_Handle_t handle,
                                  int *itemsTransferred,
                                  int *itemsRemaining )
{
  int remaining;

  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  if ( ( itemsTransferred == NULL ) || ( itemsRemaining == NULL ) )
    {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  INT_Disable();
  if ( handle->state == spidrvStateIdle ) {
    remaining = handle->remaining;
  } else {
    remaining =  1 + ( ( dmaControlBlock[ handle->initData.rxDMACh ].CTRL
                       & _DMA_CTRL_N_MINUS_1_MASK )
                     >> _DMA_CTRL_N_MINUS_1_SHIFT );
  }
  INT_Enable();

  *itemsTransferred = handle->transferCount - remaining;
  *itemsRemaining   = remaining;

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI master receive transfer.
 *
 * @note
 *    The MOSI wire will transmit @ref SPIDRV_Init_t.dummyTxValue.
 *
 * @param[in]  handle Pointer to a SPI driver handle.
 *
 * @param[out] buffer Receive data buffer.
 *
 * @param[in]  count Number of bytes in transfer.
 *
 * @param[in]  callback Transfer completion callback.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_MReceive( SPIDRV_Handle_t handle,
                         void *buffer,
                         int count,
                         SPIDRV_Callback_t callback )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvSlave ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiPrologue( handle, buffer, count ) )
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  StartReceiveDMA( handle, buffer, count, callback );

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI master blocking receive transfer.
 *
 * @note
 *    The MOSI wire will transmit @ref SPIDRV_Init_t.dummyTxValue.
 *    @n This function is blocking and returns when the transfer has completed,
 *    or when @ref SPIDRV_AbortTransfer() is called.
 *
 * @param[in]  handle Pointer to a SPI driver handle.
 *
 * @param[out] buffer Receive data buffer.
 *
 * @param[in]  count Number of bytes in transfer.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success or @ref ECODE_EMDRV_SPIDRV_ABORTED
 *    if @ref SPIDRV_AbortTransfer() has been called. On failure an appropriate
 *    SPIDRV @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_MReceiveB( SPIDRV_Handle_t handle,
                          void *buffer,
                          int count )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvSlave ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiBlockingPrologue( handle, buffer, count ) )
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  StartReceiveDMA( handle, buffer, count, BlockingComplete );

  while ( handle->blockingCompleted == false );

  return handle->transferStatus;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI master transfer.
 *
 * @param[in]  handle Pointer to a SPI driver handle.
 *
 * @param[in]  txBuffer Transmit data buffer.
 *
 * @param[out] rxBuffer Receive data buffer.
 *
 * @param[in]  count Number of bytes in transfer.
 *
 * @param[in]  callback Transfer completion callback.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_MTransfer( SPIDRV_Handle_t handle,
                          const void *txBuffer,
                          void *rxBuffer,
                          int count,
                          SPIDRV_Callback_t callback )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvSlave ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiPrologue( handle, (void*)txBuffer, count ) )
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  if ( rxBuffer == NULL ) {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  StartTransferDMA( handle, txBuffer, rxBuffer, count, callback );

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI master blocking transfer.
 *
 * @note
 *    This function is blocking and returns when the transfer has completed,
 *    or when @ref SPIDRV_AbortTransfer() is called.
 *
 * @param[in]  handle Pointer to a SPI driver handle.
 *
 * @param[in]  txBuffer Transmit data buffer.
 *
 * @param[out] rxBuffer Receive data buffer.
 *
 * @param[in]  count Number of bytes in transfer.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success or @ref ECODE_EMDRV_SPIDRV_ABORTED
 *    if @ref SPIDRV_AbortTransfer() has been called. On failure an appropriate
 *    SPIDRV @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_MTransferB( SPIDRV_Handle_t handle,
                           const void *txBuffer,
                           void *rxBuffer,
                           int count )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvSlave ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiBlockingPrologue( handle, (void*)txBuffer, count ))
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  if ( rxBuffer == NULL ) {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  StartTransferDMA( handle, txBuffer, rxBuffer, count, BlockingComplete );

  while ( handle->blockingCompleted == false );

  return handle->transferStatus;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI master blocking single item (frame) transfer.
 *
 * @note
 *    This function is blocking and returns when the transfer has completed,
 *    or when @ref SPIDRV_AbortTransfer() is called.
 *
 * @param[in] handle Pointer to a SPI driver handle.
 *
 * @param[in] txValue Value to transmit.
 *
 * @param[out] rxValue Value received.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success or @ref ECODE_EMDRV_SPIDRV_ABORTED
 *    if @ref SPIDRV_AbortTransfer() has been called. On failure an appropriate
 *    SPIDRV @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_MTransferSingleItemB( SPIDRV_Handle_t handle,
                                     uint32_t txValue,
                                     void *rxValue )
{
  void *pRx;
  uint32_t rxBuffer;

  if ( handle->initData.type == spidrvSlave ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  if ( INT_Disable() > 1 )
  {
    INT_Enable();
    return ECODE_EMDRV_SPIDRV_ILLEGAL_OPERATION;
  }

  if ( handle->state != spidrvStateIdle ) {
    INT_Enable();
    return ECODE_EMDRV_SPIDRV_BUSY;
  }
  handle->state = spidrvStateTransferring;
  INT_Enable();

  if ( ( pRx = rxValue ) == NULL ) {
    pRx = &rxBuffer;
  }

  StartTransferDMA( handle, &txValue, pRx, 1, BlockingComplete );

  while ( handle->blockingCompleted == false );

  return handle->transferStatus;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI master transmit transfer.
 *
 * @note
 *    The data received on the MISO wire is discarded.
 *
 * @param[in] handle Pointer to a SPI driver handle.
 *
 * @param[in] buffer Transmit data buffer.
 *
 * @param[in] count Number of bytes in transfer.
 *
 * @param[in] callback Transfer completion callback.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_MTransmit( SPIDRV_Handle_t handle,
                          const void *buffer,
                          int count,
                          SPIDRV_Callback_t callback )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvSlave ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiPrologue( handle, (void*)buffer, count ) )
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  StartTransmitDMA( handle, buffer, count, callback );

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI master blocking transmit transfer.
 *
 * @note
 *    The data received on the MISO wire is discarded.
 *    @n This function is blocking and returns when the transfer is completed.
 *
 * @param[in] handle Pointer to a SPI driver handle.
 *
 * @param[in] buffer Transmit data buffer.
 *
 * @param[in] count Number of bytes in transfer.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success or @ref ECODE_EMDRV_SPIDRV_ABORTED
 *    if @ref SPIDRV_AbortTransfer() has been called. On failure an appropriate
 *    SPIDRV @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_MTransmitB( SPIDRV_Handle_t handle,
                           const void *buffer,
                           int count )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvSlave ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiBlockingPrologue( handle, (void*)buffer, count ) )
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  StartTransmitDMA( handle, buffer, count, BlockingComplete );

  while ( handle->blockingCompleted == false );

  return handle->transferStatus;
}

/***************************************************************************//**
 * @brief
 *    Set SPI bus bitrate.
 *
 * @param[in] handle Pointer to a SPI driver handle.
 *
 * @param[in] bitRate New SPI bus bitrate.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_SetBitrate( SPIDRV_Handle_t handle, uint32_t bitRate )
{
  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  INT_Disable();
  if ( handle->state != spidrvStateIdle ) {
    INT_Enable();
    return ECODE_EMDRV_SPIDRV_BUSY;
  }

  handle->initData.bitRate = bitRate;
  USART_BaudrateSyncSet( handle->initData.port, 0, bitRate );
  INT_Enable();

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Set SPI framelength.
 *
 * @param[in] handle Pointer to a SPI driver handle.
 *
 * @param[in] frameLength New SPI bus framelength.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_SetFramelength( SPIDRV_Handle_t handle, uint32_t frameLength )
{
  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  frameLength -= 3;
  if ( ( frameLength < _USART_FRAME_DATABITS_FOUR )
       || ( frameLength > _USART_FRAME_DATABITS_SIXTEEN ) ) {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  INT_Disable();
  if ( handle->state != spidrvStateIdle ) {
    INT_Enable();
    return ECODE_EMDRV_SPIDRV_BUSY;
  }

  handle->initData.frameLength = frameLength + 3;
  handle->initData.port->FRAME = ( handle->initData.port->FRAME
                                   & ~_USART_FRAME_DATABITS_MASK )
                                  | ( frameLength
                                      << _USART_FRAME_DATABITS_SHIFT );
  INT_Enable();

  return ECODE_EMDRV_SPIDRV_OK;
}

#if defined( EMDRV_SPIDRV_INCLUDE_SLAVE )
/***************************************************************************//**
 * @brief
 *    Start a SPI slave receive transfer.
 *
 * @note
 *    The MISO wire will transmit @ref SPIDRV_Init_t.dummyTxValue.
 *
 * @param[in]  handle Pointer to a SPI driver handle.
 *
 * @param[out] buffer Receive data buffer.
 *
 * @param[in]  count Number of bytes in transfer.
 *
 * @param[in]  callback Transfer completion callback.
 *
 * @param[in]  timeoutMs Transfer timeout in milliseconds.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_SReceive( SPIDRV_Handle_t handle,
                         void *buffer,
                         int count,
                         SPIDRV_Callback_t callback,
                         int timeoutMs )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvMaster ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiPrologue( handle, buffer, count ) )
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  if ( timeoutMs ) {
    RTCDRV_StartTimer( handle->timer,
                       rtcdrvTimerTypeOneshot,
                       timeoutMs,
                       SlaveTimeout,
                       handle );
  }

  if ( handle->initData.slaveStartMode == spidrvSlaveStartDelayed ) {
    if ( ( retVal = WaitForIdleLine( handle ) ) != ECODE_EMDRV_SPIDRV_OK ) {
      return retVal;
    }
  }

  StartReceiveDMA( handle, buffer, count, callback );

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI slave blocking receive transfer.
 *
 * @note
 *    The MISO wire will transmit @ref SPIDRV_Init_t.dummyTxValue.
 *    @n This function is blocking and returns when the transfer has completed,
 *    or on timeout or when @ref SPIDRV_AbortTransfer() is called.
 *
 * @param[in]  handle Pointer to a SPI driver handle.
 *
 * @param[out] buffer Receive data buffer.
 *
 * @param[in]  count Number of bytes in transfer.
 *
 * @param[in]  timeoutMs Transfer timeout in milliseconds.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success, @ref ECODE_EMDRV_SPIDRV_TIMEOUT on
 *    timeout or @ref ECODE_EMDRV_SPIDRV_ABORTED if @ref SPIDRV_AbortTransfer()
 *    has been called. On failure an appropriate SPIDRV @ref Ecode_t is
 *    returned.
 ******************************************************************************/
Ecode_t SPIDRV_SReceiveB( SPIDRV_Handle_t handle,
                          void *buffer,
                          int count,
                          int timeoutMs )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvMaster ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiBlockingPrologue( handle, buffer, count ) )
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  if ( timeoutMs ) {
    RTCDRV_StartTimer( handle->timer,
                       rtcdrvTimerTypeOneshot,
                       timeoutMs,
                       SlaveTimeout,
                       handle );
  }

  if ( handle->initData.slaveStartMode == spidrvSlaveStartDelayed ) {
    if ( ( retVal = WaitForIdleLine( handle ) ) != ECODE_EMDRV_SPIDRV_OK ) {
      return retVal;
    }
  }

  StartReceiveDMA( handle, buffer, count, BlockingComplete );

  while ( handle->blockingCompleted == false );

  return handle->transferStatus;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI slave transfer.
 *
 * @param[in]  handle Pointer to a SPI driver handle.
 *
 * @param[in]  txBuffer Transmit data buffer.
 *
 * @param[out] rxBuffer Receive data buffer.
 *
 * @param[in]  count Number of bytes in transfer.
 *
 * @param[in]  callback Transfer completion callback.
 *
 * @param[in]  timeoutMs Transfer timeout in milliseconds.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_STransfer( SPIDRV_Handle_t handle,
                          const void *txBuffer,
                          void *rxBuffer,
                          int count,
                          SPIDRV_Callback_t callback,
                          int timeoutMs )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvMaster ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiPrologue( handle, (void*)txBuffer, count ) )
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  if ( rxBuffer == NULL ) {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  if ( timeoutMs ) {
    RTCDRV_StartTimer( handle->timer,
                       rtcdrvTimerTypeOneshot,
                       timeoutMs,
                       SlaveTimeout,
                       handle );
  }

  if ( handle->initData.slaveStartMode == spidrvSlaveStartDelayed ) {
    if ( ( retVal = WaitForIdleLine( handle ) ) != ECODE_EMDRV_SPIDRV_OK ) {
      return retVal;
    }
  }

  StartTransferDMA( handle, txBuffer, rxBuffer, count, callback );

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI slave blocking transfer.
 *
 * @note
 *    @n This function is blocking and returns when the transfer has completed,
 *    or on timeout or when @ref SPIDRV_AbortTransfer() is called.
 *
 * @param[in]  handle Pointer to a SPI driver handle.
 *
 * @param[in]  txBuffer Transmit data buffer.
 *
 * @param[out] rxBuffer Receive data buffer.
 *
 * @param[in]  count Number of bytes in transfer.
 *
 * @param[in]  timeoutMs Transfer timeout in milliseconds.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success, @ref ECODE_EMDRV_SPIDRV_TIMEOUT on
 *    timeout or @ref ECODE_EMDRV_SPIDRV_ABORTED if @ref SPIDRV_AbortTransfer()
 *    has been called. On failure an appropriate SPIDRV @ref Ecode_t is
 *    returned.
 ******************************************************************************/
Ecode_t SPIDRV_STransferB( SPIDRV_Handle_t handle,
                           const void *txBuffer,
                           void *rxBuffer,
                           int count,
                           int timeoutMs )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvMaster ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiBlockingPrologue( handle, (void*)txBuffer, count ))
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  if ( rxBuffer == NULL ) {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  if ( timeoutMs ) {
    RTCDRV_StartTimer( handle->timer,
                       rtcdrvTimerTypeOneshot,
                       timeoutMs,
                       SlaveTimeout,
                       handle );
  }

  if ( handle->initData.slaveStartMode == spidrvSlaveStartDelayed ) {
    if ( ( retVal = WaitForIdleLine( handle ) ) != ECODE_EMDRV_SPIDRV_OK ) {
      return retVal;
    }
  }

  StartTransferDMA( handle, txBuffer, rxBuffer, count, BlockingComplete );

  while ( handle->blockingCompleted == false );

  return handle->transferStatus;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI slave transmit transfer.
 *
 * @note
 *    The data received on the MOSI wire is discarded.
 *
 * @param[in]  handle Pointer to a SPI driver handle.
 *
 * @param[in]  buffer Transmit data buffer.
 *
 * @param[in]  count Number of bytes in transfer.
 *
 * @param[in]  callback Transfer completion callback.
 *
 * @param[in]  timeoutMs Transfer timeout in milliseconds.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success. On failure an appropriate SPIDRV
 *    @ref Ecode_t is returned.
 ******************************************************************************/
Ecode_t SPIDRV_STransmit( SPIDRV_Handle_t handle,
                          const void *buffer,
                          int count,
                          SPIDRV_Callback_t callback,
                          int timeoutMs )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvMaster ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiPrologue( handle, (void*)buffer, count ) )
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  if ( timeoutMs ) {
    RTCDRV_StartTimer( handle->timer,
                       rtcdrvTimerTypeOneshot,
                       timeoutMs,
                       SlaveTimeout,
                       handle );
  }

  if ( handle->initData.slaveStartMode == spidrvSlaveStartDelayed ) {
    if ( ( retVal = WaitForIdleLine( handle ) ) != ECODE_EMDRV_SPIDRV_OK ) {
      return retVal;
    }
  }

  StartTransmitDMA( handle, buffer, count, callback );

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief
 *    Start a SPI slave blocking transmit transfer.
 *
 * @note
 *    The data received on the MOSI wire is discarded.
 *    @n This function is blocking and returns when the transfer has completed,
 *    or on timeout or when @ref SPIDRV_AbortTransfer() is called.
 *
 * @param[in]  handle Pointer to a SPI driver handle.
 *
 * @param[in]  buffer Transmit data buffer.
 *
 * @param[in]  count Number of bytes in transfer.
 *
 * @param[in]  timeoutMs Transfer timeout in milliseconds.
 *
 * @return
 *    @ref ECODE_EMDRV_SPIDRV_OK on success, @ref ECODE_EMDRV_SPIDRV_TIMEOUT on
 *    timeout or @ref ECODE_EMDRV_SPIDRV_ABORTED if @ref SPIDRV_AbortTransfer()
 *    has been called. On failure an appropriate SPIDRV @ref Ecode_t is
 *    returned.
 ******************************************************************************/
Ecode_t SPIDRV_STransmitB( SPIDRV_Handle_t handle,
                           const void *buffer,
                           int count,
                           int timeoutMs )
{
  Ecode_t retVal;

  if ( handle->initData.type == spidrvMaster ) {
    return ECODE_EMDRV_SPIDRV_MODE_ERROR;
  }

  if ( ( retVal = TransferApiBlockingPrologue( handle, (void*)buffer, count ) )
       != ECODE_EMDRV_SPIDRV_OK ) {
    return retVal;
  }

  if ( timeoutMs ) {
    RTCDRV_StartTimer( handle->timer,
                       rtcdrvTimerTypeOneshot,
                       timeoutMs,
                       SlaveTimeout,
                       handle );
  }

  if ( handle->initData.slaveStartMode == spidrvSlaveStartDelayed ) {
    if ( ( retVal = WaitForIdleLine( handle ) ) != ECODE_EMDRV_SPIDRV_OK ) {
      return retVal;
    }
  }

  StartTransmitDMA( handle, buffer, count, BlockingComplete );

  while ( handle->blockingCompleted == false );

  return handle->transferStatus;
}
#endif

/// @cond DO_NOT_INCLUDE_WITH_DOXYGEN

/***************************************************************************//**
 * @brief
 *    Transfer complete callback function used by blocking transfer API
 *    functions. Called by DMA interrupt handler, timer timeout handler
 *    or @ref SPIDRV_AbortTransfer() function.
 ******************************************************************************/
static void BlockingComplete( SPIDRV_Handle_t handle,
                              Ecode_t transferStatus,
                              int itemsTransferred )
{
  (void)itemsTransferred;

  handle->transferStatus    = transferStatus;
  handle->blockingCompleted = true;
}

/***************************************************************************//**
 * @brief Configure/deconfigure SPI GPIO pins.
 ******************************************************************************/
static Ecode_t ConfigGPIO( SPIDRV_Handle_t handle, bool enable )
{
  uint32_t location;
  int mosiPin, misoPin, clkPin;
  int mosiPort, misoPort, clkPort;

  location = handle->initData.portLocation;

  if ( 0 ) {
  #if defined( USART0 )
  } else if ( handle->initData.port == USART0 ) {
    mosiPort       = AF_USART0_TX_PORT(  location );
    misoPort       = AF_USART0_RX_PORT(  location );
    clkPort        = AF_USART0_CLK_PORT( location );
    handle->csPort = AF_USART0_CS_PORT(  location );
    mosiPin        = AF_USART0_TX_PIN(   location );
    misoPin        = AF_USART0_RX_PIN(   location );
    clkPin         = AF_USART0_CLK_PIN(  location );
    handle->csPin  = AF_USART0_CS_PIN(   location );
  #endif
  #if defined( USART1 )
  } else if ( handle->initData.port == USART1 ) {
    mosiPort       = AF_USART1_TX_PORT(  location );
    misoPort       = AF_USART1_RX_PORT(  location );
    clkPort        = AF_USART1_CLK_PORT( location );
    handle->csPort = AF_USART1_CS_PORT(  location );
    mosiPin        = AF_USART1_TX_PIN(   location );
    misoPin        = AF_USART1_RX_PIN(   location );
    clkPin         = AF_USART1_CLK_PIN(  location );
    handle->csPin  = AF_USART1_CS_PIN(   location );
  #endif
  #if defined( USART2 )
  } else if ( handle->initData.port == USART2 ) {
    mosiPort       = AF_USART2_TX_PORT(  location );
    misoPort       = AF_USART2_RX_PORT(  location );
    clkPort        = AF_USART2_CLK_PORT( location );
    handle->csPort = AF_USART2_CS_PORT(  location );
    mosiPin        = AF_USART2_TX_PIN(   location );
    misoPin        = AF_USART2_RX_PIN(   location );
    clkPin         = AF_USART2_CLK_PIN(  location );
    handle->csPin  = AF_USART2_CS_PIN(   location );
  #endif
  } else {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  if ( enable ) {
    if ( handle->initData.type == spidrvMaster ) {
      GPIO_PinModeSet( (GPIO_Port_TypeDef)mosiPort, mosiPin,
                       gpioModePushPull, 0 );
      GPIO_PinModeSet( (GPIO_Port_TypeDef)misoPort, misoPin,
                       gpioModeInputPull, 0 );

      if (    ( handle->initData.clockMode == spidrvClockMode0 )
           || ( handle->initData.clockMode == spidrvClockMode1 ) ) {
        GPIO_PinModeSet( (GPIO_Port_TypeDef)clkPort, clkPin,
                         gpioModePushPull, 0 );
      } else {
        GPIO_PinModeSet( (GPIO_Port_TypeDef)clkPort, clkPin,
                         gpioModePushPull, 1 );
      }

      GPIO_PinModeSet( (GPIO_Port_TypeDef)handle->csPort, handle->csPin,
                       gpioModePushPull, 1 );
    } else {
      GPIO_PinModeSet( (GPIO_Port_TypeDef)mosiPort, mosiPin,
                       gpioModeInputPull, 0 );
      GPIO_PinModeSet( (GPIO_Port_TypeDef)misoPort, misoPin,
                       gpioModePushPull, 0 );

      if (    ( handle->initData.clockMode == spidrvClockMode0 )
           || ( handle->initData.clockMode == spidrvClockMode1 ) ) {
        GPIO_PinModeSet( (GPIO_Port_TypeDef)clkPort, clkPin,
                         gpioModeInputPull, 0 );
      } else {
        GPIO_PinModeSet( (GPIO_Port_TypeDef)clkPort, clkPin,
                         gpioModeInputPull, 1 );
      }

      GPIO_PinModeSet( (GPIO_Port_TypeDef)handle->csPort, handle->csPin,
                       gpioModeInputPull, 1 );
    }
  } else {
    GPIO_PinModeSet( (GPIO_Port_TypeDef)mosiPort, mosiPin, gpioModeInputPull,0);
    GPIO_PinModeSet( (GPIO_Port_TypeDef)misoPort, misoPin, gpioModeInputPull,0);

    if (    ( handle->initData.clockMode == spidrvClockMode0 )
         || ( handle->initData.clockMode == spidrvClockMode1 ) ) {
      GPIO_PinModeSet( (GPIO_Port_TypeDef)clkPort, clkPin, gpioModeInputPull,0);
    } else {
      GPIO_PinModeSet( (GPIO_Port_TypeDef)clkPort, clkPin, gpioModeInputPull,1);
    }

    GPIO_PinModeSet( (GPIO_Port_TypeDef)handle->csPort, handle->csPin,
                     gpioModeDisabled, 0);
  }

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief DMA transfer completion callback. Called by DMA interrupt handler.
 ******************************************************************************/
static void RxDMAComplete( unsigned int channel, bool primary, void *user )
{
  SPIDRV_Handle_t handle;
  (void)channel;
  (void)primary;

  INT_Disable();

  handle = (SPIDRV_Handle_t)user;

  handle->transferStatus = ECODE_EMDRV_SPIDRV_OK;
  handle->state          = spidrvStateIdle;
  handle->remaining      = 0;

#if defined( EMDRV_SPIDRV_INCLUDE_SLAVE )
  if ( handle->initData.type == spidrvSlave ) {
    RTCDRV_StopTimer( handle->timer );
  }
#endif

  if ( handle->userCallback != NULL ) {
    handle->userCallback( handle, ECODE_EMDRV_SPIDRV_OK, handle->transferCount);
  }

  INT_Enable();
}

#if defined( EMDRV_SPIDRV_INCLUDE_SLAVE )
/***************************************************************************//**
 * @brief Slave transfer timeout callback function.
 ******************************************************************************/
static void SlaveTimeout( RTCDRV_TimerID_t id, void *user )
{
  SPIDRV_Handle_t handle;
  (void)id;

  handle = (SPIDRV_Handle_t)user;

  if ( handle->state == spidrvStateTransferring ) {
    if ( DMA_ChannelEnabled( handle->initData.rxDMACh ) ) {
      // Stop running DMA's
      DMA_ChannelEnable( handle->initData.rxDMACh, false );
      DMA_ChannelEnable( handle->initData.txDMACh, false );
      handle->remaining = 1 +
                          ( ( dmaControlBlock[ handle->initData.rxDMACh ].CTRL
                              & _DMA_CTRL_N_MINUS_1_MASK )
                            >> _DMA_CTRL_N_MINUS_1_SHIFT );
    } else {
      // DMA is either completed or not yet started
      if ( DMA->IF & ( 1 << handle->initData.rxDMACh ) ) {
          // We have a pending DMA interrupt, let the DMA handler do the rest
          return;
      }
      handle->remaining = handle->transferCount;
    }
    handle->transferStatus = ECODE_EMDRV_SPIDRV_TIMEOUT;
    handle->state          = spidrvStateIdle;

    if ( handle->userCallback != NULL ) {
      handle->userCallback( handle,
                            ECODE_EMDRV_SPIDRV_TIMEOUT,
                            handle->transferCount - handle->remaining );
    }
  }
}
#endif

/***************************************************************************//**
 * @brief Start a SPI receive DMA.
 ******************************************************************************/
static void StartReceiveDMA( SPIDRV_Handle_t handle,
                             void *buffer,
                             int count,
                             SPIDRV_Callback_t callback )
{
  void *rxPort, *txPort;
  DMA_CfgDescr_TypeDef cfgDesc;

  handle->blockingCompleted  = false;
  handle->transferCount      = count;
  handle->initData.port->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
  handle->userCallback       = callback;

  if ( handle->initData.frameLength > 8 ) {
    cfgDesc.size = dmaDataSize2;
  } else {
    cfgDesc.size = dmaDataSize1;
  }
  cfgDesc.arbRate = dmaArbitrate1;
  cfgDesc.hprot   = 0;

  cfgDesc.srcInc = dmaDataIncNone;
  if ( handle->initData.frameLength > 8 ) {
    cfgDesc.dstInc = dmaDataInc2;
    rxPort = (void *)&(handle->initData.port->RXDOUBLE);
    txPort = (void *)&(handle->initData.port->TXDOUBLE);
  } else {
    cfgDesc.dstInc = dmaDataInc1;
    rxPort = (void *)&(handle->initData.port->RXDATA);
    txPort = (void *)&(handle->initData.port->TXDATA);
  }
  DMA_CfgDescr( handle->initData.rxDMACh, true, &cfgDesc );

  cfgDesc.dstInc = dmaDataIncNone;
  DMA_CfgDescr( handle->initData.txDMACh, true, &cfgDesc );

  DMA_ActivateBasic( handle->initData.rxDMACh,
                     true,
                     false,
                     buffer,
                     rxPort,
                     count - 1 );

  DMA_ActivateBasic( handle->initData.txDMACh,
                     true,
                     false,
                     txPort,
                     (void *)&(handle->initData.dummyTxValue),
                     count - 1 );
}

/***************************************************************************//**
 * @brief Start a SPI transmit/receive DMA.
 ******************************************************************************/
static void StartTransferDMA( SPIDRV_Handle_t handle,
                              const void *txBuffer,
                              void *rxBuffer,
                              int count,
                              SPIDRV_Callback_t callback )
{
  void *rxPort, *txPort;
  DMA_CfgDescr_TypeDef cfgDesc;

  handle->blockingCompleted  = false;
  handle->transferCount      = count;
  handle->initData.port->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
  handle->userCallback       = callback;

  if ( handle->initData.frameLength > 8 ) {
    cfgDesc.size = dmaDataSize2;
  } else {
    cfgDesc.size = dmaDataSize1;
  }
  cfgDesc.arbRate = dmaArbitrate1;
  cfgDesc.hprot   = 0;

  if ( handle->initData.frameLength > 8 ) {
    cfgDesc.srcInc = dmaDataInc2;
    txPort = (void *)&(handle->initData.port->TXDOUBLE);
  } else {
    cfgDesc.srcInc = dmaDataInc1;
    txPort = (void *)&(handle->initData.port->TXDATA);
  }
  cfgDesc.dstInc = dmaDataIncNone;
  DMA_CfgDescr( handle->initData.txDMACh, true, &cfgDesc );

  cfgDesc.srcInc = dmaDataIncNone;
  if ( handle->initData.frameLength > 8 ) {
    cfgDesc.dstInc = dmaDataInc2;
    rxPort = (void *)&(handle->initData.port->RXDOUBLE);
  } else {
    cfgDesc.dstInc = dmaDataInc1;
    rxPort = (void *)&(handle->initData.port->RXDATA);
  }
  DMA_CfgDescr( handle->initData.rxDMACh, true, &cfgDesc );

  DMA_ActivateBasic( handle->initData.rxDMACh,
                     true,
                     false,
                     rxBuffer,
                     rxPort,
                     count - 1 );

  DMA_ActivateBasic( handle->initData.txDMACh,
                     true,
                     false,
                     txPort,
                     (void*)txBuffer,
                     count - 1 );
}

/***************************************************************************//**
 * @brief Start a SPI transmit DMA.
 ******************************************************************************/
static void StartTransmitDMA( SPIDRV_Handle_t handle,
                               const void *buffer,
                               int count,
                               SPIDRV_Callback_t callback )
{
  void *rxPort, *txPort;
  DMA_CfgDescr_TypeDef cfgDesc;

  handle->blockingCompleted  = false;
  handle->transferCount      = count;
  handle->initData.port->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
  handle->userCallback       = callback;

  if ( handle->initData.frameLength > 8 ) {
    cfgDesc.size = dmaDataSize2;
  } else {
    cfgDesc.size = dmaDataSize1;
  }
  cfgDesc.arbRate = dmaArbitrate1;
  cfgDesc.hprot   = 0;

  if ( handle->initData.frameLength > 8 ) {
    cfgDesc.srcInc = dmaDataInc2;
    rxPort = (void *)&(handle->initData.port->RXDOUBLE);
    txPort = (void *)&(handle->initData.port->TXDOUBLE);
  } else {
    cfgDesc.srcInc = dmaDataInc1;
    rxPort = (void *)&(handle->initData.port->RXDATA);
    txPort = (void *)&(handle->initData.port->TXDATA);
  }
  cfgDesc.dstInc = dmaDataIncNone;
  DMA_CfgDescr( handle->initData.txDMACh, true, &cfgDesc );

  cfgDesc.srcInc = dmaDataIncNone;
  DMA_CfgDescr( handle->initData.rxDMACh, true, &cfgDesc );

  // Receive DMA runs only to get precise numbers for SPIDRV_GetTransferStatus()
  DMA_ActivateBasic( handle->initData.rxDMACh,
                     true,
                     false,
                     (void *)&(handle->dummyRx),
                     rxPort,
                     count - 1 );

  DMA_ActivateBasic( handle->initData.txDMACh,
                     true,
                     false,
                     txPort,
                     (void*)buffer,
                     count - 1 );
}

/***************************************************************************//**
 * @brief Parameter checking function for blocking transfer API functions.
 ******************************************************************************/
static Ecode_t TransferApiBlockingPrologue( SPIDRV_Handle_t handle,
                                            void *buffer,
                                            int count )
{
  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  if ( ( buffer == NULL ) || ( count == 0 ) ) {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  if ( INT_Disable() > 1 )
  {
    INT_Enable();
    return ECODE_EMDRV_SPIDRV_ILLEGAL_OPERATION;
  }

  if ( handle->state != spidrvStateIdle ) {
    INT_Enable();
    return ECODE_EMDRV_SPIDRV_BUSY;
  }
  handle->state = spidrvStateTransferring;
  INT_Enable();

  return ECODE_EMDRV_SPIDRV_OK;
}

/***************************************************************************//**
 * @brief Parameter checking function for non-blocking transfer API functions.
 ******************************************************************************/
static Ecode_t TransferApiPrologue( SPIDRV_Handle_t handle,
                                    void *buffer,
                                    int count )
{
  if ( handle == NULL ) {
    return ECODE_EMDRV_SPIDRV_ILLEGAL_HANDLE;
  }

  if ( ( buffer == NULL ) || ( count == 0 ) ) {
    return ECODE_EMDRV_SPIDRV_PARAM_ERROR;
  }

  INT_Disable();
  if ( handle->state != spidrvStateIdle ) {
    INT_Enable();
    return ECODE_EMDRV_SPIDRV_BUSY;
  }
  handle->state = spidrvStateTransferring;
  INT_Enable();

  return ECODE_EMDRV_SPIDRV_OK;
}

#if defined( EMDRV_SPIDRV_INCLUDE_SLAVE )
/***************************************************************************//**
 * @brief Wait for CS deassertion. Used by slave transfer API functions.
 ******************************************************************************/
static Ecode_t WaitForIdleLine( SPIDRV_Handle_t handle )
{
  while ( !GPIO_PinInGet( (GPIO_Port_TypeDef)handle->csPort, handle->csPin )
          && ( handle->state != spidrvStateIdle ) );

  if ( handle->state == spidrvStateIdle ) {
    return handle->transferStatus;
  }

  return ECODE_EMDRV_SPIDRV_OK;
}
#endif

/// @endcond

/******** THE REST OF THE FILE IS DOCUMENTATION ONLY !**********************//**
 * @{

@page spidrv_doc SPIDRV Serial Peripheral Interface driver

  The source files for the SPI driver library resides in the
  emdrv/spidrv folder, and are named spidrv.c and spidrv.h.

  @li @ref spidrv_intro
  @li @ref spidrv_conf
  @li @ref spidrv_api
  @li @ref spidrv_example

@n @section spidrv_intro Introduction
  The SPI driver support the SPI capabilities of EFM32 USARTs. The driver
  is fully reentrant and several drivers can coexist. The driver does not
  buffer or queue data. The driver has SPI transfer functions for both master
  and slave SPI mode. Both synchronous and asynchronous transfer functions are
  present. Synchronous transfer functions are blocking and will not return to
  caller before the transfer has completed. Asynchronous transfer functions
  report transfer completion with callback functions. Transfers are done using
  DMA.

  @note Transfer completion callback functions are called from within the DMA
  interrupt handler with interrupts disabled.

@n @section spidrv_conf Configuration Options

  Some properties of the SPIDRV driver are compile-time configurable. These
  properties are stored in a file named @ref spidrv_config.h. A template for this
  file, containing default values, resides in the emdrv/config folder.
  Currently the configuration options are:
  @li Inclusion of slave API transfer functions.
  @li Interrupt priority of the DMA interrupt.

  To configure SPIDRV, provide your own configuration file. Here is a
  sample @ref spidrv_config.h file:
  @verbatim
#ifndef __SILICON_LABS_SPIDRV_CONFIG_H__
#define __SILICON_LABS_SPIDRV_CONFIG_H__

// SPIDRV configuration option. Use this define to include the
// slave part of the SPIDRV API.
#define EMDRV_SPIDRV_INCLUDE_SLAVE

// SPIDRV configuration option. Set SPI transfer DMA IRQ priority.
// Range is 0..7, 0 is highest priority.
#define EMDRV_SPIDRV_DMA_IRQ_PRIORITY 4

#endif
  @endverbatim

  The properties of each SPI driver instance are set at run-time via the
  @ref SPIDRV_Init_t data structure input parameter to the @ref SPIDRV_Init()
  function.

@n @section spidrv_api The API

  This section contain brief descriptions of the functions in the API. You will
  find detailed information on input and output parameters and return values by
  clicking on the hyperlinked function names. Most functions return an error
  code, @ref ECODE_EMDRV_SPIDRV_OK is returned on success,
  see @ref ecode.h and @ref spidrv.h for other error codes.

  Your application code must include one header file: @em spidrv.h.

  @ref SPIDRV_Init(), @ref SPIDRV_DeInit() @n
    These functions initializes or deinitializes the SPIDRV driver. Typically
    @htmlonly SPIDRV_Init() @endhtmlonly is called once in your startup code.

  @ref SPIDRV_GetTransferStatus() @n
    Query the status of a transfer. Reports number of items (frames) transmitted
    and remaining.

  @ref SPIDRV_AbortTransfer() @n
    Stop an ongoing transfer.

  @ref SPIDRV_SetBitrate(), @ref SPIDRV_GetBitrate() @n
    Set or query SPI bus bitrate.

  @ref SPIDRV_SetFramelength(), @ref SPIDRV_GetFramelength() @n
    Set or query SPI bus frame length.

  SPIDRV_MReceive(), SPIDRV_MReceiveB() @n
  SPIDRV_MTransfer(), SPIDRV_MTransferB(), SPIDRV_MTransferSingleItemB() @n
  SPIDRV_MTransmit(), SPIDRV_MTransmitB() @n
  SPIDRV_SReceive(), SPIDRV_SReceiveB() @n
  SPIDRV_STransfer(), SPIDRV_STransferB() @n
  SPIDRV_STransmit(), SPIDRV_STransmitB() @n
    SPI transfer functions for SPI masters have an uppercase M in their name,
    the slave counterparts have an S.

    Transfer functions come in both synchronous and asynchronous versions,
    the synchronous versions have an uppercase B (for Blocking) at the end of
    their function name. Synchronous functions will not return before the transfer
    has completed. The aynchronous functions signal transfer completion with a
    callback function.

    @em Transmit functions discards received data, @em receive functions transmit
    a fixed data pattern set when the driver is initialized
    (@ref SPIDRV_Init_t.dummyTxValue). @em Transfer functions both receive and
    transmit data.

    All slave transfer functions have a millisecond timeout parameter. Use 0
    for no (infinite) timeout.

@n @section spidrv_example Example
  @verbatim
#include "spidrv.h"

SPIDRV_HandleData_t handleData;
SPIDRV_Handle_t handle = &handleData;

void TransferComplete( SPIDRV_Handle_t handle,
                       Ecode_t transferStatus,
                       int itemsTransferred)
{
  if ( transferStatus == ECODE_EMDRV_SPIDRV_OK ) {

    // Success !

  }
}

int main( void )
{
  uint8_t buffer[10];
  SPIDRV_Init_t initData = SPIDRV_MASTER_USART2;

  // Initialize a SPI driver instance
  SPIDRV_Init( handle, &initData );

  // Transmit data using a blocking transmit function
  SPIDRV_MTransmitB( handle, buffer, 10 );

  // Transmit data using a callback to catch transfer completion.
  SPIDRV_MTransmit( handle, buffer, 10, TransferComplete );
}
  @endverbatim

 * @}**************************************************************************/
