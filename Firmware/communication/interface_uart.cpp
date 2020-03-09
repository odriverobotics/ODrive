/*******************************************************************************
* File          : interface_uart.cpp
*
* Description   :
*
* Project       :
*
* Author        :
*
* Created on    :
*
*******************************************************************************/

/******************************************************************************
INCLUDES
*******************************************************************************/

#include "interface_uart.h"
#include "ascii_protocol.hpp"
#include <MotorControl/utils.h>
#include <fibre/protocol.hpp>
#include <cmsis_os.h>
#include <freertos_vars.h>
#include "UARTControlTask.hpp"

/*******************************************************************************
NAMESPACE
*******************************************************************************/

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

/*******************************************************************************
TYPES
*******************************************************************************/

/*******************************************************************************
GLOBAL VARIABLES
*******************************************************************************/

// FIXME: the stdlib doesn't know about CMSIS threads, so this is just a global variable
osThreadId uart_thread;
const uint32_t stack_size_uart_thread = 2048;  // Bytes

/*******************************************************************************
MODULE VARIABLES
*******************************************************************************/

uint32_t UARTStack[stack_size_uart_thread / sizeof(uint32_t)];

/*******************************************************************************
MODULE CLASS INSTANCES
*******************************************************************************/

UARTSender uart_stream_output(&huart4);
StreamBasedPacketSink uart4_packet_output(uart_stream_output);
BidirectionalPacketBasedChannel uart4_channel(uart4_packet_output);
StreamToPacketSegmenter uart4_stream_input(uart4_channel);

CUARTControlTask UARTControl("UART Controller"
                            , 0
                            , osPriorityNormal
                            , UARTStack
                            , sizeof(UARTStack)
                            , &huart4
                            , &uart4_stream_input
                            , uart_stream_output);

/*******************************************************************************
INTERNAL FUNCTION DEFINTIONS
*******************************************************************************/

/*******************************************************************************
FUNCTION DECLARATIONS
*******************************************************************************/

/**\brief   Constructor
 *
 * \param   pHuart  - pointer to the UART handle
 *
 * \return  None
 */
UARTSender::UARTSender(UART_HandleTypeDef * pHuart)
    : m_pHuart(pHuart)
    , TXCircularBuffer(TXBuffer, ARRAY_LEN(TXBuffer))
{}

/**\brief   Processes the incoming data and outputs it to the UART port
 *
 * \param   buffer  - pointer to the incoming data
 * \param   length  - number of characters in the buffer
 * \param   processed_bytes - pointer to variable of the number of processed bytes
 *
 * \return  0 on success, -1 on error
 */
int UARTSender::process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes)
{
    auto written = 0u;

    while (written < length)
    {
        written += TXCircularBuffer.write(buffer, length);
        if(nullptr != processed_bytes)
        {
                *processed_bytes = written;
        }
        // Loop to ensure all bytes get sent
        while (!TXCircularBuffer.isEmpty())
        {
            if (osSemaphoreWait(sem_uart_dma, PROTOCOL_SERVER_TIMEOUT_MS) != osOK)
            {
                return -1;
            }
            else
            {
                // transmit chunk
                if (HAL_UART_Transmit_DMA(m_pHuart, tx_buf_, TXCircularBuffer.read(tx_buf_, sizeof(tx_buf_))) != HAL_OK)
                {
                    return -1;
                }
            }
        }
    }

    return 0;
}

/**\brief   Gets the remaining space in the buffer
 *
 * \param   pHuart  - pointer to the UART handle
 *
 * \return  Free space in the buffer
 */
size_t UARTSender::get_free_space(void)
{
    return TXCircularBuffer.remainingSpace();
}

/**\brief   Starts the UART server
 *
 * \param   None
 *
 * \return  None
 */
void start_uart_server(void)
{
    uart_thread = UARTControl.getHandle();
    // Start UART communication thread
    UARTControl.start(osPriorityNormal);
}

/**
  * @brief  UART error callbacks.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(&huart4 == huart)
    {
        HAL_UART_AbortReceive(huart);
        HAL_UART_Receive_DMA(huart, huart->pRxBuffPtr, huart->RxXferSize);
    }
}

 /**
   * @brief  Tx Transfer completed callbacks.
   * @param  huart pointer to a UART_HandleTypeDef structure that contains
   *                the configuration information for the specified UART module.
   * @retval None
   */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if(&huart4 == huart)
    {
        osSemaphoreRelease(sem_uart_dma);
    }
}

/**
  * @brief  Rx Half Transfer completed callbacks.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if(&huart4 == huart)
    {
        UARTControl.writeData(&huart->pRxBuffPtr[0], 1);
    }
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(&huart4 == huart)
    {
        UARTControl.writeData(&huart->pRxBuffPtr[1], 1);
    }
}
