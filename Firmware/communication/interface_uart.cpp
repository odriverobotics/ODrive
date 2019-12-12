
#include "interface_uart.h"
#include "ascii_protocol.hpp"
#include "CircularBuffer.hpp"
#include <MotorControl/utils.h>
#include <fibre/protocol.hpp>
#include <usart.h>
#include <cmsis_os.h>
#include <freertos_vars.h>

#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 64

// DMA open loop continous circular buffer
// 1ms delay periodic, chase DMA ptr around
static uint8_t dma_rx_buffer[2];
// static uint32_t dma_last_rcv_idx;

static uint8_t RXBuffer[UART_RX_BUFFER_SIZE];
CCBBuffer<uint8_t> CRXCircularBuffer(RXBuffer, sizeof(RXBuffer), false, false);

// FIXME: the stdlib doesn't know about CMSIS threads, so this is just a global variable
// static thread_local uint32_t deadline_ms = 0;

osThreadId uart_thread;


class UART4Sender 
    : public StreamSink
{
public:
    UART4Sender()
        : CTXCircularBuffer(TXBuffer, sizeof(TXBuffer), false, false){}
    int process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes)
    {
        // Loop to ensure all bytes get sent
        while (length)
        {
            size_t chunk = (length < UART_TX_BUFFER_SIZE) ? length : UART_TX_BUFFER_SIZE;
            // wait for USB interface to become ready
            // TODO: implement ring buffer to get a more continuous stream of data
            // if (osSemaphoreWait(sem_uart_dma, deadline_to_timeout(deadline_ms)) != osOK)
            if (osSemaphoreWait(sem_uart_dma, PROTOCOL_SERVER_TIMEOUT_MS) != osOK)
            {
                return -1;
            }
            else
            {
                // transmit chunk
                if (HAL_UART_Transmit_DMA(&huart4, (uint8_t *)memcpy(tx_buf_, buffer, chunk), chunk) != HAL_OK)
                {
                    return -1;
                }
                else
                {
                    buffer += chunk;
                    length -= chunk;
                    if (processed_bytes)
                    {
                        *processed_bytes += chunk;
                    }
                }
            }
        }
        return 0;
    }

    size_t get_free_space() { return SIZE_MAX; }
private:
    uint8_t tx_buf_[UART_TX_BUFFER_SIZE];
    uint8_t TXBuffer[UART_TX_BUFFER_SIZE];
    CCBBuffer<uint8_t> CTXCircularBuffer;
} uart4_stream_output;

StreamSink * uart4_stream_output_ptr = &uart4_stream_output;
StreamBasedPacketSink uart4_packet_output(uart4_stream_output);
BidirectionalPacketBasedChannel uart4_channel(uart4_packet_output);
StreamToPacketSegmenter uart4_stream_input(uart4_channel);

static void uart_server_thread(void * ctx)
{
    (void) ctx;

    for (;;)
    {
        uint8_t RXData[UART_RX_BUFFER_SIZE] = {0};
        size_t length = CRXCircularBuffer.read(RXData, sizeof(RXData));

        uart4_stream_input.process_bytes(RXData, length, nullptr); // TODO: use process_all
        ASCII_protocol_parse_stream(RXData, length, uart4_stream_output);

        osThreadSuspend(uart_thread);
        // osDelay(1);
    };
}

void start_uart_server()
{
    // DMA is set up to recieve in a circular buffer forever.
    // We dont use interrupts to fetch the data, instead we periodically read
    // data out of the circular buffer into a parse buffer, controlled by a state machine
    HAL_UART_Receive_DMA(&huart4, dma_rx_buffer, sizeof(dma_rx_buffer));
    // dma_last_rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;

    // Start UART communication thread
    osThreadDef(uart_server_thread_def,
                uart_server_thread,
                osPriorityNormal,
                0,
                1024 /* the ascii protocol needs considerable stack space */);
    uart_thread = osThreadCreate(osThread(uart_server_thread_def), NULL);
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
        HAL_UART_AbortReceive(&huart4);
        HAL_UART_Receive_DMA(&huart4, dma_rx_buffer, sizeof(dma_rx_buffer));
    }
}

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
        CRXCircularBuffer.write(&huart->pRxBuffPtr[0], 1);
        osThreadResume(uart_thread);
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
        CRXCircularBuffer.write(&huart->pRxBuffPtr[1], 1);
        osThreadResume(uart_thread);
    }
}
