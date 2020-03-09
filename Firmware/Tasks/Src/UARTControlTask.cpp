/*******************************************************************************
* File          : UARTControlTask.cpp
*
* Description   : UART receive and transmit handling task
*
* Project       :
*
* Author        : S.Gilbert
*
* Created on    : 14 Jan 2020
*
*******************************************************************************/

/******************************************************************************
INCLUDES
*******************************************************************************/

#include "UARTControlTask.hpp"
#include "ascii_protocol.hpp"

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

/*******************************************************************************
MODULE VARIABLES
*******************************************************************************/

/*******************************************************************************
INTERNAL FUNCTION DEFINTIONS
*******************************************************************************/

/*******************************************************************************
FUNCTION DECLARATIONS
*******************************************************************************/

/**\brief   Constructor
 *
 * \param   pTaskName   - pointer to the task name
 * \param   freq        - task frequency. Set to zero for non periodic task
 * \param   priority    - task priority
 * \param   pStack      - pointer to stack space
 * \param   stackSize   - stack size in bytes
 * \param   pHuart      - pointer to the UART handle
 * \param   pStreamIn   - pointer to the input parser
 * \param   rStreamOut  - reference to the output stream handler
 * \param   pWatchdog   - pointer to the watchdog class
 * \param   timeoutMS   - watchdog timeout
 *
 * \return  None
 */
CUARTControlTask::CUARTControlTask(char const * const pTaskName
                                         , const float freq
                                         , const int32_t priority
                                         , void * const pStack
                                         , const size_t stackSize
                                         , UART_HandleTypeDef * pHuart
                                         , StreamToPacketSegmenter * pStreamIn
                                         , StreamSink & rStreamOut
                                         , CWatchdogBase * pWatchdog
                                         , int32_t timeoutMS)
    : threadCore::CTaskBase(pTaskName, freq, priority, pStack, stackSize, pWatchdog, timeoutMS)
    , m_pHuart(pHuart)
    , m_RXCircularBuffer(m_RXBuffer, ARRAY_LEN(m_RXBuffer))
    , m_pStreamIn(pStreamIn)
    , m_rStreamOut(rStreamOut)
{}

/**\brief   Sets up the UART to receive
 *
 * \param   None
 *
 * \return  None
 */
void CUARTControlTask::funcBegin(void)
{
    // DMA open loop continuous circular buffer
    static uint8_t dma_rx_buffer[2];

    // DMA is set up to receive in a circular buffer forever.
    // We don't use interrupts to fetch the data, instead we periodically read
    // data out of the circular buffer into a parse buffer, controlled by a state machine
    HAL_UART_Receive_DMA(m_pHuart, dma_rx_buffer, sizeof(dma_rx_buffer));
}

/**\brief   Main thread function, this is called when the task resumes from a
 *          a suspend, and suspends the thread when it returns. Treat this as
 *          called from within a while loop.
 *          Wakes on receiving serial data, reads data out of circular buffer
 *          and sends it to the parser/handler.
 *
 * \param   None
 *
 * \return  None
 */
void CUARTControlTask::funcMain(void)
{
    uint8_t RXData[UART_RX_BUFFER_SIZE] = {0};
    size_t length = m_RXCircularBuffer.read(RXData, ARRAY_LEN(RXData));

    m_pStreamIn->process_bytes(RXData, length, nullptr); // TODO: use process_all
    ASCII_protocol_parse_stream(RXData, (uint16_t)length, m_rStreamOut);
}

/**\brief   Read's data in and writes it to the circular buffer, then wakes the
 *          task to process the data.
 *
 * \param   pData       - pointer to the data to read in
 * \param   length      - number of characters in the buffer
 *
 * \return  None
 */
void CUARTControlTask::readData(uint8_t * pData, size_t length)
{
    (void)m_RXCircularBuffer.write(pData, length);
    this->resumeTaskFromISR();
}
