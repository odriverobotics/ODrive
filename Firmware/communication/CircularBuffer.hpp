/*******************************************************************************
* File          : circularBuffer.hpp
*
* Description   : Implementation file for circular buffers
*
* Project       :
*
* Author        : S.Gilbert
*
* Created on    : 09 Dec 2019
*
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
NAMESPACE
*******************************************************************************/

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

typedef enum
{
	CB_OK = 0,
	CB_FAIL,
	CB_SIZE_ERROR,
	CB_POINTER_ERROR
}CB_Status_t;

/*******************************************************************************
TYPES
*******************************************************************************/

/*******************************************************************************
GLOBAL VARIABLES
*******************************************************************************/

/*******************************************************************************
FUNCTION PROTOTYPES
*******************************************************************************/

template <class dataType_t>
class CCBBuffer
{
    typedef struct
    {
        size_t numElements;                                                     /* maximum number of elements */
        volatile int32_t start;                                                 /* index of oldest element */
        volatile int32_t end;                                                   /* index at which to write new element */
    }CBTracker_t;

    typedef struct
    {
        CBTracker_t tracker;                                                    /* buffer size and start/end counters */
        dataType_t * pArray;                                                    /* pointer to array of elements. to be set on buffer init */
    }circularBuffer_t;
public:

public:

    CCBBuffer(dataType_t * pArray = nullptr, const size_t size = 0, const bool useMutex = false, const bool overwriteOldData = false);
    ~CCBBuffer() = default;
    bool isFull(void);
    bool isEmpty(void);
    size_t remainingSpace(void);
    size_t remainingSpaceLinear(void);
    size_t usedSpace(void);
    size_t usedSpaceLinear(void);
    bool flushBuffer(void);
    int32_t write(const dataType_t * const elem, size_t length);
    int32_t write(const dataType_t Data);
    int32_t peak(dataType_t * pData, size_t length);
    int32_t read(dataType_t * elem, size_t length);
    bool readNewest(dataType_t * pData);

private:
    bool isPowerOfTwo(size_t x);

private:
    circularBuffer_t m_cb;
    bool m_useMutex;
    bool m_overwriteOldData;

}; /* CCBBuffer */

/*******************************************************************************
INLINE FUNCTIONS
*******************************************************************************/

/**\brief   Constructor
 *
 * \param   pArray              - pointer to external memory
 * \param   numElements         - number of dataType_t's in ring buffer to be created, must be a value ^2 if doing bulk writes
 * \param   useMutex            - enables or disables mutex locking. Defaults to false
 * \param   overwriteOldData    - enables discarding of old data. Defaults to false
 *
 * \return  None
 */
template <class dataType_t>
CCBBuffer<dataType_t>::CCBBuffer(dataType_t * pArray, const size_t numElements, const bool useMutex, const bool overwriteOldData)
    : m_cb {{numElements, 0, 0}, pArray}
    , m_useMutex(useMutex)
    , m_overwriteOldData(overwriteOldData)
{
//        assert(this->isPowerOfTwo(size), "The size must be of ^2 otherwise overflow condition of start/end tracker values will corrupt data in buffer ");
    (void)flushBuffer();
}

/**\brief   Checks if the ring buffer is full
 *
 * \param   None
 *
 * \return  TRUE for full FALSE for not
 */
template <class dataType_t>
bool CCBBuffer<dataType_t>::isFull(void)
{
    return (((m_cb.tracker.end + 1) % (int32_t)m_cb.tracker.numElements) == m_cb.tracker.start);
}

/**\brief   Checks if the ring buffer is empty
 *
 * \param   None
 *
 * \return  FALSE for non-empty TRUE for not
 */
template <class dataType_t>
bool CCBBuffer<dataType_t>::isEmpty(void)
{
    return (m_cb.tracker.end == m_cb.tracker.start);
}

/**\brief   Checks available space in the buffer
 *
 * \param   None
 *
 * \return  Unused space in buffer
 */
template <class dataType_t>
size_t CCBBuffer<dataType_t>::remainingSpace(void)
{
    return (m_cb.tracker.numElements - usedSpace());
}

/**\brief   Checks linearly free space in the buffer from end pointer to array
 *          start OR string array which ever is reached first (no wrap around)
 *
 * \param   None
 *
 * \return  Used space in buffer
 */
template <class dataType_t>
size_t CCBBuffer<dataType_t>::remainingSpaceLinear(void)
{
    size_t length = 0;

/* if end is less than start index then data goes from start pointer to size of
 * array and start of array to end index. free space exists between end index
 * and start index. If start is less than end index, then data does from end
 * index to numElements of array and start of array to start index.
 */
    if ((m_overwriteOldData) || !isFull())
    {
        length = (size_t)(((m_cb.tracker.end < m_cb.tracker.start) ? m_cb.tracker.start : (int32_t)m_cb.tracker.numElements) - m_cb.tracker.end);
    }

    return length;
}

/**\brief   Checks used space in the buffer
 *
 * \param   None
 *
 * \return  Used space in buffer
 */
template <class dataType_t>
size_t CCBBuffer<dataType_t>::usedSpace(void)
{
    return (size_t)(m_cb.tracker.end + ((m_cb.tracker.end < m_cb.tracker.start) ? (int32_t)m_cb.tracker.numElements : 0) - m_cb.tracker.start);
}

/**\brief   Checks linearly used space in the buffer from start pointer to array
 *          end OR string array which ever is reached first (no wrap around)
 *
 * \param   None
 *
 * \return  Used space in buffer
 */
template <class dataType_t>
size_t CCBBuffer<dataType_t>::usedSpaceLinear(void)
{
    return (size_t)(((m_cb.tracker.end < m_cb.tracker.start) ? (int32_t)m_cb.tracker.numElements : m_cb.tracker.end) - m_cb.tracker.start);
}

/**\brief   Resets indexers effectively emptying buffer
 *
 * \param   None
 *
 * \return  TRUE if successful FALSE if fail
 */
template <class dataType_t>
bool CCBBuffer<dataType_t>::flushBuffer(void)
{
    if(m_useMutex)
    {

    }

    memset(m_cb.pArray, 0, (m_cb.tracker.numElements * sizeof(dataType_t)));
    m_cb.tracker.start = 0;
    m_cb.tracker.end   = 0;

    return (m_cb.tracker.start == m_cb.tracker.end);
}

/**\brief   Writes a number of elements to the buffer, exiting if buffer is full
 *
 * \param   pData   - Pointer to data element to be stored
 * \param   length  - number of entries written to the buffer
 *
 * \return  number of entries written
 */
template <class dataType_t>
int32_t CCBBuffer<dataType_t>::write(const dataType_t * const pData, size_t length)
{
    int32_t writeCnt = 0;
    size_t toWrite = 0;

    if(m_useMutex)
    {

    }

    for (writeCnt = 0; ((writeCnt < (int32_t)length)); writeCnt += toWrite)
    {
        size_t linearLength = remainingSpaceLinear();
        size_t leftToWrite = length - writeCnt;
        toWrite = (leftToWrite < linearLength) ? leftToWrite : linearLength;

        if(isFull())                                                            /* check if the buffer is full */
        {
            if(!m_overwriteOldData)                                             /* if we can't overwrite old data */
            {
                break;                                                          /* get out */
            }
            else                                                                /* else */
            {
                m_cb.tracker.start = ((m_cb.tracker.start + toWrite) % (int32_t)m_cb.tracker.numElements);     /* make space by move the start pointer forward the necessary elements */
            }
        }

        memcpy(&m_cb.pArray[m_cb.tracker.end], &pData[writeCnt], toWrite * sizeof(dataType_t));
        m_cb.tracker.end = ((m_cb.tracker.end + toWrite) % (int32_t)m_cb.tracker.numElements);
    }

    return writeCnt;
}

/**\brief   Writes a number of elements to the buffer, exiting if buffer is full
 *
 * \param   Data    - Single data element to be stored
 *
 * \return  number of entries written
 */
template <class dataType_t>
int32_t CCBBuffer<dataType_t>::write(const dataType_t Data)
{
    return write(&Data, 1);
}

/**\brief   peaks at a length of data starting from oldest element. Does not remove from buffer
 *
 * \param   pData   - Pointer to place where data is to be returned
 * \param   length  - number of entries to be read from the buffer
 *
 * \return  number of elements read
 */
template <class dataType_t>
int32_t CCBBuffer<dataType_t>::peak(dataType_t * pData, size_t length)
{
    int32_t readCnt = 0;
    size_t toRead = 0;

    if(m_useMutex)
    {

    }

    for (readCnt = 0; (!isEmpty() && readCnt < (int32_t)length); readCnt =+ toRead)
    {
        size_t linearLength = usedSpaceLinear();
        size_t leftToRead = length - readCnt;
        toRead = (leftToRead < linearLength) ? leftToRead : linearLength;
        memcpy(&pData[readCnt], &m_cb.pArray[m_cb.tracker.start], toRead * sizeof(m_cb.pArray[0]));
    }

    return readCnt;
}

/**\brief   Reads a length of data starting from oldest element.
 *
 * \param   pData   - Pointer to place where data is to be returned
 * \param   length  - number of entries to be read from the buffer
 *
 * \return  number of elements read
 */
template <class dataType_t>
int32_t CCBBuffer<dataType_t>::read(dataType_t * pData, size_t length)
{
    int32_t readCnt = 0;
    size_t toRead = 0;

    if(m_useMutex)
    {

    }

    for (readCnt = 0; (!isEmpty() && readCnt < (int32_t)length); readCnt =+ toRead)
    {
        size_t linearLength = usedSpaceLinear();
        size_t leftToRead = length - readCnt;
        toRead = (leftToRead < linearLength) ? leftToRead : linearLength;
        memcpy(&pData[readCnt], &m_cb.pArray[m_cb.tracker.start], toRead * sizeof(dataType_t));
        m_cb.tracker.start = ((m_cb.tracker.start + toRead) % (int32_t)m_cb.tracker.numElements);
    }

    return readCnt;
}

/**\brief   Read newest element.
 *
 * \param   pData   - Pointer to place where data is to be returned
 *
 * \return  false for no data
 */
template <class dataType_t>
bool CCBBuffer<dataType_t>::readNewest(dataType_t * pData)
{
    bool empty = this->isEmpty();

    if(m_useMutex)
    {

    }

    if(!empty)
    {
        size_t endButOne = ((m_cb.tracker.end + ((int32_t)m_cb.tracker.numElements - 1)) % (int32_t)m_cb.tracker.numElements);
        *pData = m_cb.pArray[endButOne];
    }

    return !empty;
}

/**\brief   Calculates whether a value in the argument is to the power of 2
 *          by taking the bit pattern of x and logically ANDing it with the bit
 *          pattern of (x-1). If the result is 0 then the number is = ^2
 *
 * \param   x   - value to test
 *
 * \return  true if value is non-zero and to the power of 2, else returns false
 */
template <class dataType_t>
bool CCBBuffer<dataType_t>::isPowerOfTwo(size_t x)
{
    return (x != 0u) && ((x & (x - 1u)) == 0u);
}

#endif /* CIRCULAR_BUFFER_H --------------------------------------------------*/
