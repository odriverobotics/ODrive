/*******************************************************************************
* File          : LinkedList.hpp
*
* Description   : 
*
* Project       :
*
* Author        : s.gilbert
*
* Created on    : 21 Feb 2020
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LINKEDLIST_HPP
#define LINKEDLIST_HPP

/******************************************************************************
INCLUDES
*******************************************************************************/

#include "stddef.h"
#include "stdint.h"
#include "string.h"

/*******************************************************************************
DEFINITIONS
*******************************************************************************/

#define LL_SUCCESS (0)
#define LL_FAIL (-1)

/*******************************************************************************
TYPES
*******************************************************************************/

/*******************************************************************************
GLOBAL VARIABLES
*******************************************************************************/

/*******************************************************************************
CONSTANTS
*******************************************************************************/

/*******************************************************************************
NAMESPACE
*******************************************************************************/

template <class MyType>
class CNode
{
using node_t = CNode<MyType>;

public:
    CNode(MyType const * const pData, node_t const * const pPrevious = nullptr, node_t const * const pNext = nullptr);
    void populateNode(MyType const * const pData, node_t * pNext, node_t * pPrevious);
    void flushNode(void);
    void getData(MyType * pData);
    node_t * getPrevious(void);
    void setPrevious(node_t * pPrevious);
    node_t * getNext(void);
    void setNext(node_t * pNext);

private:
    MyType m_data;                                                              /* stored data */
    node_t * m_pPrevious;                                                       /* pointer to previous data node */
    node_t * m_pNext;                                                           /* pointer to next data node */
};

/*******************************************************************************
 *
 ******************************************************************************/

template <class MyType>
class CLinkedList
{
using node_t = CNode<MyType>;

public:
    CLinkedList(void * pTable = nullptr, size_t size = 0);
    ~CLinkedList() = default;
    int32_t pushToFront(MyType const * const pData);
    int32_t pushToBack(MyType const * const pData);
    int32_t pushBefore(node_t const * const pNextNode, MyType const * const pData);
    int32_t pushAfter(node_t const * const pPrevNode, MyType const * const pData);
    int32_t popFromFront(MyType const * const pData);
    int32_t popFromBack(MyType const * const pData);
    int32_t popFromNode(uint32_t nodeID, MyType * pData);
    int32_t peakFromNode(uint32_t nodeID, MyType * pData);
    size_t countNodes(void);
    size_t getMaxNodes(void);
    bool deleteNode(size_t nodeID);
    void * findData(MyType const * const pData, bool findFirst);
    void * findNode(uint32_t nodeID);

private:
    node_t * newNode(void);
    void freeNode(node_t * pNode);
    bool removeNode(node_t * pDelete);

private:
    node_t * m_pTable;
    size_t m_length;                                                            /* length of the list */
    size_t m_activeNodes;                                                       /* nodes in list */
    node_t * m_pHead;                                                           /* pointer to the first node in the chain */
};

/*******************************************************************************
INLINE FUNCTION DEFINITIONS
*******************************************************************************/

/**\brief   Constructor.
 *
 * \param   pData       - pointer to the data to store
 * \param   pNext       - pointer to next node
 * \param   pPrevious   - pointer to previous node
 *
 * \return  None
 */
template <class MyType>
CNode<MyType>::CNode(MyType const * const pData, node_t const * const pPrevious, node_t const * const pNext)
    : m_data(* pData)
    , m_pPrevious(pPrevious)
    , m_pNext(pNext)
{}

/**\brief   Populates current node with the data, and next and previous pointers
 *
 * \param   pData       - pointer to the data to store
 * \param   pNext       - pointer to next node
 * \param   pPrevious   - pointer to previous node
 *
 * \return  None
 */
template <class MyType>
void CNode<MyType>::populateNode(MyType const * const pData, node_t * pNext, node_t * pPrevious)
{
    (void)memcpy(&m_data, pData, sizeof(MyType));                               /* copy the data in to the node */
    m_pNext = pNext;
    m_pPrevious = pPrevious;
}

/**\brief   Erases data in the node and resets the pointers
 *
 * \param   None
 *
 * \return  None
 */
template <class MyType>
void CNode<MyType>::flushNode(void)
{
    MyType tempData;
    populateNode((MyType *)memset((void *)&tempData, 0, sizeof(MyType)), nullptr, nullptr);
}

/**\brief   Peaks at element
 *
 * \param   pData   - pointer to where to write the data to
 *
 * \return  None
 */
template <class MyType>
void CNode<MyType>::getData(MyType * pData)
{
    (void)memcpy(pData, &m_data, sizeof(MyType));                               /* copy out the data to the target pointer */
}

/**\brief   Gets the address of the previous node.
 *
 * \param   None
 *
 * \return  pointer to previous node
 */
template <class MyType>
CNode<MyType> * CNode<MyType>::getPrevious(void)
{
    return m_pPrevious;
}

/**\brief   Sets the address of the previous node.
 *
 * \param   pointer to previous node
 *
 * \return  None
 */
template <class MyType>
void CNode<MyType>::setPrevious(node_t * pPrevious)
{
    m_pPrevious = pPrevious;
}

/**\brief   Gets the address of the next node.
 *
 * \param   None
 *
 * \return  pointer to next node
 */
template <class MyType>
CNode<MyType> * CNode<MyType>::getNext(void)
{
    return m_pNext;
}

/**\brief   Sets the address of the next node.
 *
 * \param   None
 *
 * \return  pointer to next node
 */
template <class MyType>
void CNode<MyType>::setNext(node_t * pNext)
{
    m_pNext = pNext;
}

/*******************************************************************************
 *
 ******************************************************************************/

/**\brief   Constructor
 *
 * \param   pTable  - pointer to memory space to be used to store link list
 * \param   size    - byte count of the array
 *
 * \return  None
 */
template <class MyType>
inline CLinkedList<MyType>::CLinkedList(void * pTable, size_t size)
    : m_pTable((node_t *)pTable)
    , m_length(size / sizeof(node_t))
    , m_activeNodes(0)
    , m_pHead(nullptr)
{
    memset(m_pTable, 0, size);
    for(auto i = 0u; i < m_length; ++i)
    {
        node_t * pTable = &m_pTable[i];
        pTable->setNext((i < (m_length - 1))    ? &pTable[1]    : nullptr);
        pTable->setPrevious((i == 0)            ? nullptr       : &pTable[-1]);
    }
}

/**\brief   Push data element to the front of the list
 *
 * \param   ppHeadRef   - pointer to the list
 * \param   pData       - pointer to the data to store
 *
 * \return  LL_SUCCESS on successful allocation else LL_FAIL
 */
template <class MyType>
inline int32_t CLinkedList<MyType>::pushToFront(MyType const * const pData)
{
    int32_t returnVal = LL_FAIL;
    node_t * pNewNode = newNode();

    if (nullptr != pNewNode)
    {
        /* copy in the data to the node and make 'next' of the new node point to
         * current m_pHead and set 'previous' pointer to nullptr
         */
        pNewNode->populateNode(pData, m_pHead, nullptr);

        /* set 'previous' of head node to new node */
        if (nullptr != m_pHead)
        {
            m_pHead->setPrevious(pNewNode);
        }

        /* set the head pointer to point to the new node */
        m_pHead = pNewNode;

        returnVal = LL_SUCCESS;
    }

    return returnVal;
}

/**\brief   Push data element to the back of the list
 *
 * \param   pData   - pointer to the data to store
 *
 * \return  LL_SUCCESS on successful allocation else LL_FAIL
 */
template <class MyType>
inline int32_t CLinkedList<MyType>::pushToBack(MyType const * const pData)
{
    int32_t returnVal = LL_FAIL;
    node_t * pNewNode = newNode();                                               /* allocate node */

    if (nullptr != pNewNode)
    {
        node_t * last = (node_t *)findNode((countNodes() - 1));

        /* copy in the data to the node, this new node is going to be the last
         * node in the chain, so make 'next' pointer nullptr
         */
        pNewNode->populateNode(pData, nullptr, last);

        /* Make last node as previous of new node */
        if(nullptr != last)
        {
            last->setNext(pNewNode);                                            /* Change the next of last node */
        }
        else
        {
            m_pHead = pNewNode;
        }

        returnVal = LL_SUCCESS;
    }

    return returnVal;
}

/**\brief   Insert data element after selected node.
 *
 * \param   pNextNode   - pointer to node to insert after
 * \param   pData       - pointer to the data to store
 *
 * \return  LL_SUCCESS on successful allocation else LL_FAILED
 */
template <class MyType>
inline int32_t CLinkedList<MyType>::pushBefore(node_t const * const pNextNode, MyType const * const pData)
{
    int32_t returnVal = LL_FAIL;
    node_t * newNode = newNode();                                               /* allocate node */

    if (nullptr != pNextNode)                                                   /* check if the given next_node is NULL */
    {
        /* copy in the data to the new node make 'next' of new node as 'next'
         * of prev_node and make prev_node as previous of newNode
         */
        newNode->populateNode(pData, pNextNode, pNextNode->m_pPrevious);

        pNextNode->m_pPrevious = newNode;                                       /* Make the previous of next_node as newNode */

        if (nullptr != newNode->m_pPrevious)                                    /* Change next of newNode's previous node */
        {
            newNode->m_pPrevious->m_pNext = newNode;
        }
        returnVal = LL_SUCCESS;
    }

    return returnVal;
}

/**\brief   Insert data element after selected node.
 *
 * \param   pPrevNode   - pointer to node to insert after
 * \param   pData       - pointer to the data to store
 *
 * \return  LL_SUCCESS on successful allocation else LL_FAILED
 */
template <class MyType>
inline int32_t CLinkedList<MyType>::pushAfter(node_t const * const pPrevNode, MyType const * const pData)
{
    int32_t returnVal = LL_FAIL;
    node_t * newNode = newNode();                                               /* allocate node */

    if (nullptr != pPrevNode)                                                   /* check if the given prev_node is NULL */
    {
        /* copy in the data to the new node make 'next' of new node as 'next'
         * of prev_node and make prev_node as previous of newNode
         */
        newNode->populateNode(pData, pPrevNode->m_pNext, pPrevNode);

        pPrevNode->m_pNext = newNode;                                           /* Make the next of prev_node as newNode */

        if (nullptr != newNode->m_pNext)                                        /* Change previous of newNode's next node */
        {
            newNode->m_pNext->m_pPrevious = newNode;
        }
        returnVal = LL_SUCCESS;
    }

    return returnVal;
}

/**\brief   Pop first element off the list
 *
 * \param   pData   - pointer to where to write the data to
 *
 * \return  LL_SUCCESS on successful allocation else LL_FAIL
 */
template <class MyType>
inline int32_t CLinkedList<MyType>::popFromFront(MyType const * const pData)
{
    return popFromNode(0, pData);
}

/**\brief   Pop last element off the list
 *
 * \param   pData   - pointer to where to write the data to
 *
 * \return  LL_SUCCESS on successful allocation else LL_FAIL
 */
template <class MyType>
inline int32_t CLinkedList<MyType>::popFromBack(MyType const * const pData)
{
    return popFromNode(countNodes(), pData);
}

/**\brief   Pop from specific node
 *
 * \param   nodeID  - node to read from, 0 is head
 * \param   pData   - pointer to where to write the data to
 *
 * \return  LL_SUCCESS on successful allocation else LL_FAIL
 */
template <class MyType>
inline int32_t CLinkedList<MyType>::popFromNode(uint32_t nodeID, MyType * pData)
{
    int32_t returnVal = LL_FAIL;

    if (LL_SUCCESS == peakFromNode(nodeID, pData))
    {
        returnVal = deleteNode(nodeID);
    }

    return returnVal;
}

/**\brief   Peaks at node specified in nodeID
 *
 * \param   nodeID  - node ID to check, 0 is head
 * \param   pData   - pointer to where to write the data to
 *
 * \return  LL_SUCCESS on peak, else returns node ID of last node in chain
 */
template <class MyType>
inline int32_t CLinkedList<MyType>::peakFromNode(uint32_t nodeID, MyType * pData)
{
    int32_t returnVal = LL_FAIL;
    node_t * node = (node_t *)findNode(nodeID);

    if (nullptr != node)
    {
        node->getData(pData);
        returnVal = LL_SUCCESS;
    }

    return returnVal;
}

/**\brief   Counts the number of elements in the list
 *
 * \param   None
 *
 * \return  the number of nodes
 */
template <class MyType>
inline size_t CLinkedList<MyType>::countNodes(void)
{
    return m_activeNodes;
}

/**\brief   returns the maximum number of elements the list can hold
 *
 * \param   None
 *
 * \return  the maximum number of nodes
 */
template <class MyType>
inline size_t CLinkedList<MyType>::getMaxNodes(void)
{
    return m_length;
}

/**\brief   Deletes node at the given nodeID, this is the distance from head.
 * From https://www.geeksforgeeks.org/delete-doubly-linked-list-node-given-position/
 *
 * \param   nodeID  - index of node to delete
 *
 * \return  LL_SUCCESS on successful allocation else LL_FAIL
 */
template <class MyType>
inline bool CLinkedList<MyType>::deleteNode(size_t nodeID)
{
    int32_t returnVal = LL_FAIL;
    int32_t activeNodeCount = countNodes();

    /* if list in NULL or invalid position is given */
    if ((nullptr != m_pHead) && ((int32_t)nodeID <= activeNodeCount))
    {
        // traverse up to the node to find the pointer at nodeID and delete it
        returnVal = removeNode((node_t *)findNode(nodeID));
    }

    return returnVal;
}

/**\brief   Finds if a node holds the datum and returns its address
 *
 * \param   pData       - pointer to the datum to find
 * \param   findFirst   - if true return address of first instance, else
 *                          return address of last instance
 *
 * \return  pointer to node containing the datum
 */
template <class MyType>
inline void * CLinkedList<MyType>::findData(MyType const * const pData, bool findFirst)
{
    node_t * returnVal = nullptr;
    node_t * node = m_pHead;
    MyType nodeData;
    size_t nodeCount = countNodes();

    for(auto i = 0u; i < nodeCount; ++i)
    {
        node->getData(&nodeData);
        if(*pData == nodeData)
        {
            returnVal = node;
            if(findFirst)
            {
                break;
            }
        }
        node = node->getNext();
    }

    return (void *)node;
}

/**\brief   Finds the active node at the given ID and returns address
 *
 * \param   nodeID  - index of node to find
 *
 * \return  pointer to node
 */
template <class MyType>
inline void * CLinkedList<MyType>::findNode(uint32_t nodeID)
{
    node_t * node = m_pHead;

    for(auto i = 0u; i < nodeID; ++i)
    {
        if(nullptr != node->getNext())
        {
            node = node->getNext();
        }
        else
        {
            break;
        }
    }

    return (void *)node;
}


/**\brief   Assigns a node from the inactive pool
 *
 * \param   None
 *
 * \return  pointer to the new node
 */
template <class MyType>
CNode<MyType> * CLinkedList<MyType>::newNode(void)
{
    CNode<MyType> * pNewNode = nullptr;

    if(m_activeNodes < m_length)
    {
        pNewNode = m_pTable;
        m_pTable = m_pTable->getNext();

        if(pNewNode)
        {
            ++m_activeNodes;
        }
    }

    return pNewNode;
}

/**\brief   Returns the node to the inactive pool
 *
 * \param   pDelete - pointer to node to delete
 *
 * \return  None
 */
template <class MyType>
void CLinkedList<MyType>::freeNode(node_t * pNode)
{
    if(pNode)
    {
        pNode->setPrevious(nullptr);
        pNode->setNext(m_pTable);
        m_pTable->setPrevious(pNode);
        m_pTable = pNode;
        --m_activeNodes;
    }
}

/**\brief   Deletes node in list and corrects the nodes either side to maintain
 *          the linked list chain.
 *
 * \param   pDelete - pointer to node to delete
 *
 * \return  bool
 */
template <class MyType>
inline bool CLinkedList<MyType>::removeNode(node_t * pDelete)
{
    int32_t returnVal = LL_FAIL;
    /* base case */
    if ((nullptr != m_pHead) && (nullptr != pDelete))
    {

        /* If node to be deleted is head node */
        if (m_pHead == pDelete)
        {
            m_pHead = pDelete->getNext();
        }

        /* Change next only if node to be deleted is NOT the last node */
        if (nullptr != pDelete->getNext())
        {
            pDelete->getNext()->setPrevious(pDelete->getPrevious());
        }

        /* Change prev only if node to be deleted is NOT the first node */
        if (nullptr != pDelete->getPrevious())
        {
            pDelete->getPrevious()->setNext(pDelete->getNext());
        }

        pDelete->flushNode();
        freeNode(pDelete);
        returnVal = LL_SUCCESS;
    }

    return returnVal;
}

#endif /* LINKEDLIST_HPP */
