/*******************************************************************************
* File          : ThreadTemplate.hpp
*
* Description   : 
*
* Project       : 
*
* Author        : s.gilbert
*
* Created on    : 11 Jan 2020
*
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef THREADTEMPLATE_HPP
#define THREADTEMPLATE_HPP

/*******************************************************************************
INCLUDES
*******************************************************************************/

#include "ThreadBase.hpp"

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
NAMESPACE
*******************************************************************************/

namespace threadCore
{

template <typename ObjT>
class CThreadTemplate
        : public CThreadBase
{
public:
    using FuncT = void(ObjT::*)();

public:
    CThreadTemplate(void * const pStack = nullptr, const size_t stackSize = 0);
    CThreadTemplate(FuncT func, ObjT * const pObj);
    void Set(FuncT func, ObjT* const pObj);
    void threadFunc() override;

protected:
    ObjT * m_pObj;
    FuncT m_func;
};

/*******************************************************************************
INLINE FUNCTION DEFINITIONS
*******************************************************************************/

/**\brief   Constructor.
 *
 * \param   pStack      - Pointer to the statically declared stack
 * \param   stackSize   - size of the stack size
 *
 * \return  None
 */
template <typename ObjT>
CThreadTemplate<ObjT>::CThreadTemplate(void * const pStack, const size_t stackSize)
    : CThreadBase(pStack, stackSize)
    , m_pObj(nullptr)
    , m_func(nullptr)
{}

/**\brief   Constructor.
 *
 * \param   func        -
 * \param   pObj        -
 *
 * \return  None
 */
template <typename ObjT>
CThreadTemplate<ObjT>::CThreadTemplate(FuncT func, ObjT * const pObj)
    : m_pObj(pObj)
    , m_func(func)
{}

/**\brief   Set.
 *
 * \param   func        -
 * \param   pObj        -
 *
 * \return  None
 */
template <typename ObjT>
void CThreadTemplate<ObjT>::Set(FuncT func, ObjT * const pObj)
{
    m_pObj = pObj;
    m_func = func;
}

/**\brief   Main thread loop.
 *
 * \param   None
 *
 * \return  None
 */
template <typename ObjT>
void CThreadTemplate<ObjT>::threadFunc()
{
    (m_pObj->*m_func)();
    while (this->m_isRunning)
    {
        CThreadBase::sleepMilliseconds(10);
    }
    this->m_thread.resume();
};

} /* namespace threadCore */

#endif /* THREADTEMPLATE_HPP */
