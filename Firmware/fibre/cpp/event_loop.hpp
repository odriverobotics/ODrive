#ifndef __FIBRE_EVENT_LOOP_HPP
#define __FIBRE_EVENT_LOOP_HPP

#include <stdint.h>

struct EventLoopTimer;

/**
 * @brief Base class for event loops.
 * 
 * Thread-safety: The functions of this class must not be assumed to be thread-safe.
 * Generally the functions of an event loop are only safe to be called from the
 * event loop's thread itself.
 */
class EventLoop {
public:
    /**
     * @brief Registers a callback for immediate execution on the event loop thread.
     */
    virtual int post(void (*callback)(void*), void *ctx) = 0;

    virtual int register_event(int event_fd, uint32_t events, void (*callback)(void*), void* ctx) = 0;
    virtual int deregister_event(int event_fd) = 0;

    /**
     * @brief Registers a callback to be called at a later point in time.
     * 
     * This returns an opaque handler which can be used to cancel the timer.
     * 
     * @param delay: The delay from now in seconds.
     *               TOOD: specify if OS sleep time is counted in.
     */
    virtual struct EventLoopTimer* call_later(float delay, void (*callback)(void*), void *ctx) = 0;

    /**
     * @brief Cancels a timer which was previously started by call_later().
     * 
     * Must not be called after invokation of the callback has started.
     * This also means that cancel_timer() must not be called from within the
     * callback of the timer itself.
     */
    virtual int cancel_timer(EventLoopTimer* timer) = 0;
};

#endif // __FIBRE_EVENT_LOOP_HPP