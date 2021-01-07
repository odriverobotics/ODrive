
#include "epoll_event_loop.hpp"
#include "../logging.hpp"

#include <sys/epoll.h>
#include <sys/types.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <string.h>

using namespace fibre;

DEFINE_LOG_TOPIC(EVENT_LOOP);
USE_LOG_TOPIC(EVENT_LOOP);


bool EpollEventLoop::start(Callback<void> on_started) {
    if (epoll_fd_ >= 0) {
        FIBRE_LOG(E) << "already started";
        return false;
    }

    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) {
        FIBRE_LOG(E) << "epoll_create1() failed";
        return false;
    }

    bool ok = true;

    post_fd_ = eventfd(0, 0);

    bool post_fd_ok = (post_fd_ >= 0)
            && register_event(post_fd_, EPOLLIN, MEMBER_CB(this, run_callbacks))
            && post(on_started);

    if (!post_fd_ok) {
        FIBRE_LOG(E) << "failed to create an event for posting callbacks onto the event loop";
        ok = false;
    }

    // Run for as long as there are callbacks pending posted or there's at least
    // one file descriptor other than post_fd_ registerd.
    while (pending_callbacks_.size() || (context_map_.size() > 1)) {
        iterations_++;

        do {
            FIBRE_LOG(D) << "epoll_wait...";
            n_triggered_events_ = epoll_wait(epoll_fd_, triggered_events_, max_triggered_events_, -1);
            FIBRE_LOG(D) << "epoll_wait unblocked by " << n_triggered_events_ << " events";
            if (errno == EINTR) {
                FIBRE_LOG(D) << "interrupted";
            }
        } while (n_triggered_events_ < 0 && errno == EINTR); // ignore syscall interruptions. This happens for instance during suspend.

        if (n_triggered_events_ <= 0) {
            FIBRE_LOG(E) << "epoll_wait() failed with " <<  n_triggered_events_ << ": " << sys_err() << " - Terminating worker thread.";
            ok = false;
            break;
        }

        // Handle events
        for (int i = 0; i < n_triggered_events_; ++i) {
            EventContext* ctx = (EventContext*)triggered_events_[i].data.ptr;
            if (ctx) {
                try { // TODO: not sure if using "try" without throwing exceptions will do unwanted things with the stack
                    ctx->callback.invoke(triggered_events_[i].events);
                } catch (...) {
                    FIBRE_LOG(E) << "worker callback threw an exception.";
                }
            }
        }
    }

    FIBRE_LOG(D) << "epoll loop exited";

    if ((post_fd_ >= 0) && !deregister_event(post_fd_)) {
        FIBRE_LOG(E) << "deregister_event() failed";
        ok = false;
    }

    if ((post_fd_ >= 0) && close(post_fd_) != 0) {
        FIBRE_LOG(E) << "close() failed: " << sys_err();
        ok = false;
    }
    post_fd_ = -1;

    if (close(epoll_fd_) != 0) {
        FIBRE_LOG(E) << "close() failed: " << sys_err();
        ok = false;
    }
    epoll_fd_ = -1;

    return ok;
}

bool EpollEventLoop::post(Callback<void> callback) {
    if (epoll_fd_ < 0) {
        FIBRE_LOG(E) << "not started";
        return false;
    }

    {
        std::unique_lock<std::mutex> lock(pending_callbacks_mutex_);
        pending_callbacks_.push_back(callback);
    }

    const uint64_t val = 1;
    if (write(post_fd_, &val, sizeof(val)) != sizeof(val)) {
        FIBRE_LOG(E) << "write() failed" << sys_err();
        return false;
    }
    return true;
}

bool EpollEventLoop::register_event(int event_fd, uint32_t events, Callback<void, uint32_t> callback) {
    if (epoll_fd_ < 0) {
        FIBRE_LOG(E) << "not initialized";
        return false;
    }
    
    if (event_fd < 0) {
        FIBRE_LOG(E) << "invalid argument";
        return false;
    }

    EventContext* ctx = new EventContext{callback};
    struct epoll_event ev = {
        .events = events,
        .data = { .ptr = ctx }
    };
    context_map_[event_fd] = ctx;

    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, event_fd, &ev) != 0) {
        FIBRE_LOG(E) << "epoll_ctl(" << event_fd << "...) failed: " << sys_err();
        delete ctx;
        return false;
    }

    FIBRE_LOG(D) << "registered epoll event " << event_fd;

    return true;
}

bool EpollEventLoop::deregister_event(int event_fd) {
    if (epoll_fd_ < 0) {
        FIBRE_LOG(E) << "not running";
        return false;
    }

    int result = true;

    if (epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, event_fd, nullptr) != 0) {
        FIBRE_LOG(E) << "epoll_ctl() failed: " << sys_err();
        result = false;
    }

    EventContext* callback = context_map_[event_fd];

    auto it = context_map_.find(event_fd);
    if (it == context_map_.end()) {
        FIBRE_LOG(E) << "event context not found";
        return false;
    }

    for (int i = 0; i < n_triggered_events_; ++i) {
        if ((EventContext*)(triggered_events_[i].data.ptr) == it->second) {
            triggered_events_[i].data.ptr = nullptr;
        }
    }

    context_map_.erase(it);
    
    return result;
}

struct EventLoopTimer* EpollEventLoop::call_later(float delay, Callback<void> callback) {
    FIBRE_LOG(E) << "not implemented"; // TODO: implement
    return nullptr;
}

bool EpollEventLoop::cancel_timer(EventLoopTimer* timer) {
    FIBRE_LOG(E) << "not implemented"; // TODO: implement
    return false;
}

void EpollEventLoop::run_callbacks(uint32_t) {
    // TODO: warn if read fails
    uint64_t val;
    if (read(post_fd_, &val, sizeof(val)) != sizeof(val)) {
        FIBRE_LOG(E) << "failed to read from post file descriptor";
    }

    std::vector<Callback<void>> pending_callbacks;

    {
        std::unique_lock<std::mutex> lock(pending_callbacks_mutex_);
        std::swap(pending_callbacks, pending_callbacks_);
    }

    for (auto& cb: pending_callbacks) {
        cb.invoke();
    }
}
