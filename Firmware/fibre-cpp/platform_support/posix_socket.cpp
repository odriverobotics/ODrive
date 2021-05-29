
#include "posix_socket.hpp"
#include "../logging.hpp"
#include "../print_utils.hpp"

#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netdb.h>
#include <signal.h>
#include <sys/epoll.h>

DEFINE_LOG_TOPIC(SOCKET);
USE_LOG_TOPIC(SOCKET);

#define MAX_CONCURRENT_CONNECTIONS 128

using namespace fibre;

namespace fibre {
/**
 * @brief Tag type to print the last socket error.
 * 
 * This is very similar to sys_err(), except that on Windows it uses
 * WSAGetLastError() instead of `errno` to fetch the last error code.
 */
struct sock_err {
    sock_err() :
#if defined(_WIN32) || defined(_WIN64)
        error_number(WSAGetLastError()) {}
#else
        error_number(errno) {}
#endif

    sock_err(int error_number) : error_number(error_number) {}

    int error_number;
};
}

namespace std {
std::ostream& operator<<(std::ostream& stream, const struct sockaddr_storage& val) {
    char buf[128];

    if ((val.ss_family == AF_INET) && (inet_ntop(val.ss_family, ((struct sockaddr*)&val)->sa_data+2, buf, sizeof(buf)))) {
        return stream << buf;
    } else if ((val.ss_family == AF_INET6) && (inet_ntop(val.ss_family, ((struct sockaddr*)&val)->sa_data+6, buf, sizeof(buf)))) {
        return stream << buf;
    } else {
        return stream << "(invalid address)";
    }
}

std::ostream& operator<<(std::ostream& stream, const fibre::sock_err& err) {
    return stream << strerror(err.error_number) << " (" << err.error_number << ")";
}
}

struct fibre::AddressResolutionContext {
    struct addrinfo hints{};
    std::string address_str;
    std::string port_str;
    EventLoop* event_loop;
    Callback<void, std::optional<cbufptr_t>> callback;
    int cmpl_fd;
    struct gaicb gaicb{};
    struct gaicb* list[1];

    void on_gai_completed();
};

bool fibre::start_resolving_address(EventLoop* event_loop, std::tuple<std::string, int> address, bool passive, AddressResolutionContext** handle, Callback<void, std::optional<cbufptr_t>> callback) {
    // deleted in on_gai_completed()
    AddressResolutionContext* ctx = new AddressResolutionContext();

    ctx->address_str = std::get<0>(address);
    ctx->port_str = std::to_string(std::get<1>(address));
    ctx->event_loop = event_loop;
    ctx->callback = callback;

    ctx->hints = {
        .ai_flags = (passive ? AI_PASSIVE : 0),
        .ai_family = AF_UNSPEC,
        .ai_socktype = 0, // this makes apparently no difference for numerical addresses
    };

    ctx->gaicb = {
        .ar_name = ctx->address_str.c_str(),
        .ar_service = ctx->port_str.c_str(),
        .ar_request = &ctx->hints
    };
    ctx->list[0] = &ctx->gaicb;

    // An extra thread will be created once getaddrinfo_a() completes. This
    // thread will post a callback onto the original event loop to do the actual
    // handling of the result. This is of course exceedingly stupid but it's
    // less bad than throwing around with actual signals that could hit threads
    // that don't expect it.

    struct sigevent sig = {
        .sigev_value = { .sival_ptr = ctx },
        //.sigev_signo = SIGRTMIN,
        .sigev_notify = SIGEV_THREAD,
    };
    sig.sigev_notify_function = [](union sigval sigval) {
        auto ctx = ((AddressResolutionContext*)sigval.sival_ptr);
        ctx->event_loop->post(MEMBER_CB(ctx, on_gai_completed));
    };

    FIBRE_LOG(D) << "starting address resolution for " << ctx->address_str;
    if (getaddrinfo_a(GAI_NOWAIT, ctx->list, 1, &sig) != 0) {
        FIBRE_LOG(E) << "getaddrinfo_a() failed";
        delete ctx;
        return false;
    }

    return true;
}

void fibre::cancel_resolving_address(AddressResolutionContext* handle) {
    gai_cancel(&handle->gaicb);
}

void AddressResolutionContext::on_gai_completed() {
    FIBRE_LOG(D) << "address resolution complete";
    if (gai_error(&gaicb) != 0) {
        FIBRE_LOG(W) << "failed to resolve " << address_str << ": " << sys_err();
    } else {
        // this returns multiple addresses
        for (struct addrinfo* addr = gaicb.ar_result; addr; addr = addr->ai_next) {
            FIBRE_LOG(D) << "resolved IP: " << *(struct sockaddr_storage*)addr->ai_addr;
            cbufptr_t buf = {(const uint8_t*)addr->ai_addr, (size_t)addr->ai_addrlen};
            callback.invoke(buf);
        }
    }
    freeaddrinfo(gaicb.ar_result);
    callback.invoke(std::nullopt); // Announce completion of the request
    delete this;
}

struct fibre::ConnectionContext {
    EventLoop* event_loop;
    socket_id_t socket_id;
    Callback<void, std::optional<socket_id_t>> callback;

    void on_connection_complete(uint32_t mask);
    void on_accept(uint32_t mask);
};

bool fibre::start_connecting(EventLoop* event_loop, cbufptr_t addr, int type, int protocol, ConnectionContext** ctx, Callback<void, std::optional<socket_id_t>> on_connected) {
    auto the_addr = reinterpret_cast<const struct sockaddr*>(addr.begin());

    ConnectionContext* context = new ConnectionContext();
    context->event_loop = event_loop;
    context->socket_id = socket(the_addr->sa_family, type | SOCK_NONBLOCK, protocol);
    context->callback = on_connected;

    if (IS_INVALID_SOCKET(context->socket_id)) {
        FIBRE_LOG(E) << "failed to open socket: " << sock_err();
        goto fail0;
    }
   
    if (connect(context->socket_id, the_addr, addr.size()) == 0) {
        if (errno != EINPROGRESS) {
            FIBRE_LOG(E) << "connect() failed: " << sock_err();
            goto fail1;
        }
    }

    if (!event_loop->register_event(context->socket_id, EPOLLOUT, MEMBER_CB(context, on_connection_complete))) {
            FIBRE_LOG(E) << "failed to register event: " << sock_err();
        goto fail1;
    }

    if (ctx) {
        *ctx = context;
    }

    return true;

fail1:
    close(context->socket_id);
fail0:
    delete context;
    return false;
}

void fibre::stop_connecting(ConnectionContext* ctx) {
    if (!ctx->event_loop->deregister_event(ctx->socket_id)) {
        FIBRE_LOG(W) << "failed to deregister event";
    }
    if (close(ctx->socket_id) != 0) {
        FIBRE_LOG(W) << "failed to close socket";
    }
    ctx->socket_id = INVALID_SOCKET;
    ctx->callback.invoke_and_clear(std::nullopt);
    delete ctx;
}

void fibre::ConnectionContext::on_connection_complete(uint32_t mask) {
    bool failed;
    int error_code;
    socklen_t error_code_size = sizeof(error_code);
    if (getsockopt(socket_id, SOL_SOCKET, SO_ERROR, &error_code, &error_code_size) != 0) {
        FIBRE_LOG(W) << "connection failed (unknown error)";
        failed = true;
    } else if (error_code != 0) {
        FIBRE_LOG(W) << "connection failed: " << sock_err{error_code};
        failed = true;
    } else {
        failed = false;
    }
    
    event_loop->deregister_event(socket_id);
    callback.invoke(failed ? std::nullopt : std::make_optional(socket_id));
    close(socket_id); // The callback must duplicate the socket id if it intends
                      // to keep using it.
    delete this;
}

bool fibre::start_listening(EventLoop* event_loop, cbufptr_t addr, int type, int protocol, ConnectionContext** ctx, Callback<void, std::optional<socket_id_t>> on_connected) {
    auto the_addr = reinterpret_cast<const struct sockaddr*>(addr.begin());
    int flag = 1;

    ConnectionContext* context = new ConnectionContext();
    context->event_loop = event_loop;
    context->socket_id = socket(the_addr->sa_family, type | SOCK_NONBLOCK, protocol);
    context->callback = on_connected;

    if (IS_INVALID_SOCKET(context->socket_id)) {
        FIBRE_LOG(E) << "failed to open socket: " << sock_err();
        goto fail0;
    }

    // Reuse local address.
    // This helps reusing ports that were previously not closed cleanly and
    // are therefore still lingering in the TIME_WAIT state.
    if (setsockopt(context->socket_id, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
        FIBRE_LOG(E) << "failed to make socket reuse addresses: " << sock_err();
        goto fail1;
    }
   
    if (bind(context->socket_id, the_addr, addr.size())) {
        FIBRE_LOG(E) << "failed to bind socket: " << sock_err();
        goto fail1;
    }

    // make this socket a passive socket
    if (listen(context->socket_id, MAX_CONCURRENT_CONNECTIONS) != 0) {
        FIBRE_LOG(E) << "failed to listen on TCP: " << sys_err();
        goto fail1;
    }

    if (!event_loop->register_event(context->socket_id, EPOLLIN, MEMBER_CB(context, on_accept))) {
            FIBRE_LOG(E) << "failed to register event: " << sock_err();
        goto fail1;
    }

    return true;

fail1:
    close(context->socket_id);
fail0:
    delete context;
    return false;
}

void fibre::stop_listening(ConnectionContext* ctx) {
    stop_connecting(ctx); // same implementation
}

void fibre::ConnectionContext::on_accept(uint32_t mask) {
    struct sockaddr_storage remote_addr;
    socklen_t slen = sizeof(remote_addr);

    FIBRE_LOG(D) << "incoming TCP connection";
    int new_socket_id = accept(socket_id, reinterpret_cast<struct sockaddr *>(&remote_addr), &slen);
    if (IS_INVALID_SOCKET(new_socket_id)) {
        FIBRE_LOG(E) << "accept() returned invalid socket: " << sock_err();
        return; // ignore and wait for next incoming connection
    }

    callback.invoke(std::make_optional(new_socket_id));
    close(new_socket_id); // The callback must duplicate the socket id if it intends
                          // to keep using it.
}

bool PosixSocket::init(EventLoop* event_loop, socket_id_t socket_id) {
    if (!IS_INVALID_SOCKET(socket_id_)) {
        FIBRE_LOG(E) << "already initialized";
        return false;
    }

    socket_id = dup(socket_id);
    if (IS_INVALID_SOCKET(socket_id)) {
        FIBRE_LOG(E) << "failed to duplicate socket: " << sock_err();
        return false;
    }

    //if (!event_loop->register_event(socket_id, 0, MEMBER_CB(this, on_event))) {
    //    FIBRE_LOG(E) << "failed to register socket event";
    //    close(socket_id);
    //    return false;
    //}

    event_loop_ = event_loop;
    socket_id_ = socket_id;
    return true;
}

bool PosixSocket::deinit() {
    if (IS_INVALID_SOCKET(socket_id_)) {
        FIBRE_LOG(E) << "not initialized";
        return false;
    }

    bool result = true;
    if (::close(socket_id_)) {
        FIBRE_LOG(E) << "close() failed: " << sock_err();
        result = false;
    }

    socket_id_ = INVALID_SOCKET;
    return result;
}

void PosixSocket::start_read(bufptr_t buffer, TransferHandle* handle, Callback<void, ReadResult> completer) {
    if (rx_callback_) {
        FIBRE_LOG(E) << "RX request already pending";
        completer.invoke({kStreamError});
        return;
    }

    if (handle) {
        *handle = reinterpret_cast<TransferHandle>(this);
    }

    auto result = read_sync(buffer);
    if (result.has_value()) {
        completer.invoke(*result);
    } else {
        rx_buf_ = buffer;
        rx_callback_ = completer;
        update_subscription();
    }
}

void PosixSocket::cancel_read(TransferHandle transfer_handle) {
    if (transfer_handle != reinterpret_cast<TransferHandle>(this)) {
        FIBRE_LOG(E) << "invalid handle";
    } else if (!rx_callback_) {
        FIBRE_LOG(E) << "no RX pending";
    } else {
        rx_callback_.invoke_and_clear({kStreamCancelled, rx_buf_.begin()});
    }
}

void PosixSocket::start_write(cbufptr_t buffer, TransferHandle* handle, Callback<void, WriteResult> completer) {
    if (tx_callback_) {
        FIBRE_LOG(E) << "TX request already pending";
        completer.invoke({kStreamError});
        return;
    }

    if (handle) {
        *handle = reinterpret_cast<TransferHandle>(this);
    }

    auto result = write_sync(buffer);
    if (result.has_value()) {
        completer.invoke(*result);
    } else {
        tx_buf_ = buffer;
        tx_callback_ = completer;
        update_subscription();
    }
}

void PosixSocket::cancel_write(TransferHandle transfer_handle) {
    if (transfer_handle != reinterpret_cast<TransferHandle>(this)) {
        FIBRE_LOG(E) << "invalid handle";
    } else if (!tx_callback_) {
        FIBRE_LOG(E) << "no TX pending";
    } else {
        tx_callback_.invoke_and_clear({kStreamCancelled, tx_buf_.begin()});
    }
}

std::optional<ReadResult> PosixSocket::read_sync(bufptr_t buffer) {
    if (buffer.size() == 0) {
        // Empty buffers mess with our socket-close detection
        FIBRE_LOG(W) << "empty buffer not permitted";
    }

    socklen_t slen = sizeof(remote_addr_);
    ssize_t n_received = recvfrom(socket_id_, buffer.begin(), buffer.size(),
            MSG_DONTWAIT, reinterpret_cast<struct sockaddr *>(&remote_addr_), &slen);
    
    if (n_received < 0) {
        // If recvfrom returns -1 an errno is set to indicate the error.
        auto err = sock_err{};
        if (err.error_number == EAGAIN || err.error_number == EWOULDBLOCK) {
            return std::nullopt;
        } else {
            FIBRE_LOG(E) << "Socket read failed: " << err;
            return {{kStreamError, buffer.end()}}; // the function might have written to the buffer
        }

    } else if ((size_t)n_received > buffer.size()) {
        FIBRE_LOG(E) << "received too many bytes";
        return {{kStreamError, buffer.end()}};

    } else if (n_received == 0) {
        FIBRE_LOG(D) << "socket closed (RX half)";
        return {{kStreamClosed, buffer.begin()}};

    } else {
        FIBRE_LOG(D) << "Received " << n_received << " bytes from " << remote_addr_;
        return {{kStreamOk, buffer.begin() + n_received}};
    }
}

std::optional<WriteResult> PosixSocket::write_sync(cbufptr_t buffer) {
    if (buffer.size() == 0) {
        // Empty buffers mess with our socket-close detection
        FIBRE_LOG(W) << "empty buffer not permitted";
    }

    int n_sent = sendto(socket_id_, buffer.begin(), buffer.size(), MSG_DONTWAIT,
            reinterpret_cast<struct sockaddr*>(&remote_addr_), sizeof(remote_addr_));
    if (n_sent < 0) {
        // If sendto returns -1 an errno is set to indicate the error.
        auto err = sock_err{};
        if (err.error_number == EAGAIN || err.error_number == EWOULDBLOCK) {
            return std::nullopt;
        } else {
            FIBRE_LOG(E) << "Socket write failed: " << err;
            return {{kStreamError, buffer.end()}}; // the function might have written to the buffer
        }

    } else if ((size_t)n_sent > buffer.size()) {
        FIBRE_LOG(E) << "sent too many bytes";
        return {{kStreamError, buffer.end()}};

    } else if (n_sent == 0) {
        FIBRE_LOG(D) << "socket closed (TX half)";
        return {{kStreamClosed, buffer.begin()}};

    } else {
        FIBRE_LOG(D) << "Sent " << n_sent << " bytes to " << remote_addr_;
        return {{kStreamOk, buffer.begin() + n_sent}};
    }
}

void PosixSocket::update_subscription() {
    uint32_t new_mask = (tx_callback_ ? EPOLLOUT : 0)
                      | (rx_callback_ ? EPOLLIN : 0);
    if (new_mask != mask_) {
        if (mask_) {
            event_loop_->deregister_event(socket_id_);
        }
        mask_ = new_mask;
        if (new_mask) {
            event_loop_->register_event(socket_id_, new_mask, MEMBER_CB(this, on_event));
        }
    }
}

void PosixSocket::on_event(uint32_t mask) {

    if (mask & EPOLLIN) {
        // The socket is ready for RX. If an RX request is pending, handle it
        // here, otherwise ignore the event.

        if (rx_callback_) {
            auto result = read_sync(rx_buf_);
            rx_buf_ = {};
            if (result.has_value()) {
                rx_callback_.invoke_and_clear(*result);
            }
        }
    }

    if (mask & EPOLLOUT) {
        // The socket is ready for RX. If an RX request is pending, handle it
        // here, otherwise ignore the event.

        if (tx_callback_) {
            auto result = write_sync(tx_buf_);
            tx_buf_ = {};
            if (result.has_value()) {
                tx_callback_.invoke_and_clear(*result);
            }
        }
    }

    if (mask & ~(EPOLLIN | EPOLLOUT)) {
        FIBRE_LOG(E) << "unknown event mask: " << as_hex(mask);
    }

    update_subscription();
}
