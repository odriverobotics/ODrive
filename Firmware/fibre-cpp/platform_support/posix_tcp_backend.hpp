#ifndef __FIBRE_POSIX_TCP_BACKEND_HPP
#define __FIBRE_POSIX_TCP_BACKEND_HPP

#include <fibre/event_loop.hpp>
#include "posix_socket.hpp"
#include <fibre/channel_discoverer.hpp>
#include <string>
#include <netdb.h>

namespace fibre {

/**
 * TCP client and TCP server implementations are identical up to the function
 * that is used to convert an address to one or more connected socket IDs.
 * The client uses the posix function `connect` to do so, while the server uses
 * the posix functions `listen` and `accept`.
 */
class PosixTcpBackend : public ChannelDiscoverer {
public:
    bool init(EventLoop* event_loop);
    bool deinit();

    void start_channel_discovery(Domain* domain, const char* specs, size_t specs_len, ChannelDiscoveryContext** handle) final;
    int stop_channel_discovery(ChannelDiscoveryContext* handle) final;

private:
    struct TcpChannelDiscoveryContext {
        PosixTcpBackend* parent;
        std::tuple<std::string, int> address;
        Domain* domain;
        AddressResolutionContext* addr_resolution_ctx;
        ConnectionContext* connection_ctx;
        float lookup_period = 1.0f; // wait 1s for next address resolution

        struct AddrContext {
            std::vector<uint8_t> addr;
            ConnectionContext* connection_ctx;
        };

        std::vector<AddrContext> known_addresses;
        void resolve_address();
        void on_found_address(std::optional<cbufptr_t> addr);
        void on_connected(std::optional<socket_id_t> socket_id);
        void on_disconnected();
    };

    virtual bool start_opening_connections(EventLoop* event_loop, cbufptr_t addr, int type, int protocol, ConnectionContext** ctx, Callback<void, std::optional<socket_id_t>> on_connected) = 0;
    virtual void cancel_opening_connections(ConnectionContext* ctx) = 0;

    EventLoop* event_loop_ = nullptr;
    size_t n_discoveries_ = 0;
};

class PosixTcpClientBackend : public PosixTcpBackend {
public:
    constexpr static const char* get_name() { return "tcp-client"; }

    bool start_opening_connections(EventLoop* event_loop, cbufptr_t addr, int type, int protocol, ConnectionContext** ctx, Callback<void, std::optional<socket_id_t>> on_connected) final {
        return start_connecting(event_loop, addr, type, protocol, ctx, on_connected);
    }
    void cancel_opening_connections(ConnectionContext* ctx) final {
        stop_connecting(ctx);
    }
};

class PosixTcpServerBackend : public PosixTcpBackend {
public:
    constexpr static const char* get_name() { return "tcp-server"; }

    bool start_opening_connections(EventLoop* event_loop, cbufptr_t addr, int type, int protocol, ConnectionContext** ctx, Callback<void, std::optional<socket_id_t>> on_connected) final {
        return start_listening(event_loop, addr, type, protocol, ctx, on_connected);
    }
    void cancel_opening_connections(ConnectionContext* ctx) final {
        stop_listening(ctx);
    }
};

}

#endif // __FIBRE_POSIX_TCP_BACKEND_HPP