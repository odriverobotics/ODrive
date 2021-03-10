
#include "posix_tcp_backend.hpp"
#include "posix_socket.hpp"
#include "../logging.hpp"
#include <fibre/fibre.hpp>
#include <signal.h>
#include <unistd.h>
#include <algorithm>
#include <string.h>

DEFINE_LOG_TOPIC(TCP);
USE_LOG_TOPIC(TCP);

using namespace fibre;

bool PosixTcpBackend::init(EventLoop* event_loop) {
    if (event_loop_) {
        FIBRE_LOG(E) << "already initialized";
        return false;
    }
    event_loop_ = event_loop;
    return true;
}

bool PosixTcpBackend::deinit() {
    if (!event_loop_) {
        FIBRE_LOG(E) << "not initialized";
        return false;
    }
    if (n_discoveries_) {
        FIBRE_LOG(W) << "some discoveries still ongoing";
    }
    event_loop_ = nullptr;
    return true;
}

void PosixTcpBackend::start_channel_discovery(Domain* domain, const char* specs, size_t specs_len, ChannelDiscoveryContext** handle) {
    const char* address_begin;
    const char* address_end;
    int port;

    if (!event_loop_) {
        FIBRE_LOG(E) << "not initialized";
        //on_found_channels.invoke({kFibreInvalidArgument, nullptr, nullptr, 0});
        return; // TODO: error reporting
    }

    if (!try_parse_key(specs, specs + specs_len, "address", &address_begin, &address_end)) {
        FIBRE_LOG(E) << "no address specified";
        //on_found_channels.invoke({kFibreInvalidArgument, nullptr, nullptr, 0});
        return; // TODO: error reporting
    }

    if (!try_parse_key(specs, specs + specs_len, "port", &port)) {
        FIBRE_LOG(E) << "no port specified";
        //on_found_channels.invoke({kFibreInvalidArgument, nullptr, nullptr, 0});
        return; // TODO: error reporting
    }

    n_discoveries_++;

    TcpChannelDiscoveryContext* ctx = new TcpChannelDiscoveryContext(); // TODO: free
    ctx->parent = this;
    ctx->address = {{address_begin, address_end}, port};
    ctx->domain = domain;
    ctx->resolve_address();
}

int PosixTcpBackend::stop_channel_discovery(ChannelDiscoveryContext* handle) {
    // TODO
    n_discoveries_--;
    return 0;
}

void PosixTcpBackend::TcpChannelDiscoveryContext::resolve_address() {
    if (addr_resolution_ctx) {
        FIBRE_LOG(E) << "already resolving";
        return;
    }

    if (!start_resolving_address(parent->event_loop_, address, false, &addr_resolution_ctx, MEMBER_CB(this, on_found_address))) {
        FIBRE_LOG(E) << "cannot start address resolution";
        return;
    }
}

void PosixTcpBackend::TcpChannelDiscoveryContext::on_found_address(std::optional<cbufptr_t> addr) {
    FIBRE_LOG(D) << "found address";

    if (addr.has_value()) {
        // Resolved an address. If it wasn't already known, try to connect to it.
        std::vector<uint8_t> vec{addr->begin(), addr->end()};
        bool is_known = std::find_if(known_addresses.begin(), known_addresses.end(),
            [&](AddrContext& val){ return val.addr == vec; }) != known_addresses.end();

        if (!is_known) {
            AddrContext ctx = {.addr = vec};
            if (parent->start_opening_connections(parent->event_loop_, *addr, SOCK_STREAM, IPPROTO_TCP, &ctx.connection_ctx, MEMBER_CB(this, on_connected))) {
                known_addresses.push_back(ctx);
            } else {
                // TODO
            }
        }
    } else {
        // No more addresses.
        addr_resolution_ctx = nullptr;
        if (known_addresses.size() == 0) {
            // No addresses could be found. Try again using exponential backoff.
            parent->event_loop_->call_later(lookup_period, MEMBER_CB(this, resolve_address));
            lookup_period = std::min(lookup_period * 3.0f, 3600.0f); // exponential backoff with at most 1h period
        } else {
            // Some addresses are known from this lookup or from a previous
            // lookup. Resolve addresses again in 1h.
            parent->event_loop_->call_later(3600.0, MEMBER_CB(this, resolve_address));
        }
    }
}

void PosixTcpBackend::TcpChannelDiscoveryContext::on_connected(std::optional<socket_id_t> socket_id) {
    if (socket_id.has_value()) {
        auto socket = new PosixSocket{}; // TODO: free
        if (socket->init(parent->event_loop_, *socket_id)) {
            domain->add_channels({kFibreOk, socket, socket, SIZE_MAX});
            return;
        }
        delete socket;
    }

    FIBRE_LOG(D) << "not connected";
    // Try to reconnect soon
    lookup_period = 1.0f;
    resolve_address();
}

void PosixTcpBackend::TcpChannelDiscoveryContext::on_disconnected() {
    lookup_period = 1.0f; // reset exponential backoff
    resolve_address();
}
