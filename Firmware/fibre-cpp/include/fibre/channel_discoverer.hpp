#ifndef __FIBRE_CHANNEL_DISCOVERER
#define __FIBRE_CHANNEL_DISCOVERER

#include "async_stream.hpp"
#include <fibre/callback.hpp>
#include <fibre/status.hpp>

namespace fibre {

struct ChannelDiscoveryResult {
    Status status;
    AsyncStreamSource* rx_channel;
    AsyncStreamSink* tx_channel;
    size_t mtu;
};

struct ChannelDiscoveryContext {};

class Domain; // defined in fibre.hpp

class ChannelDiscoverer {
public:
    // TODO: maybe we should remove "handle" because a discovery can also be
    // uniquely identified by domain.
    virtual void start_channel_discovery(
        Domain* domain,
        const char* specs, size_t specs_len,
        ChannelDiscoveryContext** handle) = 0;
    virtual int stop_channel_discovery(ChannelDiscoveryContext* handle) = 0;

protected:
    bool try_parse_key(const char* begin, const char* end, const char* key, const char** val_begin, const char** val_end);
    bool try_parse_key(const char* begin, const char* end, const char* key, int* val);
};

}

#endif // __FIBRE_CHANNEL_DISCOVERER