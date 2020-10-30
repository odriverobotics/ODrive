#ifndef __FIBRE_CHANNEL_DISCOVERER
#define __FIBRE_CHANNEL_DISCOVERER

#include "async_stream.hpp"
#include <fibre/libfibre.h>

namespace fibre {

struct ChannelDiscoveryResult {
    FibreStatus status;
    AsyncStreamSource* rx_channel;
    AsyncStreamSink* tx_channel;
    size_t mtu;
};

struct ChannelDiscoveryContext {};

class ChannelDiscoverer {
public:
    virtual void start_channel_discovery(
        const char* specs, size_t specs_len,
        ChannelDiscoveryContext** handle,
        Completer<ChannelDiscoveryResult>& on_found_channels) = 0;
    virtual int stop_channel_discovery(ChannelDiscoveryContext* handle) = 0;
};

}

#endif // __FIBRE_CHANNEL_DISCOVERER