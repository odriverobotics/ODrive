
#include <fibre/libfibre.h>
#include "logging.hpp"
#include "print_utils.hpp"
#include "legacy_protocol.hpp"
#include "legacy_object_client.hpp"
#include "event_loop.hpp"
#include "channel_discoverer.hpp"
#include "string.h"
#include <algorithm>
#include "fibre/simple_serdes.hpp"

#ifdef FIBRE_ENABLE_LIBUSB
#include "platform_support/libusb_transport.hpp"
#endif

DEFINE_LOG_TOPIC(LIBFIBRE);
USE_LOG_TOPIC(LIBFIBRE);

static const struct LibFibreVersion libfibre_version = { 0, 1, 0 };

class FIBRE_PRIVATE ExternalEventLoop : public EventLoop {
public:
    ExternalEventLoop(post_cb_t post,
                      register_event_cb_t register_event,
                      deregister_event_cb_t deregister_event,
                      call_later_cb_t call_later,
                      cancel_timer_cb_t cancel_timer) :
        post_(post),
        register_event_(register_event),
        deregister_event_(deregister_event),
        call_later_(call_later),
        cancel_timer_(cancel_timer) {}

    int post(void (*callback)(void*), void *ctx) final {
        return (*post_)(callback, ctx);
    }

    int register_event(int event_fd, uint32_t events, void (*callback)(void*), void* ctx) final {
        return (*register_event_)(event_fd, events, callback, ctx);
    }

    int deregister_event(int event_fd) final {
        return (*deregister_event_)(event_fd);
    }

    struct EventLoopTimer* call_later(float delay, void (*callback)(void*), void *ctx) final {
        return (*call_later_)(delay, callback, ctx);
    }

    int cancel_timer(struct EventLoopTimer* timer) final {
        return (*cancel_timer_)(timer);
    }

private:
    post_cb_t post_;
    register_event_cb_t register_event_;
    deregister_event_cb_t deregister_event_;
    call_later_cb_t call_later_;
    cancel_timer_cb_t cancel_timer_;
};

namespace fibre {

class AsyncStreamLink : public AsyncStreamSink, public AsyncStreamSource {
public:
    void start_write(cbufptr_t buffer, TransferHandle* handle, Completer<WriteResult>& completer) final;
    void cancel_write(TransferHandle transfer_handle) final;
    void start_read(bufptr_t buffer, TransferHandle* handle, Completer<ReadResult>& completer) final;
    void cancel_read(TransferHandle transfer_handle) final;
    void close(StreamStatus status);

    Completer<ReadResult>* read_completer_ = nullptr;
    bufptr_t read_buf_;
    Completer<WriteResult>* write_completer_ = nullptr;
    cbufptr_t write_buf_;
};

void AsyncStreamLink::start_write(cbufptr_t buffer, TransferHandle* handle, Completer<WriteResult>& completer) {
    if (read_completer_) {
        size_t n_copy = std::min(read_buf_.size(), buffer.size());
        memcpy(read_buf_.begin(), buffer.begin(), n_copy);
        safe_complete(read_completer_, {kStreamOk, read_buf_.begin() + n_copy});
        completer.complete({kStreamOk, buffer.begin() + n_copy});
    } else {
        if (handle) {
            *handle = reinterpret_cast<uintptr_t>(this);
        }
        write_buf_ = buffer;
        write_completer_ = &completer;
    }
}

void AsyncStreamLink::cancel_write(TransferHandle transfer_handle) {
    safe_complete(write_completer_, {kStreamCancelled, write_buf_.begin()});
}

void AsyncStreamLink::start_read(bufptr_t buffer, TransferHandle* handle, Completer<ReadResult>& completer) {
    if (write_completer_) {
        FIBRE_LOG(W) << "start_read: completing writer";
        size_t n_copy = std::min(buffer.size(), write_buf_.size());
        memcpy(buffer.begin(), write_buf_.begin(), n_copy);
        safe_complete(write_completer_, {kStreamOk, write_buf_.begin() + n_copy});
        completer.complete({kStreamOk, buffer.begin() + n_copy});
    } else {
        //FIBRE_LOG(W) << "start_read: waiting for writer";
        if (handle) {
            *handle = reinterpret_cast<uintptr_t>(this);
        }
        read_buf_ = buffer;
        read_completer_ = &completer;
    }
}

void AsyncStreamLink::cancel_read(TransferHandle transfer_handle) {
    safe_complete(read_completer_, {kStreamCancelled, read_buf_.begin()});
}

void AsyncStreamLink::close(StreamStatus status) {
    safe_complete(write_completer_, {status, write_buf_.begin()});
    safe_complete(read_completer_, {status, read_buf_.begin()});
}

}

FibreStatus convert_status(fibre::StreamStatus status) {
    switch (status) {
        case fibre::kStreamOk: return kFibreOk;
        case fibre::kStreamCancelled: return kFibreCancelled;
        case fibre::kStreamClosed: return kFibreClosed;
        default: return kFibreInternalError; // TODO: this may not always be appropriate
    }
}

fibre::StreamStatus convert_status(FibreStatus status) {
    switch (status) {
        case kFibreOk: return fibre::kStreamOk;
        case kFibreCancelled: return fibre::kStreamCancelled;
        case kFibreClosed: return fibre::kStreamClosed;
        default: return fibre::kStreamError; // TODO: this may not always be appropriate
    }
}

struct FIBRE_PRIVATE LibFibreCtx {
    ExternalEventLoop* event_loop;
    construct_object_cb_t on_construct_object;
    destroy_object_cb_t on_destroy_object;
    void* cb_ctx;
    size_t n_discoveries = 0;

    std::unordered_map<std::string, std::shared_ptr<fibre::ChannelDiscoverer>> discoverers;
};

struct FIBRE_PRIVATE LibFibreDiscoveryCtx :
        fibre::Completer<fibre::ChannelDiscoveryResult>,
        fibre::Completer<fibre::LegacyObjectClient*, std::shared_ptr<fibre::LegacyObject>>,
        fibre::Completer<fibre::LegacyObjectClient*>,
        fibre::Completer<fibre::LegacyProtocolPacketBased*, fibre::StreamStatus>
{
    void complete(fibre::ChannelDiscoveryResult result) final;
    void complete(fibre::LegacyObjectClient* obj_client, std::shared_ptr<fibre::LegacyObject> intf) final;
    void complete(fibre::LegacyObjectClient* obj_client) final;
    void complete(fibre::LegacyProtocolPacketBased* protocol, fibre::StreamStatus status) final;

    std::unordered_map<std::string, fibre::ChannelDiscoveryContext*> context_handles;

    on_found_object_cb_t on_found_object;
    void* cb_ctx;
    LibFibreCtx* ctx;

    // A LibFibreDiscoveryCtx is created when the application starts discovery
    // and is deleted when the application stopped discovery _and_ all protocol
    // instances that arose from this discovery instance were also stopped.
    size_t use_count = 1;
};

struct LibFibreTxStream : fibre::Completer<fibre::WriteResult> {
    void complete(fibre::WriteResult result) {
        if (on_completed) {
            (*on_completed)(ctx, this, convert_status(result.status), result.end);
        }
    }

    fibre::AsyncStreamSink* sink;
    fibre::TransferHandle handle;
    on_tx_completed_cb_t on_completed;
    void* ctx;
    void (*on_closed)(LibFibreTxStream*, void*, fibre::StreamStatus);
    void* on_closed_ctx;
};

struct LibFibreRxStream : fibre::Completer<fibre::ReadResult> {
    void complete(fibre::ReadResult result) {
        if (on_completed) {
            (*on_completed)(ctx, this, convert_status(result.status), result.end);
        }
    }

    fibre::AsyncStreamSource* source;
    fibre::TransferHandle handle;
    on_rx_completed_cb_t on_completed;
    void* ctx;
    void (*on_closed)(LibFibreRxStream*, void*, fibre::StreamStatus);
    void* on_closed_ctx;
};


// Callback for start_channel_discovery()
void LibFibreDiscoveryCtx::complete(fibre::ChannelDiscoveryResult result) {
    FIBRE_LOG(D) << "found channels!";

    if (result.status != kFibreOk) {
        FIBRE_LOG(W) << "discoverer stopped";
        return;
    }

    if (!result.rx_channel || !result.tx_channel) {
        FIBRE_LOG(W) << "unidirectional operation not supported yet";
        return;
    }

    use_count++;

    auto protocol = new fibre::LegacyProtocolPacketBased(result.rx_channel, result.tx_channel, result.mtu);
    protocol->client_.user_data_ = ctx;
    protocol->start(*this, *this, *this);
}

// on_found_root_object callback for LegacyProtocolPacketBased::start()
void LibFibreDiscoveryCtx::complete(fibre::LegacyObjectClient* obj_client, std::shared_ptr<fibre::LegacyObject> obj) {
    auto obj_cast = reinterpret_cast<LibFibreObject*>(obj.get()); // corresponding reverse cast in libfibre_get_attribute()
    auto intf_cast = reinterpret_cast<LibFibreInterface*>(obj->intf.get()); // corresponding reverse cast in libfibre_subscribe_to_interface()

    for (auto& obj: obj_client->objects_) {
        // If the callback handler calls libfibre_get_attribute() before
        // all objects were announced to the application then it's possible
        // that during that function call some objects are already announced
        // on-demand.
        if (!obj->known_to_application) {
            obj->known_to_application = true;
            //FIBRE_LOG(D) << "constructing root object " << fibre::as_hex(reinterpret_cast<uintptr_t>(obj.get()));
            if (ctx->on_construct_object) {
                (*ctx->on_construct_object)(ctx->cb_ctx,
                    reinterpret_cast<LibFibreObject*>(obj.get()),
                    reinterpret_cast<LibFibreInterface*>(obj->intf.get()),
                    obj->intf->name.size() ? obj->intf->name.data() : nullptr, obj->intf->name.size());
            }
        }
    }

    if (on_found_object) {
        FIBRE_LOG(D) << "announcing root object " << fibre::as_hex(reinterpret_cast<uintptr_t>(obj_cast));
        (*on_found_object)(cb_ctx, obj_cast);
    }
}

// on_lost_root_object for LegacyProtocolPacketBased::start()
void LibFibreDiscoveryCtx::complete(fibre::LegacyObjectClient* obj_client) {
    if (ctx->on_destroy_object) {
        for (auto obj: obj_client->objects_) {
            auto obj_cast = reinterpret_cast<LibFibreObject*>(obj.get());
            //FIBRE_LOG(D) << "destroying subobject " << fibre::as_hex(reinterpret_cast<uintptr_t>(obj_cast));
            (*ctx->on_destroy_object)(ctx->cb_ctx, obj_cast);
        }

        obj_client->objects_.clear();
    }
}

// on_stopped callback for LegacyProtocolPacketBased::start()
void LibFibreDiscoveryCtx::complete(fibre::LegacyProtocolPacketBased* protocol, fibre::StreamStatus status) {
    delete protocol;

    if (--use_count == 0) {
        FIBRE_LOG(D) << "deleting discovery context";
        delete this;
    }
}

const struct LibFibreVersion* libfibre_get_version() {
    return &libfibre_version;
}

LibFibreCtx* libfibre_open(
        post_cb_t post,
        register_event_cb_t register_event,
        deregister_event_cb_t deregister_event,
        call_later_cb_t call_later,
        cancel_timer_cb_t cancel_timer,
        construct_object_cb_t construct_object,
        destroy_object_cb_t destroy_object,
        void* cb_ctx)
{
    //if (!register_event || !deregister_event) {
    //    FIBRE_LOG(E) << "invalid argument";
    //    return nullptr;
    //}
    FIBRE_LOG(D) << "object constructor: " << reinterpret_cast<uintptr_t>(construct_object);
    LibFibreCtx* ctx = new LibFibreCtx();
    ctx->event_loop = new ExternalEventLoop(post, register_event, deregister_event, call_later, cancel_timer);
    ctx->on_construct_object = construct_object;
    ctx->on_destroy_object = destroy_object;
    ctx->cb_ctx = cb_ctx;

#ifdef FIBRE_ENABLE_LIBUSB
    auto libusb_discoverer = std::make_shared<fibre::LibusbDiscoverer>();
    if (libusb_discoverer->init(ctx->event_loop) != 0) {
        delete ctx;
        FIBRE_LOG(E) << "failed to init libusb transport layer";
        return nullptr;
    }
    ctx->discoverers["usb"] = libusb_discoverer;
#endif

    FIBRE_LOG(D) << "opened (" << fibre::as_hex((uintptr_t)ctx) << ")";
    return ctx;
}

void libfibre_close(LibFibreCtx* ctx) {
    if (ctx->n_discoveries) {
        FIBRE_LOG(W) << "there are still discovery processes ongoing";
    }

    ctx->discoverers.clear();

    delete ctx->event_loop;
    delete ctx;

    FIBRE_LOG(D) << "closed (" << fibre::as_hex((uintptr_t)ctx) << ")";
}

struct LibFibreChannelDiscoveryCtx : fibre::ChannelDiscoveryContext {
    fibre::Completer<fibre::ChannelDiscoveryResult>* completer;
};

class ExternalDiscoverer : public fibre::ChannelDiscoverer {
    void start_channel_discovery(
        const char* specs, size_t specs_len,
        fibre::ChannelDiscoveryContext** handle,
        fibre::Completer<fibre::ChannelDiscoveryResult>& on_found_channels) final;
    int stop_channel_discovery(fibre::ChannelDiscoveryContext* handle) final;
public:
    on_start_discovery_cb_t on_start_discovery;
    on_stop_discovery_cb_t on_stop_discovery;
    void* cb_ctx;
};

void ExternalDiscoverer::start_channel_discovery(const char* specs, size_t specs_len, fibre::ChannelDiscoveryContext** handle, fibre::Completer<fibre::ChannelDiscoveryResult>& on_found_channels) {
    LibFibreChannelDiscoveryCtx* ctx = new LibFibreChannelDiscoveryCtx{};
    ctx->completer = &on_found_channels;
    if (handle) {
        *handle = ctx;
    }
    if (on_start_discovery) {
        (*on_start_discovery)(cb_ctx, ctx, specs, specs_len);
    }
}

int ExternalDiscoverer::stop_channel_discovery(fibre::ChannelDiscoveryContext* handle) {
    LibFibreChannelDiscoveryCtx* ctx = static_cast<LibFibreChannelDiscoveryCtx*>(handle);
    if (on_stop_discovery) {
        (*on_stop_discovery)(cb_ctx, ctx);
    }
    delete ctx;
    return 0;
}

void libfibre_register_discoverer(LibFibreCtx* ctx, const char* name, size_t name_length, on_start_discovery_cb_t on_start_discovery, on_stop_discovery_cb_t on_stop_discovery, void* cb_ctx) {
    std::string name_str = {name, name + name_length};
    if (ctx->discoverers.find(name_str) != ctx->discoverers.end()) {
        FIBRE_LOG(W) << "Discoverer " << name_str << " already registered";
        return; // TODO: report status
    }

    auto disc = std::make_shared<ExternalDiscoverer>();
    disc->on_start_discovery = on_start_discovery;
    disc->on_stop_discovery = on_stop_discovery;
    disc->cb_ctx = cb_ctx;
    ctx->discoverers[name_str] = disc;
}

void libfibre_add_channels(LibFibreCtx* ctx, LibFibreChannelDiscoveryCtx* discovery_ctx, LibFibreRxStream** tx_channel, LibFibreTxStream** rx_channel, size_t mtu) {
    fibre::AsyncStreamLink* tx_link = new fibre::AsyncStreamLink();
    fibre::AsyncStreamLink* rx_link = new fibre::AsyncStreamLink();
    LibFibreRxStream* tx = new LibFibreRxStream();
    LibFibreTxStream* rx = new LibFibreTxStream();
    tx->source = tx_link;
    rx->sink = rx_link;

    tx->on_closed = [](LibFibreRxStream* stream, void* ctx, fibre::StreamStatus status) {
        auto link = reinterpret_cast<fibre::AsyncStreamLink*>(ctx);
        link->close(status);
        delete link;
        delete stream;
    };
    tx->on_closed_ctx = tx_link;
    rx->on_closed = [](LibFibreTxStream* stream, void* ctx, fibre::StreamStatus status) {
        auto link = reinterpret_cast<fibre::AsyncStreamLink*>(ctx);
        link->close(status);
        delete link;
        delete stream;
    };
    rx->on_closed_ctx = rx_link;

    if (tx_channel) {
        *tx_channel = tx;
    }

    if (rx_channel) {
        *rx_channel = rx;
    }

    fibre::ChannelDiscoveryResult result = {kFibreOk, rx_link, tx_link, mtu};
    discovery_ctx->completer->complete(result);
}

void libfibre_start_discovery(LibFibreCtx* ctx, const char* specs, size_t specs_len, struct LibFibreDiscoveryCtx** handle,
        on_found_object_cb_t on_found_object, on_stopped_cb_t on_stopped, void* cb_ctx) {
    if (!ctx) {
        FIBRE_LOG(E) << "invalid argument";
        if (on_stopped) {
            (*on_stopped)(cb_ctx, kFibreInvalidArgument);
        }
        return;
    }

    const char* prev_delim = specs;

    FIBRE_LOG(D) << "starting discovery with path \"" << std::string(specs, specs_len) << "\"";

    LibFibreDiscoveryCtx* discovery_ctx = new LibFibreDiscoveryCtx();
    discovery_ctx->on_found_object = on_found_object;
    discovery_ctx->cb_ctx = cb_ctx;
    discovery_ctx->ctx = ctx;

    if (handle) {
        *handle = discovery_ctx;
    }

    while (prev_delim < specs + specs_len) {
        const char* next_delim = std::find(prev_delim, specs + specs_len, ';');
        const char* colon = std::find(prev_delim, next_delim, ':');
        const char* colon_end = std::min(colon + 1, next_delim);

        std::string name{prev_delim, colon};
        auto it = ctx->discoverers.find(name);

        if (it == ctx->discoverers.end()) {
            FIBRE_LOG(W) << "transport layer \"" << name << "\" not implemented";
        } else {
            discovery_ctx->context_handles[name] = nullptr;
            it->second->start_channel_discovery(colon_end, next_delim - colon_end,
                    &discovery_ctx->context_handles[name], *discovery_ctx);
        }

        prev_delim = std::min(next_delim + 1, specs + specs_len);
    }

    ctx->n_discoveries++;
}

void libfibre_stop_discovery(LibFibreCtx* ctx, LibFibreDiscoveryCtx* discovery_ctx) {
    if (!ctx->n_discoveries) {
        FIBRE_LOG(W) << "stopping a discovery process but none is active";
    } else {
        ctx->n_discoveries--;
    }

    for (auto& it: discovery_ctx->context_handles) {
        ctx->discoverers[it.first]->stop_channel_discovery(it.second);
    }
    discovery_ctx->context_handles.clear();

    if (--discovery_ctx->use_count == 0) {
        FIBRE_LOG(D) << "deleting discovery context";
        delete discovery_ctx;
    }
}

const char* transform_codec(std::string& codec) {
    if (codec == "endpoint_ref") {
        return "object_ref";
    } else {
        return codec.data();
    }
}

void libfibre_subscribe_to_interface(LibFibreInterface* interface,
        on_attribute_added_cb_t on_attribute_added,
        on_attribute_removed_cb_t on_attribute_removed,
        on_function_added_cb_t on_function_added,
        on_function_removed_cb_t on_function_removed,
        void* cb_ctx)
{
    auto intf = reinterpret_cast<fibre::FibreInterface*>(interface); // corresponding reverse cast in LibFibreDiscoveryCtx::complete() and libfibre_subscribe_to_interface()

    for (auto& func: intf->functions) {
        std::vector<const char*> input_names;
        std::vector<const char*> input_codecs;
        std::vector<const char*> output_names;
        std::vector<const char*> output_codecs;
        for (auto& arg: func.second.inputs) {
            input_names.push_back(arg.name.data());
            input_codecs.push_back(transform_codec(arg.codec));
        }
        for (auto& arg: func.second.outputs) {
            output_names.push_back(arg.name.data());
            output_codecs.push_back(transform_codec(arg.codec));
        }
        input_names.push_back(nullptr);
        input_codecs.push_back(nullptr);
        output_names.push_back(nullptr);
        output_codecs.push_back(nullptr);

        if (on_function_added) {
            (*on_function_added)(cb_ctx,
                reinterpret_cast<LibFibreFunction*>(&func.second), // corresponding reverse cast in libfibre_start_call()
                func.first.data(), func.first.size(),
                input_names.data(), input_codecs.data(),
                output_names.data(), output_codecs.data());
        }
    }

    for (auto& attr: intf->attributes) {
        if (on_attribute_added) {
            (*on_attribute_added)(cb_ctx,
                reinterpret_cast<LibFibreAttribute*>(&attr.second), // corresponding reverse cast in libfibre_get_attribute()
                attr.first.data(), attr.first.size(),
                reinterpret_cast<LibFibreInterface*>(attr.second.object->intf.get()), // corresponding reverse cast in libfibre_subscribe_to_interface()
                attr.second.object->intf->name.size() ? attr.second.object->intf->name.data() : nullptr, attr.second.object->intf->name.size()
            );
        }
    }
}

FibreStatus libfibre_get_attribute(LibFibreObject* parent_obj, LibFibreAttribute* attr, LibFibreObject** child_obj_ptr) {
    if (!parent_obj || !attr) {
        return kFibreInvalidArgument;
    }

    fibre::LegacyObject* parent_obj_cast = reinterpret_cast<fibre::LegacyObject*>(parent_obj);
    fibre::LegacyFibreAttribute* attr_cast = reinterpret_cast<fibre::LegacyFibreAttribute*>(attr); // corresponding reverse cast in libfibre_subscribe_to_interface()
    auto& attributes = parent_obj_cast->intf->attributes;

    bool is_member = std::find_if(attributes.begin(), attributes.end(),
        [&](std::pair<const std::string, fibre::LegacyFibreAttribute>& kv) {
            return &kv.second == attr_cast;
        }) != attributes.end();

    if (!is_member) {
        FIBRE_LOG(W) << "attempt to fetch attribute from an object that does not implement it";
        return kFibreInvalidArgument;
    }

    LibFibreCtx* libfibre_ctx = reinterpret_cast<LibFibreCtx*>(parent_obj_cast->client->user_data_);
    fibre::LegacyObject* child_obj = attr_cast->object.get();

    if (!attr_cast->object->known_to_application) {
        attr_cast->object->known_to_application = true;

        if (libfibre_ctx->on_construct_object) {
            //FIBRE_LOG(D) << "constructing subobject " << fibre::as_hex(reinterpret_cast<uintptr_t>(child_obj));
            (*libfibre_ctx->on_construct_object)(libfibre_ctx->cb_ctx,
                reinterpret_cast<LibFibreObject*>(child_obj),
                reinterpret_cast<LibFibreInterface*>(child_obj->intf.get()),
                child_obj->intf->name.size() ? child_obj->intf->name.data() : nullptr, child_obj->intf->name.size());
        }
    }

    if (child_obj_ptr) {
        *child_obj_ptr = reinterpret_cast<LibFibreObject*>(child_obj);
    }

    return kFibreOk;
}

/**
 * @brief Inserts or removes the specified number of elements
 * @param delta: Positive value: insert elements, negative value: remove elements
 */
void resize_at(std::vector<uint8_t>& vec, size_t pos, ssize_t delta) {
    if (delta > 0) {
        std::fill_n(std::inserter(vec, vec.begin() + pos), delta, 0);
    } else {
        vec.erase(std::min(vec.begin() + pos, vec.end()),
                  std::min(vec.begin() + pos + -delta, vec.end()));
    }
}

class ArgEncoder : public fibre::AsyncStreamSink, fibre::Completer<fibre::WriteResult> {
    void start_write(fibre::cbufptr_t buffer, fibre::TransferHandle* handle, Completer<fibre::WriteResult>& completer) final;
    void cancel_write(fibre::TransferHandle transfer_handle) final;
    void complete(fibre::WriteResult result) final;

public:
    LibFibreCallContext* call_ = nullptr;
    fibre::AsyncStreamSink* encoded_stream_ = nullptr;
    fibre::cbufptr_t decoded_buf_; // application-owned buffer
    std::vector<uint8_t> encoded_buf_; // libfibre-owned buffer used after transcoding from application buffer
    size_t encoded_offset_ = 0; // offset in the TX stream after transcoding from application-facing format
    fibre::TransferHandle transfer_handle_ = 0;
    fibre::Completer<fibre::WriteResult>* completer_ = nullptr;
};

class ArgDecoder : public fibre::AsyncStreamSource, fibre::Completer<fibre::ReadResult> {
    void start_read(fibre::bufptr_t buffer, fibre::TransferHandle* handle, Completer<fibre::ReadResult>& completer) final;
    void cancel_read(fibre::TransferHandle transfer_handle) final;
    void complete(fibre::ReadResult result) final;

public:
    LibFibreCallContext* call_ = nullptr;
    fibre::AsyncStreamSource* encoded_stream_ = nullptr;
    fibre::bufptr_t decoded_buf_; // application-owned buffer
    std::vector<uint8_t> encoded_buf_; // libfibre-owned buffer used before transcoding to application buffer
    size_t encoded_offset_ = 0; // offset in the RX stream before transcoding to application-facing format
    fibre::TransferHandle transfer_handle_ = 0;
    fibre::Completer<fibre::ReadResult>* completer_ = nullptr;
};

struct FIBRE_PRIVATE LibFibreCallContext : fibre::Completer<FibreStatus> {
    void complete(FibreStatus status) final;

    template<typename Func>
    bool iterate_over_args_at(size_t encoded_offset, size_t max_encoded_length, size_t max_decoded_length, Func visitor);

    uint8_t n_active_transfers = 0;
    fibre::LegacyObject* obj = nullptr;
    fibre::LegacyFibreFunction* func = nullptr;
    on_call_completed_cb_t on_call_completed_ = nullptr;
    void* call_cb_ctx_ = nullptr;
    fibre::LegacyObjectClient::CallContext* handle_ = nullptr;
    LibFibreTxStream tx_stream_;
    LibFibreRxStream rx_stream_;
    ArgDecoder decoder_;
    ArgEncoder encoder_;
};

void libfibre_start_call(LibFibreObject* obj, LibFibreFunction* func,
                         LibFibreCallContext** handle,
                         LibFibreTxStream** tx_stream,
                         LibFibreRxStream** rx_stream,
                         on_call_completed_cb_t on_completed, void* cb_ctx) {
    if (!obj || !func) {
        if (on_completed) {
            (*on_completed)(cb_ctx, kFibreInvalidArgument);
        }
        return;
    }

    fibre::LegacyObject* obj_cast = reinterpret_cast<fibre::LegacyObject*>(obj);
    fibre::LegacyFibreFunction* func_cast = reinterpret_cast<fibre::LegacyFibreFunction*>(func);

    bool is_member = std::find_if(obj_cast->intf->functions.begin(), obj_cast->intf->functions.end(),
        [&](std::pair<const std::string, fibre::LegacyFibreFunction>& kv) {
            return &kv.second == func_cast;
        }) != obj_cast->intf->functions.end();

    if (!is_member) {
        FIBRE_LOG(W) << "attempt to invoke function on an object that does not implement it";
        if (on_completed) {
            (*on_completed)(cb_ctx, kFibreInvalidArgument);
        }
        return;
    }


    auto completer = new LibFibreCallContext();
    completer->obj = obj_cast;
    completer->func = func_cast;
    completer->on_call_completed_ = on_completed;
    completer->call_cb_ctx_ = cb_ctx;
    completer->encoder_.call_ = completer;
    completer->decoder_.call_ = completer;
    completer->tx_stream_.sink = &completer->encoder_;
    completer->rx_stream_.source = &completer->decoder_;

    if (handle) {
        *handle = completer;
    }
    if (tx_stream) {
        *tx_stream = &completer->tx_stream_;
    }
    if (rx_stream) {
        *rx_stream = &completer->rx_stream_;
    }

    obj_cast->client->start_call(obj_cast->ep_num, func_cast,
        &completer->handle_, *completer);
        
    completer->encoder_.encoded_stream_ = completer->handle_;
    completer->decoder_.encoded_stream_ = completer->handle_;
}

void libfibre_end_call(LibFibreCallContext* handle) {
    if (handle) {
        handle->obj->client->cancel_call(handle->handle_);
    }
}

void LibFibreCallContext::complete(FibreStatus status) {
    if (on_call_completed_) {
        (*on_call_completed_)(call_cb_ctx_, status);
    }
    delete this;
}

bool encode_for_transport(fibre::LegacyObjectClient* client, fibre::cbufptr_t src, fibre::bufptr_t dst, const fibre::LegacyFibreArg& arg) {
    if (arg.codec == "endpoint_ref") {
        if (src.size() < sizeof(uintptr_t) || dst.size() < 4) {
            return false;
        }

        uintptr_t val = *reinterpret_cast<const uintptr_t*>(src.begin());
        auto obj = reinterpret_cast<fibre::LegacyObject*>(val);
        write_le<uint16_t>(obj ? obj->ep_num : 0, &dst);
        write_le<uint16_t>(obj ? obj->client->json_crc_ : 0, &dst);
    } else {
        if (src.size() < arg.size || dst.size() < arg.size) {
            return false;
        }

        memcpy(dst.begin(), src.begin(), arg.size);
    }

    return true;
}

bool decode_from_transport(fibre::LegacyObjectClient* client, fibre::cbufptr_t src, fibre::bufptr_t dst, const fibre::LegacyFibreArg& arg) {
    if (arg.codec == "endpoint_ref") {
        if (src.size() < 4 || dst.size() < sizeof(uintptr_t)) {
            return false;
        }

        uint16_t ep_num = *read_le<uint16_t>(&src);
        uint16_t json_crc = *read_le<uint16_t>(&src);

        fibre::LegacyObject* obj_ptr = nullptr;

        if (ep_num && json_crc == client->json_crc_) {
            for (auto& known_obj: client->objects_) {
                if (known_obj->ep_num == ep_num) {
                    obj_ptr = known_obj.get();
                }
            }
        }

        FIBRE_LOG(D) << "placing transcoded ptr " << reinterpret_cast<uintptr_t>(obj_ptr);
        *reinterpret_cast<uintptr_t*>(dst.begin()) = reinterpret_cast<uintptr_t>(obj_ptr);

    } else {
        if (src.size() < arg.size || dst.size() < arg.size) {
            return false;
        }

        memcpy(dst.begin(), src.begin(), arg.size);
    }

    return true;
}

/**
 * @brief func: A functor that takes these arguments:
 *              - size_t rel_encoded_offset (relative to the starting pos described by encoded_offset)
 *              - size_t rel_decoded_offset (relative to the starting pos described by encoded_offset)
 *              - size_t encoded_length
 *              - size_t decoded_length
 */
template<typename Func>
bool LibFibreCallContext::iterate_over_args_at(size_t encoded_offset, size_t max_encoded_length, size_t max_decoded_length, Func visitor) {
    ssize_t arg_offset = 0;
    ssize_t len_diff = 0;

    for (auto& arg: func->outputs) {
        if (arg.size >= SIZE_MAX || arg_offset + arg.size > encoded_offset) {
            ssize_t rel_encoded_offset = arg_offset - (ssize_t)encoded_offset;
            ssize_t rel_decoded_offset = arg_offset - (ssize_t)encoded_offset - len_diff;

            if (rel_decoded_offset >= max_decoded_length || rel_encoded_offset >= max_encoded_length) {
                break;
            }

            size_t encoded_arg_size = arg.size;
            size_t decoded_arg_size = arg.codec == "endpoint_ref" ? sizeof(uintptr_t) : arg.size;
            len_diff += encoded_arg_size - decoded_arg_size;

            if (rel_encoded_offset < 0) {
                encoded_arg_size += rel_encoded_offset;
            }

            if (rel_decoded_offset < 0) {
                decoded_arg_size += rel_decoded_offset;
            }

            if (encoded_arg_size < SIZE_MAX && rel_encoded_offset + encoded_arg_size > max_encoded_length) {
                encoded_arg_size -= rel_encoded_offset + encoded_arg_size - max_encoded_length;
            }

            if (decoded_arg_size < SIZE_MAX && rel_decoded_offset + decoded_arg_size > max_decoded_length) {
                decoded_arg_size -= rel_decoded_offset + decoded_arg_size - max_decoded_length;
            }

            if (!visitor(arg, rel_encoded_offset, rel_decoded_offset, encoded_arg_size, decoded_arg_size)) {
                return false;
            }
        }

        arg_offset += arg.size;
    }

    return true;
}

void libfibre_start_tx(LibFibreTxStream* tx_stream,
        const uint8_t* tx_buf, size_t tx_len, on_tx_completed_cb_t on_completed,
        void* ctx) {
    tx_stream->on_completed = on_completed;
    tx_stream->ctx = ctx;
    tx_stream->sink->start_write({tx_buf, tx_len}, &tx_stream->handle, *tx_stream);
}

void libfibre_cancel_tx(LibFibreTxStream* tx_stream) {
    tx_stream->sink->cancel_write(tx_stream->handle);
}

void libfibre_close_tx(LibFibreTxStream* tx_stream, FibreStatus status) {
    if (tx_stream->on_closed) {
        (tx_stream->on_closed)(tx_stream, tx_stream->on_closed_ctx, convert_status(status));
    }
}

void libfibre_start_rx(LibFibreRxStream* rx_stream,
        uint8_t* rx_buf, size_t rx_len, on_rx_completed_cb_t on_completed,
        void* ctx) {
    rx_stream->on_completed = on_completed;
    rx_stream->ctx = ctx;
    rx_stream->source->start_read({rx_buf, rx_len}, &rx_stream->handle, *rx_stream);
}

void libfibre_cancel_rx(LibFibreRxStream* rx_stream) {
    rx_stream->source->cancel_read(rx_stream->handle);
}

void libfibre_close_rx(LibFibreRxStream* rx_stream, FibreStatus status) {
    if (rx_stream->on_closed) {
        (rx_stream->on_closed)(rx_stream, rx_stream->on_closed_ctx, convert_status(status));
    }
}

void ArgEncoder::start_write(fibre::cbufptr_t buffer, fibre::TransferHandle* handle, Completer<fibre::WriteResult>& completer) {
    // Allocate libfibre-internal TX buffer into which the application buffer
    // will be encoded. The size can still change during transcoding.
    encoded_buf_ = std::vector<uint8_t>{};
    encoded_buf_.reserve(buffer.size());
    
    // Transcode application buffer to stream buffer
    bool ok = call_->iterate_over_args_at(encoded_offset_, SIZE_MAX, buffer.size(), [&](
            const fibre::LegacyFibreArg& arg,
            ssize_t rel_encoded_offset, ssize_t rel_decoded_offset,
            size_t encoded_arg_size, size_t decoded_arg_size) {
        encoded_buf_.resize(rel_encoded_offset + encoded_arg_size);
        fibre::bufptr_t encoded_buf = {encoded_buf_.data() + rel_encoded_offset, encoded_arg_size};
        fibre::cbufptr_t decoded_buf = {buffer.begin() + rel_decoded_offset, decoded_arg_size};
        return encode_for_transport(call_->obj->client, decoded_buf, encoded_buf, arg);
    });

    if (!ok) {
        FIBRE_LOG(W) << "Transcoding before TX failed. Note that partial transcoding of arguments is not supported.";
        completer.complete({fibre::kStreamError, buffer.begin()});
        return;
    }

    decoded_buf_ = buffer;
    call_->n_active_transfers++;
    completer_ = &completer;
    
    encoded_stream_->start_write(encoded_buf_, &transfer_handle_, *this);
}

void ArgEncoder::cancel_write(fibre::TransferHandle transfer_handle) {
    encoded_stream_->cancel_write(transfer_handle_);
}

void ArgEncoder::complete(fibre::WriteResult result) {
    size_t n_sent = (result.end - encoded_buf_.data());
    FIBRE_LOG(D) << "sent " << n_sent << " bytes ";

    if (n_sent > encoded_buf_.size()) {
        FIBRE_LOG(E) << "internal error: sent more bytes than expected";
    }

    ssize_t len_diff = encoded_buf_.size() - decoded_buf_.size();
    const uint8_t* tx_end = decoded_buf_.begin() + n_sent - len_diff;

    decoded_buf_ = {};
    encoded_buf_ = {};
    encoded_offset_ += n_sent;
    call_->n_active_transfers--;

    safe_complete(completer_, {result.status, tx_end});
}

void ArgDecoder::start_read(fibre::bufptr_t buffer, fibre::TransferHandle* handle, Completer<fibre::ReadResult>& completer) {
    size_t encoded_size = 0;

    bool ok = call_->iterate_over_args_at(encoded_offset_, SIZE_MAX, buffer.size(), [&](
            const fibre::LegacyFibreArg& arg,
            ssize_t rel_encoded_offset, ssize_t rel_decoded_offset,
            size_t encoded_arg_size, size_t decoded_arg_size) {
        encoded_size += encoded_arg_size;
        return true;
    });

    if (!ok) {
        FIBRE_LOG(W) << "Transcoding preparation before RX failed. Note that partial transcoding of arguments is not supported.";
        completer.complete({fibre::kStreamError, buffer.begin()});
        return;
    }
    
    encoded_buf_ = {};
    encoded_buf_.resize(encoded_size);
    decoded_buf_ = buffer;
    completer_ = &completer;
    call_->n_active_transfers++;

    encoded_stream_->start_read(encoded_buf_, &transfer_handle_, *this);
}

void ArgDecoder::cancel_read(fibre::TransferHandle transfer_handle) {
    encoded_stream_->cancel_read(transfer_handle_);
}

void ArgDecoder::complete(fibre::ReadResult result) {
    transfer_handle_ = 0;

    size_t n_recv = result.end - encoded_buf_.data();
    FIBRE_LOG(D) << "received " << n_recv << " bytes ";

    if (n_recv > encoded_buf_.size()) {
        FIBRE_LOG(E) << "internal error: received more bytes than expected";
    }

    ssize_t len_diff = encoded_buf_.size() - decoded_buf_.size();
    uint8_t* rx_end = decoded_buf_.begin() + n_recv - len_diff;

    size_t arg_offset = 0;

    // Transcode stream buffer to application buffer
    bool ok = call_->iterate_over_args_at(encoded_offset_, n_recv, decoded_buf_.size(), [&](
            const fibre::LegacyFibreArg& arg,
            ssize_t rel_encoded_offset, ssize_t rel_decoded_offset,
            size_t encoded_arg_size, size_t decoded_arg_size) {
        fibre::cbufptr_t encoded_buf = {encoded_buf_.data() + rel_encoded_offset, encoded_arg_size};
        fibre::bufptr_t decoded_buf = {decoded_buf_.begin() + rel_decoded_offset, decoded_arg_size};
        return decode_from_transport(call_->obj->client, encoded_buf, decoded_buf, arg);
    });

    if (!ok) {
        FIBRE_LOG(W) << "Transcoding after RX failed. Partial transcoding of arguments is not supported.";
        rx_end = decoded_buf_.begin();
        result.status = fibre::kStreamError;
    } else if (rx_end > decoded_buf_.end()) {
        FIBRE_LOG(E) << "miscalculated pointer: beyond buffer end";
        rx_end = decoded_buf_.end();
        result.status = fibre::kStreamError;
    }

    decoded_buf_ = {};
    encoded_buf_ = {};
    encoded_offset_ += n_recv;
    call_->n_active_transfers--;

    safe_complete(completer_, {result.status, rx_end});
}
