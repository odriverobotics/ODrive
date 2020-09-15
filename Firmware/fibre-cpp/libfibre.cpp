
#include <fibre/libfibre.h>
#include "platform_support/libusb_transport.hpp"
#include "logging.hpp"
#include "print_utils.hpp"
#include "legacy_protocol.hpp"
#include "legacy_object_client.hpp"
#include "stdio.h" // TODO: remove
#include "string.h"
#include <algorithm>
#include "fibre/simple_serdes.hpp"

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

struct FIBRE_PRIVATE LibFibreCtx {
    ExternalEventLoop* event_loop;
    construct_object_cb_t on_construct_object;
    destroy_object_cb_t on_destroy_object;
    void* cb_ctx;
    fibre::LibusbDiscoverer libusb_discoverer;
    size_t n_discoveries = 0;
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

    fibre::LibusbDiscoverer::ChannelDiscoveryContext* libusb_discovery_ctx = nullptr;
    on_found_object_cb_t on_found_object;
    void* cb_ctx;
    LibFibreCtx* ctx;
    std::vector<fibre::LegacyProtocolPacketBased*> protocol_instances;
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

    const size_t mtu = 64; // TODO: get MTU from channel specific data

    auto protocol = new fibre::LegacyProtocolPacketBased(result.rx_channel, result.tx_channel, mtu);
    protocol->client_.user_data_ = ctx;
    protocol_instances.push_back(protocol);
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
    if (!register_event || !deregister_event) {
        FIBRE_LOG(E) << "invalid argument";
        return nullptr;
    }
    
    LibFibreCtx* ctx = new LibFibreCtx();
    ctx->event_loop = new ExternalEventLoop(post, register_event, deregister_event, call_later, cancel_timer);
    ctx->on_construct_object = construct_object;
    ctx->on_destroy_object = destroy_object;
    ctx->cb_ctx = cb_ctx;

    if (ctx->libusb_discoverer.init(ctx->event_loop) != 0) {
        delete ctx;
        FIBRE_LOG(E) << "failed to init libusb transport layer";
        return nullptr;
    }

    FIBRE_LOG(D) << "opened (" << fibre::as_hex((uintptr_t)ctx) << ")";
    return ctx;
}

void libfibre_close(LibFibreCtx* ctx) {
    if (ctx->n_discoveries) {
        FIBRE_LOG(W) << "there are still discovery processes ongoing";
    }

    ctx->libusb_discoverer.deinit();
    delete ctx->event_loop;
    delete ctx;

    FIBRE_LOG(D) << "closed (" << fibre::as_hex((uintptr_t)ctx) << ")";
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
        
        if ((colon - prev_delim) == strlen("usb") && std::equal(prev_delim, colon, "usb")) {
            ctx->libusb_discoverer.start_channel_discovery(colon_end, next_delim - colon_end,
                    &discovery_ctx->libusb_discovery_ctx, *discovery_ctx);
        } else {
            FIBRE_LOG(W) << "transport layer \"" << std::string(prev_delim, colon - prev_delim) << "\" not implemented";
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

    if (discovery_ctx->libusb_discovery_ctx) {
        // TODO: implement "stopped" callback
        ctx->libusb_discoverer.stop_channel_discovery(discovery_ctx->libusb_discovery_ctx);
    }

    delete discovery_ctx;
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

void transcode(fibre::LegacyObjectClient* client, std::vector<uint8_t>& buffer, const std::vector<fibre::LegacyFibreArg>& args, bool to) {
    size_t offset = 0;

    for (auto& arg: args) {
        if (to && arg.codec == "endpoint_ref") {
            fibre::cbufptr_t orig_range = fibre::cbufptr_t{buffer}.skip(offset).take(sizeof(uintptr_t));

            uintptr_t val = *reinterpret_cast<const uintptr_t*>(orig_range.begin());

            resize_at(buffer, offset, (ssize_t)4 - (ssize_t)sizeof(uintptr_t));
            fibre::bufptr_t transcoded_range = fibre::bufptr_t{buffer}.skip(offset).take(4);

            auto obj = reinterpret_cast<fibre::LegacyObject*>(val);
            write_le<uint16_t>(obj ? obj->ep_num : 0, &transcoded_range);
            write_le<uint16_t>(obj ? obj->client->json_crc_ : 0, &transcoded_range);

            offset += 4;

        } else if (!to && arg.codec == "endpoint_ref") {
            fibre::cbufptr_t orig_range = fibre::cbufptr_t{buffer}.skip(offset).take(4);

            uint16_t ep_num = *read_le<uint16_t>(&orig_range);
            uint16_t json_crc = *read_le<uint16_t>(&orig_range);

            resize_at(buffer, offset, (ssize_t)sizeof(uintptr_t) - (ssize_t)4);
            fibre::bufptr_t transcoded_range = fibre::bufptr_t{buffer}.skip(offset).take(sizeof(uintptr_t));

            fibre::LegacyObject* obj_ptr = nullptr;

            if (ep_num && json_crc == client->json_crc_) {
                for (auto& known_obj: client->objects_) {
                    if (known_obj->ep_num == ep_num) {
                        obj_ptr = known_obj.get();
                    }
                }
            }

            FIBRE_LOG(D) << "placing transcoded ptr " << reinterpret_cast<uintptr_t>(obj_ptr);
            *reinterpret_cast<uintptr_t*>(transcoded_range.begin()) = reinterpret_cast<uintptr_t>(obj_ptr);

            offset += sizeof(uintptr_t);

        } else {
            offset += arg.size;
        }
    }
}

struct FIBRE_PRIVATE LibFibreCallContext : fibre::Completer<fibre::LegacyObjectClient::CallResult> {
    void complete(fibre::LegacyObjectClient::CallResult output) final {

        FIBRE_LOG(D) << "received " << (output.end - rx_vec.data()) << " bytes ";

        // Prune vector to the end of valid data
        rx_vec.erase(rx_vec.begin() + (output.end - rx_vec.data()), rx_vec.end());

        transcode(obj->client, rx_vec, func->outputs, false);

        size_t len = std::min(rx_vec.size(), rx_buf.size());
        std::copy(rx_vec.begin(), rx_vec.begin() + len, rx_buf.begin());
        FIBRE_LOG(D) << "result is [" << as_hex(rx_buf) << "]";

        if (on_completed) {
            (*on_completed)(ctx, output.status, rx_buf.begin() + len);
        }
        delete this;
    }

    fibre::LegacyObject* obj = nullptr;
    fibre::LegacyFibreFunction* func = nullptr;
    void (*on_completed)(void*, FibreStatus, uint8_t*);
    void* ctx;
    fibre::cbufptr_t tx_buf; // application-owned buffer
    fibre::bufptr_t rx_buf; // application-owned buffer
    std::vector<uint8_t> tx_vec; // libfibre-owned buffer used after transcoding from application buffer
    std::vector<uint8_t> rx_vec; // libfibre-owned buffer used before transcoding to application buffer
    fibre::LegacyObjectClient::CallContext* handle;
};

void libfibre_start_call(LibFibreObject* obj, LibFibreFunction* func,
                         const uint8_t *input, size_t input_length,
                         uint8_t *output, size_t output_length,
                         LibFibreCallContext** handle,
                         on_call_completed_cb_t on_completed, void* cb_ctx) {
    if (!obj || !func || !input || !output) {
        if (on_completed) {
            (*on_completed)(cb_ctx, kFibreInvalidArgument, output);
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
            (*on_completed)(cb_ctx, kFibreInvalidArgument, output);
        }
        return;
    }

    std::vector<uint8_t> tx_vec;
    tx_vec.insert(tx_vec.begin(), input, input + input_length);


    auto completer = new LibFibreCallContext();
    completer->obj = obj_cast;
    completer->func = func_cast;
    completer->on_completed = on_completed;
    completer->ctx = cb_ctx;
    completer->tx_buf = fibre::cbufptr_t{input, input + input_length};
    completer->rx_buf = fibre::bufptr_t{output, output + output_length};

    completer->tx_vec.insert(completer->tx_vec.begin(), completer->tx_buf.begin(), completer->tx_buf.end());
    transcode(obj_cast->client, completer->tx_vec, func_cast->inputs, true);

    size_t output_size = 0;
    for (auto& arg: func_cast->outputs)
        output_size += arg.size;
    FIBRE_LOG(D) << "sizing output to " << output_size;
    completer->rx_vec.resize(output_size);

    if (handle) {
        *handle = completer;
    }

    obj_cast->client->start_call(obj_cast->ep_num, func_cast,
        fibre::cbufptr_t{completer->tx_vec}, fibre::bufptr_t{completer->rx_vec},
        &completer->handle, *completer);
}

void libfibre_cancel_call(LibFibreCallContext* handle) {
    if (handle && handle->on_completed) {
        handle->obj->client->cancel_call(handle->handle);
    }
}
