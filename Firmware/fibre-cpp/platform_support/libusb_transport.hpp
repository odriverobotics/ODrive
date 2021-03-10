#ifndef __FIBRE_USB_DISCOVERER_HPP
#define __FIBRE_USB_DISCOVERER_HPP

#include <fibre/event_loop.hpp>
#include <fibre/async_stream.hpp>
#include <fibre/channel_discoverer.hpp>

#include <libusb.h>
#include <thread>
#include <vector>
#include <unordered_map>

namespace fibre {

class LibusbBulkInEndpoint;
class LibusbBulkOutEndpoint;

template<typename TRes> class LibusbBulkEndpoint;

class LibusbDiscoverer : public ChannelDiscoverer {
public:

    struct InterfaceSpecs {
        int bus = -1; // -1 to ignore
        int address = -1; // -1 to ignore
        int vendor_id = -1; // -1 to ignore
        int product_id = -1; // -1 to ignore
        int interface_class = -1; // -1 to ignore
        int interface_subclass = -1; // -1 to ignore
        int interface_protocol = -1; // -1 to ignore
    };

    struct MyChannelDiscoveryContext : ChannelDiscoveryContext {
        InterfaceSpecs interface_specs;
        Domain* domain;
    };

    constexpr static const char* get_name() { return "usb"; }
    bool init(EventLoop* event_loop);
    bool deinit() { return deinit(INT_MAX); }
    void start_channel_discovery(Domain* domain, const char* specs, size_t specs_len, ChannelDiscoveryContext** handle) final;
    int stop_channel_discovery(ChannelDiscoveryContext* handle) final;

private:
    friend class LibusbBulkEndpoint<ReadResult>;
    friend class LibusbBulkEndpoint<WriteResult>;

    struct Device {
        struct libusb_device* dev;
        struct libusb_device_handle* handle;
        std::vector<LibusbBulkInEndpoint*> ep_in;
        std::vector<LibusbBulkOutEndpoint*> ep_out;
    };

    bool deinit(int stage);
    void internal_event_loop();
    void on_event_loop_iteration();
    void on_event_loop_iteration2(uint32_t) { on_event_loop_iteration(); }
    void on_add_pollfd(int fd, short events);
    void on_remove_pollfd(int fd);
    int on_hotplug(struct libusb_device *dev, libusb_hotplug_event event);
    void poll_devices_now();
    void consider_device(struct libusb_device *device, MyChannelDiscoveryContext* subscription);

    EventLoop* event_loop_ = nullptr;
    bool using_sparate_libusb_thread_; // true on Windows. Initialized in init()
    libusb_context *libusb_ctx_ = nullptr; // libusb session
    libusb_hotplug_callback_handle hotplug_callback_handle_ = 0;
    bool run_internal_event_loop_ = false;
    std::thread* internal_event_loop_thread_;
    EventLoopTimer* device_polling_timer_;
    EventLoopTimer* event_loop_timer_ = nullptr;
    std::unordered_map<uint16_t, Device> known_devices_; // key: bus_number << 8 | dev_number
    std::vector<MyChannelDiscoveryContext*> subscriptions_;
};

template<typename TRes>
class LibusbBulkEndpoint {
public:
    bool init(LibusbDiscoverer* parent, struct libusb_device_handle* handle, uint8_t endpoint_id);
    bool deinit();

protected:
    void start_transfer(bufptr_t buffer, TransferHandle* handle, Callback<void, TRes> completer);
    void cancel_transfer(TransferHandle transfer_handle);

private:
    void submit_transfer();
    void on_transfer_finished();

    LibusbDiscoverer* parent_ = nullptr;
    struct libusb_device_handle* handle_ = nullptr;
    uint8_t endpoint_id_ = 0;
    struct libusb_transfer* transfer_ = nullptr;
    Callback<void, TRes> completer_ = nullptr;
};

class LibusbBulkInEndpoint final : public LibusbBulkEndpoint<ReadResult>, public AsyncStreamSource {
public:
    void start_read(bufptr_t buffer, TransferHandle* handle, Callback<void, ReadResult> completer) final {
        start_transfer(buffer, handle, completer);
    }

    void cancel_read(TransferHandle transfer_handle) final {
        cancel_transfer(transfer_handle);
    }
};

class LibusbBulkOutEndpoint final : public LibusbBulkEndpoint<WriteResult>, public AsyncStreamSink {
public:
    void start_write(cbufptr_t buffer, TransferHandle* handle, Callback<void, WriteResult> completer) final {
        start_transfer({
            (unsigned char*)buffer.begin(),
            buffer.size()
        }, handle, completer);
    }

    void cancel_write(TransferHandle transfer_handle) final {
        cancel_transfer(transfer_handle);
    }
};

}

#endif // __FIBRE_USB_DISCOVERER_HPP