#ifndef __FIBRE_USB_DISCOVERER_HPP
#define __FIBRE_USB_DISCOVERER_HPP

#include "../event_loop.hpp"
#include "../async_stream.hpp"
#include <fibre/libfibre.h>

#include <libusb.h>
#include <thread>
#include <vector>
#include <unordered_map>

namespace fibre {

class LibusbBulkInEndpoint;
class LibusbBulkOutEndpoint;

struct ChannelDiscoveryResult {
    FibreStatus status;
    AsyncStreamSource* rx_channel;
    AsyncStreamSink* tx_channel;
};

template<typename TRes> class FIBRE_PRIVATE LibusbBulkEndpoint;

class FIBRE_PRIVATE LibusbDiscoverer {
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

    struct ChannelDiscoveryContext {
        InterfaceSpecs interface_specs;
        Completer<ChannelDiscoveryResult>* on_found_channels;
    };

    int init(EventLoop* event_loop);
    int deinit() { return deinit(INT_MAX); }
    void start_channel_discovery(const char* specs, size_t specs_len, ChannelDiscoveryContext** handle, Completer<ChannelDiscoveryResult>& on_found_channels);
    int stop_channel_discovery(ChannelDiscoveryContext* handle);

private:
    friend class LibusbBulkEndpoint<ReadResult>;
    friend class LibusbBulkEndpoint<WriteResult>;

    struct Device {
        struct libusb_device* dev;
        struct libusb_device_handle* handle;
        std::vector<LibusbBulkInEndpoint*> ep_in;
        std::vector<LibusbBulkOutEndpoint*> ep_out;
    };

    int deinit(int stage);
    void internal_event_loop();
    void on_event_loop_iteration();
    void on_add_pollfd(int fd, short events);
    void on_remove_pollfd(int fd);
    int on_hotplug(struct libusb_device *dev, libusb_hotplug_event event);
    void poll_devices_now();
    void consider_device(struct libusb_device *device, ChannelDiscoveryContext* subscription);

    EventLoop* event_loop_ = nullptr;
    bool using_sparate_libusb_thread_; // true on Windows. Initialized in init()
    libusb_context *libusb_ctx_ = nullptr; // libusb session
    libusb_hotplug_callback_handle hotplug_callback_handle_ = 0;
    bool run_internal_event_loop_ = false;
    std::thread* internal_event_loop_thread_;
    EventLoopTimer* device_polling_timer_;
    EventLoopTimer* event_loop_timer_ = nullptr;
    std::unordered_map<uint16_t, Device> known_devices_; // key: bus_number << 8 | dev_number
    std::vector<ChannelDiscoveryContext*> subscriptions_;
};

template<typename TRes>
class FIBRE_PRIVATE LibusbBulkEndpoint {
public:
    bool init(LibusbDiscoverer* parent, struct libusb_device_handle* handle, uint8_t endpoint_id);
    bool deinit();

protected:
    void start_transfer(bufptr_t buffer, TransferHandle* handle, Completer<TRes>& completer);
    void cancel_transfer(TransferHandle transfer_handle);

private:
    void submit_transfer();
    void on_transfer_finished();

    LibusbDiscoverer* parent_ = nullptr;
    struct libusb_device_handle* handle_ = nullptr;
    uint8_t endpoint_id_ = 0;
    struct libusb_transfer* transfer_ = nullptr;
    Completer<TRes>* completer_ = nullptr;
};

class FIBRE_PRIVATE LibusbBulkInEndpoint : public LibusbBulkEndpoint<ReadResult>, public AsyncStreamSource {
public:
    void start_read(bufptr_t buffer, TransferHandle* handle, Completer<ReadResult>& completer) final {
        start_transfer(buffer, handle, completer);
    }

    void cancel_read(TransferHandle transfer_handle) final {
        cancel_transfer(transfer_handle);
    }
};

class FIBRE_PRIVATE LibusbBulkOutEndpoint : public LibusbBulkEndpoint<WriteResult>, public AsyncStreamSink {
public:
    void start_write(cbufptr_t buffer, TransferHandle* handle, Completer<WriteResult>& completer) final {
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