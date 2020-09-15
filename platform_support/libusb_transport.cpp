/**
 * @brief Transport provider: libusb
 * 
 * Platform Compatibility: Linux, Windows, macOS
 */

#include "libusb_transport.hpp"
#include "../logging.hpp"
#include "../print_utils.hpp"

#include <algorithm>

using namespace fibre;

DEFINE_LOG_TOPIC(USB);
USE_LOG_TOPIC(USB);

constexpr unsigned int kBulkTimeoutMs = 2000;


/* LibusbDiscoverer ----------------------------------------------------------*/

/**
 * @brief Initializes the discoverer.
 * 
 * Asynchronous tasks will be executed on the provided event_loop.
 * 
 * @param event_loop: The event loop that is used to execute background tasks. The
 *        pointer must be non-null and initialized when this function is called.
 *        It must remain initialized until deinit() of this discoverer was called.
 */
int LibusbDiscoverer::init(EventLoop* event_loop) {
    if (!event_loop)
        return -1;
    event_loop_ = event_loop;

    if (libusb_init(&libusb_ctx_) != LIBUSB_SUCCESS) {
        FIBRE_LOG(E) << "libusb_init() failed: " << sys_err();
        return deinit(0), -1;
    }

    // Fetch initial list of file-descriptors we have to monitor.
    // Note: this will fail on Windows. Since this is used for epoll, we need a
    // different approach for Windows anyway.
    const struct libusb_pollfd** pollfds = libusb_get_pollfds(libusb_ctx_);
    using_sparate_libusb_thread_ = !pollfds;

    if (!using_sparate_libusb_thread_) {
        // This code path is taken on Linux
        FIBRE_LOG(D) << "Using externally provided event loop";

        // Check if libusb needs special time-based polling on this platform
        if (libusb_pollfds_handle_timeouts(libusb_ctx_) == 0) {
            FIBRE_LOG(D) << "Using time-based polling";
        }

        // libusb maintains a (dynamic) list of file descriptors that need to be
        // monitored (via select/poll/epoll) so that I/O events can be processed when
        // needed. Since we use the async libusb interface, we do the monitoring
        // ourselves. That means we always need keep track of the libusb file
        // descriptor list.

        // Subscribe to changes to the list of file-descriptors we have to monitor.
        libusb_set_pollfd_notifiers(libusb_ctx_,
                [](int fd, short events, void *user_data) {
                    ((LibusbDiscoverer*)user_data)->on_add_pollfd(fd, events);
                },
                [](int fd, void *user_data) {
                    ((LibusbDiscoverer*)user_data)->on_remove_pollfd(fd);
                }, this);

        // Fetch initial list of file-descriptors we have to monitor.
        // Note: this will fail on Windows. Since this is used for epoll, we need a
        // different approach for Windows anyway.
        const struct libusb_pollfd** pollfds = libusb_get_pollfds(libusb_ctx_);
        if (!pollfds) {
            return deinit(2), -1;
        }

        for (size_t i = 0; pollfds[i]; ++i) {
            on_add_pollfd(pollfds[i]->fd, pollfds[i]->events);
        }
        libusb_free_pollfds(pollfds);
        pollfds = nullptr;

    } else {
        FIBRE_LOG(D) << "Using internal event loop thread";

        // This code path is taken on Windows (which does not support epoll)
        run_internal_event_loop_ = true;
        internal_event_loop_thread_ = new std::thread([](void* ctx) {
            ((LibusbDiscoverer*)ctx)->internal_event_loop();
        }, this);
    }

    if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        // This code path is taken on Linux
        FIBRE_LOG(D) << "Using libusb native hotplug detection";

        // Subscribe to hotplug events
        int result = libusb_hotplug_register_callback(libusb_ctx_,
                             (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
                             LIBUSB_HOTPLUG_ENUMERATE /* trigger callback for all currently connected devices too */,
                             LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
                             [](struct libusb_context *ctx, struct libusb_device *dev, libusb_hotplug_event event, void *user_data){
                                 return ((LibusbDiscoverer*)user_data)->on_hotplug(dev, event);
                             }, this, &hotplug_callback_handle_);
        if (LIBUSB_SUCCESS != result) {
            FIBRE_LOG(E) << "Error subscribing to hotplug events";
            hotplug_callback_handle_ = 0;
            return deinit(3), -1;
        }

    } else {
        // This code path is taken on Windows
        FIBRE_LOG(D) << "Using periodic polling to discover devices";

        poll_devices_now(); // this will also start a timer to poll again periodically
    }

    if (!pollfds && libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        // The hotplug callback handler above is not yet thread-safe. To make it thread-safe
        // we'd need to post it on the application's event loop.
        FIBRE_LOG(E) << "Hotplug detection with separate libusb thread will cause trouble.";
    }

    return 0;
}

int LibusbDiscoverer::deinit(int stage) {
    // TODO: verify that all devices are closed and hotplug detection is disabled

    if (stage > 3 && libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        libusb_hotplug_deregister_callback(libusb_ctx_, hotplug_callback_handle_);
    }
    
    if (stage > 3 && device_polling_timer_) {
        event_loop_->cancel_timer(device_polling_timer_);
        device_polling_timer_ = nullptr;
    }

    if (stage > 2 && !run_internal_event_loop_) {
        // Deregister libusb events from our event loop.
        const struct libusb_pollfd** pollfds = libusb_get_pollfds(libusb_ctx_);
        if (pollfds) {
            for (size_t i = 0; pollfds[i]; ++i) {
                on_remove_pollfd(pollfds[i]->fd);
            }
            libusb_free_pollfds(pollfds);
            pollfds = nullptr;
        }
    }

    if (stage > 1 && !run_internal_event_loop_) {
        libusb_set_pollfd_notifiers(libusb_ctx_, nullptr, nullptr, nullptr);
    }

    if (stage > 0 && run_internal_event_loop_) {
        run_internal_event_loop_ = false;
        libusb_interrupt_event_handler(libusb_ctx_);
        internal_event_loop_thread_->join();
        delete internal_event_loop_thread_;
        internal_event_loop_thread_ = nullptr;
    }

    if (stage > 0) {
        // TODO: we should probably deinit and close all connected channels
        for (auto& dev: known_devices_) {
            libusb_unref_device(dev.second.dev);
        }
    }

    // FIXME: the libusb_hotplug_deregister_callback call will still trigger a
    // usb_handler event. We need to wait until this has finished before we
    // truly discard libusb resources
    // Update: is this still relevant?
    //usleep(100000);

    if (stage > 0) {
        libusb_exit(libusb_ctx_);
        libusb_ctx_ = nullptr;
    }

    event_loop_ = nullptr;

    return 0;
}

bool try_parse_key(const char* begin, const char* end, const char* key, int* val) {
    char buf[end - begin + 1];
    memcpy(buf, begin, end - begin);
    buf[end - begin] = 0;

    char fmt1[strlen(key) + 6];
    memcpy(fmt1, key, strlen(key));
    memcpy(fmt1 + strlen(key), "=0x%x", 6);

    char fmt2[strlen(key) + 4];
    memcpy(fmt2, key, strlen(key));
    memcpy(fmt2 + strlen(key), "=%d", 4);

    return sscanf(buf, fmt1, val) == 1
        || sscanf(buf, fmt2, val) == 1;
}

/**
 * @brief Starts looking for Fibre devices accessible through USB.
 *
 * Multiple discovery requests can be active at the same time but beware that a
 * channel will be announced to all matching subscribers so be careful with access
 * multiplexing.
 * 
 * If the function succeeds, an opaque context pointer is returned which must be
 * passed to stop_channel_discovery() to terminate this particular request.
 * 
 * @param specs: Specifies the constraints to consider. Must be either empty or
 *        of the format "key1=val1,key2=val2" where the available keys are:
 *
 *          bus, address, idProduct, idVendor, bInterfaceClass,
 *          bInterfaceSubClass, bInterfaceProtocol
 *
 *        The value can be either a integer in decimal or hexadecimal notation
 *        (0x1234).
 *        Omitted keys are ignored during filtering.
 *
 * @param on_found_channels: Invoked when a matching pair of RX/TX channels is found.
 *        This callback will also be called for any matching channels that already exist when
 *        the discovery is started.
 */
void LibusbDiscoverer::start_channel_discovery(const char* specs, size_t specs_len, ChannelDiscoveryContext** handle, Completer<ChannelDiscoveryResult>& on_found_channels) {
    FIBRE_LOG(D) << "starting discovery with filter \"" << std::string(specs, specs_len) << "\"";

    const char* prev_delim = specs;

    InterfaceSpecs interface_specs;

    while (prev_delim < specs + specs_len) {
        const char* next_delim = std::find(prev_delim, specs + specs_len, ',');
        
        bool success = try_parse_key(prev_delim, next_delim, "bus", &interface_specs.bus)
                    || try_parse_key(prev_delim, next_delim, "address", &interface_specs.address)
                    || try_parse_key(prev_delim, next_delim, "idVendor", &interface_specs.vendor_id)
                    || try_parse_key(prev_delim, next_delim, "idProduct", &interface_specs.product_id)
                    || try_parse_key(prev_delim, next_delim, "bInterfaceClass", &interface_specs.interface_class)
                    || try_parse_key(prev_delim, next_delim, "bInterfaceSubClass", &interface_specs.interface_subclass)
                    || try_parse_key(prev_delim, next_delim, "bInterfaceProtocol", &interface_specs.interface_protocol);

        if (!success) {
            FIBRE_LOG(E) << "could not interpret channel discovery specs";
            on_found_channels.complete({kFibreInvalidArgument, nullptr, nullptr});
            return;
        }

        prev_delim = std::min(next_delim + 1, specs + specs_len);
    }

    ChannelDiscoveryContext* subscription = new ChannelDiscoveryContext{interface_specs, &on_found_channels};
    subscriptions_.push_back(subscription);

    for (auto& dev: known_devices_) {
        consider_device(dev.second.dev, subscription);
    }

    if (handle) {
        *handle = subscription;
    }

    return;
}

/**
 * @brief Stops an object discovery process that was started with start_channel_discovery().
 * 
 * Channels which were already discovered will remain open. However if the discovery is restarted
 * it is possible that the same channels are returned again (their pointers need not match the old instance).
 * 
 * The discovery must be considered still in progress until the callback is
 * invoked with kFibreCancelled.
 */
int LibusbDiscoverer::stop_channel_discovery(ChannelDiscoveryContext* handle) {
    auto it = std::find(subscriptions_.begin(), subscriptions_.end(), handle);

    if (it == subscriptions_.end()) {
        FIBRE_LOG(E) << "Not an active subscription";
        return -1;
    }

    subscriptions_.erase(it);
    delete handle;
    return 0;
}

/**
 * @brief Runs the event handling loop. This function blocks until
 * run_internal_event_loop_ is false.
 * 
 * This loop is only executed on Windows. On other platforms the provided EventLoop is used.
 */
void LibusbDiscoverer::internal_event_loop() {
    while (run_internal_event_loop_)
        libusb_handle_events(libusb_ctx_);
}

void LibusbDiscoverer::on_event_loop_iteration() {
    if (event_loop_timer_) {
        FIBRE_LOG(D) << "cancelling event loop timer";
        event_loop_->cancel_timer(event_loop_timer_);
        event_loop_timer_ = nullptr;
    }

    timeval tv = { .tv_sec = 0, .tv_usec = 0 };
    if (libusb_handle_events_timeout(libusb_ctx_, &tv) != 0) {
        FIBRE_LOG(E) << "libusb_handle_events_timeout() failed";
    }

    timeval timeout;
    if (libusb_get_next_timeout(libusb_ctx_, &timeout)) {
        float timeout_sec = (float)timeout.tv_sec + (float)timeout.tv_usec * 1e-6;
        FIBRE_LOG(D) << "setting event loop timeout to " << timeout_sec << " s";
        event_loop_timer_ = event_loop_->call_later(timeout_sec, [](void* ctx) {
            ((LibusbDiscoverer*)ctx)->on_event_loop_iteration();
        }, this);
    }
}

/**
 * @brief Called when libusb wants to add a file descriptor to our event loop.
 */
void LibusbDiscoverer::on_add_pollfd(int fd, short events) {
    event_loop_->register_event(fd, events, [](void* ctx) {
        ((LibusbDiscoverer*)ctx)->on_event_loop_iteration();
    }, this);
}

/**
 * @brief Called when libusb wants to remove a file descriptor to our event loop.
 */
void LibusbDiscoverer::on_remove_pollfd(int fd) {
    event_loop_->deregister_event(fd);
}

/**
 * @brief Called by libusb when a USB device was plugged in or out.
 *
 * If this function returns a non-zero value, libusb removes this filter.
 */
int LibusbDiscoverer::on_hotplug(struct libusb_device *dev,
                                 libusb_hotplug_event event) {
    uint8_t bus_number = libusb_get_bus_number(dev);
    uint8_t dev_number = libusb_get_device_address(dev);
    
    if (LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED == event) {
        FIBRE_LOG(D) << "device arrived: bus " << (int)bus_number << ", " << (int)dev_number;

        for (auto& subscription: subscriptions_) {
            consider_device(dev, subscription);
        }

        // add empty placeholder to the list of known devices
        known_devices_[bus_number << 8 | dev_number] = {
            .dev = libusb_ref_device(dev),
            .handle = nullptr
        };

    } else if (LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT == event) {
        FIBRE_LOG(D) << "device left: bus " << (int)bus_number << ", " << (int)dev_number;

        auto it = known_devices_.find(bus_number << 8 | dev_number);

        if (it != known_devices_.end()) {
            for (auto& ep: it->second.ep_in) {
                ep->deinit();
            }
            for (auto& ep: it->second.ep_out) {
                ep->deinit();
            }
            if (it->second.handle) {
                libusb_close(it->second.handle);
            }

            known_devices_.erase(it);
        }

        libusb_unref_device(dev);

    } else {
        FIBRE_LOG(W) << "Unexpected event: " << event;
    }
    
    return 0;
}

void LibusbDiscoverer::poll_devices_now() {
    FIBRE_LOG(D) << "poll_devices_now() called.";

    device_polling_timer_ = nullptr;

    libusb_device** list = nullptr;
    ssize_t n_devices = libusb_get_device_list(libusb_ctx_, &list);
    std::unordered_map<uint16_t, libusb_device*> current_devices;

    if (n_devices < 0) {
        FIBRE_LOG(W) << "libusb_get_device_list() failed.";
    } else {
        for (ssize_t i = 0; i < n_devices; ++i) {
            uint8_t bus_number = libusb_get_bus_number(list[i]);
            uint8_t dev_number = libusb_get_device_address(list[i]);
            current_devices[bus_number << 8 | dev_number] = list[i];
        }

        // Call on_hotplug for all new devices
        for (auto& dev: current_devices) {
            if (known_devices_.find(dev.first) == known_devices_.end()) {
                on_hotplug(dev.second, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED);
            }
        }

        // Call on_hotplug for all lost devices

        std::vector<libusb_device*> lost_devices;

        for (auto& dev: known_devices_) {
            if (current_devices.find(dev.first) == current_devices.end()) {
                lost_devices.push_back(dev.second.dev);
            }
        }

        for (auto& dev: lost_devices) {
            on_hotplug(dev, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT);
        }

        libusb_free_device_list(list, 1 /* unref the devices */);
    }

    // It's possible that the discoverer was deinited during this function.
    if (event_loop_) {
        device_polling_timer_ = event_loop_->call_later(1.0, [](void* ctx) {
            ((LibusbDiscoverer*)ctx)->poll_devices_now();
        }, this);
    }
}

void LibusbDiscoverer::consider_device(struct libusb_device *device, ChannelDiscoveryContext* subscription) {
    uint8_t bus_number = libusb_get_bus_number(device);
    uint8_t dev_number = libusb_get_device_address(device);

    bool mismatch = (subscription->interface_specs.bus != -1 && bus_number != subscription->interface_specs.bus)
                 || (subscription->interface_specs.address != -1 && dev_number != subscription->interface_specs.address);

    if (mismatch) {
        return;
    }

    if (subscription->interface_specs.vendor_id != -1 || subscription->interface_specs.product_id != -1) {
        struct libusb_device_descriptor dev_desc;
        int result = libusb_get_device_descriptor(device, &dev_desc);
        if (result != LIBUSB_SUCCESS) {
            FIBRE_LOG(W) << "Failed to get device descriptor: " << result;
        }

        mismatch = (subscription->interface_specs.vendor_id != -1 && dev_desc.idVendor != subscription->interface_specs.vendor_id)
                || (subscription->interface_specs.product_id != -1 && dev_desc.idProduct != subscription->interface_specs.product_id);

        if (mismatch) {
            return;
        }
    }
    
    static libusb_device_handle *handle = nullptr;
    struct libusb_config_descriptor* config_desc = nullptr;

    if (libusb_get_active_config_descriptor(device, &config_desc) != LIBUSB_SUCCESS) {
        FIBRE_LOG(E) << "Failed to get active config descriptor: " << sys_err();
    } else {
        for (uint8_t i = 0; i < config_desc->bNumInterfaces; ++i) {
            for (int j = 0; j < config_desc->interface[i].num_altsetting; ++j) {
                // TODO: probably we should only chose one alt setting
                const struct libusb_interface_descriptor* intf_desc = &(config_desc->interface[i].altsetting[j]);

                mismatch = (subscription->interface_specs.interface_class != -1 && intf_desc->bInterfaceClass != subscription->interface_specs.interface_class)
                        || (subscription->interface_specs.interface_subclass != -1 && intf_desc->bInterfaceSubClass != subscription->interface_specs.interface_subclass)
                        || (subscription->interface_specs.interface_protocol != -1 && intf_desc->bInterfaceProtocol != subscription->interface_specs.interface_protocol);
                if (mismatch) {
                    continue;
                }

                // We found a matching interface. Now find one bulk IN and one bulk OUT endpoint.
                const libusb_endpoint_descriptor* libusb_ep_in = nullptr;
                const libusb_endpoint_descriptor* libusb_ep_out = nullptr;
                for (uint8_t k = 0; k < intf_desc->bNumEndpoints; ++k) {
                    if ((intf_desc->endpoint[k].bmAttributes & 0x03) == LIBUSB_TRANSFER_TYPE_BULK
                        && (intf_desc->endpoint[k].bEndpointAddress & 0x80) == LIBUSB_ENDPOINT_IN) {
                        libusb_ep_in = &intf_desc->endpoint[k];
                    } else if ((intf_desc->endpoint[k].bmAttributes & 0x03) == LIBUSB_TRANSFER_TYPE_BULK
                               && (intf_desc->endpoint[k].bEndpointAddress & 0x80) == LIBUSB_ENDPOINT_OUT) {
                        libusb_ep_out = &intf_desc->endpoint[k];
                    }
                }

                Device& my_dev = known_devices_[bus_number << 8 | dev_number];

                // If the same device was already returned in a previous discovery
                // then it will already be open.

                if (!my_dev.handle) {
                    int result = libusb_open(device, &my_dev.handle);
                    if (LIBUSB_SUCCESS != result) {
                        FIBRE_LOG(E) << "Could not open USB device: " << result;
                        continue;
                    }
                }

                int result = libusb_claim_interface(my_dev.handle, i);
                if (LIBUSB_SUCCESS != result) {
                    FIBRE_LOG(E) << "Could not claim interface " << i << " on USB device: " << result;
                    continue;
                }

                EventLoop* event_loop = using_sparate_libusb_thread_ ? event_loop_ : nullptr;

                LibusbBulkInEndpoint* ep_in = new LibusbBulkInEndpoint();
                if (libusb_ep_in && ep_in->init(event_loop, my_dev.handle, libusb_ep_in->bEndpointAddress)) {
                    my_dev.ep_in.push_back(ep_in);
                } else {
                    delete ep_in;
                    ep_in = nullptr;
                }

                LibusbBulkOutEndpoint* ep_out = new LibusbBulkOutEndpoint();
                if (libusb_ep_out && ep_out->init(event_loop, my_dev.handle, libusb_ep_out->bEndpointAddress)) {
                    my_dev.ep_out.push_back(ep_out);
                } else {
                    delete ep_out;
                    ep_out = nullptr;
                }

                if (subscription->on_found_channels) {
                    subscription->on_found_channels->complete({kFibreOk, ep_in, ep_out});
                }
            }
        }

        libusb_free_config_descriptor(config_desc);
        config_desc = nullptr;
    }
}


/* LibusbBulkEndpoint --------------------------------------------------------*/

template<typename TRes>
bool LibusbBulkEndpoint<TRes>::init(EventLoop* event_loop, libusb_device_handle* handle, uint8_t endpoint_id) {
    event_loop_ = event_loop;
    handle_ = handle;
    transfer_ = libusb_alloc_transfer(0);
    endpoint_id_ = endpoint_id;
    return true;
}

template<typename TRes>
bool LibusbBulkEndpoint<TRes>::deinit() {
    if (completer_) {
        FIBRE_LOG(E) << "Transfer still in progress. This is gonna be messy.";
    }

    libusb_free_transfer(transfer_);
    transfer_ = nullptr;
    return true;
}

template<typename TRes>
void LibusbBulkEndpoint<TRes>::start_transfer(bufptr_t buffer, TransferHandle* handle, Completer<TRes>& completer) {
    if (handle) {
        *handle = reinterpret_cast<TransferHandle>(this);
    }

    if (completer_) {
        FIBRE_LOG(E) << "transfer already in progress";
        completer.complete({kStreamError, nullptr});
        return;
    }

    if (!handle_) {
        FIBRE_LOG(E) << "device not open";
        completer.complete({kStreamError, nullptr});
        return;
    }

    auto direct_callback = [](struct libusb_transfer* transfer){
        ((LibusbBulkEndpoint<TRes>*)transfer->user_data)->on_transfer_finished();
    };

    // This callback is used if we start our own libusb thread
    // separate from the application's event loop thread
    auto indirect_callback = [](struct libusb_transfer* transfer){
        ((LibusbBulkEndpoint<TRes>*)transfer->user_data)->event_loop_->post(
            [](void* ctx) {
                ((LibusbBulkEndpoint<TRes>*)ctx)->on_transfer_finished();
            }, transfer->user_data
        );
    };

    //FIBRE_LOG(D) << "transfer of size " << buffer.size();
    libusb_fill_bulk_transfer(transfer_, handle_, endpoint_id_,
        buffer.begin(), buffer.size(),
        event_loop_ ? indirect_callback : direct_callback,
        this, kBulkTimeoutMs);
    
    completer_ = &completer;
    submit_transfer();
}

template<typename TRes>
void LibusbBulkEndpoint<TRes>::cancel_transfer(TransferHandle transfer_handle) {
    if (!completer_) {
        FIBRE_LOG(E) << "transfer not in progress";
        return;
    }

    libusb_cancel_transfer(transfer_);
}

template<typename TRes>
void LibusbBulkEndpoint<TRes>::submit_transfer() {
    int result = libusb_submit_transfer(transfer_);
    if (LIBUSB_SUCCESS == result) {
        // ok
        FIBRE_LOG(T) << "started USB transfer on EP " << as_hex(endpoint_id_);
    } else if (LIBUSB_ERROR_NO_DEVICE == result) {
        FIBRE_LOG(W) << "couldn't start USB transfer on EP " << as_hex(endpoint_id_) << ": " << libusb_error_name(result);
        safe_complete(completer_, {kStreamClosed, nullptr});
    } else {
        FIBRE_LOG(W) << "couldn't start USB transfer on EP " << as_hex(endpoint_id_) << ": " << libusb_error_name(result);
        safe_complete(completer_, {kStreamError, nullptr});
    }
}

template<typename TRes>
void LibusbBulkEndpoint<TRes>::on_transfer_finished() {
    // We ignore timeouts here and just retry. If the application wishes to have
    // a timeout on the transfer it can just call cancel_transfer() after a while.
    if (transfer_->status == LIBUSB_TRANSFER_TIMED_OUT) {
        submit_transfer();
        return;
    }

    // On linux we get LIBUSB_TRANSFER_STALL on the RX pipe when the cable is plugged out
    StreamStatus status = LIBUSB_TRANSFER_COMPLETED == transfer_->status ? kStreamOk :
                          LIBUSB_TRANSFER_CANCELLED == transfer_->status ? kStreamCancelled :
                          LIBUSB_TRANSFER_STALL == transfer_->status ? kStreamClosed :
                          LIBUSB_TRANSFER_NO_DEVICE == transfer_->status ? kStreamClosed :
                          kStreamError;

    (status == kStreamError ? FIBRE_LOG(W) : FIBRE_LOG(T))
        << "USB transfer on EP " << as_hex(endpoint_id_) << " finished with " << libusb_error_name(transfer_->status);

    uint8_t* end = std::max(transfer_->buffer + transfer_->actual_length, transfer_->buffer);

    safe_complete(completer_, {status, end});
}
