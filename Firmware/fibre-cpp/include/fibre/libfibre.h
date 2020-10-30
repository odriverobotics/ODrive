/**
 * @brief Fibre C library
 * 
 * The library is fully asynchronous and runs on an application-managed event
 * loop. This integration happens with the call to libfibre_open(), where the
 * application must pass a couple of functions that libfibre will use to put
 * tasks on the event loop.
 * 
 * Some general things to note:
 *  - None of the library's functions are blocking.
 *  - None of the library's functions can be expected to be thread-safe, they
 *    should not be invoked from any other thread than the one that runs the
 *    event loop.
 *  - Callbacks that the user passes to a libfibre function are always executed
 *    on the event loop thread.
 *  - All of the library's functions can be expected reentry-safe. That means
 *    you can call into any libfibre function from any callback handler that
 *    libfibre invokes.
 */

#ifndef __LIBFIBRE_H
#define __LIBFIBRE_H

#include <stdint.h>
#include <stdlib.h>

#if defined(_MSC_VER)
#   define DLL_EXPORT __declspec(dllexport)
#   define DLL_IMPORT __declspec(dllimport)
#elif defined(__GNUC__)
#   define DLL_EXPORT __attribute__((visibility("default")))
#   define DLL_IMPORT
#   if __GNUC__ > 4
#       define DLL_LOCAL __attribute__((visibility("hidden")))
#   else
#       define DLL_LOCAL
#   endif
#else
#   error("Don't know how to export shared object libraries")
#endif

#ifdef FIBRE_COMPILE
#   ifndef FIBRE_PUBLIC
#       define FIBRE_PUBLIC DLL_EXPORT
#   endif
#   define FIBRE_PRIVATE DLL_LOCAL
#else
#   define FIBRE_PUBLIC DLL_IMPORT
#endif


#define FIBRE_PRIVATE DLL_LOCAL

#ifdef __cplusplus
extern "C" {
#endif

struct LibFibreCtx;
struct LibFibreChannelDiscoveryCtx;
struct LibFibreDiscoveryCtx;
struct LibFibreCallContext;
struct LibFibreObject;
struct LibFibreInterface;
struct LibFibreFunction;
struct LibFibreAttribute;
struct LibFibreTxStream;
struct LibFibreRxStream;

enum FibreStatus {
    kFibreOk,
    kFibreCancelled,
    kFibreClosed,
    kFibreInvalidArgument,
    kFibreInternalError
};

struct LibFibreVersion {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
};


typedef int (*post_cb_t)(void (*callback)(void*), void* cb_ctx);
typedef int (*register_event_cb_t)(int fd, uint32_t events, void (*callback)(void*), void* cb_ctx);
typedef int (*deregister_event_cb_t)(int fd);
typedef struct EventLoopTimer* (*call_later_cb_t)(float delay, void (*callback)(void*), void* cb_ctx);
typedef int (*cancel_timer_cb_t)(struct EventLoopTimer* timer);

/**
 * @brief construct_object callback type for libfibre_open().
 * 
 * @param ctx: The user data that was passed to libfibre_open().
 * @param obj: An object handle. This handle is valid until the invokation of
 *        destroy_object(). It is unique at any point in time but can be reused
 *        after destroy_object().
 * @param intf: A handle for the interface that this object implements.
 *        The handle may be identical to an interface handle announced for a
 *        previous object.
 *        The interface handle is valid until the last object that implements it
 *        is destroyed.
 * @param intf_name: The ASCII-encoded name of the interface. Can be NULL for
 *        anonymous interfaces. If not NULL, it is only valid for the duration
 *        of the callback and must not be freed by the application.
 */
typedef void (*construct_object_cb_t)(void* ctx, LibFibreObject* obj, LibFibreInterface* intf, const char* intf_name, size_t intf_name_length);
typedef void (*destroy_object_cb_t)(void* ctx, LibFibreObject* obj);

/**
 * @brief on_start_discovery callback type for libfibre_register_discoverer().
 * 
 * For every channel pair that the application finds that matches the filter of
 * this discoverer the application should call libfibre_add_channels().
 * 
 * @param discovery_handle: An opaque handle that libfibre will pass to the
 *        corresponding on_stop_discovery callback to stop the discovery.
 * @param specs, specs_length: The specs string that specifies discoverer-specific
 *        filter parameters.
 */
typedef void (*on_start_discovery_cb_t)(void* ctx, LibFibreChannelDiscoveryCtx* discovery_ctx, const char* specs, size_t specs_length);
typedef void (*on_stop_discovery_cb_t)(void* ctx, LibFibreChannelDiscoveryCtx* discovery_ctx);

typedef void (*on_found_object_cb_t)(void*, LibFibreObject*);
typedef void (*on_stopped_cb_t)(void*, FibreStatus);

typedef void (*on_attribute_added_cb_t)(void*, LibFibreAttribute*, const char* name, size_t name_length, LibFibreInterface*, const char* intf_name, size_t intf_name_length);
typedef void (*on_attribute_removed_cb_t)(void*, LibFibreAttribute*);

/**
 * @brief on_function_added callback type for libfibre_subscribe_to_interface().
 * 
 * @param ctx: The user data that was passed to libfibre_subscribe_to_interface().
 * @param func: A handle for the function. Remains valid until the corresponding
 *        call to on_function_removed().
 * @param name: The ASCII-encoded name of the function.
 * @param name_length: Length in bytes of the name.
 * @param input_names: A null-terminated list of null-terminated ASCII-encoded
 *        strings. Each string corresponds to the name of one input argument.
 *        The list and the string buffers are only valid for the duration of the
 *        callback. They must not be freed by the application.
 * @param input_codecs: A null-terminated list of null-terminated ASCII-encoded
 *        strings. Each string names the codec of one input argument.
 *        The list and the string buffers are only valid for the duration of the
 *        callback. They must not be freed by the application.
 * @param output_names: Analogous to input_names.
 * @param output_codecs: Analogous to output names.
 */
typedef void (*on_function_added_cb_t)(void* ctx, LibFibreFunction* func, const char* name, size_t name_length, const char** input_names, const char** input_codecs, const char** output_names, const char** output_codecs);

typedef void (*on_function_removed_cb_t)(void*, LibFibreFunction*);
typedef void (*on_call_completed_cb_t)(void*, FibreStatus);

/**
 * @brief TX completion callback type for libfibre_start_tx().
 * 
 * @param ctx: The user data that was passed to libfibre_start_tx().
 * @param tx_stream: The TX stream on which the TX operation completed.
 * @param status: The status of the last TX operation.
 *         - kFibreOk: The indicated range of the TX buffer was successfully
 *           transmitted and the stream might accept more data.
 *         - kFibreClosed: The indicated range of the TX buffer was successfully
 *           transmitted and the stream will no longer accept any data.
 *         - Any other status: Successful transmission of the data cannot be
 *           guaranteed and no more data can be sent on this stream.
 * @param tx_end: Points to the address after the last byte read from the
 *        TX buffer. This pointer always points to a valid position in the
 *        buffer (or the end of the buffer), even if the transmission failed.
 *        However if the status is something other than kFibreOk and
 *        kFibreClosed then the pointer may not precisely indicate the
 *        transmitted data range.
 */
typedef void (*on_tx_completed_cb_t)(void* ctx, LibFibreTxStream* tx_stream, FibreStatus status, const uint8_t* tx_end);

/**
 * @brief RX completion callback type for libfibre_start_rx().
 * 
 * @param ctx: The user data that was passed to libfibre_start_rx().
 * @param rx_stream: The RX stream on which the RX operation completed.
 * @param status: The status of the last RX operation.
 *         - kFibreOk: The indicated range of the RX buffer was successfully
 *           filled with received data and the stream might emit more data.
 *         - kFibreClosed: The indicated range of the RX buffer was successfully
 *           filled with received data and the stream will emit no more data.
 *         - Any other status: Successful transmission of the data cannot be
 *           guaranteed and no more data can be sent on this stream.
 * @param rx_end: Points to the address after the last byte written to the
 *        RX buffer. This pointer always points to a valid position in the
 *        buffer (or the end of the buffer), even if the reception failed.
 *        However if the status is something other than kFibreOk and
 *        kFibreClosed then the pointer may not precisely indicate the
 *        received data range.
 */
typedef void (*on_rx_completed_cb_t)(void* ctx, LibFibreRxStream* rx_stream, FibreStatus status, uint8_t* rx_end);

/**
 * @brief Returns the version of the libfibre library.
 * 
 * The returned struct must not be freed.
 * 
 * The version adheres to Semantic Versioning, that means breaking changes of
 * the ABI can be detected by an increment of the major version number (unless
 * it's zero).
 * 
 * Even if breaking changes are introduced, we promise to keep this function
 * backwards compatible.
 */
FIBRE_PUBLIC const struct LibFibreVersion* libfibre_get_version();

/**
 * @brief Opens and initializes a Fibre context.
 * 
 * @param post: Called by libfibre when it wants the application to run
 *        a callback on the application's event loop.
 *        This is the only callback that libfibre can invoke from a different
 *        thread than the event loop thread itself. The application must
 *        ensure that this callback is thread-safe.
 *        This allows libfibre to run other threads internally while keeping
 *        threading promises made to the application.
 * @param register_event: TODO: this is a Linux specific callback. Need to use
 *        IOCP on Windows.
 * @param deregister_event: TODO: this is a Linux specific callback. Need to use
 *        IOCP on Windows.
 * @param call_later: Called by libfibre to ask the application to call a
 *        certain callback after a certain amount of time on the same thread
 *        on which libfibre_open() was called. The application should return
 *        an opaque handle that libfibre can use to cancel the timer.
 * @param cancel_timer: Called by libfibre to ask the application to cancel a
 *        callback timer previously enqueued with call_later().
 * @param construct_object: Called by libfibre for every remote object that is
 *        allocated. This can be a root object that was just discovered or an
 *        object that was created as a result of operations like
 *        libfibre_get_attribute(). The application can use this to construct
 *        a corresponding object in the application's environment.
 * @param destroy_object: Called by libfibre when a remote object is lost, for
 *        instance because all channels that provided connection to the object
 *        broke down.
 *        An object pointer must no longer be used during or after the call to
 *        destroy_object().
 * @param cb_ctx: Arbitrary user data passed to construct_object() and
 *        destroy_object().
 */
FIBRE_PUBLIC struct LibFibreCtx* libfibre_open(
    post_cb_t post,
    register_event_cb_t register_event,
    deregister_event_cb_t deregister_event,
    call_later_cb_t call_later,
    cancel_timer_cb_t cancel_timer,
    construct_object_cb_t construct_object,
    destroy_object_cb_t destroy_object,
    void* cb_ctx);

/**
 * @brief Closes a context that was previously opened with libfibre_open().
 *
 * This function must not be invoked before all ongoing discovery processes
 * are stopped and all channels are closed.
 */
FIBRE_PUBLIC void libfibre_close(struct LibFibreCtx* ctx);

/**
 * @brief Registers an external channel discovery provider.
 * 
 * Libfibre starts and stops the discoverer on demand as a result of calls
 * to libfibre_start_discovery() and libfibre_stop_discovery().
 * This can be used by applications to implement transport providers which are
 * not supported natively in libfibre.
 */
FIBRE_PUBLIC void libfibre_register_discoverer(LibFibreCtx* ctx, const char* name, size_t name_length, on_start_discovery_cb_t on_start_discovery, on_stop_discovery_cb_t on_stop_discovery, void* cb_ctx);

/**
 * @brief Registers new TX and RX channels as part of an ongoing discovery
 * operation.
 * 
 * The channels can be closed with libfibre_close_tx() and libfibre_close_rx().
 */
FIBRE_PUBLIC void libfibre_add_channels(LibFibreCtx* ctx, LibFibreChannelDiscoveryCtx* discovery_ctx, LibFibreRxStream** tx_channel, LibFibreTxStream** rx_channel, size_t mtu);

/**
 * @brief Starts looking for Fibre objects that match the specifications.
 *
 * TODO: specify if specs needs to remain valid for the duration of discovery.
 * 
 * @param ctx: The libfibre context that was obtained from libfibre_open().
 * @param specs: Pointer to an ASCII string encoding the channel specifications.
 *        Must remain valid for the duration of the discovery.
 * 
 *        The specification has the format:
 *          "transport_provider1:args1;transport_provider2:args2"
 *        Transport providers are for example "usb", "serial", etc.
 *        Refer to the transport provider's documentation to see what arguments
 *        it takes.
 * 
 *        Example:
 *          "usb:idVendor=0x1209,idVendor=0x0d32;serial:path=/dev/ttyACM0"
 *        This will look for channels on USB devices with VID:PID 1209:0d32 and
 *        on the serial port /dev/ttyACM0.
 * @param on_found_object: Invoked for every object that is found. Objects are
 *        first passed to the construct_object() callback of libfibre_open()
 *        before they are passed to this callback. An application should use
 *        the destroy_object() callback of the libfibre_open() function to
 *        detect the loss of objects.
 * @param on_stopped: Invoked when the discovery stops for any reason, including
 *        a corresponding call to libfibre_stop_discovery().
 * @param cb_ctx: Arbitrary user data passed to the callbacks.
 * @returns: An opaque handle which should be passed to libfibre_stop_discovery().
 */
FIBRE_PUBLIC void libfibre_start_discovery(LibFibreCtx* ctx,
    const char* specs, size_t specs_len, LibFibreDiscoveryCtx** handle,
    on_found_object_cb_t on_found_object,
    on_stopped_cb_t on_stopped, void* cb_ctx);

/**
 * @brief Stops an ongoing discovery process that was previously started with
 * libfibre_start_discovery().
 *
 * The discovery is stopped asynchronously. That means it must still be
 * considered ongoing until the on_stopped callback which was passed to
 * libfibre_start_discovery() is invoked. Once this callback is invoked,
 * libfibre_stop_discovery() must no longer be called.
 */
FIBRE_PUBLIC void libfibre_stop_discovery(LibFibreCtx* ctx, LibFibreDiscoveryCtx* discovery_ctx);

/**
 * @brief Subscribes to changes on the interface.
 * 
 * All functions and attributes which are already part of the interface by the
 * time this function is called are also announced to the subscriber.
 * 
 * @param interface: An interface handle that was obtained in the callback of
 *        libfibre_start_discovery().
 * @param on_attribute_added: Invoked when an attribute is added to the
 *        interface.
 * @param on_attribute_removed: Invoked when an attribute is removed from the
 *        interface, including when the interface is being torn down. This is
 *        called exactly once for every call to on_attribute_added().
 * @param on_function_added: Invoked when a function is added to the
 *        interface. The input_names, input_codecs, output_names and
 *        output_codecs arguments are null terminated lists of null terminated
 *        strings. The name buffer and the four lists are only valid for the
 *        duration of the callback and must not be freed by the application.
 *        The function handle remains valid until the corresponding call to
 *        on_function_removed().
 * @param on_function_removed: Invoked when a function is removed from the
 *        interface, including when the interface is being torn down. This is
 *        called exactly once for every call to on_function_added().
 * @param cb_ctx: Arbitrary user data passed to the callbacks.
 */
FIBRE_PUBLIC void libfibre_subscribe_to_interface(LibFibreInterface* interface,
    on_attribute_added_cb_t on_attribute_added,
    on_attribute_removed_cb_t on_attribute_removed,
    on_function_added_cb_t on_function_added,
    on_function_removed_cb_t on_function_removed,
    void* cb_ctx);

/**
 * @brief Returns the object that corresponds the the specified attribute of
 * another object.
 * 
 * This function runs purely locally and therefore returns a result immediately.
 * 
 * TODO: it might be useful to allow this operation to go through to the remote
 * device.
 * TODO: Specify whether the returned object handle must be identical for
 * repeated calls.
 * 
 * @param parent_obj: An object handle that was obtained in the callback of
 *        libfibre_start_discovery() or from a previous call to
 *        libfibre_get_attribute().
 * @param attr: An attribute handle that was obtained in the on_attribute_added()
 *        callback of libfibre_subscribe_to_interface().
 * @param child_obj_ptr: If and only if the function succeeds, the variable that
 *        this argument points to is set to the requested subobject. The returned
 *        object handle is only guaranteed to remain valid until the next
 *        iteration of the libfibre event loop or until any other libfibre
 *        function (other than libfibre_ref_obj()) is invoked. If the
 *        application intends to keep the object handle around it must call
 *        libfibre_ref_obj() immediately.
 * @returns: kFibreOk or kFibreInvalidArgument
 */
FIBRE_PUBLIC FibreStatus libfibre_get_attribute(LibFibreObject* parent_obj, LibFibreAttribute* attr, LibFibreObject** child_obj_ptr);

/**
 * @brief Starts a remote procedure call.
 * 
 * This function returns a TX/RX pair of streams that can be used by the
 * application to send inputs to the remote procedure and receive outputs from
 * it.
 * 
 * libfibre is not required to send anything on the underlying transport
 * layer(s) until a TX operation is started on the call's tx_stream. This means
 * that for functions with no input the application must start a zero-length
 * operation for the call to be started.
 * 
 * The operation must be considered in progress until on_completed is invoked.
 * This usually happens directly after both the tx_stream and rx_stream are
 * closed (or failed), either by the server or due to a call to
 * libfibre_cancel_call().
 * 
 * @param obj: An object handle that was obtained in the callback of
 *        libfibre_start_discovery() or from a call to libfibre_get_attribute().
 * @param func: A function handle that was obtained in the on_function_added()
 *        callback of libfibre_subscribe_to_interface().
 * @param handle: The variable being pointed to by this argument is set to a
 *        handle that can be passed to libfibre_cancel_call() to cancel the
 *        started call. If on_complete() is invoked directly during
 *        libfibre_start_call() then the handle variable is not updated later
 *        than that invokation.
 * @param tx_stream: The variable being pointed to by this argument is set to
 *        a TX stream handle that can be used to send data on this call.
 *        This handle remains valid until the application calls
 *        libfibre_end_call().
 * @param rx_stream: The variable being pointed to by this argument is set to
 *        an RX stream handle that can be used to receive data on this call.
 *        This handle remains valid until the application calls
 *        libfibre_end_call().
 * @param on_completed: Called when the call completes, whether successful or
 *        not.
 * @param cb_ctx: An opaque handle which will be passed to on_completed().
 */
FIBRE_PUBLIC void libfibre_start_call(LibFibreObject* obj, LibFibreFunction* func, LibFibreCallContext** handle, LibFibreTxStream** tx_stream, LibFibreRxStream** rx_stream, on_call_completed_cb_t on_completed, void* cb_ctx);

/**
 * @brief Ends an ongoing function call.
 * 
 * Note that this does not request semantic cancellation (or reversal) of
 * actions triggered by this call.
 * 
 * The application must still wait for the on_complete callback to be called
 * before the call can be considered finished.
 * In the meantime if the TX and/or RX stream of this call is still open then
 * it is closed.
 * libfibre_end_call must not be called twice for the same call.
 * The completion callback may be called with kFibreCancelled or any other
 * status.
 * 
 * @param handle: The function call handle that was obtained by a call to
 *        libfibre_start_call().
 */
FIBRE_PUBLIC void libfibre_end_call(LibFibreCallContext* handle);

/**
 * @brief Starts sending data on the specified TX stream.
 * 
 * The TX operation must be considered in progress until the on_completed
 * callback is called. Until then the application must not start another TX
 * operation on the same stream. In the meantime the application can call
 * libfibre_cancel_tx() at any time to abort the operation.
 * 
 * @param tx_stream: The stream on which to send data.
 * @param tx_buf: The buffer to transmit. Must remain valid until the operation
 *        completes.
 * @param tx_len: Length of tx_buf.
 * @param on_completed: Called when the operation completes, whether successful
 *        or not.
 * @param ctx: Arbitrary user data passed to the on_completed callback.
 */
FIBRE_PUBLIC void libfibre_start_tx(LibFibreTxStream* tx_stream, const uint8_t* tx_buf, size_t tx_len, on_tx_completed_cb_t on_completed, void* ctx);

/**
 * @brief Cancels an ongoing TX operation.
 * 
 * Must only be called if there is actually a TX operation in progress for which
 * cancellation has not yet been requested.
 * The application must still wait for the on_complete callback to be called
 * before the operation can be considered finished. The completion callback may
 * be called with kFibreCancelled or any other status.
 * 
 * TODO: specify if streams can be restarted (current doc of on_tx_completed_cb_t implies no)
 * 
 * @param tx_stream: The TX stream on which to cancel the ongoing TX operation.
 */
FIBRE_PUBLIC void libfibre_cancel_tx(LibFibreTxStream* tx_stream);

/**
 * @brief Permanently close TX stream.
 * 
 * Must not be called while a transfer is ongoing.
 */
FIBRE_PUBLIC void libfibre_close_tx(LibFibreTxStream* tx_stream, FibreStatus status);

/**
 * @brief Starts receiving data on the specified RX stream.
 * 
 * The RX operation must be considered in progress until the on_completed
 * callback is called. Until then the application must not start another RX
 * operation on the same stream. In the meantime the application can call
 * libfibre_cancel_rx() at any time to abort the operation.
 * 
 * @param rx_stream: The stream on which to receive data.
 * @param rx_buf: The buffer to receive to. Must remain valid until the
 *        operation completes.
 * @param rx_len: Length of rx_buf.
 * @param on_completed: Called when the operation completes, whether successful
 *        or not.
 * @param ctx: Arbitrary user data passed to the on_completed callback.
 */
FIBRE_PUBLIC void libfibre_start_rx(LibFibreRxStream* rx_stream, uint8_t* rx_buf, size_t rx_len, on_rx_completed_cb_t on_completed, void* ctx);

/**
 * @brief Cancels an ongoing RX operation.
 * 
 * Must only be called if there is actually a RX operation in progress for which
 * cancellation has not yet been requested.
 * The application must still wait for the on_complete callback to be called
 * before the operation can be considered finished. The completion callback may
 * be called with kFibreCancelled or any other status.
 * 
 * TODO: specify if streams can be restarted (current doc of on_rx_completed_cb_t implies no)
 * 
 * @param rx_stream: The RX stream on which to cancel the ongoing RX operation.
 */
FIBRE_PUBLIC void libfibre_cancel_rx(LibFibreRxStream* rx_stream);

/**
 * @brief Permanently close RX stream.
 * 
 * Must not be called while a transfer is ongoing.
 */
FIBRE_PUBLIC void libfibre_close_rx(LibFibreRxStream* rx_stream, FibreStatus status);

#ifdef __cplusplus
}
#endif

#endif // __LIBFIBRE_H