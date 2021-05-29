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
struct LibFibreDiscoveryCtx;
struct LibFibreCallContext;
struct LibFibreObject;
struct LibFibreInterface;
struct LibFibreFunction;
struct LibFibreAttribute;
struct LibFibreTxStream;
struct LibFibreRxStream;
struct LibFibreDomain;

// This enum must remain identical to fibre::Status.
enum LibFibreStatus {
    kFibreOk,
    kFibreBusy, //<! The request will complete asynchronously
    kFibreCancelled, //!< The operation was cancelled due to a request by the application or the remote peer
    kFibreClosed, //!< The operation has finished orderly or shall be finished orderly
    kFibreInvalidArgument, //!< Bug in the application
    kFibreInternalError, //!< Bug in the local fibre implementation
    kFibreProtocolError, //!< A remote peer is misbehaving (indicates bug in the remote peer)
    kFibreHostUnreachable, //!< The remote peer can no longer be reached
    //kFibreInsufficientData, // maybe we will introduce this to tell the caller that the granularity of the data is too small
};

struct LibFibreVersion {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
};

typedef int (*post_cb_t)(void (*callback)(void*), void* cb_ctx);
typedef int (*register_event_cb_t)(int fd, uint32_t events, void (*callback)(void*, uint32_t), void* cb_ctx);
typedef int (*deregister_event_cb_t)(int fd);
typedef struct EventLoopTimer* (*call_later_cb_t)(float delay, void (*callback)(void*), void* cb_ctx);
typedef int (*cancel_timer_cb_t)(struct EventLoopTimer* timer);

struct LibFibreEventLoop {
    /**
     * @brief Called by libfibre when it wants the application to run a callback
     * on the application's event loop.
     * 
     * This is the only callback that libfibre can invoke from a different
     * thread than the event loop thread itself. The application must ensure
     * that this callback is thread-safe.
     * This allows libfibre to run other threads internally while keeping
     * threading promises made to the application.
     */
    post_cb_t post;

    /**
     * @brief TODO: this is a Unix specific callback. Need to use IOCP on Windows.
     */
    register_event_cb_t register_event;

    /**
     * @brief TODO: this is a Unix specific callback. Need to use IOCP on Windows.
     */
    deregister_event_cb_t deregister_event;

    /**
     * @brief Called by libfibre to ask the application to call a certain
     * callback after a certain amount of time.
     * 
     * The callback must be invoked on the same thread on which libfibre_open()
     * was called. The application should return an opaque handle that
     * libfibre can use to cancel the timer.
     */
    call_later_cb_t call_later;

    /**
     * @brief Called by libfibre to ask the application to cancel a callback
     * timer previously enqueued with call_later().
     */
    cancel_timer_cb_t cancel_timer;
};

/**
 * @brief on_start_discovery callback type for libfibre_register_backend().
 * 
 * For every channel pair that the application finds that matches the filter of
 * this discoverer the application should call libfibre_add_channels().
 * 
 * @param discovery_handle: An opaque handle that libfibre will pass to the
 *        corresponding on_stop_discovery callback to stop the discovery.
 * @param specs, specs_length: The specs string that specifies discoverer-specific
 *        filter parameters.
 */
typedef void (*on_start_discovery_cb_t)(void* ctx, LibFibreDomain* domain, const char* specs, size_t specs_length);
typedef void (*on_stop_discovery_cb_t)(void* ctx, LibFibreDomain* domain);

/**
 * @brief on_found_object callback type for libfibre_start_discovery().
 * @param obj: The object handle.
 * @param intf: The interface handle. Valid for as long as any handle of an
 *        object that implements it is valid.
 */
typedef void (*on_found_object_cb_t)(void*, LibFibreObject* obj, LibFibreInterface* intf);

/**
 * @brief on_lost_object callback type for libfibre_start_discovery().
 */
typedef void (*on_lost_object_cb_t)(void*, LibFibreObject* obj);

typedef void (*on_stopped_cb_t)(void*, LibFibreStatus);

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

/**
 * @brief Callback type for libfibre_call().
 * 
 * For an overview of the coroutine call control flow see libfibre_call().
 * 
 * @param ctx: The context pointer that was passed to libfibre_call().
 * @param tx_end: End of the range of data that was accepted by libfibre. This
 *        is always in the interval [tx_buf, tx_buf + tx_len] where `tx_buf` and
 *        `tx_len` are the arguments of the corresponding libfibre_call() call.
 * @param tx_end: End of the range of data that was returned by libfibre. This
 *        is always in the interval [rx_buf, rx_buf + rx_len] where `rx_buf` and
 *        `rx_len` are the arguments of the corresponding libfibre_call() call.
 * @param tx_buf: The application should set this to the next buffer to
 *        transmit. The buffer must remain valid until the next callback
 *        invokation.
 * @param tx_len: The length of tx_buf. Must be zero if tx_buf is NULL.
 * @param rx_buf: The application should set this to the buffer into which data
 *        should be written. The buffer must remain allocated until the next
 *        callback invokation.
 * @param rx_len: The length of rx_buf. Must be zero if rx_buf is NULL.
 * 
 * @retval kFibreOk: The application set tx_buf and rx_buf to valid or empty
 *         buffers and libfibre should invoke the callback again when it has
 *         made progress.
 * @retval kFibreBusy: The application cannot provide a new tx_buf or rx_buf at
 *         the moment. The application will eventually call libfibre_call() for
 *         this coroutine call again.
 * @retval kFibreClosed: The application may have returned non-empty buffers and
 *         if libfibre manages to fully handle these buffers it shall consider
 *         the call ended.
 * @retval kFibreCancelled: The application did not set valid tx and rx buffers
 *         and libfibre should consider the call cancelled. Libfibre will not
 *         invoke the callback anymore.
 */
typedef LibFibreStatus (*libfibre_call_cb_t)(void* ctx,
        LibFibreStatus status,
        const unsigned char* tx_end, unsigned char* rx_end,
        const unsigned char** tx_buf, size_t* tx_len,
        unsigned char** rx_buf, size_t* rx_len);

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
typedef void (*on_tx_completed_cb_t)(void* ctx, LibFibreTxStream* tx_stream, LibFibreStatus status, const uint8_t* tx_end);

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
typedef void (*on_rx_completed_cb_t)(void* ctx, LibFibreRxStream* rx_stream, LibFibreStatus status, uint8_t* rx_end);

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
 * @param event_loop: The event loop on which libfibre will run. Some function
          of the event loop can be left unimplemented (set to NULL) depending on
          the platform and the backends used (TODO: elaborate).
          The event loop must be single threaded and all calls to libfibre must
          happen on the event loop thread.
 */
FIBRE_PUBLIC struct LibFibreCtx* libfibre_open(LibFibreEventLoop event_loop);

/**
 * @brief Closes a context that was previously opened with libfibre_open().
 *
 * This function must not be invoked before all ongoing discovery processes
 * are stopped and all channels are closed.
 */
FIBRE_PUBLIC void libfibre_close(struct LibFibreCtx* ctx);

/**
 * @brief Registers an external channel provider.
 * 
 * Libfibre starts and stops the discoverer on demand as a result of calls
 * to libfibre_start_discovery() and libfibre_stop_discovery().
 * This can be used by applications to implement transport providers which are
 * not supported natively in libfibre.
 */
FIBRE_PUBLIC void libfibre_register_backend(LibFibreCtx* ctx, const char* name,
    size_t name_length, on_start_discovery_cb_t on_start_discovery,
    on_stop_discovery_cb_t on_stop_discovery, void* cb_ctx);

/**
 * @brief Creates a communication domain from the specified spec string.
 * 
 * @param ctx: The libfibre context that was obtained from libfibre_open().
 * @param specs: Pointer to an ASCII string encoding the channel specifications.
 *        Must remain valid for the life time of the discovery.
 *        See README of the main Fibre repository for details.
 *        (https://github.com/samuelsadok/fibre/tree/devel).
 * @returns: An opaque handle which can be passed to libfibre_start_discovery().
 */
FIBRE_PUBLIC LibFibreDomain* libfibre_open_domain(LibFibreCtx* ctx,
    const char* specs, size_t specs_len);

/**
 * @brief Closes a domain that was previously opened with libfibre_open_domain().
 */
FIBRE_PUBLIC void libfibre_close_domain(LibFibreDomain* domain);

/**
 * @brief Adds new TX and RX channels to a domain.
 * 
 * The channels can be closed with libfibre_close_tx() and libfibre_close_rx().
 */
FIBRE_PUBLIC void libfibre_add_channels(LibFibreDomain* domain, LibFibreRxStream** tx_channel, LibFibreTxStream** rx_channel, size_t mtu);

/**
 * @brief Starts looking for Fibre objects that match the specifications.
 *
 * @param domain: The domain obtained from libfibre_open_domain() on which to
 *        discover objects.
 * @param on_found_object: Invoked for every matching object that is found.
 *        The application must expect the same object handle to appear more than
 *        once.
 *        libfibre increments the internal reference count of the object before
 *        this call and decrements it after the corresponding call to
 *        on_lost_object. When the reference count reaches zero the application
 *        must no longer use it. The reference count is always non-negative.
 * @param on_lost_object: Invoked when an object is lost.
 * @param on_stopped: Invoked when the discovery stops for any reason, including
 *        a corresponding call to libfibre_stop_discovery().
 * @param cb_ctx: Arbitrary user data passed to the callbacks.
 * @returns: An opaque handle which should be passed to libfibre_stop_discovery().
 */
FIBRE_PUBLIC void libfibre_start_discovery(LibFibreDomain* domain,
    LibFibreDiscoveryCtx** handle, on_found_object_cb_t on_found_object,
    on_lost_object_cb_t on_lost_object,
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
FIBRE_PUBLIC void libfibre_stop_discovery(LibFibreDiscoveryCtx* handle);

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
 *        object handle is only guaranteed to remain valid for as long as the
 *        parent object handle is valid.
 * @returns: kFibreOk or kFibreInvalidArgument
 */
FIBRE_PUBLIC LibFibreStatus libfibre_get_attribute(LibFibreObject* parent_obj, LibFibreAttribute* attr, LibFibreObject** child_obj_ptr);

/**
 * @brief Starts a remote coroutine call or continues or cancels an ongoing call.
 * 
 * A remote coroutine call can be considered a continuous exchange of the
 * following tuples:
 * 
 *  Client Application  ===== (tx_buf, rx_buf, status) ====> libfibre
 *  Client Application  <==== (tx_end, rx_end, status) ===== libfibre
 * 
 * These tuples are exchanged through the input/output arguments of
 * libfibre_call() or libfibre_call()'s callback.
 * 
 * If during an ongoing call either of the two parties is unable to respond
 * immediately it responds with kFibreBusy and will thus get the responsibility
 * to resume the call when able. kFibreCancelled can be issued by either party
 * at any time.
 * 
 * Each party must make progress during every control transfer to the other
 * party.
 * 
 * For the application this means for every call to libfibre_call() and every
 * return from libfibre_call()'s callback the arguments passed from application
 * to libfibre must satisfy at least one of the following:
 * 
 *  - The call handle is NULL
 *  - tx_len is non-zero
 *  - rx_len is non-zero
 *  - The status is different from kFibreOk
 * 
 * For libfibre this means every for return from libfibre_call() and every
 * call to libfibre_call()'s callback the arguments passed from libfibre to
 * application satisfy at least one of the following:
 * 
 *  - tx_end is larger than the corresponding tx_buf
 *  - rx_end is larger than the corresponding rx_buf
 *  - The status is different from kFibreOk
 * 
 * @param func: A function handle that was obtained in the on_function_added()
 *        callback of libfibre_subscribe_to_interface().
 * @param handle: The variable being pointed to by this argument identifies the
 *        coroutine call. If the variable is NULL it will be set to a new opaque
 *        handle. If the variable is not NULL the active function call is
 *        continued or cancelled (depending on status).
 * @param tx_buf: The buffer to transmit. If libfibre_call() returns kFibreBusy
 *        then this buffer must remain valid until `callback` is invoked.
 *        Otherwise it can be freed immediately after this call.
 * @param tx_len: Length of tx_buf. Must be zero if tx_buf is NULL.
 * @param rx_buf: The buffer into which the received data should be written. If
 *        libfibre_call() returns kFibreBusy then this buffer must remain
 *        allocated until `callback` is invoked. Otherwise it can be freed
 *        immediately after this call.
 * @param rx_len: Length of rx_buf. Must be zero if rx_buf is NULL.
 * @param tx_end: End of the range of data that was accepted by libfibre. This
 *        is always in the interval [tx_buf, tx_buf + tx_len] unless
 *        libfibre_call() returns kFibreBusy, in which case this is NULL.
 *        This value does not give any delivery guarantees.
 * @param rx_end: End of the range of data that was returned by libfibre. This
 *        is always in the interval [rx_buf, rx_buf + rx_len] unless
 *        libfibre_call() returns kFibreBusy, in which case this is NULL.
 * @param callback: Will be invoked eventually if and only if libfibre_call()
 *        returns kFibreBusy. This callback is never invoked from inside
 *        libfibre_call().
 * @param cb_ctx: An opaque application-defined handle that gets passed to
 *        `callback`.
 * 
 * @retval kFibreOk: libfibre accepted some or all of the tx_buf or filled some
 *         or all of the rx_buf with data and can immediately accept more TX
 *         data or provide more RX data.
 * @retval kFibreBusy: libfibre will complete the request asynchronously by
 *         calling `callback`. If this value is returned, then the application
 *         must not invoke libfibre_call() on the same call handle again until
 *         `callback` is invoked except for cancelling the call with a status
 *         of `kFibreCancelled`.
 * @retval kFibreClosed: the remote server completed the call and will not
 *         accept or return any more data on this call. The application must not
 *         pass the closed call context handle to libfibre_call() anymore.
 * @retval kFibreCancelled: the application's cancellation request was honored
 *         or the remote server cancelled the call. The application must not
 *         pass the cancelled call context handle to libfibre_call() anymore.
 */
FIBRE_PUBLIC LibFibreStatus libfibre_call(LibFibreFunction* func, LibFibreCallContext** handle,
        LibFibreStatus status,
        const unsigned char* tx_buf, size_t tx_len,
        unsigned char* rx_buf, size_t rx_len,
        const unsigned char** tx_end,
        unsigned char** rx_end,
        libfibre_call_cb_t callback, void* cb_ctx);

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
FIBRE_PUBLIC void libfibre_close_tx(LibFibreTxStream* tx_stream, LibFibreStatus status);

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
FIBRE_PUBLIC void libfibre_close_rx(LibFibreRxStream* rx_stream, LibFibreStatus status);

#ifdef __cplusplus
}
#endif

#endif // __LIBFIBRE_H