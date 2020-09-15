#ifndef __FIBRE_ASYNC_STREAM_HPP
#define __FIBRE_ASYNC_STREAM_HPP

#include "include/fibre/bufptr.hpp" // TODO: move this header
#include <stdint.h>

namespace fibre {

enum StreamStatus {
    kStreamOk,
    kStreamCancelled,
    kStreamClosed,
    kStreamError
};

template<typename ... TResults>
class Completer {
public:
    virtual void complete(TResults ... result) = 0;

    static Completer& get_dummy() {
        static struct DummyCompleter : Completer {
            void complete(TResults ... result) {}
        } dummy;
        return dummy;
    }
};

/**
 * @brief Safe wrapper around Completer::complete.
 * 
 * This function takes a reference to a completer pointer and only invokes the
 * completer if it's not null. Before invoking the completer, the pointer is
 * cleared.
 */
template<typename ... TResults>
static void safe_complete(Completer<TResults...>*& completer, TResults ... results) {
    Completer<TResults...>* tmp = completer;
    completer = nullptr;
    if (tmp) {
        tmp->complete(results...);
    }
}

struct ReadResult {
    StreamStatus status;

    /**
     * @brief The pointer to one position after the last byte that was
     * transferred.
     * This must always be in [buffer.begin(), buffer.end()], even if the
     * transfer was not succesful.
     * If the status is kStreamError or kStreamCancelled then the accuracy
     * of this field is not guaranteed.
     */
    unsigned char* end;
};

struct WriteResult {
    StreamStatus status;

    /**
     * @brief The pointer to one position after the last byte that was
     * transferred.
     * This must always be in [buffer.begin(), buffer.end()], even if the
     * transfer was not succesful.
     * If the status is kStreamError or kStreamCancelled then the accuracy
     * of this field is not guaranteed.
     */
    const unsigned char* end;
};

struct WriteCompleter : Completer<WriteResult> {
    virtual void on_write_finished(WriteResult result) = 0;

    void complete(WriteResult result) final {
        on_write_finished(result);
    }
};

struct ReadCompleter : Completer<ReadResult> {
    virtual void on_read_finished(ReadResult result) = 0;

    void complete(ReadResult result) final {
        on_read_finished(result);
    }
};


using TransferHandle = uintptr_t;

/**
 * @brief Base class for asynchronous stream sources.
 */
class AsyncStreamSource {
public:
    /**
     * @brief Starts a read operation. Once the read operation completes,
     * on_finished.complete() is called.
     *
     * Most implementations only allow one transfer to be active at a time.
     *
     * TODO: specify if `completer` can be called directly within this function.
     *
     * @param buffer: The buffer where the data to be written shall be fetched from.
     *        Must remain valid until `completer` is satisfied.
     * @param handle: The variable pointed to by this argument is set to an
     *        opaque transfer handle that can be passed to cancel_read() as
     *        long as the operation has not yet completed.
     *        If the completer is invoked directly from start_read() then the
     *        handle is not modified after this invokation. That means it's safe
     *        for the completion handler to reuse the handle variable.
     * @param completer: The completer that will be completed once the operation
     *        finishes, whether successful or not.
     *        Must remain valid until it is satisfied.
     */
    virtual void start_read(bufptr_t buffer, TransferHandle* handle, Completer<ReadResult>& completer) = 0;

    /**
     * @brief Cancels an operation that was previously started with start_read().
     *
     * The transfer is cancelled asynchronously and the associated completer
     * will eventually be completed with kStreamCancelled. Until then the
     * transfer must be considered still in progress and associated resources
     * must not be freed.
     * 
     * TODO: specify if an implementation is allowed to return something other
     * than kStreamCancelled when the transfer was cancelled.
     *
     * This function must not be called once the stream has started to invoke
     * the associated completion handler. It must also not be called twice for
     * the same transfer.
     */
    virtual void cancel_read(TransferHandle transfer_handle) = 0;
};

/**
 * @brief Base class for asynchronous stream sources.
 *
 * Thread-safety: Implementations are generally not required to provide thread
 * safety. Users should only call the functions of this class on the same thread
 * as the event loop on which the stream runs.
 */
class AsyncStreamSink {
public:
    /**
     * @brief Starts a write operation. Once the write operation completes,
     * on_finished.complete() is called.
     *
     * Most implementations only allow one transfer to be active at a time.
     *
     * TODO: specify if `completer` can be called directly within this function.
     *
     * @param buffer: The buffer where the data to be written shall be fetched from.
     *        Must remain valid until `completer` is satisfied.
     * @param handle: The variable pointed to by this argument is set to an
     *        opaque transfer handle that can be passed to cancel_write() as
     *        long as the operation has not yet completed.
     *        If the completer is invoked directly from start_write() then the
     *        handle is not modified after this invokation. That means it's safe
     *        for the completion handler to reuse the handle variable.
     * @param completer: The completer that will be completed once the operation
     *        finishes, whether successful or not.
     *        Must remain valid until it is satisfied.
     */
    virtual void start_write(cbufptr_t buffer, TransferHandle* handle, Completer<WriteResult>& completer) = 0;

    /**
     * @brief Cancels an operation that was previously started with start_write().
     *
     * The transfer is cancelled asynchronously and the associated completer
     * will eventually be completed with kStreamCancelled. Until then the
     * transfer must be considered still in progress and associated resources
     * must not be freed.
     * 
     * TODO: specify if an implementation is allowed to return something other
     * than kStreamCancelled when the transfer was cancelled.
     *
     * This function must not be called once the stream has started to invoke
     * the associated completion handler. It must also not be called twice for
     * the same transfer.
     */
    virtual void cancel_write(TransferHandle transfer_handle) = 0;
};

}

#endif // __FIBRE_ASYNC_STREAM_HPP