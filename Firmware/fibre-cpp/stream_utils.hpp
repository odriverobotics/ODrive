#ifndef __FIBRE_STREAM_UTILS_HPP
#define __FIBRE_STREAM_UTILS_HPP

#include <fibre/async_stream.hpp>
#include <string.h>

namespace fibre {

template<size_t I>
class BufferedStreamSink {
public:
    BufferedStreamSink(AsyncStreamSink& sink) : sink_(sink) {}

    /**
     * @brief Enqueues as much of the specified buffer as possible.
     * 
     * Thread safety: only one write call is allowed at a time. The write call
     * can be on a different thread from the underlying stream's event loop.
     * (TODO: this is not true yet, see comment in function)
     */
    void write(cbufptr_t buf) {
        size_t read_idx = read_idx_; // read_idx_ could change during this function

        if ((read_idx + 1) % I == write_idx_) {
            return;
        }

        // We subtract 1 from the read index because we never want the write
        // pointer to catch up with the read pointer, cause then
        // `write_idx_ == read_idx_` could mean both "full" and "empty".

        read_idx = (read_idx + I - 1) % I; 

        if (write_idx_ > read_idx) {
            size_t n_copy = std::min(I - write_idx_, buf.size());
            memcpy(buffer_ + write_idx_, buf.begin(), n_copy);
            write_idx_ = (write_idx_ + n_copy) % I;
            buf = buf.skip(n_copy);
        }

        size_t n_copy = std::min(read_idx - write_idx_, buf.size());
        memcpy(buffer_ + write_idx_, buf.begin(), n_copy);
        write_idx_ = (write_idx_ + n_copy) % I;

        //if (!__atomic_exchange_n(&is_active_, true, __ATOMIC_SEQ_CST)) {
        //    // TODO: calling the sink in here breaks the rule that async
        //    // functions must only be called on the event loop thread.
        //    // But to do that we need to implement a proper event loop where we
        //    // can enqueue calls.
        //    maybe_start_async_write();
        //}
    }

    void maybe_start_async_write() {
        if (is_active_) {
            // nothing to do
        } else if (read_idx_ < write_idx_) {
            is_active_ = true;
            sink_.start_write({buffer_ + read_idx_, buffer_ + write_idx_}, &transfer_handle_, MEMBER_CB(this, on_write_complete));
        } else if (read_idx_ > write_idx_) {
            is_active_ = true;
            sink_.start_write({buffer_ + read_idx_, buffer_ + I}, &transfer_handle_, MEMBER_CB(this, on_write_complete));
        } else {
            // nothing to do
        }
    }

private:
    void on_write_complete(WriteResult result) {
        is_active_ = false;
        transfer_handle_ = 0;

        if (result.status == kStreamOk) {
            if (result.end < buffer_ || result.end > (buffer_ + I)) {
                for (;;)
                    transfer_handle_ = 0;
            }
            read_idx_ = (result.end - buffer_) % I;
            maybe_start_async_write();
        }
    }

    uint8_t buffer_[I];

    // Both indices are in [0, I)
    // They are equal if the buffer is empty (no valid data).
    size_t write_idx_ = 0; // [0, I)
    size_t read_idx_ = 0; // [0, I)
    bool is_active_ = false;
    TransferHandle transfer_handle_ = 0;

    AsyncStreamSink& sink_;
};

/**
 * @brief Buffers up to NSlots concurrent async write requests.
 * 
 * This can be used to wrap sinks that can only handle one concurrent write
 * operation at a time but are written to by multiple independent sources.
 */
template<size_t NSlots>
class AsyncStreamSinkMultiplexer : public AsyncStreamSink {
public:
    AsyncStreamSinkMultiplexer(AsyncStreamSink& sink) : sink_(sink) {}

    void start_write(cbufptr_t buffer, TransferHandle* handle, Callback<void, WriteResult> completer) final {
        for (size_t i = 0; i < NSlots; ++i) {
            auto& [slot_in_use, slot_buf, slot_completer] = slots_[i];
            if (!__atomic_exchange_n(&slot_in_use, true, __ATOMIC_SEQ_CST)) {
                slot_buf = buffer;
                slot_completer = completer;

                if (handle) {
                    *handle = i + 1; // returning a valid handle of 0 is not a good idea
                }

                // If the underlying sink wasn't busy, start it now.
                if (active_slot_ == 0) {
                    active_slot_ = i + 1;
                    sink_.start_write(slot_buf, &transfer_handle_, MEMBER_CB(this, on_write_complete));
                }

                return;
            }
        }

        if (handle) {
            *handle = 0;
        }
        completer.invoke({kStreamError, buffer.begin()});
    }

    void cancel_write(TransferHandle transfer_handle) final {
        if (transfer_handle == active_slot_) {
            // This transfer is the one that the underlying sink is busy with.
            sink_.cancel_write(transfer_handle_);
        } else {
            // This transfer is only enqueued but not yet started.
            auto& [slot_in_use, slot_buf, slot_completer] = slots_[transfer_handle - 1];
            auto completer = slot_completer;
            auto end = slot_buf.end();
            slot_in_use = false;
            completer.invoke_and_clear({kStreamCancelled, end});
        }
    }

private:
    void on_write_complete(fibre::WriteResult result) {
        transfer_handle_ = 0;

        auto& [slot_in_use, slot_buf, slot_completer] =  slots_[active_slot_ - 1];
        (void) slot_buf;
        auto completer = slot_completer;
        slot_in_use = false;

        completer.invoke_and_clear(result);

        // Select new slot before announcing completion of the old
        size_t active_slot = 0;
        for (size_t i = 0; i < NSlots; ++i) {
            auto& [slot_in_use, slot_buf, slot_completer] = slots_[i];
            (void) slot_buf;
            (void) slot_completer;
            if (slot_in_use) {
                active_slot = i + 1;
                break;
            }
        }

        // Start next slot
        active_slot_ = active_slot;
        if (active_slot) {
            auto& [slot_in_use, slot_buf, slot_completer] = slots_[active_slot - 1];
            (void) slot_in_use;
            (void) slot_completer;
            sink_.start_write(slot_buf, &transfer_handle_, MEMBER_CB(this, on_write_complete));
        }
    }
    
    AsyncStreamSink& sink_;
    std::tuple<bool, cbufptr_t, Callback<void, WriteResult>> slots_[NSlots];
    size_t active_slot_ = 0;
    TransferHandle transfer_handle_ = 0;
};

}

#endif // __FIBRE_STREAM_UTILS_HPP
