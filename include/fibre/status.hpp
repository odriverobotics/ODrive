#ifndef __FIBRE_STATUS_HPP
#define __FIBRE_STATUS_HPP

namespace fibre {

enum Status {
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

}

#endif // __FIBRE_STATUS_HPP