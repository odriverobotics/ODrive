#ifndef __FIBRE_LEGACY_OBJECT_MODEL_HPP
#define __FIBRE_LEGACY_OBJECT_MODEL_HPP

#include <fibre/async_stream.hpp>
#include <unordered_map>
#include <vector>
#include <memory>
#include <string>
#include <fibre/callback.hpp>
#include <fibre/cpp_utils.hpp> // std::variant and std::optional C++ backport
#include <fibre/fibre.hpp>

struct json_value;

namespace fibre {

struct EndpointOperationResult {
    StreamStatus status;
    const uint8_t* tx_end;
    uint8_t* rx_end;
};

// Lower 16 bits are the seqno. Upper 16 bits are all 1 for valid handles
// (such that seqno 0 doesn't cause the handle to be 0)
using EndpointOperationHandle = uint32_t;

struct LegacyProtocolPacketBased;

struct LegacyFibreArg {
    std::string name;
    std::string protocol_codec;
    std::string app_codec;
    size_t protocol_size;
    size_t app_size;
    size_t ep_num;
};

struct LegacyObject;

struct LegacyFunction : Function {
    LegacyFunction(std::vector<LegacyFibreArg> inputs, std::vector<LegacyFibreArg> outputs)
        : ep_num(0), obj_(nullptr), inputs(inputs), outputs(outputs) {}
    LegacyFunction(size_t ep_num, LegacyObject* obj, std::vector<LegacyFibreArg> inputs, std::vector<LegacyFibreArg> outputs)
        : ep_num(ep_num), obj_(obj), inputs(inputs), outputs(outputs) {}

    std::optional<CallBufferRelease>
    call(void**, CallBuffers, Callback<std::optional<CallBuffers>, CallBufferRelease>) final;

    size_t ep_num; // 0 for property read/write/exchange functions
    LegacyObject* obj_; // null for property read/write/exchange functions (all other functions are associated with one object only)
    std::vector<LegacyFibreArg> inputs;
    std::vector<LegacyFibreArg> outputs;
};

struct FibreInterface;
class LegacyObjectClient;

struct LegacyFibreAttribute {
    std::shared_ptr<LegacyObject> object;
};

struct FibreInterface {
    std::string name;
    std::unordered_map<std::string, LegacyFunction> functions;
    std::unordered_map<std::string, LegacyFibreAttribute> attributes;
};

struct LegacyObject {
    LegacyObjectClient* client;
    size_t ep_num;
    std::shared_ptr<FibreInterface> intf;
    bool known_to_application;
};

struct LegacyCallContext {
    LegacyFunction* func_;

    size_t progress = 0; //!< 0: expecting more tx data
                         //!< [1...n_inputs]: endpoint operations for sending inputs
                         //!< n_inputs + 1: trigger endpoint operation
                         //!< [n_inputs + 2, n_inputs + 2 + n_outputs]: endpoint operations for receiving outputs
                         //!< n_inputs + 3 + n_outputs: reporting outputs to application
                         
    EndpointOperationHandle op_handle_ = 0;

    std::vector<uint8_t> tx_buf_;
    size_t tx_pos_ = 0;
    std::vector<uint8_t> rx_buf_;
    size_t rx_pos_ = 0;

    const uint8_t* app_tx_end_;
    bufptr_t app_rx_buf_;

    Callback<std::optional<CallBuffers>, CallBufferRelease> callback;
    std::optional<EndpointOperationResult> ep_result;

    LegacyObject* obj_;

    std::optional<CallBufferRelease>
    resume_from_app(CallBuffers, Callback<std::optional<CallBuffers>, CallBufferRelease>);

    void resume_from_protocol(EndpointOperationResult result);

    struct ContinueWithProtocol {
        LegacyProtocolPacketBased* client;
        size_t ep_num;
        cbufptr_t tx_buf;
        bufptr_t rx_buf;
    };

    using ContinueWithApp = CallBufferRelease;
    using ResultFromProtocol = EndpointOperationResult;
    using ResultFromApp = CallBuffers;
    struct InternalError {};

    // Returns control either to the application or to the next endpoint operation
    std::variant<ContinueWithApp, ContinueWithProtocol, InternalError> get_next_task(std::variant<ResultFromApp, ResultFromProtocol> continue_from);
};

class LegacyObjectClient {
public:
    LegacyObjectClient(LegacyProtocolPacketBased* protocol) : protocol_(protocol) {}

    void start(Callback<void, LegacyObjectClient*, std::shared_ptr<LegacyObject>> on_found_root_object, Callback<void, LegacyObjectClient*, std::shared_ptr<LegacyObject>> on_lost_root_object);
    bool transcode(cbufptr_t src, bufptr_t dst, std::string src_codec, std::string dst_codec);

    // For direct access by LegacyProtocolPacketBased and libfibre.cpp
    uint16_t json_crc_ = 0;
    Callback<void, LegacyObjectClient*, std::shared_ptr<LegacyObject>> on_lost_root_object_;
    std::shared_ptr<LegacyObject> root_obj_;
    std::vector<std::shared_ptr<LegacyObject>> objects_;
    void* user_data_; // used by libfibre to store the libfibre context pointer
    LegacyProtocolPacketBased* protocol_;

private:
    std::shared_ptr<FibreInterface> get_property_interfaces(std::string codec, bool write);
    std::shared_ptr<LegacyObject> load_object(json_value list_val);
    void receive_more_json();
    void on_received_json(EndpointOperationResult result);

    Callback<void, LegacyObjectClient*, std::shared_ptr<LegacyObject>> on_found_root_object_;
    uint8_t tx_buf_[4] = {0xff, 0xff, 0xff, 0xff};
    EndpointOperationHandle op_handle_ = 0;
    std::vector<uint8_t> json_;
    //std::vector<LegacyCallContext*> pending_calls_;
    std::unordered_map<std::string, std::shared_ptr<FibreInterface>> rw_property_interfaces;
    std::unordered_map<std::string, std::shared_ptr<FibreInterface>> ro_property_interfaces;
};

}

#endif // __FIBRE_LEGACY_OBJECT_MODEL_HPP