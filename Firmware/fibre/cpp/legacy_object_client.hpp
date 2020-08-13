#ifndef __FIBRE_LEGACY_OBJECT_MODEL_HPP
#define __FIBRE_LEGACY_OBJECT_MODEL_HPP

//#include "legacy_protocol.hpp"
#include "async_stream.hpp"
#include <fibre/libfibre.h>
#include <unordered_map>
#include <vector>
#include <memory>
#include <string>

struct json_value;

namespace fibre {

struct EndpointOperationResult {
    StreamStatus status;
    uint8_t* rx_end;
};

using EndpointOperationHandle = uint32_t;

struct LegacyProtocolPacketBased;

struct LegacyFibreArg {
    std::string name;
    std::string codec;
    size_t ep_num;
    size_t size;
};

struct LegacyFibreFunction {
    size_t ep_num; // 0 for property read/write/exchange functions
    std::vector<LegacyFibreArg> inputs;
    std::vector<LegacyFibreArg> outputs;
};

struct FibreInterface;
struct LegacyObjectClient;
struct LegacyObject;

struct LegacyFibreAttribute {
    std::shared_ptr<LegacyObject> object;
};

struct FibreInterface {
    std::string name;
    std::unordered_map<std::string, LegacyFibreFunction> functions;
    std::unordered_map<std::string, LegacyFibreAttribute> attributes;
};

struct LegacyObject {
    LegacyObjectClient* client;
    size_t ep_num;
    std::shared_ptr<FibreInterface> intf;
    bool known_to_application;
};

class LegacyObjectClient : Completer<EndpointOperationResult> {
public:
    struct CallResult {
        FibreStatus status;
        uint8_t* end;
    };
    struct CallContext {
        size_t progress = 0;
        size_t ep_num;
        cbufptr_t tx_buf;
        bufptr_t rx_buf;
        LegacyFibreFunction* func;
        Completer<CallResult>* completer;
    };

    LegacyObjectClient(LegacyProtocolPacketBased* protocol) : protocol_(protocol) {}

    void start(Completer<LegacyObjectClient*, std::shared_ptr<LegacyObject>>& on_found_root_object, Completer<LegacyObjectClient*>& on_lost_root_object);

    void start_call(size_t ep_num, LegacyFibreFunction* func, cbufptr_t input, bufptr_t output, CallContext** handle, Completer<CallResult>& completer);
    void cancel_call(CallContext* handle);

    // For direct access by LegacyProtocolPacketBased and libfibre.cpp
    uint16_t json_crc_ = 0;
    Completer<LegacyObjectClient*>* on_lost_root_object_;
    std::shared_ptr<LegacyObject> root_obj_;
    std::vector<std::shared_ptr<LegacyObject>> objects_;
    void* user_data_; // used by libfibre to store the libfibre context pointer

private:
    std::shared_ptr<FibreInterface> get_property_interfaces(std::string codec, bool write);
    std::shared_ptr<LegacyObject> load_object(json_value list_val);
    void receive_more_json();
    void complete(EndpointOperationResult result);

    LegacyProtocolPacketBased* protocol_;
    Completer<LegacyObjectClient*, std::shared_ptr<LegacyObject>>* on_found_root_object_;
    uint8_t tx_buf_[4] = {0xff, 0xff, 0xff, 0xff};
    EndpointOperationHandle op_handle_ = 0;
    std::vector<uint8_t> json_;
    CallContext* call_ = nullptr; // active call
    std::vector<CallContext*> pending_calls_;
    std::unordered_map<std::string, std::shared_ptr<FibreInterface>> rw_property_interfaces;
    std::unordered_map<std::string, std::shared_ptr<FibreInterface>> ro_property_interfaces;
};

}

#endif // __FIBRE_LEGACY_OBJECT_MODEL_HPP