
#include "legacy_object_client.hpp"
#include "legacy_protocol.hpp"
#include "include/fibre/simple_serdes.hpp"
#include "logging.hpp"
#include "print_utils.hpp"
#include "include/fibre/crc.hpp"
#include <variant>
#include <algorithm>

DEFINE_LOG_TOPIC(LEGACY_OBJ);
USE_LOG_TOPIC(LEGACY_OBJ);

using namespace fibre;

struct json_error {
    const char* ptr;
    std::string str;
};

struct json_value;
using json_list = std::vector<std::shared_ptr<json_value>>;
using json_dict = std::vector<std::pair<std::shared_ptr<json_value>, std::shared_ptr<json_value>>>;
using json_value_variant = std::variant<std::string, int, json_list, json_dict, json_error>;

struct json_value : json_value_variant {
    //json_value(const json_value_variant& v) : json_value_variant{v} {}
    template<typename T> json_value(T&& arg) : json_value_variant{std::forward<T>(arg)} {}
    //json_value_variant v;
};

// helper functions
bool json_is_str(json_value val) { return val.index() == 0; }
bool json_is_int(json_value val) { return val.index() == 1; }
bool json_is_list(json_value val) { return val.index() == 2; }
bool json_is_dict(json_value val) { return val.index() == 3; }
bool json_is_err(json_value val) { return val.index() == 4; }
std::string json_as_str(json_value val) { return std::get<0>(val); }
int json_as_int(json_value val) { return std::get<1>(val); }
json_list json_as_list(json_value val) { return std::get<2>(val); }
json_dict json_as_dict(json_value val) { return std::get<3>(val); }
json_error json_as_err(json_value val) { return std::get<4>(val); }

json_value json_make_error(const char* ptr, std::string str) {
    return {json_error{ptr, str}};
}


void json_skip_whitespace(const char** begin, const char* end) {
    while (*begin < end && std::isspace(**begin)) {
        (*begin)++;
    }
}

bool json_comp(const char* begin, const char* end, char c) {
    return begin < end && *begin == c;
}

json_value json_parse(const char** begin, const char* end) {
    // skip whitespace

    if (*begin >= end) {
        return json_make_error(*begin, "expected value but got EOF");
    }

    if (json_comp(*begin, end, '{')) {
        // parse dict
        (*begin)++; // consume leading '{'
        json_dict dict;
        bool expect_comma = false;

        json_skip_whitespace(begin, end);
        while (!json_comp(*begin, end, '}')) {
            if (expect_comma) {
                if (!json_comp(*begin, end, ',')) {
                    return json_make_error(*begin, "expected ',' or '}'");
                }
                (*begin)++; // consume comma
                json_skip_whitespace(begin, end);
            }
            expect_comma = true;

            // Parse key-value pair
            json_value key = json_parse(begin, end);
            if (json_is_err(key)) return key;
            json_skip_whitespace(begin, end);
            if (!json_comp(*begin, end, ':')) {
                return json_make_error(*begin, "expected :");
            }
            (*begin)++;
            json_value val = json_parse(begin, end);
            if (json_is_err(val)) return val;
            dict.push_back({std::make_shared<json_value>(key), std::make_shared<json_value>(val)});

            json_skip_whitespace(begin, end);
        }

        (*begin)++;
        return {dict};

    } else if (json_comp(*begin, end, '[')) {
        // parse list
        (*begin)++; // consume leading '['
        json_list list;
        bool expect_comma = false;

        json_skip_whitespace(begin, end);
        while (!json_comp(*begin, end, ']')) {
            if (expect_comma) {
                if (!json_comp(*begin, end, ',')) {
                    return json_make_error(*begin, "expected ',' or ']'");
                }
                (*begin)++; // consume comma
                json_skip_whitespace(begin, end);
            }
            expect_comma = true;

            // Parse item
            json_value val = json_parse(begin, end);
            if (json_is_err(val)) return val;
            list.push_back(std::make_shared<json_value>(val));

            json_skip_whitespace(begin, end);
        }

        (*begin)++; // consume trailing ']'
        return {list};

    } else if (json_comp(*begin, end, '"')) {
        // parse string
        (*begin)++; // consume leading '"'
        std::string str;

        while (!json_comp(*begin, end, '"')) {
            if (*begin >= end) {
                return json_make_error(*begin, "expected '\"' but got EOF");
            }
            if (json_comp(*begin, end, '\\')) {
                return json_make_error(*begin, "escaped strings not supported");
            }
            str.push_back(**begin);
            (*begin)++;
        }

        (*begin)++; // consume trailing '"'
        return {str};

    } else if (std::isdigit(**begin)) {
        // parse int

        std::string str;
        while (*begin < end && std::isdigit(**begin)) {
            str.push_back(**begin);
            (*begin)++;
        }

        return {std::stoi(str)}; // note: this can throw an exception if the int is too long

    } else {
        return json_make_error(*begin, "unexpected character '" + std::string(*begin, *begin + 1) + "'");
    }
}

json_value json_dict_find(json_dict dict, std::string key) {
    auto it = std::find_if(dict.begin(), dict.end(),
        [&](std::pair<std::shared_ptr<json_value>, std::shared_ptr<json_value>>& kv){
            return json_is_str(*kv.first) && json_as_str(*kv.first) == key;
        });
    return (it == dict.end()) ? json_make_error(nullptr, "key not found") : *it->second;
}

std::unordered_map<std::string, size_t> codecs = {
    {"bool", 1},
    {"int8", 1},
    {"uint8", 1},
    {"int16", 2},
    {"uint16", 2},
    {"int32", 4},
    {"uint32", 4},
    {"int64", 8},
    {"uint64", 8},
    {"float", 4},
    {"endpoint_ref", 4}
};

size_t get_codec_size(std::string codec) {
    auto it = codecs.find(codec);
    return (it == codecs.end()) ? 0 : it->second;
}

std::vector<LegacyFibreArg> parse_arglist(const json_value& list_val) {
    std::vector<LegacyFibreArg> arglist;

    for (auto& arg : json_is_list(list_val) ? json_as_list(list_val) : json_list()) {
        if (!json_is_dict(*arg)) {
            FIBRE_LOG(W) << "arglist is invalid";
            continue;
        }
        auto dict = json_as_dict(*arg);
        
        json_value name_val = json_dict_find(dict, "name");
        json_value id_val = json_dict_find(dict, "id");
        json_value type_val = json_dict_find(dict, "type");

        if (!json_is_str(name_val) || !json_is_int(id_val) || ((int)(size_t)json_as_int(id_val) != json_as_int(id_val)) || !json_is_str(type_val)) {
            FIBRE_LOG(W) << "arglist is invalid";
            continue;
        }

        arglist.push_back({
            json_as_str(name_val),
            json_as_str(type_val),
            (size_t)json_as_int(id_val),
            get_codec_size(json_as_str(type_val))
        });
    }

    return arglist;
}

void LegacyObjectClient::start(Completer<LegacyObjectClient*, std::shared_ptr<LegacyObject>>& on_found_root_object, Completer<LegacyObjectClient*>& on_lost_root_object) {
    FIBRE_LOG(D) << "start";
    on_found_root_object_ = &on_found_root_object;
    on_lost_root_object_ = &on_lost_root_object;
    json_.clear();
    receive_more_json();
}

void LegacyObjectClient::start_call(size_t ep_num, LegacyFibreFunction* func, CallContext** handle, Completer<FibreStatus>& completer) {
    CallContext* call = new CallContext();
    call->ep_num = ep_num;
    call->func = func;
    call->protocol_ = protocol_;
    call->completer_ = &completer;
    
    if (handle) {
        *handle = call;
    }
}

void LegacyObjectClient::cancel_call(CallContext* handle) {
    if (handle->op_handle_) {
        handle->protocol_->cancel_endpoint_operation(op_handle_);
        handle->cancelling_ = true;
    } else {
        handle->complete_call(kFibreCancelled);
    }
}

std::shared_ptr<FibreInterface> LegacyObjectClient::get_property_interfaces(std::string codec, bool write) {
    auto& dict = write ? rw_property_interfaces : ro_property_interfaces;

    auto it = dict.find(codec);
    if (it != dict.end()) {
        return it->second;
    }

    FibreInterface intf;
    size_t size = get_codec_size(codec);

    if (!size) {
        FIBRE_LOG(W) << "unknown size for codec " << codec;
    }
    
    intf.name = std::string{} + "fibre.Property<" + (write ? "readwrite" : "readonly") + " " + codec + ">";
    intf.functions["read"] = {0, {}, {{"value", codec, 0, size}}};
    if (write) {
        intf.functions["exchange"] = {0, {{"newval", codec, 0, size}}, {{"oldval", codec, 0, size}}};
    }
    
    return dict[codec] = std::make_shared<FibreInterface>(intf);
}

std::shared_ptr<LegacyObject> LegacyObjectClient::load_object(json_value list_val) {
    FibreInterface intf;

    if (!json_is_list(list_val)) {
        FIBRE_LOG(W) << "interface members must be a list";
        return nullptr;
    }

    for (auto& item: json_as_list(list_val)) {
        if (!json_is_dict(*item)) {
            FIBRE_LOG(W) << "expected dict";
            continue;
        }
        auto dict = json_as_dict(*item);

        json_value type = json_dict_find(dict, "type");
        json_value name_val = json_dict_find(dict, "name");
        std::string name = json_is_str(name_val) ? json_as_str(name_val) : "[anonymous]";

        if (json_is_str(type) && json_as_str(type) == "object") {
            std::shared_ptr<LegacyObject> subobj = load_object(json_dict_find(dict, "members"));
            intf.attributes[name] = {subobj};

        } else if (json_is_str(type) && json_as_str(type) == "function") {
            json_value id = json_dict_find(dict, "id");
            if (!json_is_int(id) || ((int)(size_t)json_as_int(id) != json_as_int(id))) {
                continue;
            }
            intf.functions[name] = {
                (size_t)json_as_int(id),
                parse_arglist(json_dict_find(dict, "inputs")),
                parse_arglist(json_dict_find(dict, "outputs")),
            };

        } else if (json_is_str(type) && json_as_str(type) == "json") {
            // Ignore

        } else if (json_is_str(type)) {
            std::string type_str = json_as_str(type);
            json_value access = json_dict_find(dict, "access");
            std::string access_str = json_is_str(access) ? json_as_str(access) : "r";
            bool can_write = access_str.find('w') != std::string::npos;

            json_value id = json_dict_find(dict, "id");
            if (!json_is_int(id) || ((int)(size_t)json_as_int(id) != json_as_int(id))) {
                continue;
            }

            LegacyObject subobj{
                .client = this,
                .ep_num = (size_t)json_as_int(id),
                .intf = get_property_interfaces(type_str, can_write),
                .known_to_application = false
            };
            auto subobj_ptr = std::make_shared<LegacyObject>(subobj);
            objects_.push_back(subobj_ptr);
            intf.attributes[name] = {subobj_ptr};

        } else {
            FIBRE_LOG(W) << "unsupported codec";
        }
    }

    LegacyObject obj{
        .client = this,
        .ep_num = 0,
        .intf = std::make_shared<FibreInterface>(intf),
        .known_to_application = false
    };
    auto obj_ptr = std::make_shared<LegacyObject>(obj);
    objects_.push_back(obj_ptr);
    return obj_ptr;
}

void LegacyObjectClient::receive_more_json() {
    write_le<uint32_t>(json_.size(), tx_buf_);
    json_.resize(json_.size() + 1024);
    bufptr_t rx_buf = {json_.data() + json_.size() - 1024, json_.data() + json_.size()};
    protocol_->start_endpoint_operation(0, tx_buf_, rx_buf, &op_handle_, *this);
}

void LegacyObjectClient::complete(EndpointOperationResult result) {
    // The JSON read operation completed

    op_handle_ = 0;

    if (result.status == kStreamCancelled) {
        return;
    } else if (result.status == kStreamClosed) {
        return;
    } else if (result.status != kStreamOk) {
        FIBRE_LOG(W) << "JSON read operation failed"; // TODO: add retry logic
        return;
    }

    size_t n_received = result.rx_end - json_.data() - json_.size() + 1024;
    json_.resize(json_.size() - 1024 + n_received);

    if (n_received) {
        receive_more_json();

    } else {

        FIBRE_LOG(D) << "received JSON of length " << json_.size();
        //FIBRE_LOG(D) << "JSON: " << str{json_.data(), json_.data() + json_.size()};

        const char *begin = reinterpret_cast<const char*>(json_.data());
        auto val = json_parse(&begin, begin + json_.size());

        if (json_is_err(val)) {
            size_t pos = json_as_err(val).ptr - reinterpret_cast<const char*>(json_.data());
            FIBRE_LOG(E) << "JSON parsing error: " << json_as_err(val).str << " at position " << pos;
            return;
        } else if (!json_is_list(val)) {
            FIBRE_LOG(E) << "JSON data must be a list";
            return;
        }

        FIBRE_LOG(D) << "sucessfully parsed JSON";
        root_obj_ = load_object(val);
        json_crc_ = calc_crc16<CANONICAL_CRC16_POLYNOMIAL>(PROTOCOL_VERSION, json_.data(), json_.size());
        if (root_obj_) {
            safe_complete(on_found_root_object_, this, root_obj_);
        }
    }
}

void LegacyObjectClient::CallContext::start_write(cbufptr_t buffer, TransferHandle* handle, Completer<WriteResult>& completer) {
    if (tx_completer_) {
        FIBRE_LOG(W) << "TX operation already in progress";
        completer.complete({kStreamError, buffer.begin()});
        return;
    }

    tx_completer_ = &completer;

    if (handle) {
        *handle = reinterpret_cast<uintptr_t>(this);
    }

    if (ep_num) {
        // Single-endpoint function
        if (progress == 0) {
            protocol_->start_endpoint_operation(ep_num, buffer, rx_buf_, &op_handle_, *this);
        } else {
            safe_complete(tx_completer_, {kStreamClosed, buffer.begin()});
        }

    } else {
        // Multi-endpoint function (deprecated)
        if (progress < func->inputs.size()) {
            // Write input arg
            size_t argnum = progress;
            if (buffer.size() < func->inputs[argnum].size) {
                FIBRE_LOG(W) << "TX granularity too small";
                safe_complete(tx_completer_, {kStreamError, buffer.begin()});
            } else {
                protocol_->start_endpoint_operation(func->inputs[argnum].ep_num,
                    buffer.take(func->inputs[argnum].size), {}, &op_handle_, *this);
            }
        } else if (progress == func->inputs.size()) {
            // Trigger function
            protocol_->start_endpoint_operation(func->ep_num, buffer.take(0), {}, &op_handle_, *this);
        } else {
            safe_complete(tx_completer_, {kStreamClosed, buffer.begin()});
        }
    }
}

void LegacyObjectClient::CallContext::cancel_write(TransferHandle transfer_handle) {
    // not implemented
}

void LegacyObjectClient::CallContext::start_read(bufptr_t buffer, TransferHandle* handle, Completer<ReadResult>& completer) {
    if (rx_completer_) {
        FIBRE_LOG(W) << "RX operation already in progress";
        completer.complete({kStreamError, buffer.begin()});
        return;
    }

    rx_completer_ = &completer;

    if (handle) {
        *handle = reinterpret_cast<uintptr_t>(this);
    }

    if (ep_num) {
        // Single-endpoint function
        if (progress == 0 && !op_handle_) {
            // Transfer has not yet started. Prepare RX buffer for when it starts.
            rx_buf_ = buffer;
        } else {
            // Transfer has already started or completed. Cannot start RX anymore.
            safe_complete(rx_completer_, {kStreamClosed, buffer.begin()});
        }

    } else {
        // Multi-endpoint function (deprecated)
        if (progress <= func->inputs.size()) {
            // Not yet in the receive phase. Store the RX pointer for later use.
            rx_buf_ = buffer;
        } else if (progress < func->inputs.size() + 1 + func->outputs.size()) {
            // Read output arg
            size_t argnum = progress - func->inputs.size() - 1;
            if (buffer.size() < func->outputs[argnum].size) {
                FIBRE_LOG(W) << "RX granularity too small";
                safe_complete(rx_completer_, {kStreamError, buffer.begin()});
            } else {
                protocol_->start_endpoint_operation(func->inputs[argnum].ep_num,
                    {}, buffer.take(func->outputs[argnum].size), &op_handle_, *this);
            }
        } else {
            safe_complete(rx_completer_, {kStreamClosed, buffer.begin()});
        }
    }
}

void LegacyObjectClient::CallContext::cancel_read(TransferHandle transfer_handle) {
    // not implemented
}

void LegacyObjectClient::CallContext::complete(EndpointOperationResult result) {
    if (result.status == kStreamCancelled || (result.status == kStreamOk && cancelling_)) {
        safe_complete(tx_completer_, {kStreamCancelled, result.tx_end});
        safe_complete(rx_completer_, {kStreamCancelled, result.rx_end});
        complete_call(kFibreCancelled);
        return;
    } else if (result.status == kStreamClosed) {
        safe_complete(tx_completer_, {kStreamClosed, result.tx_end});
        safe_complete(rx_completer_, {kStreamClosed, result.rx_end});
        complete_call(kFibreClosed);
        return;
    } else if (result.status != kStreamOk) {
        FIBRE_LOG(W) << "endpoint operation failed"; // TODO: add retry logic
        safe_complete(tx_completer_, {kStreamError, result.tx_end});
        safe_complete(rx_completer_, {kStreamError, result.rx_end});
        complete_call(kFibreInternalError);
        return;
    }

    progress++;

    if (ep_num) {
        // Single-endpoint function
        if (progress == 1) {
            safe_complete(tx_completer_, {kStreamClosed, result.tx_end});
            safe_complete(rx_completer_, {kStreamClosed, result.rx_end});
            complete_call(kFibreOk);
        }

    } else {
        // Multi-endpoint function (deprecated)

        if (progress < func->inputs.size() + 1) {
            safe_complete(tx_completer_, {kStreamOk, result.tx_end});
        } else if (progress == func->inputs.size() + 1) {
            safe_complete(tx_completer_, {kStreamClosed, result.tx_end});
            // If the application already prepared an RX operation complete this
            // operation with length 0 to make the application restart this
            // operation.
            safe_complete(rx_completer_, {kStreamOk, rx_buf_.begin()});
        } else if (progress < func->inputs.size() + 1 + func->outputs.size()) {
            safe_complete(rx_completer_, {kStreamOk, result.rx_end});
        } else if (progress == func->inputs.size() + 1 + func->outputs.size()) {
            safe_complete(rx_completer_, {kStreamClosed, result.rx_end});
            complete_call(kFibreOk);
        } else {
            FIBRE_LOG(W) << "progress is further than expected";
        }
    }
}

void LegacyObjectClient::CallContext::complete_call(FibreStatus result) {
    safe_complete(completer_, result);
    delete this;
}
