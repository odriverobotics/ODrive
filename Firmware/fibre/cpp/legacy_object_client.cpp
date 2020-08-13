
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
    {"int64", 6},
    {"uint64", 6},
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

void LegacyObjectClient::start_call(size_t ep_num, LegacyFibreFunction* func, cbufptr_t input, bufptr_t output, CallContext** handle, Completer<CallResult>& completer) {
    CallContext* call = new CallContext();
    call->ep_num = ep_num;
    call->tx_buf = input;
    call->rx_buf = output;
    call->func = func;
    call->completer = &completer;

    if (op_handle_) {
        FIBRE_LOG(D) << "Call in progress. Enqueuing this call.";
        // An operation is already in progress. Enqueue this one.
        pending_calls_.push_back(call);
    } else {
        // No endpoint operation is in progress. Start this call immediately
        FIBRE_LOG(D) << "No call in progress. Starting call now.";
        call_ = call;
        complete({kStreamOk, nullptr});
    }
}

void LegacyObjectClient::cancel_call(CallContext* handle) {
    if (call_ == handle) {
        protocol_->cancel_endpoint_operation(op_handle_);
    } else {
        auto it = std::find(pending_calls_.begin(), pending_calls_.end(), handle);
        if (it != pending_calls_.end()) {
            CallContext* call = *it;
            pending_calls_.erase(it);
            safe_complete(call->completer, {kFibreCancelled, call->rx_buf.end()});
            delete call;
        }
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
    op_handle_ = 0;

    if (result.status == kStreamCancelled) {
        if (call_) {
            auto call = call_;
            call_ = nullptr;
            safe_complete(call->completer, {kFibreCancelled, call->rx_buf.end()});
            delete call;
        }
        return;
    } else if (result.status == kStreamClosed) {
        if (call_) {
            auto call = call_;
            call_ = nullptr;
            safe_complete(call->completer, {kFibreClosed, call->rx_buf.end()});
            delete call;
        }
        return;
    } else if (result.status != kStreamOk) {
        FIBRE_LOG(W) << "endpoint operation failed"; // TODO: add retry logic
        if (call_) {
            auto call = call_;
            call_ = nullptr;
            safe_complete(call->completer, {kFibreInternalError, call->rx_buf.end()});
            delete call;
        }
        return;
    }

    if (call_) {
        // The endpoint operation that completed belongs to the active call

        LegacyFibreFunction* func = call_->func;

        if (call_->ep_num && !call_->progress) {
            // Read/write/exchange property
            FIBRE_LOG(D) << "starting property transaction on " << call_->ep_num << " with tx buf len " << call_->tx_buf.size() << " and rx len " << call_->rx_buf.size();
            call_->progress++;
            protocol_->start_endpoint_operation(call_->ep_num, call_->tx_buf, call_->rx_buf, &op_handle_, *this);

        } else if (!call_->ep_num && call_->progress < func->inputs.size()) {
            // Write input arg
            size_t argnum = call_->progress;
            call_->progress++;
            cbufptr_t current_buf = call_->tx_buf.take(func->inputs[argnum].size);
            call_->tx_buf = call_->tx_buf.skip(func->inputs[argnum].size);
            protocol_->start_endpoint_operation(func->inputs[argnum].ep_num, current_buf, call_->rx_buf.take(0), &op_handle_, *this);

        } else if (!call_->ep_num && call_->progress == func->inputs.size()) {
            // Trigger
            // call_->tx_buf should be empty by now
            call_->progress++;
            protocol_->start_endpoint_operation(func->ep_num, call_->tx_buf, call_->rx_buf.take(0), &op_handle_, *this);

        } else if (!call_->ep_num && call_->progress < func->inputs.size() + 1 + func->outputs.size()) {
            // Read output arg
            size_t argnum = call_->progress - func->inputs.size() - 1;
            call_->progress++;
            bufptr_t current_buf = call_->rx_buf.take(func->outputs[argnum].size);
            call_->rx_buf = call_->rx_buf.skip(func->outputs[argnum].size);
            protocol_->start_endpoint_operation(func->outputs[argnum].ep_num, call_->tx_buf, current_buf, &op_handle_, *this);

        } else {
            CallContext* call = call_;
            call_ = nullptr;
            FIBRE_LOG(D) << "call completed!";
            safe_complete(call->completer, {kFibreOk, call->rx_buf.end()});
            delete call;
        }

    } else {
        // The endpoint operation that completed belongs to the JSON fetch process

        size_t n_received = result.rx_end - json_.data() - json_.size() + 1024;
        json_.resize(json_.size() - 1024 + n_received);

        if (n_received) {
            receive_more_json();
            return;

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

    // Start next call in the queue if any
    // It's possible that the next function call was already started on one of
    // the callbacks above.
    if (!call_ && pending_calls_.size()) {
        call_ = pending_calls_[0];
        pending_calls_.erase(pending_calls_.begin());
        complete({kStreamOk, nullptr});
    }
}