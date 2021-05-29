
#include "legacy_object_client.hpp"
#include "legacy_protocol.hpp"
#include <fibre/simple_serdes.hpp>
#include "logging.hpp"
#include "print_utils.hpp"
#include "crc.hpp"
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

// not sure if this function exists in the STL
template<typename TIt, typename TFunc, typename TNum = decltype(std::declval<TFunc>()(*std::declval<TIt>()))>
TNum calc_sum(TIt begin, TIt end, TFunc func) {
    TNum s = {};
    for (TIt it = begin; it != end; ++it) {
        s += func(*it);
    }
    return s;
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
            (json_as_str(type_val) == "endpoint_ref") ? "object_ref" : json_as_str(type_val),
            get_codec_size(json_as_str(type_val)),
            (json_as_str(type_val) == "endpoint_ref") ? sizeof(uintptr_t) : get_codec_size(json_as_str(type_val)),
            (size_t)json_as_int(id_val),
        });
    }

    return arglist;
}

void LegacyObjectClient::start(Callback<void, LegacyObjectClient*, std::shared_ptr<LegacyObject>> on_found_root_object, Callback<void, LegacyObjectClient*, std::shared_ptr<LegacyObject>> on_lost_root_object) {
    FIBRE_LOG(D) << "start";
    on_found_root_object_ = on_found_root_object;
    on_lost_root_object_ = on_lost_root_object;
    json_.clear();
    receive_more_json();
}

std::shared_ptr<FibreInterface> LegacyObjectClient::get_property_interfaces(std::string codec, bool write) {
    auto& dict = write ? rw_property_interfaces : ro_property_interfaces;

    auto it = dict.find(codec);
    if (it != dict.end()) {
        return it->second;
    }

    auto intf_ptr = std::make_shared<FibreInterface>();
    dict[codec] = intf_ptr;
    FibreInterface& intf = *intf_ptr;
    size_t size = get_codec_size(codec);
    std::string app_codec = codec == "endpoint_ref" ? "object_ref" : codec;
    size_t app_codec_size = codec == "endpoint_ref" ? sizeof(uintptr_t) : size;

    if (!size || !app_codec_size) {
        FIBRE_LOG(W) << "unknown size for codec " << codec;
    }
    
    intf.name = std::string{} + "fibre.Property<" + (write ? "readwrite" : "readonly") + " " + codec + ">";
    intf.functions.emplace("read", LegacyFunction{0, nullptr, {}, {{"value", codec, app_codec, size, app_codec_size, 0}}});
    if (write) {
        intf.functions.emplace("exchange", LegacyFunction{0, nullptr, {{"newval", codec, app_codec, size, app_codec_size, 0}}, {{"oldval", codec, app_codec, size, app_codec_size, 0}}});
    }
    
    return intf_ptr;
}

std::shared_ptr<LegacyObject> LegacyObjectClient::load_object(json_value list_val) {
    

    if (!json_is_list(list_val)) {
        FIBRE_LOG(W) << "interface members must be a list";
        return nullptr;
    }

    LegacyObject obj{
        .client = this,
        .ep_num = 0,
        .intf = std::make_shared<FibreInterface>(),
        .known_to_application = false
    };
    auto obj_ptr = std::make_shared<LegacyObject>(obj);
    FibreInterface& intf = *obj_ptr->intf;

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
            intf.functions.emplace(name, LegacyFunction{
                (size_t)json_as_int(id),
                obj_ptr.get(),
                parse_arglist(json_dict_find(dict, "inputs")),
                parse_arglist(json_dict_find(dict, "outputs"))
            });

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

    objects_.push_back(obj_ptr);
    return obj_ptr;
}

void LegacyObjectClient::receive_more_json() {
    write_le<uint32_t>(json_.size(), tx_buf_);
    json_.resize(json_.size() + 1024);
    bufptr_t rx_buf = {json_.data() + json_.size() - 1024, json_.data() + json_.size()};
    protocol_->start_endpoint_operation(0, tx_buf_, rx_buf, &op_handle_, MEMBER_CB(this, on_received_json));
}

void LegacyObjectClient::on_received_json(EndpointOperationResult result) {
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
            on_found_root_object_.invoke_and_clear(this, root_obj_);
        }
    }
}


std::optional<CallBufferRelease> LegacyFunction::call(void** call_handle,
        CallBuffers buffers,
        Callback<std::optional<CallBuffers>, CallBufferRelease> callback) {
    
    LegacyCallContext* ctx;

    if (!*call_handle) {
        // Instantiate new call
        ctx = new LegacyCallContext();
        ctx->func_ = this;

        size_t total_tx_decoded_size = sizeof(uintptr_t);
        for (auto& arg: inputs) {
            total_tx_decoded_size += arg.app_size;
        }

        size_t total_rx_encoded_size = 0;
        for (auto& arg: outputs) {
            total_rx_encoded_size += arg.protocol_size;
        }

        ctx->tx_buf_.resize(total_tx_decoded_size);
        ctx->rx_buf_.resize(total_rx_encoded_size);

        *call_handle = ctx;
    } else {
        // Resume call
        ctx = reinterpret_cast<LegacyCallContext*>(*call_handle);
    }


    std::variant<LegacyCallContext::ResultFromApp,
                 LegacyCallContext::ResultFromProtocol> result = buffers;

    // Run endpoint operations for as long as we can do this synchronously.
    for (;;) {
        auto continuation = ctx->get_next_task(result);
        if (continuation.index() == 0) {
            return std::get<0>(continuation);
        } else if (continuation.index() == 1) {
            auto proto_continuation = std::get<1>(continuation);
            proto_continuation.client->start_endpoint_operation(
                    proto_continuation.ep_num, proto_continuation.tx_buf,
                    proto_continuation.rx_buf, &ctx->op_handle_,
                    MEMBER_CB(ctx, resume_from_protocol));

            if (!ctx->ep_result.has_value()) {
                ctx->callback = callback;
                return std::nullopt; // protocol will resume asynchronously
            }

            result = *ctx->ep_result;
            ctx->ep_result = std::nullopt;

            // TODO: ensure progress
        } else {
            return CallBufferRelease{kFibreInternalError, ctx->app_tx_end_, ctx->app_rx_buf_.begin()};
        }
    }

    return ctx->resume_from_app(buffers, callback);
}


void LegacyCallContext::resume_from_protocol(EndpointOperationResult result) {
    if (!callback) {
        // No callback configured. This means that this function is being executed
        // synchronously from inside LegacyFunction::call(). Set result and return.
        ep_result = result;
        return;
    }
    op_handle_ = 0;

    std::variant<ResultFromApp, ResultFromProtocol> res = result; 

    for (;;) {
        auto continuation = get_next_task(res);
        if (continuation.index() == 0) {
            auto app_result = callback.invoke(std::get<0>(continuation));
            if (!app_result.has_value()) {
                return; // app will resume asynchronously
            } else if (std::get<0>(continuation).status != kFibreOk) {
                if (app_result->status != kFibreClosed || app_result->rx_buf.size() || app_result->tx_buf.size()) {
                    FIBRE_LOG(W) << "app tried to continue a closed call";
                }
                FIBRE_LOG(T) << "closing call";
                return;
            } else {
                res = *app_result;
            }
        } else if (continuation.index() == 1) {
            auto proto_continuation = std::get<1>(continuation);
            proto_continuation.client->start_endpoint_operation(
                    proto_continuation.ep_num, proto_continuation.tx_buf,
                    proto_continuation.rx_buf, &op_handle_,
                    MEMBER_CB(this, resume_from_protocol));
            return; // protocol will return asynchronously
        } else {
            callback.invoke({kFibreInternalError, app_tx_end_, app_rx_buf_.begin()});
            return;
        }
    }
}

bool LegacyObjectClient::transcode(cbufptr_t src, bufptr_t dst, std::string src_codec, std::string dst_codec) {
    if (src_codec == "object_ref" && dst_codec == "endpoint_ref") {
        if (src.size() < sizeof(uintptr_t) || dst.size() < 4) {
            return false;
        }

        uintptr_t val = *reinterpret_cast<const uintptr_t*>(src.begin());
        LegacyObject* obj = reinterpret_cast<LegacyObject*>(val);
        write_le<uint16_t>(obj ? obj->ep_num : 0, &dst);
        write_le<uint16_t>(obj ? obj->client->json_crc_ : 0, &dst);

    } else if (src_codec == "endpoint_ref" && dst_codec == "object_ref") {
        if (src.size() < 4 || dst.size() < sizeof(uintptr_t)) {
            return false;
        }

        uint16_t ep_num = *read_le<uint16_t>(&src);
        uint16_t json_crc = *read_le<uint16_t>(&src);

        LegacyObject* obj_ptr = nullptr;

        if (ep_num && json_crc == json_crc_) {
            for (auto& known_obj: objects_) {
                if (known_obj->ep_num == ep_num) {
                    obj_ptr = known_obj.get();
                }
            }
        }

        FIBRE_LOG(D) << "placing transcoded ptr " << reinterpret_cast<uintptr_t>(obj_ptr);
        *reinterpret_cast<uintptr_t*>(dst.begin()) = reinterpret_cast<uintptr_t>(obj_ptr);

    } else {
        if (src.size() != dst.size()) {
            return false;
        }

        memcpy(dst.begin(), src.begin(), src.size());
    }

    return true;
}


std::variant<LegacyCallContext::ContinueWithApp, LegacyCallContext::ContinueWithProtocol, LegacyCallContext::InternalError> LegacyCallContext::get_next_task(std::variant<ResultFromApp, ResultFromProtocol> continue_from) {
    if (progress == 0) {
        if (continue_from.index() != 0) {
            FIBRE_LOG(E) << "expected continuation from app";
            return InternalError{};
        }

        ResultFromApp result_from_app = std::get<0>(continue_from);

        size_t n_copy = std::min(tx_buf_.size() - tx_pos_, result_from_app.tx_buf.size());
        std::copy_n(result_from_app.tx_buf.begin(), n_copy, tx_buf_.begin() + tx_pos_);
        result_from_app.tx_buf = result_from_app.tx_buf.skip(n_copy);
        tx_pos_ += n_copy;

        app_tx_end_ = result_from_app.tx_buf.begin();
        app_rx_buf_ = result_from_app.rx_buf;

        if (tx_pos_ < tx_buf_.size()) {
            // application specified kFibreOk? => return kFibreOk
            // application specified kFibreClosed? => return kFibreClosed
            return ContinueWithApp{result_from_app.status, app_tx_end_, app_rx_buf_.begin()};
        }

    } else if (progress <= func_->inputs.size() + 1 + func_->outputs.size()) {
        if (continue_from.index() != 1) {
            FIBRE_LOG(E) << "expected continuation from protocol";
            return InternalError{};
        }

        ResultFromProtocol result_from_protocol = std::get<1>(continue_from);

        if (result_from_protocol.status == kStreamClosed) {
            return ContinueWithApp{kFibreHostUnreachable, app_tx_end_, app_rx_buf_.begin()};
        } else if (result_from_protocol.status != kStreamOk) {
            FIBRE_LOG(W) << "protocol failed with " << result_from_protocol.status << " - propagating error to application";
            return ContinueWithApp{kFibreHostUnreachable, app_tx_end_, app_rx_buf_.begin()};
        }

        tx_pos_ = result_from_protocol.tx_end - tx_buf_.data();
        if (result_from_protocol.rx_end) {
            rx_pos_ = result_from_protocol.rx_end - rx_buf_.data();
        }

    } else if (progress == func_->inputs.size() + 2 + func_->outputs.size()) {
        if (continue_from.index() != 0) {
            FIBRE_LOG(E) << "expected continuation from app";
            return InternalError{};
        }

        ResultFromApp result_from_app = std::get<0>(continue_from);

        if (result_from_app.status != kFibreOk && result_from_app.status != kFibreClosed) {
            FIBRE_LOG(W) << "application failed with " << result_from_app.status << " - dropping this call";
            return InternalError{};
        }

        app_tx_end_ = result_from_app.tx_buf.begin();
        app_rx_buf_ = result_from_app.rx_buf;
    }

    if (progress == 0) {
        // Transcode from application codec to protocol codec

        obj_ = *reinterpret_cast<LegacyObject**>(tx_buf_.data());
        FIBRE_LOG(T) << "object is " << as_hex(reinterpret_cast<uintptr_t>(obj_));
        FIBRE_LOG(T) << "tx buf is " << as_hex(cbufptr_t{tx_buf_});

        std::vector<uint8_t> transcoded;
        size_t transcoded_size = calc_sum(func_->inputs.begin(), func_->inputs.end(),
            [](LegacyFibreArg& arg) { return arg.protocol_size; });
        FIBRE_LOG(T) << "transcoding " << func_->inputs.size() << " inputs from " << tx_buf_.size() << " B to " << transcoded_size << " B";
        transcoded.resize(transcoded_size);
        
        tx_pos_ = sizeof(uintptr_t);
        size_t transcoded_pos = 0;
        for (auto& arg: func_->inputs) {
            if (!obj_->client->transcode({tx_buf_.data() + tx_pos_, arg.app_size},
                    {transcoded.data() + transcoded_pos, arg.protocol_size},
                    arg.app_codec, arg.protocol_codec)) {
                return ContinueWithApp{kFibreInternalError, app_tx_end_, app_rx_buf_.begin()};
            }
            transcoded_pos += arg.protocol_size;
            tx_pos_ += arg.app_size;
        }

        tx_buf_ = transcoded;
        tx_pos_ = 0;

    } else if (progress == func_->inputs.size() + 1 + func_->outputs.size()) {
        // Transcode from protocol codec to application codec

        std::vector<uint8_t> transcoded;
        for (auto& arg: func_->outputs) {
            FIBRE_LOG(T) << "arg size " << arg.app_size;
        }
        size_t transcoded_size = calc_sum(func_->outputs.begin(), func_->outputs.end(),
            [](LegacyFibreArg& arg) { return arg.app_size; });
        FIBRE_LOG(T) << "transcoding " << func_->outputs.size() << " outputs from " << rx_buf_.size() << " B to " << transcoded_size << " B";
        transcoded.resize(transcoded_size);
        
        rx_pos_ = 0;
        size_t transcoded_pos = 0;
        for (auto& arg: func_->outputs) {
            if (!obj_->client->transcode({rx_buf_.data() + rx_pos_, arg.protocol_size},
                    {transcoded.data() + transcoded_pos, arg.app_size},
                    arg.protocol_codec, arg.app_codec)) {
                return ContinueWithApp{kFibreInternalError, app_tx_end_, app_rx_buf_.begin()};
            }
            transcoded_pos += arg.app_size;
            rx_pos_ += arg.protocol_size;
        }

        rx_buf_ = transcoded;
        rx_pos_ = 0;

        FIBRE_LOG(T) << "rx buf is " << as_hex(cbufptr_t{rx_buf_});
    }

    progress++;

    if (progress == 1 && obj_->ep_num) {
        // Single Endpoint Function - exchange everything in one go
        progress = func_->inputs.size() + 1 + func_->outputs.size();
        return ContinueWithProtocol{obj_->client->protocol_, obj_->ep_num, tx_buf_, rx_buf_};

    } else if (progress <= func_->inputs.size()) {
        // send arg
        auto arg = func_->inputs[progress - 1];
        return ContinueWithProtocol{obj_->client->protocol_, arg.ep_num, {tx_buf_.data() + tx_pos_, arg.protocol_size}, {}};
    } else if (progress == func_->inputs.size() + 1) {
        // send trigger
        return ContinueWithProtocol{obj_->client->protocol_, func_->ep_num, {}, {}};
    } else if (progress <= func_->inputs.size() + 1 + func_->outputs.size()) {
        // receive arg
        auto arg = func_->outputs[progress - 2 - func_->inputs.size()];
        return ContinueWithProtocol{obj_->client->protocol_, arg.ep_num, {}, {rx_buf_.data() + rx_pos_, arg.protocol_size}};
    } else if (progress == func_->inputs.size() + 2 + func_->outputs.size()) {
        // return data to application
        size_t n_copy = std::min(rx_buf_.size() - rx_pos_, app_rx_buf_.size());
        std::copy_n(rx_buf_.data() + rx_pos_, n_copy, app_rx_buf_.begin());
        app_rx_buf_ = app_rx_buf_.skip(n_copy);
        rx_pos_ += n_copy;
        return ContinueWithApp{rx_pos_ == rx_buf_.size() ? kFibreClosed : kFibreOk, app_tx_end_, app_rx_buf_.begin()};
    }

    return InternalError{};
}
