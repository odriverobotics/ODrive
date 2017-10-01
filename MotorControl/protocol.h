
// TODO: resolve assert
#define assert(expr)


#ifdef __cplusplus

#include <functional>

typedef std::function<void(const void *ctx)> PrintCallback;
typedef std::function<void(const char* buffer, void *ctx)> ScanCallback;

typedef enum {
    AS_FLOAT,
    AS_INT,
    AS_BOOL,
    AS_UINT16,
    BEGIN_TREE,
    END_TREE
} TypeInfo_t;

// The order in this list must correspond to the order in TypeInfo_t
const char *_type_names[] = {
    "float",
    "int",
    "bool",
    "uint16",
    "tree"
};


// Default getters/setters

PrintCallback print_float = std::bind(printf, "%f", std::placeholders::_1);
ScanCallback scan_float = std::bind(sscanf, std::placeholders::_1, "%f", std::placeholders::_2);

PrintCallback print_int = std::bind(printf, "%d", std::placeholders::_1);
ScanCallback scan_int = std::bind(sscanf, std::placeholders::_1, "%d", std::placeholders::_2);

PrintCallback print_bool = std::bind(printf, "%d", std::placeholders::_1);
ScanCallback scan_bool = std::bind(sscanf, std::placeholders::_1, "%d", std::placeholders::_2);

PrintCallback print_uint16 = std::bind(printf, "%d", std::placeholders::_1);
ScanCallback scan_uint16 = std::bind(sscanf, std::placeholders::_1, "%d", std::placeholders::_2);

class Endpoint {
private:
    const TypeInfo_t _type_info;
    const PrintCallback _print_callback;
    ScanCallback _scan_callback;
    const void* const _ctx;

public:
    const char* const _name;

    Endpoint(const char* name, TypeInfo_t type_info, PrintCallback print_callback, const void *ctx) :
        _type_info(type_info),
        _print_callback(print_callback),
        _scan_callback(nullptr),
        _ctx(ctx),
        _name(name)
    {
    }

    Endpoint(const char* name, TypeInfo_t type_info, PrintCallback print_callback, ScanCallback scan_callback, void *ctx) :
        _type_info(type_info),
        _print_callback(print_callback),
        _scan_callback(scan_callback),
        _ctx(ctx),
        _name(name)
    {
    }

    Endpoint(const char* name, const float& ctx) :
        Endpoint(name, AS_FLOAT, print_float, &ctx) {}
    Endpoint(const char* name, float& ctx) :
        Endpoint(name, AS_FLOAT, print_float, scan_float, &ctx) {}

    Endpoint(const char* name, const int& ctx) :
        Endpoint(name, AS_INT, print_int, &ctx) {}
    Endpoint(const char* name, int& ctx) :
        Endpoint(name, AS_INT, print_int, scan_int, &ctx) {}

    Endpoint(const char* name, const bool& ctx) :
        Endpoint(name, AS_BOOL, print_bool, &ctx) {}
    Endpoint(const char* name, bool& ctx) :
        Endpoint(name, AS_BOOL, print_bool, scan_bool, &ctx) {}

    Endpoint(const char* name, const uint16_t& ctx) :
        Endpoint(name, AS_UINT16, print_uint16, &ctx) {}
    Endpoint(const char* name, uint16_t& ctx) :
        Endpoint(name, AS_UINT16, print_uint16, scan_uint16, &ctx) {}

    void print_json(size_t id, bool& need_comma) {
        if (_type_info == END_TREE) {
            printf("]}");
            need_comma = true;
            return;
        } else if (_type_info < END_TREE) {
            assert(_name);
            assert(_type_names[_type_info]);
            if (need_comma)
                printf(",");
            printf("{\"name\":\"%s\",\"id\":%u,\"type\":\"%s\"", _name, id, _type_names[_type_info]);
            if (_type_info == BEGIN_TREE) {
                printf(",\"content\":[");
                need_comma = false;
            } else {
                printf("}");
                need_comma = true;
            }
        }
    }

    void print_value(void) {
        if (_print_callback)
            _print_callback(_ctx);
    }

    void scan_value(const char* buffer) {
        if (_scan_callback)
            _scan_callback(buffer, const_cast<void*>(_ctx));
    }
};


extern "C" {
#endif

void Protocol_parse_cmd(uint8_t* buffer, int len);

#ifdef __cplusplus
}
#endif
