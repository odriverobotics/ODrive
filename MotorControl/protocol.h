
// TODO: resolve assert
#define assert(expr)

#ifdef __cplusplus

#include <functional>

typedef enum {
    AS_FLOAT,
    AS_INT,
    AS_BOOL,
    AS_UINT16,
    BEGIN_TREE,
    END_TREE
} TypeInfo_t;

typedef std::function<int(TypeInfo_t type_info, const void *ctx)> PrintCallback;
typedef std::function<int(TypeInfo_t type_info, const char* buffer, void *ctx)> ScanCallback;

// The order in this list must correspond to the order in TypeInfo_t
const char *type_names_[] = {
    "float",
    "int",
    "bool",
    "uint16",
    "tree"
};


int Protocol_print_simple_type(TypeInfo_t type_info, const void *ctx);
int Protocol_scan_simple_type(TypeInfo_t type_info, const char* buffer, void *ctx);

// Default getters/setters

class Endpoint {
public:
    const char* const name_;

    Endpoint(const char* name, TypeInfo_t type_info, PrintCallback print_callback, const void *ctx) :
        name_(name),
        type_info_(type_info),
        print_callback_(print_callback),
        scan_callback_(nullptr),
        ctx_(ctx)
    {
    }

    Endpoint(const char* name, TypeInfo_t type_info, PrintCallback print_callback, ScanCallback scan_callback, void *ctx) :
        name_(name),
        type_info_(type_info),
        print_callback_(print_callback),
        scan_callback_(scan_callback),
        ctx_(ctx)
    {
    }

    Endpoint(const char* name, const float& ctx) :
        Endpoint(name, AS_FLOAT, Protocol_print_simple_type, &ctx) {}
    Endpoint(const char* name, float& ctx) :
        Endpoint(name, AS_FLOAT, Protocol_print_simple_type, Protocol_scan_simple_type, &ctx) {}

    Endpoint(const char* name, const int& ctx) :
        Endpoint(name, AS_INT, Protocol_print_simple_type, &ctx) {}
    Endpoint(const char* name, int& ctx) :
        Endpoint(name, AS_INT, Protocol_print_simple_type, Protocol_scan_simple_type, &ctx) {}

    Endpoint(const char* name, const bool& ctx) :
        Endpoint(name, AS_BOOL, Protocol_print_simple_type, &ctx) {}
    Endpoint(const char* name, bool& ctx) :
        Endpoint(name, AS_BOOL, Protocol_print_simple_type, Protocol_scan_simple_type, &ctx) {}

    Endpoint(const char* name, const uint16_t& ctx) :
        Endpoint(name, AS_UINT16, Protocol_print_simple_type, &ctx) {}
    Endpoint(const char* name, uint16_t& ctx) :
        Endpoint(name, AS_UINT16, Protocol_print_simple_type, Protocol_scan_simple_type, &ctx) {}

    void print_json(size_t id, bool& need_comma);

    void print_value(void) {
        if (print_callback_)
            print_callback_(type_info_, ctx_);
    }

    void scan_value(const char* buffer) {
        if (scan_callback_)
            scan_callback_(type_info_, buffer, const_cast<void*>(ctx_));
    }

private:
    const TypeInfo_t type_info_;
    const PrintCallback print_callback_;
    ScanCallback scan_callback_;
    const void* const ctx_;
};


extern "C" {
#endif

void Protocol_parse_cmd(uint8_t* buffer, int len);

#ifdef __cplusplus
}
#endif
