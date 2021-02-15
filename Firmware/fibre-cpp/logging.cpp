
#include "logging.hpp"

#if !defined(_WIN32) && !defined(_WIN64) && !defined(__linux__) && !defined(__APPLE__) && !defined(EMSCRIPTEN)

namespace std {
StdoutStream cerr;
}

#endif

namespace fibre {

Logger logger{};

Logger* get_logger() {
    return &logger;
}

}
