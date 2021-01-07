/**
 * @brief Provides logging facilities
 * 
 * Log entries are associated with user defined topics. A user can define a
 * topic using DEFINE_LOG_TOPIC(topicname) and activate the topic for the
 * current scope using USE_LOG_TOPIC(topicname).
 * 
 * Currently all log entries are posted to stderr in a thread-safe way.
 *
 * Whether an event is actually logged depends on the current log verbosity of
 * the corresponding topic. The log verbosity is defined by the following
 * sources (in order of their precedence).
 * 
 *  0. maximum log verbosity setting (see below)
 *  1. runtime environment variable "FIBRE_LOG_[topicname]"
 *  2. runtime environment variable "FIBRE_LOG"
 *  3. topic specific default log verbosity defined using CONFIG_LOG_TOPIC(...)
 *  4. FIBRE_DEFAULT_LOG_VERBOSITY defined before this file (using #define or -D compiler flag)
 *  5. FIBRE_DEFAULT_LOG_VERBOSITY defined in this file
 * 
 * The maximum log verbosity can be defined separately from the default log
 * verbosity. The maximum log verbosity bound always applies, regardless of how
 * the actual log verbosity is specified. This allows keeping the binary small
 * by optimizing away unnecessary log entries at compile time.
 * The maximum log verbosity is defined by the following sources (in order of
 * their precedence):
 * 
 *  1. topic specific max log verbosity defined using CONFIG_LOG_TOPIC(...)
 *  2. FIBRE_MAX_LOG_VERBOSITY defined before this file (using #define or -D compiler flag)
 *  3. FIBRE_MAX_LOG_VERBOSITY defined in this file
 * 
 * TODO: ensure that the optimizer can indeed strip the unused strings (currently not the case)
 * 
 * 
 * Example:
 * 
 * @code
 * 
 * DEFINE_LOG_TOPIC(MAIN);
 * USE_LOG_TOPIC(MAIN);
 * 
 * int main(void) {
 *     FIBRE_LOG(D) << "Hello Log!";
 *     if (open("inexistent_file", O_RDONLY) < 0) {
 *         FIBRE_LOG(E) << "Could not open file: " << sys_err();
 *     }
 *     return 0;
 * }
 * 
 * @endcode
 * 
 * Using logging in header files is possible but undocumented (TODO: fix)
 */

#ifndef __FIBRE_LOGGING_HPP
#define __FIBRE_LOGGING_HPP

/**
 * @brief Tag type to print the last system error
 * 
 * The statement `std::out << sys_err();` will print the last system error
 * in the following format: "error description (errno)".
 * This is based on `GetLastError()` (Windows) or `errno` (all other systems).
 */
struct sys_err {};


// TODO: support lite-version of logging on embedded systems
#if FIBRE_MAX_LOG_VERBOSITY

#include <fibre/cpp_utils.hpp>

#include <string.h>
#include <chrono>

#if defined(_WIN32) || defined(_WIN64)
#include "windows.h"
#endif

#if defined(_WIN32) || defined(_WIN64) || defined(__linux__) || defined(__APPLE__) || defined(EMSCRIPTEN)
#include <iostream>
#include <iomanip>
#else

// We don't want <iostream> included on an embedded system as it makes the
// binary huge.

struct StdoutStream : std::ostream {
    void operator <<(const char * str) {
        printf("%s", str);
    }
};

namespace std {
extern StdoutStream cerr;
}

#endif

#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)

#include <mutex>
using TMutex = std::mutex;
using TLock = std::unique_lock<TMutex>;

#else

using TMutex = int;
struct TLock {
    TLock() {}
    TLock(TMutex) {}
};

#endif

namespace fibre {

// Maximum log verbosity that should be compiled into the binary.
// Log entries with a higher verbosity should be optimized away.
#ifndef FIBRE_MAX_LOG_VERBOSITY
#  define FIBRE_MAX_LOG_VERBOSITY LOG_LEVEL_T
#endif

// Default log verbosity that should be used for all topic. This may be
// overridden by other sources, see description in the beginning of this file.
#ifndef FIBRE_DEFAULT_LOG_VERBOSITY
#  define FIBRE_DEFAULT_LOG_VERBOSITY LOG_LEVEL_W
#endif

/**
 * @brief Generates one log entry.
 * 
 * The log entry will be associates with the topic specified in "USE_LOG_TOPIC".
 * 
 * Note that the log entry must only be used in the statement it is generated. (TODO: fix)
 * 
 * @param level: Can be one of "E", "W", "D", or other levels defined in
 *        log_level_t.
 * @returns a stream for writing into the log entry
 */
#define FIBRE_LOG(level) \
    fibre::make_log_entry<current_log_topic, fibre::LOG_LEVEL_ ## level>( \
        fibre::get_file_name(MAKE_SSTRING(__FILE__){}), __LINE__, __func__ \
    ).get_stream()

/**
 * @brief Defines a log topic. A log topic must be defined exactly once in every
 * translation unit it is used.
 */
#define DEFINE_LOG_TOPIC(name) \
    struct LOG_TOPIC_ ## name { \
        static const char * get_label() { \
            static const char label[] = #name; \
            return label; \
        } \
    }

/**
 * @brief Activates the use of the specified log topic for the current scope
 * (and all subscopes)
 */
#define USE_LOG_TOPIC(name) using current_log_topic = LOG_TOPIC_ ## name

/**
 * @brief Overrides the general log verbosity settings for a specific topic.
 * If used, this should be placed in the same scope as the corresponding
 * DEFINE_LOG_TOPIC.
 */
#define CONFIG_LOG_TOPIC(topic, default_verbosity, max_verbosity) \
template<> constexpr log_level_t get_default_log_verbosity<LOG_TOPIC_ ## topic>() { return (default_verbosity); } \
template<> constexpr log_level_t get_max_log_verbosity<LOG_TOPIC_ ## topic>() { return (max_verbosity); }


/** @brief Log verbosity levels */
enum log_level_t {
    LOG_LEVEL_F = 0, // fatal
    LOG_LEVEL_E = 1, // error
    LOG_LEVEL_W = 2, // warning
    LOG_LEVEL_I = 3, // info
    LOG_LEVEL_D = 4, // debug
    LOG_LEVEL_T = 5, // trace
};

class NullBuffer : public std::streambuf {
public:
    int overflow(int c) { return c; }
};

// Source: https://stackoverflow.com/questions/15845505/how-to-get-higher-precision-fractions-of-a-second-in-a-printout-of-current-tim
static std::string get_local_time() {
  auto now(std::chrono::system_clock::now());
  auto seconds_since_epoch(
      std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()));

  // Construct time_t using 'seconds_since_epoch' rather than 'now' since it is
  // implementation-defined whether the value is rounded or truncated.
  std::time_t now_t(
      std::chrono::system_clock::to_time_t(
          std::chrono::system_clock::time_point(seconds_since_epoch)));

  char temp[10];
  if (!std::strftime(temp, 10, "%H:%M:%S.", std::localtime(&now_t)))
    return "";

  return std::string(temp) +
      std::to_string((now.time_since_epoch() - seconds_since_epoch).count());
}

class Logger {
public:
    class Entry {
    public:
        Entry() : base_stream_(null_stream), lock_() {}

        Entry(std::ostream& base_stream, log_level_t level, const char* topic, const char* filename, size_t line_no, const char *funcname, TMutex& mutex)
            : base_stream_(base_stream), lock_(mutex)
        {
            switch (level) {
            case LOG_LEVEL_W:
                base_stream << "\x1b[93;1m";
                break;
            case LOG_LEVEL_E:
            case LOG_LEVEL_F:
                base_stream << "\x1b[91;1m";
                break;
            default:
                break;
            }
            base_stream << get_local_time() << " ";
            base_stream << std::dec << "[" << topic << "] ";
            //base_stream << std::dec << filename << ":" << line_no << " in " << funcname << "(): ";
        }
        ~Entry() { get_stream() << "\x1b[0m" << std::endl; }
        std::ostream& get_stream() { return base_stream_; };
    private:
        NullBuffer null_buffer{};
        std::ostream null_stream{&null_buffer};
        std::ostream& base_stream_;
        TLock lock_;
    };
    
    TMutex mutex_;
};



template<typename TOPIC>
constexpr log_level_t get_default_log_verbosity() { return (log_level_t)FIBRE_DEFAULT_LOG_VERBOSITY; }

template<typename TOPIC>
constexpr log_level_t get_max_log_verbosity() { return (log_level_t)FIBRE_MAX_LOG_VERBOSITY; }

/**
 * @brief Resolves the currently active log verbosity for the given topic.
 * See top of this file for a detailed description of the algorithm.
 */
template<typename TOPIC>
log_level_t get_current_log_verbosity() {
    char var_name[sizeof("FIBRE_LOG_") + strlen(TOPIC::get_label())];
    strcpy(var_name, "FIBRE_LOG_");
    strcat(var_name, TOPIC::get_label());

    // TODO: provide a way to disable the 
    const char * var_val = std::getenv(var_name);
    if (!var_val) {
        var_val = std::getenv("FIBRE_LOG");
    }

    log_level_t log_level = get_default_log_verbosity<TOPIC>();
    if (var_val) {
        unsigned long num = strtoul(var_val, nullptr, 10);
        log_level = (log_level_t)num;
    }

    if (log_level > get_max_log_verbosity<TOPIC>()) {
        log_level = get_max_log_verbosity<TOPIC>();
    }
    return log_level;
}



/*
template<typename TStream, typename T, typename... Ts>
void send_to_stream(TStream&& stream);

template<typename TStream>
void send_to_stream(TStream&& stream) { }

template<typename TStream, typename T, typename... Ts>
void send_to_stream(TStream&& stream, T&& value, Ts&&... values) {
    send_to_stream(std::forward<TStream>(stream) << std::forward<T>(value), std::forward<Ts>(values)...);
}*/


Logger* get_logger(); // defined in logging.cpp

template<typename TOPIC, log_level_t LEVEL>
Logger::Entry make_log_entry(const char *filename, size_t line_no, const char *funcname) {
    if (get_current_log_verbosity<TOPIC>() < LEVEL) {
        return {};
    } else {
        Logger* logger = get_logger();
        return { std::cerr, LEVEL, TOPIC::get_label(), filename, line_no, funcname, logger->mutex_ };
    }
}

template<typename TFilepath>
constexpr const char * get_file_name(TFilepath file_path) {
    return (file_path /*file_path.after_last_index_of('/')*/).c_str(); // TODO: extract file name (without path)
}

}


namespace std {
static inline std::ostream& operator<<(std::ostream& stream, const sys_err&) {
#if defined(_WIN32) || defined(_WIN64)
    auto error_code = GetLastError();
#else
    auto error_code = errno;
#endif
    return stream << strerror(error_code) << " (" << error_code << ")";
}
}


#else

#define DEFINE_LOG_TOPIC(topic)
#define USE_LOG_TOPIC(topic)

struct NullStream {
    template<typename T> NullStream& operator<<(T val) { return *this; }
};

#define FIBRE_LOG(level) NullStream()

#endif // FIBRE_MAX_LOG_VERBOSITY

#endif // __FIBRE_LOGGING_HPP
