
#include <fibre/channel_discoverer.hpp>
#include <string.h>
#include <stdio.h>
#include <algorithm>

using namespace fibre;

bool ChannelDiscoverer::try_parse_key(const char* begin, const char* end, const char* key, const char** val_begin, const char** val_end) {
    ssize_t keylen = strlen(key);

    while (begin != end) {
        const char* next_delim = std::find(begin, end, ',');
        
        if ((next_delim - begin >= keylen) && (memcmp(begin, key, keylen) == 0)) {
            if (next_delim - begin == keylen) {
                // The key exists but has no value
                *val_begin = *val_end = next_delim;
                return true;
            } else if (begin[keylen] == '=') {
                *val_begin = begin + keylen + 1;
                *val_end = next_delim;
                return true;
            }
        }

        begin = std::min(next_delim + 1, end);
    }

    return false; // key not found
}

bool ChannelDiscoverer::try_parse_key(const char* begin, const char* end, const char* key, int* val) {
    const char* val_begin;
    const char* val_end;
    if (!try_parse_key(begin, end, key, &val_begin, &val_end)) {
        return false;
    }

    // Copy value to a null-terminated buffer
    char buf[val_end - val_begin + 1];
    memcpy(buf, val_begin, val_end - val_begin);
    buf[val_end - val_begin] = 0;

    return sscanf(buf, "0x%x", val) == 1
        || sscanf(buf, "%d", val) == 1;
}
