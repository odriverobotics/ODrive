#ifndef __FILE_HPP
#define __FILE_HPP

#include <stdint.h>
#include <stdlib.h>

/**
 * @brief Represents a non-volatile file, typically stored on flash memory.
 * 
 * The state machine works like this:
 * 
 * 
 *              power cycle                       write()
 *                .----.                          .----.
 *                v    |                          v    |
 *            .-----------.   start_write()   .-----------.
 *            |  Erased   |------------------>|  Erased   |
 *            |   Idle    |<------------------|  Writing  |
 *            '-----------'    power cycle    '-----------'
 *                       ^                          |
 *                        \     earse()             | close()
 *                         '----------------.       |
 *                                           \      v
 *            .-----------.   start_read()    \-----------.
 *            |   Valid   |<------------------|  Valid    |
 *            |  Reading  |------------------>|   Idle    |
 *            '-----------'      close()      '-----------'
 *               ^    |        power cycle        ^    |
 *               '----'                           '----'
 *               read()                         power cycle
 *
 * 
 * All function calls that are not shown for a particular state are not allowed
 * in that state and will return false if issued.
 */
struct File {
    /**
     * @brief Indicates if this file is valid (true) or erased (false).
     */
    virtual bool is_valid() = 0;
    
    /**
     * @brief Opens the file for writing.
     * 
     * The file must be erased when calling this function.
     */
    virtual bool open_write() = 0;

    /**
     * @brief Tentatively appends data to the file.
     * 
     * The data is not committed until close().
     */
    virtual bool write(const uint8_t* buffer, size_t length) = 0;

    /**
     * @brief Opens the file for read access.
     * 
     * The file must be valid when calling this function.
     * 
     * @param length: set to the total length of the file which can be 0.
     */
    virtual bool open_read(size_t* length) = 0;

    /**
     * @brief Reads data from the file. This advances the read pointer.
     * 
     * Returns false on error, for example if the user tries to read beyond the
     * end of the file.
     */
    virtual bool read(uint8_t* buffer, size_t length) = 0;

    /**
     * @brief Closes an ongoing write or read operation.
     */
    virtual bool close() = 0;

    /**
     * @brief Erases the file.
     * 
     * The file must be valid when this function is called.
     */
    virtual bool erase() = 0;

    template<typename T>
    bool write(T* data) { return write((uint8_t*)data, sizeof(T)); }
    template<typename T>
    bool read(T* data) { return read((uint8_t*)data, sizeof(T)); }
};

#endif // __FILE_HPP
