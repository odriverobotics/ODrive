#ifndef __FIBRE_BUFPTR_HPP
#define __FIBRE_BUFPTR_HPP

#include <stdlib.h>
#include <vector>

namespace fibre {

static inline bool soft_assert(bool expr) { return expr; } // TODO: implement

/**
 * @brief Holds a reference to a buffer and a length.
 * Since this class implements begin() and end(), you can use it with many
 * standard algorithms that operate on iterable objects.
 */
template<typename T>
struct generic_bufptr_t {
    using iterator = T*;
    using const_iterator = const T*;

    generic_bufptr_t(T* begin, size_t length) : begin_(begin), end_(begin + length) {}

    generic_bufptr_t(T* begin, T* end) : begin_(begin), end_(end) {}

    generic_bufptr_t() : begin_(nullptr), end_(nullptr) {}

    template<size_t I>
    generic_bufptr_t(T (&begin)[I]) : generic_bufptr_t(begin, I) {}

    generic_bufptr_t(std::vector<typename std::remove_const<T>::type>& vector)
        : generic_bufptr_t(vector.data(), vector.size()) {}

    generic_bufptr_t(const std::vector<typename std::remove_const<T>::type>& vector)
        : generic_bufptr_t(vector.data(), vector.size()) {}

    generic_bufptr_t(const generic_bufptr_t<typename std::remove_const<T>::type>& other)
        : generic_bufptr_t(other.begin(), other.end()) {}

    generic_bufptr_t& operator+=(size_t num) {
        if (!soft_assert(num <= size())) {
            num = size();
        }
        begin_ += num;
        return *this;
    }

    generic_bufptr_t operator++(int) {
        generic_bufptr_t result = *this;
        *this += 1;
        return result;
    }

    T& operator*() {
        return *begin_;
    }

    generic_bufptr_t take(size_t num) const {
        if (!soft_assert(num <= size())) {
            num = size();
        }
        generic_bufptr_t result = {begin_, num};
        return result;
    }

    generic_bufptr_t skip(size_t num, size_t* processed_bytes = nullptr) const {
        if (!soft_assert(num <= size())) {
            num = size();
        }
        if (processed_bytes)
            (*processed_bytes) += num;
        return {begin_ + num, end_};
    }

    size_t size() const {
        return end_ - begin_;
    }

    bool empty() const {
        return size() == 0;
    }

    T*& begin() { return begin_; }
    T*& end() { return end_; }
    T* const & begin() const { return begin_; }
    T* const & end() const { return end_; }
    T& front() const { return *begin(); }
    T& back() const { return *(end() - 1); }
    T& operator[](size_t idx) { return *(begin() + idx); }

private:
    T* begin_;
    T* end_;
};

using cbufptr_t = generic_bufptr_t<const unsigned char>;
using bufptr_t = generic_bufptr_t<unsigned char>;

}

#endif // __FIBRE_BUFPTR_HPP
