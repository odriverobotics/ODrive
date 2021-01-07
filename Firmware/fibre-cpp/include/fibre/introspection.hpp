#ifndef __FIBRE_INTROSPECTION_HPP
#define __FIBRE_INTROSPECTION_HPP

#include <stdlib.h>
#include <algorithm>
#include <cstring>

#pragma GCC push_options
#pragma GCC optimize ("s")

class TypeInfo;
class Introspectable;
using introspectable_storage_t = std::aligned_storage<4 * sizeof(uintptr_t), sizeof(uintptr_t)>::type;

struct PropertyInfo {
    const char * name;
    const TypeInfo* type_info;
};

/**
 * @brief Contains runtime accessible type information.
 * 
 * Specifically, this information consists of a list of PropertyInfo items which
 * enable accessing attributes of an object by a runtime string.
 * 
 * Typically, for each combination of C++ type and Fibre interface implemented
 * by this type, one (static constant) TypeInfo object will exist.
 */
class TypeInfo {
    friend class Introspectable;
public:
    TypeInfo(const PropertyInfo* property_table, size_t property_table_length)
        : property_table_(property_table), property_table_length_(property_table_length) {}

    virtual introspectable_storage_t get_child(introspectable_storage_t obj, size_t idx) const = 0;
    Introspectable get_child(const Introspectable& obj, const char * name, size_t length) const;

protected:

    template<typename T> static T& as(Introspectable& obj);
    template<typename T> static const T& as(const Introspectable& obj);
    template<typename T> static Introspectable make_introspectable(T obj, const TypeInfo* type_info);

private:
    const PropertyInfo* property_table_;
    size_t property_table_length_;
};

/**
 * @brief Wraps a reference to an application object by attaching runtime
 * accessible type information.
 * 
 * The reference that is wrapped is typically a pointer but can also be a small
 * temporary, on-demand constructed object such as a fibre::Property<...> which
 * contains multiple pointers.
 */
class Introspectable {
    friend class TypeInfo;
public:
    Introspectable() {}

    /**
     * @brief Returns an Introspectable object for the attribute referenced by
     * the specified attribute name.
     * 
     * The name can consist of multiple parts separated by dots.
     * 
     * If the attribute does not exist, an invalid Introspectable is returned.
     * 
     * @param path: The name or path of the attribute.
     * @param length: The maximum length of the name.
     */
    Introspectable get_child(const char * path, size_t length) {
        Introspectable current = *this;

        const char * begin = path;
        const char * end = std::find(begin, path + length, '\0');

        while ((begin < end) && current.type_info_) {
            const char * end_of_token = std::find(begin, end, '.');
            current = current.get_direct_child(begin, end_of_token - begin);
            begin = std::min(end, end_of_token + 1);
        }

        return current;
    };

    bool is_valid() {
        return type_info_;
    }

    const TypeInfo* get_type_info() {
        return type_info_;
    }

private:
    Introspectable get_direct_child(const char * name, size_t length) const {
        for (size_t i = 0; i < type_info_->property_table_length_; ++i) {
            if (!strncmp(name, type_info_->property_table_[i].name, length) && (length == strlen(type_info_->property_table_[i].name))) {
                Introspectable result;
                result.storage_ = type_info_->get_child(storage_, i);
                result.type_info_ = type_info_->property_table_[i].type_info;
                return result;
            }
        }
        return {};
    }

public: // these should technically be protected but are public for optimization reasons
    // We use this storage to hold generic small objects. Usually that's a pointer
    // but sometimes it's an on-demand constructed Property<...>.
    // Caution: only put objects in here which are trivially copyable, movable
    // and destructible as any custom operation wouldn't be called.
    introspectable_storage_t storage_;
    const TypeInfo* type_info_ = nullptr;
};

template<typename T> T& TypeInfo::as(Introspectable& obj) {
    static_assert(sizeof(T) <= sizeof(obj.storage_), "invalid size");
    return *(T*)&obj.storage_;
}
template<typename T> const T& TypeInfo::as(const Introspectable& obj) {
    static_assert(sizeof(T) <= sizeof(obj.storage_), "invalid size");
    return *(const T*)&obj.storage_;
}
template<typename T> Introspectable TypeInfo::make_introspectable(T obj, const TypeInfo* type_info) {
    Introspectable introspectable;
    as<T>(introspectable) = obj;
    introspectable.type_info_ = type_info;
    return introspectable;
}


// maybe_underlying_type_t<T> resolves to the underlying type of T if T is an enum type or otherwise to T itself.
template<typename T, bool = std::is_enum<T>::value> struct maybe_underlying_type;
template<typename T> struct maybe_underlying_type<T, true> { typedef std::underlying_type_t<T> type; };
template<typename T> struct maybe_underlying_type<T, false> { typedef T type; };
template<typename T> using maybe_underlying_type_t = typename maybe_underlying_type<T>::type;


struct StringConvertibleTypeInfo {
    virtual bool get_string(const Introspectable& obj, char* buffer, size_t length) const { return false; }
    virtual bool set_string(const Introspectable& obj, char* buffer, size_t length) const { return false; }
};

struct FloatSettableTypeInfo {
    //virtual bool get_float(const Introspectable& obj, float* val) const { return false; }
    virtual bool set_float(const Introspectable& obj, float val) const { return false; }
};

/* Built-in type infos ********************************************************/

template<typename T>
struct FibrePropertyTypeInfo;

// readonly property
template<typename T>
struct FibrePropertyTypeInfo<Property<const T>> : StringConvertibleTypeInfo, TypeInfo {
    using TypeInfo::TypeInfo;
    static const PropertyInfo property_table[];
    static const FibrePropertyTypeInfo<Property<const T>> singleton;

    introspectable_storage_t get_child(introspectable_storage_t obj, size_t idx) const override {
        return {};
    }

    bool get_string(const Introspectable& obj, char* buffer, size_t length) const override {
        return to_string(static_cast<maybe_underlying_type_t<T>>(as<const Property<const T>>(obj).read()), buffer, length, 0);
    }
};

template<typename T>
const PropertyInfo FibrePropertyTypeInfo<Property<const T>>::property_table[] = {};
template<typename T>
const FibrePropertyTypeInfo<Property<const T>> FibrePropertyTypeInfo<Property<const T>>::singleton{FibrePropertyTypeInfo<Property<const T>>::property_table, sizeof(FibrePropertyTypeInfo<Property<const T>>::property_table) / sizeof(FibrePropertyTypeInfo<Property<const T>>::property_table[0])};

// readwrite property
template<typename T>
struct FibrePropertyTypeInfo<Property<T>> : FloatSettableTypeInfo, StringConvertibleTypeInfo, TypeInfo {
    using TypeInfo::TypeInfo;
    static const PropertyInfo property_table[];
    static const FibrePropertyTypeInfo<Property<T>> singleton;
    static const Introspectable make_introspectable(Property<T> obj) { return TypeInfo::make_introspectable(obj, &singleton); }

    introspectable_storage_t get_child(introspectable_storage_t obj, size_t idx) const override {
        return {};
    }

    bool get_string(const Introspectable& obj, char* buffer, size_t length) const override {
        return to_string(static_cast<maybe_underlying_type_t<T>>(as<const Property<T>>(obj).read()), buffer, length, 0);
    }

    bool set_string(const Introspectable& obj, char* buffer, size_t length) const override {
        maybe_underlying_type_t<T> value{};
        if (!from_string(buffer, length, &value, 0)) {
            return false;
        }
        as<const Property<T>>(obj).exchange(static_cast<T>(value));
        return true;
    }

    bool set_float(const Introspectable& obj, float val) const override {
        maybe_underlying_type_t<T> value{};
        if (!conversion::set_from_float(val, &value)) {
            return false;
        }
        as<const Property<T>>(obj).exchange(static_cast<T>(value));
        return true;
    }
};

template<typename T>
const PropertyInfo FibrePropertyTypeInfo<Property<T>>::property_table[] = {};
template<typename T>
const FibrePropertyTypeInfo<Property<T>> FibrePropertyTypeInfo<Property<T>>::singleton{FibrePropertyTypeInfo<Property<T>>::property_table, sizeof(FibrePropertyTypeInfo<Property<T>>::property_table) / sizeof(FibrePropertyTypeInfo<Property<T>>::property_table[0])};

#pragma GCC pop_options

#endif // __FIBRE_INTROSPECTION_HPP