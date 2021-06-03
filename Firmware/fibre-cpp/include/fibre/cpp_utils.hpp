/*

## Advanced C++ Topics

This is an overview of some of the more obscure C++ techniques used in this project.
This assumes you're already familiar with templates in C++.

### Template recursion

[TODO]

### Almost perfect template forwarding

This is adapted from https://akrzemi1.wordpress.com/2013/10/10/too-perfect-forwarding/

Suppose you have a inner class, with a couple of constructors:
```
class InnerClass {
public:
    InnerClass(int arg1, int arg2);
    InnerClass(int arg1);
    InnerClass();
};
```

Now you want to create a wrapper class. This wrapper class should provide the exact same constructors as `InnerClass`, so you use perfect forwarding:
```
class WrapperClass {
public:
    template<typename ... Args>
    WrapperClass(Args&& ... args)
        : inner_object(std::forward<Args>(args)...)
    {}
    InnerClass inner_object;
};
```

Now you can almost use the wrapper class as expected, but only almost:
```
void make_wrappers(void) {
    WrapperClass wrapper1;              // ok, maps to InnerClass()
    WrapperClass wrapper2(1);           // ok, maps to InnerClass(int arg1)
    WrapperClass wrapper3(1,2);         // ok, maps to InnerClass(int arg1, arg2)
    WrapperClass wrapper4 = wrapper1;   // does not compile
}
```

The last assignment fails. What _you_ obviously wanted, is to use the copy constructor of WrapperClass.
However the compiler will use the perfect forwarding constructor of WrapperClass for this assignment.
So after template expansion it would try to use the following constructor:

```
    WrapperClass(InnerClass& arg)
        : inner_object(arg)
    {}
```

Clearly this is not what we wanted and in this case it will fail because the exists no
constructor of the form `InnerClass(WrapperClass& arg)`.

And thus we need to make the perfect forwarding a little less perfect, by telling it
"only enable this constructor if the first argument of the argument list is not of type WrapperClass".

The modified version thus looks like this:
```
class WrapperClass {
public:
    template<typename ... Args, ENABLE_IF(TypeChecker<Args...>::template first_is_not<WrapperClass>())>
    WrapperClass(Args&& ... args)
        : inner_object(std::forward<Args>(args)...)
    {} 
    InnerClass inner_object;
};
```

*/

#ifndef __CPP_UTILS_HPP
#define __CPP_UTILS_HPP

#include <limits>
#include <tuple>
#include <functional>
#include <unordered_map>
#include <stdlib.h>
//#include <ostream>

/* Backport features from C++14 and C++17 ------------------------------------*/

#if __cplusplus < 201402L
namespace std {
    template< class T >
    using underlying_type_t = typename underlying_type<T>::type;

    // source: http://en.cppreference.com/w/cpp/types/enable_if
    template< bool B, class T = void >
    using enable_if_t = typename enable_if<B,T>::type;

    // source: https://en.cppreference.com/w/cpp/types/conditional
    template< bool B, class T, class F >
    using conditional_t = typename conditional<B,T,F>::type;

    // source: http://en.cppreference.com/w/cpp/utility/tuple/tuple_element
    template <std::size_t I, class T>
    using tuple_element_t = typename tuple_element<I, T>::type;

    // source: https://en.cppreference.com/w/cpp/types/remove_cv
    template< class T >
    using remove_cv_t       = typename remove_cv<T>::type;
    template< class T >
    using remove_const_t    = typename remove_const<T>::type;
    template< class T >
    using remove_volatile_t = typename remove_volatile<T>::type;
    template< class T >
    using remove_reference_t = typename remove_reference<T>::type;

    template< class T >
    using decay_t = typename decay<T>::type;

    // integer_sequence implementation adapted from
    // https://stackoverflow.com/questions/17424477/implementation-c14-make-integer-sequence

    /// Class template integer_sequence
    template<typename _Tp, _Tp... _Idx>
    struct integer_sequence {
        using type = integer_sequence;
        typedef _Tp value_type;
        static constexpr size_t size() noexcept { return sizeof...(_Idx); }
    };

    template <class Sequence1, class Sequence2>
    struct _merge_and_renumber;

    template <typename _Tp, size_t... I1, size_t... I2>
    struct _merge_and_renumber<integer_sequence<_Tp, I1...>, integer_sequence<_Tp, I2...>>
      : integer_sequence<_Tp, I1..., (sizeof...(I1)+I2)...>
    { };

    template <typename _Tp, size_t N>
    struct make_integer_sequence
      : _merge_and_renumber<typename make_integer_sequence<_Tp, N/2>::type,
                            typename make_integer_sequence<_Tp, N - N/2>::type>
    { };

    template<typename _Tp> struct make_integer_sequence<_Tp, 0> : integer_sequence<_Tp> { };
    template<typename _Tp> struct make_integer_sequence<_Tp, 1> : integer_sequence<_Tp, 0> { };

    /// Alias template index_sequence
    template<size_t... _Idx>
    using index_sequence = integer_sequence<size_t, _Idx...>;

    /// Alias template make_index_sequence
    template<size_t _Num>
    using make_index_sequence = typename make_integer_sequence<size_t, _Num>::type;
}
#endif

namespace fibre {
    // Creates the index sequence { IFrom, IFrom + 1, IFrom + 2, ..., ITo - 1 }
    template<typename _Tp, _Tp IFrom, _Tp ITo, _Tp ... I>
    struct make_integer_sequence_from_to_impl {
        using type = typename make_integer_sequence_from_to_impl<_Tp, IFrom, ITo - 1, ITo - 1, I...>::type;
    };

    template<typename _Tp, _Tp IFrom, _Tp ... I>
    struct make_integer_sequence_from_to_impl<_Tp, IFrom, IFrom, I...> {
        using type = std::index_sequence<I...>;
    };
    
    template<typename _Tp, _Tp IFrom, _Tp ITo>
    using make_integer_sequence_from_to = typename make_integer_sequence_from_to_impl<_Tp, IFrom, ITo>::type;
}

#if __cplusplus < 201703L
namespace std {
//template<typename Fn, typename... Args,
//    std::enable_if_t<std::is_member_pointer<std::decay_t<Fn>>{}, int> = 0>
//using enable_

template <class, class, class...> struct invoke_result_impl;

template<typename Fn, typename... Args>
struct invoke_result_impl<std::enable_if_t<std::is_member_pointer<std::decay_t<Fn>>{}>,
                          Fn, Args...> {
    typedef decltype(std::mem_fn(std::declval<Fn>())(std::declval<Args>()...)) type;
};

template<typename Fn, typename... Args>
struct invoke_result_impl<std::enable_if_t<!std::is_member_pointer<std::decay_t<Fn>>{}>,
                          Fn, Args...> {
    typedef decltype(std::declval<Fn>()(std::declval<Args>()...)) type;
};

template<typename Fn, typename... Args>
using invoke_result = invoke_result_impl<void, Fn, Args...>;

template<typename Fn, typename... Args>
using invoke_result_t = typename invoke_result<Fn, Args...>::type;

template<typename Fn, typename... Args,
        std::enable_if_t<std::is_member_pointer<std::decay_t<Fn>>{}, int> = 0 >
constexpr invoke_result_t<Fn, Args...> invoke(Fn&& f, Args&&... args)
    noexcept(noexcept(std::mem_fn(f)(std::forward<Args>(args)...)))
{
    return std::mem_fn(f)(std::forward<Args>(args)...);
}

template<typename Fn, typename... Args, 
         std::enable_if_t<!std::is_member_pointer<std::decay_t<Fn>>{}, int> = 0>
constexpr invoke_result_t<Fn, Args...> invoke(Fn&& f, Args&&... args)
    noexcept(noexcept(std::forward<Fn>(f)(std::forward<Args>(args)...)))
{
    return std::forward<Fn>(f)(std::forward<Args>(args)...);
}
}

namespace std {
namespace detail {
template <class F, class Tuple, class>
struct apply_result_impl;

// TODO: apply_result is not part of C++17, therefore we should move this out of
// the #if block
template <class F, class Tuple, std::size_t... I>
struct apply_result_impl<F, Tuple, std::index_sequence<I...>> {
    //typedef std::invoke_result_t<F, std::tuple_element_t<I, Tuple>...> type;
    typedef std::invoke_result_t<F, decltype(std::get<I>(std::declval<Tuple>()))...> type;
};

template <class F, class Tuple>
using apply_result = apply_result_impl<F, Tuple, std::make_index_sequence<std::tuple_size<std::decay_t<Tuple>>::value>>;

template <class F, class Tuple>
using apply_result_t = typename apply_result<F, Tuple>::type;

template <class F, class Tuple, std::size_t... I>
constexpr apply_result_t<F, Tuple> apply_impl( F&& f, Tuple&& t, std::index_sequence<I...> )
{
    return std::invoke(std::forward<F>(f), std::get<I>(std::forward<Tuple>(t))...);
}
} // namespace detail

template <class F, class Tuple>
constexpr detail::apply_result_t<F, Tuple> apply(F&& f, Tuple&& t)
{
    return detail::apply_impl(std::forward<F>(f), std::forward<Tuple>(t),
        std::make_index_sequence<std::tuple_size<std::decay_t<Tuple>>::value>{});
}
}


namespace std {

template<typename T>
struct identity { using type = T; };

template<typename ... Ts>
struct overload_resolver;

template<>
struct overload_resolver<> { void operator()() const; };

template<typename T, typename ... Ts>
struct overload_resolver<T, Ts...> : overload_resolver<Ts...> {
    using overload_resolver<Ts...>::operator();
    identity<T> operator()(T) const;
};

template<typename TQuery, typename T, typename ... Ts>
struct index_of : integral_constant<size_t, (index_of<TQuery, Ts...>::value + 1)> {};

template<typename TQuery, typename ... Ts>
struct index_of<TQuery, TQuery, Ts...> : integral_constant<size_t, 0> {};

/**
 * @brief Heavily simplified version of the C++17 std::variant.
 * Whatever compiles should work as one would expect from the C++17 variant.
 */
template<typename ... Ts>
class variant;

// Empty variant is ill-formed. Only used for clean recursion here.
template<>
class variant<> {
public:
    using storage_t = char[0];
    storage_t content_;

    static void selective_destructor(char* storage, size_t index) {
        throw;
    }

    static void selective_copy_constuctor(char* target, const char* source, size_t index) {
        throw;
    }

    static bool selective_eq(const char* lhs, const char* rhs, size_t index) {
        throw;
    }

    static bool selective_neq(const char* lhs, const char* rhs, size_t index) {
        throw;
    }

    template<typename TFunc, typename ... TArgs>
    static void selective_invoke_const(const char* content, size_t index, TFunc functor, TArgs&&... args) {
        throw;
    }

    template<typename TFunc, typename ... TArgs>
    static void selective_invoke(const char* content, size_t index, TFunc functor, TArgs&&... args) {
        throw;
    }
};

template<typename T, typename ... Ts>
class variant<T, Ts...> {
public:
    using storage_t = char[sizeof(T) > sizeof(typename variant<Ts...>::storage_t) ? sizeof(T) : sizeof(typename variant<Ts...>::storage_t)];

    static void selective_copy_constuctor(char* target, const char* source, size_t index) {
        if (index == 0) {
            new ((T*)target) T{*(T*)source}; // in-place construction using first type's copy constructor
        } else {
            variant<Ts...>::selective_copy_constuctor(target, source, index - 1);
        }
    }

    static void selective_destructor(char* storage, size_t index) {
        if (index == 0) {
            ((T*)storage)->~T();
        } else {
            variant<Ts...>::selective_destructor(storage, index - 1);
        }
    }

    static bool selective_eq(const char* lhs, const char* rhs, size_t index) {
        if (index == 0) {
            return ((*(T*)lhs) == (*(T*)rhs));
        } else {
            return variant<Ts...>::selective_eq(lhs, rhs, index - 1);
        }
    }

    static bool selective_neq(const char* lhs, const char* rhs, size_t index) {
        if (index == 0) {
            return ((*(T*)lhs) != (*(T*)rhs));
        } else {
            return variant<Ts...>::selective_neq(lhs, rhs, index - 1);
        }
    }

    template<typename TFunc, typename ... TArgs>
    static void selective_invoke_const(const char* content, size_t index, TFunc functor, TArgs&&... args) {
        if (index == 0) {
            functor(*(T*)content, std::forward<TArgs>(args)...);
        } else {
            variant<Ts...>::selective_invoke_const(content, index - 1, functor, std::forward<TArgs>(args)...);
        }
    }

    template<typename TFunc, typename ... TArgs>
    static void selective_invoke(char* content, size_t index, TFunc functor, TArgs&&... args) {
        if (index == 0) {
            functor(*(T*)content, std::forward<TArgs>(args)...);
        } else {
            variant<Ts...>::selective_invoke(content, index - 1, functor, std::forward<TArgs>(args)...);
        }
    }

    variant() : index_(0) {
        new ((T*)content_) T{}; // in-place construction using first type's default constructor
    }

    variant(const variant & other) : index_(other.index_) {
        selective_copy_constuctor(content_, other.content_, index_);
    }

    variant(variant&& other) : index_(other.index_) {
        // TODO: implement
        selective_copy_constuctor(content_, other.content_, index_);
    }

    // Find the best match out of `T, Ts...` with `TArg` as the argument.
    template<typename TArg>
    using best_match = decltype(overload_resolver<T, Ts...>()(std::declval<TArg>()));

    template<class TArg, typename TTarget = typename best_match<TArg&&>::type> //, typename=typename std::enable_if_t<!(std::is_same<std::decay_t<U>, variant>::value)>, typename TTarget=decltype(indicator_func(std::forward<U>(std::declval<U>()))), typename TIndex=index_of<TTarget, T, Ts...>>
    variant(TArg&& arg) {
        new ((TTarget*)content_) TTarget{std::forward<TArg>(arg)};
        index_ = index_of<TTarget, T, Ts...>::value;
    }

    ~variant() {
        selective_destructor(content_, index_);
    }

    inline variant& operator=(const variant & other) {
        selective_destructor(content_, index_);
        index_ = other.index_;
        selective_copy_constuctor(content_, other.content_, index_);
        return *this;
    }

    inline bool operator==(const variant& rhs) const {
        return (index_ == rhs.index_) && selective_eq(this->content_, rhs.content_, index_);
    }

    inline bool operator!=(const variant& rhs) const {
        return (index_ != rhs.index_) || selective_neq(this->content_, rhs.content_, index_);
    }

    template<typename TFunc, typename ... TArgs>
    void invoke(TFunc functor, TArgs&&... args) const {
        selective_invoke_const(content_, index_, functor, std::forward<TArgs>(args)...);
    }

    template<typename TFunc, typename ... TArgs>
    void invoke(TFunc functor, TArgs&&... args) {
        selective_invoke(content_, index_, functor, std::forward<TArgs>(args)...);
    }

    storage_t content_;
    size_t index_;

    size_t index() const { return index_; }
};

template<size_t I, typename ... Ts>
std::tuple_element_t<I, std::tuple<Ts...>>& get(std::variant<Ts...>& val) {
    if (val.index() != I)
        throw;
    using T = std::tuple_element_t<I, std::tuple<Ts...>>;
    return *((T*)val.content_);
}

template<typename T, typename ... Ts>
T& get(std::variant<Ts...>& val) {
    constexpr size_t index = std::index_of<T, Ts...>::value;
    return std::get<index>(val);
}


/// Tag type to disengage optional objects.
struct nullopt_t {
  // Do not user-declare default constructor at all for
  // optional_value = {} syntax to work.
  // nullopt_t() = delete;

  // Used for constructing nullopt.
  enum class _Construct { _Token };

  // Must be constexpr for nullopt_t to be literal.
  explicit constexpr nullopt_t(_Construct) { }
};

constexpr nullopt_t nullopt { nullopt_t::_Construct::_Token };

template<typename T>
class optional {
public:
    using storage_t = char[sizeof(T)];

    optional() : has_value_(false) {}
    optional(nullopt_t val) : has_value_(false) {}

    optional(const optional & other) : has_value_(other.has_value_) {
        if (has_value_)
            new ((T*)content_) T{*(T*)other.content_};
    }

    optional(optional&& other) : has_value_(other.has_value_) {
        if (has_value_)
            new ((T*)content_) T{*(T*)other.content_};
    }

    optional(T& arg) {
        new ((T*)content_) T{arg};
        has_value_ = true;
    }

    optional(T&& arg) {
        new ((T*)content_) T{std::forward<T>(arg)};
        has_value_ = true;
    }

    ~optional() {
        if (has_value_)
            ((T*)content_)->~T();
    }

    inline optional& operator=(const optional & other) {
        (**this).~T();
        new (this) optional{other};
        return *this;
    }

    inline bool operator==(const optional& rhs) const {
        return (!has_value_ && !rhs.has_value_) || (*(T*)content_ == *(T*)rhs.content_);
    }

    inline bool operator!=(const optional& rhs) const {
        return !(*this == rhs);
    }

    inline T& operator*() {
        return *(T*)content_;
    }

    inline T* operator->() {
        return (T*)content_;
    }

    storage_t content_;
    size_t has_value_;

    size_t has_value() const { return has_value_; }
};

template<typename T>
optional<T> make_optional(T&& val) {
    return optional<T>{std::forward<T>(val)};
}

template<typename T>
optional<T> make_optional(T& val) {
    return optional<T>{val};
}

} // namespace std

#else
#include <variant>
#include <optional>
#endif

/* Stuff that should be in the STL but isn't ---------------------------------*/

// source: https://en.cppreference.com/w/cpp/experimental/to_array
namespace detail {
template <class T, std::size_t N, std::size_t... I>
constexpr std::array<std::remove_cv_t<T>, N>
    to_array_impl(T (&a)[N], std::index_sequence<I...>)
{
    return { {a[I]...} };
}

template <class T, std::size_t N>
constexpr std::array<std::remove_cv_t<T>, N> to_array(T (&a)[N])
{
    return detail::to_array_impl(a, std::make_index_sequence<N>{});
}
}



/* Custom utils --------------------------------------------------------------*/

// @brief Supports various queries on a list of types
template<typename ... Ts>
class TypeChecker;

template<typename T, typename ... Ts>
class TypeChecker<T, Ts...> {
public:
    using DecayedT = typename std::decay<T>::type;
    
    // @brief Returns false if type T is equal to U or inherits from U. Returns true otherwise.
    template<typename U>
    constexpr static inline bool first_is_not() {
        return !std::is_same<DecayedT, U>::value
            && !std::is_base_of<U, DecayedT>::value;
    }

    // @brief Returns true if all types [T, Ts...] are either equal to U or inherit from U.
    template<typename U>
    constexpr static inline bool all_are() {
        return std::is_base_of<U, DecayedT>::value
            && TypeChecker<Ts...>::template all_are<U>();
    }
    constexpr static const size_t count = TypeChecker<Ts...>::count + 1;
};

template<>
class TypeChecker<> {
public:
    template<typename U>
    constexpr static inline bool first_is_not() {
        return std::true_type::value;
    }
    template<typename U>
    constexpr static inline bool all_are() {
        return std::true_type::value;
    }
    constexpr static const size_t count = 0;
};

template<typename ... Ts>
TypeChecker<Ts...> make_type_checker(Ts ...) {
    return TypeChecker<Ts...>();
}

#include <type_traits>
#define ENABLE_IF(...) \
    typename = typename std::enable_if_t<__VA_ARGS__>

#define ENABLE_IF_SAME(a, b, type) \
    template<typename T = a> typename std::enable_if_t<std::is_same<T, b>::value, type>

template <class T, class M> M get_member_type(M T:: *);
#define GET_TYPE_OF(mem) decltype(get_member_type(mem))


//#include <type_traits>
// @brief Statically asserts that T is derived from type BaseType
#define EXPECT_TYPE(T, BaseType) static_assert(std::is_base_of<BaseType, typename std::decay<T>::type>::value || std::is_convertible<typename std::decay<T>::type, BaseType>::value, "expected template argument of type " #BaseType)
//#define EXPECT_TYPE(T, BaseType) static_assert(, "expected template argument of type " #BaseType)




template<typename TObj, typename TRet, typename ... TArgs>
class function_traits {
public:
    template<unsigned IUnpacked, typename ... TUnpackedArgs, ENABLE_IF(IUnpacked != sizeof...(TArgs))>
    static TRet invoke(TObj& obj, TRet(TObj::*func_ptr)(TArgs...), std::tuple<TArgs...> packed_args, TUnpackedArgs ... args) {
        return invoke<IUnpacked+1>(obj, func_ptr, packed_args, std::forward<TArgs>(args)..., std::get<IUnpacked>(packed_args));
    }

    template<unsigned IUnpacked>
    static TRet invoke(TObj& obj, TRet(TObj::*func_ptr)(TArgs...), std::tuple<TArgs...> packed_args, TArgs ... args) {
        return (obj.*func_ptr)(std::forward<TArgs>(args)...);
    }
};


/* @brief return_type<TypeList>::type represents the C++ native return type
* of a function returning 0 or more arguments.
*
* For an empty TypeList, the return type is void. For a list with
* one type, the return type is equal to that type. For a list with
* more than one items, the return type is a tuple.
*/
template<typename ... Types>
struct return_type;

template<>
struct return_type<> { typedef void type; };
template<typename T>
struct return_type<T> { typedef T type; };
template<typename T, typename ... Ts>
struct return_type<T, Ts...> { typedef std::tuple<T, Ts...> type; };



template<typename ... TInputsAndOutputs>
struct static_function_traits;

// TODO: All invoke-related functions should be superseeded by a proper std::apply implementation
#if 0
template<typename ... TInputs, typename ... TOutputs>
struct static_function_traits<std::tuple<TInputs...>, std::tuple<TOutputs...>> {
    using TRet = typename return_type<TOutputs...>::type;

    //template<TRet(*Function)(TInputs...), unsigned IUnpacked, typename ... TUnpackedInputs, ENABLE_IF(IUnpacked != sizeof...(TInputs))>
    //static std::tuple<TOutputs...> invoke(std::tuple<TInputs...> packed_args, TUnpackedInputs ... args) {
    //    return invoke<Function, IUnpacked+1>(packed_args, args..., std::get<IUnpacked>(packed_args));
    //}

    template<TRet(*Function)(TInputs...)>
    static std::tuple<TOutputs...> invoke(std::tuple<TInputs...>& packed_args) {
        return invoke_impl<Function>(packed_args, std::make_index_sequence<sizeof...(TInputs)>());
    }

    template<TRet(*Function)(TInputs...), size_t... Is>
    static std::tuple<TOutputs...> invoke_impl(std::tuple<TInputs...> packed_args, std::index_sequence<Is...>) {
        return invoke_impl_2<Function>(std::get<Is>(packed_args)...);
    }

    //template<TRet(*Function)(TInputs...)>
    //static std::enable_if_t<(sizeof...(TOutputs) == 0), std::tuple<TOutputs...>>
    template<TRet(*Function)(TInputs...), size_t IOutputs = sizeof...(TOutputs) /*, typename = typename std::enable_if_t<(IOutputs == 0)>>*/>
    static std::enable_if_t<(IOutputs == 0), std::tuple<TOutputs...>>
    invoke_impl_2(TInputs ... args) {
        Function(args...);
        return std::make_tuple<>();
    }

    //template<TRet(*Function)(TInputs...)>
    //static std::enable_if_t<(sizeof...(TOutputs) == 1), std::tuple<TOutputs...>>
    template<TRet(*Function)(TInputs...), size_t IOutputs = sizeof...(TOutputs) /*, typename = typename std::enable_if_t<(IOutputs == 1)>>*/>
    static std::enable_if_t<(IOutputs == 1), std::tuple<TOutputs...>>
    invoke_impl_2(TInputs ... args) {
        return std::make_tuple<TOutputs...>(Function(args...));
    }
//
//    template<TRet(*Function)(TInputs...), ENABLE_IF(sizeof...(TOutputs) >= 2)>
//    static /* std::enable_if_t<sizeof...(TOutputs) >= 2, */ std::tuple<TOutputs...> //>
//    invoke_impl_2(std::tuple<TInputs...> packed_args, TInputs ... args) {
//        return Function(args...);
//    }
};

/* @brief Invoke a class member function with a variable number of arguments that are supplied as a tuple

Example usage:

class MyClass {
public:
    int MyFunction(int a, int b) {
        return 0;
    }
};

MyClass my_object;
std::tuple<int, int> my_args(3, 4); // arguments are supplied as a tuple
int result = invoke_function_with_tuple(my_object, &MyClass::MyFunction, my_args);
*/
template<typename TObj, typename TRet, typename ... TArgs>
TRet invoke_function_with_tuple(TObj& obj, TRet(TObj::*func_ptr)(TArgs...), std::tuple<TArgs...> packed_args) {
    return function_traits<TObj, TRet, TArgs...>::template invoke<0>(obj, func_ptr, packed_args);
}

template<typename ... TOut, typename ... TIn, return_type<TOut...>(*Function)(TIn...)>
std::tuple<TOut...> invoke_with_tuples(std::tuple<TIn...> inputs) {
    static_function_traits<TOut..., TIn...>::template invoke<0>(inputs);
}
#endif


template<typename TInt, TInt... Is>
struct sum_impl;
template<typename TInt>
struct sum_impl<TInt> { static constexpr TInt value = 0; };
template<typename TInt, TInt I, TInt... Is>
struct sum_impl<TInt, I, Is...> { static constexpr TInt value = I + sum_impl<TInt, Is...>::value; };

template<size_t... Is>
using sum = sum_impl<size_t, Is...>;


// source: https://akrzemi1.wordpress.com/2017/05/18/asserts-in-constexpr-functions/
#if defined NDEBUG
# define X_ASSERT(CHECK) void(0)
#else
# define X_ASSERT(CHECK) \
    ( (CHECK) ? void(0) : []{assert(!#CHECK);}() )
#endif

template<class Fn, class Tuple, class>
struct for_each_in_tuple_result_impl;

template<class Fn, class Tuple, size_t... I>
struct for_each_in_tuple_result_impl<Fn, Tuple, std::index_sequence<I...>> {
    typedef std::tuple<decltype(std::forward<Fn>(std::declval<Fn>())(std::get<I>(std::declval<Tuple>())))...> type;
};

template<class Fn, class Tuple>
using for_each_in_tuple_result = for_each_in_tuple_result_impl<Fn, Tuple, std::make_index_sequence<std::tuple_size<std::decay_t<Tuple>>::value>>;

template<class Fn, class Tuple>
using for_each_in_tuple_result_t = typename for_each_in_tuple_result<Fn, Tuple>::type;

template<class Fn, class Tuple, size_t... I>
for_each_in_tuple_result_t<Fn, Tuple> for_each_in_tuple_impl(Fn&& f, Tuple&& t, std::index_sequence<I...>) {
    return for_each_in_tuple_result_t<Fn, Tuple>(std::forward<Fn>(f)(std::get<I>(t))...);
}

template<class Fn, class Tuple>
for_each_in_tuple_result_t<Fn, Tuple> for_each_in_tuple(Fn&& f, Tuple&& t) {
    return for_each_in_tuple_impl(std::forward<Fn>(f), std::forward<Tuple>(t), std::make_index_sequence<std::tuple_size<std::decay_t<Tuple>>::value>{});
}
//template<class Fn, class Tuple>
//for_each_in_tuple_result_t<Fn, Tuple> for_each_in_tuple(Fn&& f, Tuple&& t) {
//    return 5;
//}


/* constexpr strings --------------------------------------------------------*/
/* adapted from:
* https://akrzemi1.wordpress.com/2017/06/28/compile-time-string-concatenation/
*/


// TODO: the functionality
// sstring::substring, sstring::get_last_part and sstring::after_last_index_of and sstring::last_index_of
// was removed during refactoring. Add again if needed.

/**
 * @brief Represents a string that is known at compile time by encoding it as a
 * type.
 */
template<char ... CHARS>
struct sstring {
    static constexpr const char chars[] = {CHARS..., 0};
    static constexpr const char* c_str() { return chars; }
    static constexpr size_t size() { return sizeof...(CHARS); }
    static constexpr std::array<char, sizeof...(CHARS)> as_array() { return {CHARS...}; }

    template<char ... OTHER_CHARS>
    constexpr bool operator==(const sstring<OTHER_CHARS...> & other) const {
        return as_array() == other.as_array();
    }
};
template<char ... CHARS>
constexpr const char sstring<CHARS...>::chars[/*sizeof...(CHARS) + 1*/];

template<typename TStr0, typename TStr1>
struct sstring_concat_impl;

template<char ... STR0, char ... STR1>
struct sstring_concat_impl<sstring<STR0...>, sstring<STR1...>> {
    using type = sstring<STR0..., STR1...>;
};

/** @brief Represents the result type of concatenating two static strings */
template<typename TStr0, typename TStr1>
using sstring_concat_t = typename sstring_concat_impl<TStr0, TStr1>::type;

/** @brief Concatenates two static strings */
template<char ... STR0, char ... STR1>
constexpr sstring<STR0..., STR1...>  operator+(sstring<STR0...>, sstring<STR1...>) {
    return {};
}


/** @brief Helper class for the MAKE_SSTRING macro */
template<size_t LENGTH, char ... CHARS>
struct sstring_builder;

template<char CHAR, char ... CHARS>
struct sstring_builder<0, CHAR, CHARS...> {
    using type = sstring<>;
};

template<size_t LENGTH, char CHAR, char ... CHARS>
struct sstring_builder<LENGTH, CHAR, CHARS...> {
    using type = sstring_concat_t<sstring<CHAR>, typename sstring_builder<LENGTH-1, CHARS...>::type>;
};

template<size_t LENGTH, char ... CHARS>
using sstring_builder_t = typename sstring_builder<LENGTH, CHARS...>::type;

#define MACRO_GET_1(str, i) \
    (sizeof(str) > (i) ? str[(i)] : 0)

#define MACRO_GET_4(str, i) \
    MACRO_GET_1(str, i+0),  \
    MACRO_GET_1(str, i+1),  \
    MACRO_GET_1(str, i+2),  \
    MACRO_GET_1(str, i+3)

#define MACRO_GET_16(str, i) \
    MACRO_GET_4(str, i+0),   \
    MACRO_GET_4(str, i+4),   \
    MACRO_GET_4(str, i+8),   \
    MACRO_GET_4(str, i+12)

#define MACRO_GET_64(str, i) \
    MACRO_GET_16(str, i+0),  \
    MACRO_GET_16(str, i+16), \
    MACRO_GET_16(str, i+32), \
    MACRO_GET_16(str, i+48)

/**
 * @brief Builds a compile-time string type from a string literal.
 * 
 * Passing more than 64 characters will prune the string.
 * 
 * Usage:
 *  MAKE_SSTRING("hello world") my_str{};
 * or
 *  auto my_str = MAKE_SSTRING("hello world"){};
 * 
 * Both examples create a compile-time variable "my_str" of which the type
 * itself stores the content "hello world".
 */
#define MAKE_SSTRING(literal)  sstring_builder_t<sizeof(literal)-1, MACRO_GET_64(literal, 0)>

/*namespace std {
template<char ... CHARS>
static std::ostream& operator<<(std::ostream& stream, const sstring<CHARS...>& val) {
    stream << val.chars;
    return stream;
}
}*/

template<typename TDelimiter, typename ... TStr>
struct join_sstring_impl;

template<char ... DELIMITER>
struct join_sstring_impl<sstring<DELIMITER...>> {
    using type = sstring<>;
};

template<char ... DELIMITER, char ... STR0>
struct join_sstring_impl<sstring<DELIMITER...>, sstring<STR0...>> {
    using type = sstring<STR0...>;
};

template<char ... DELIMITER, char ... STR0, typename ... TStr>
struct join_sstring_impl<sstring<DELIMITER...>, sstring<STR0...>, TStr...> {
    using type = sstring_concat_t<sstring<STR0..., DELIMITER...>, typename join_sstring_impl<sstring<DELIMITER...>, TStr...>::type>;
};

template<typename TDelimiter, typename ... TStr>
using join_sstring_t = typename join_sstring_impl<TDelimiter, TStr...>::type;

template<typename TDelimiter, typename ... TStr>
constexpr join_sstring_t<TDelimiter, TStr...> join_sstring(const TDelimiter& delimiter, const TStr& ... str) {
    return {};
}

template<size_t... ILengths>
using sstring_arr = std::tuple<sstring<ILengths>...>;


// source: https://stackoverflow.com/questions/40159732/return-other-value-if-key-not-found-in-the-map
template<typename TKey, typename TValue>
TValue& get_or(std::unordered_map<TKey, TValue>& m, const TKey& key, TValue& default_value) {
    auto it = m.find(key);
    if (it == m.end()) {
        return default_value;
    } else {
        return it->second;
    }
}
template<typename TKey, typename TValue>
TValue* get_ptr(std::unordered_map<TKey, TValue>& m, const TKey& key) {
    auto it = m.find(key);
    if (it == m.end())
        return nullptr;
    else
        return &(it->second);
}

template <class T, std::size_t = sizeof(T)>
std::true_type is_complete_impl(T *);
std::false_type is_complete_impl(...);

/** @brief is_complete<T> resolves to std::true_type if T is complete
 * and to std::false_type otherwise. This can be used to check if a certain template
 * specialization exists.
 **/
template <class T>
using is_complete = decltype(is_complete_impl(std::declval<T*>()));

template<typename I, typename TRet, typename ... Ts>
struct dynamic_get_impl {
    template<typename TTuple>
    static TRet* get(size_t i, TTuple& t) {
        if (i == I::value)
            return &static_cast<TRet&>(std::get<I::value>(t));
        else if (i > I::value)
            return dynamic_get_impl<std::integral_constant<size_t, I::value + 1>, TRet, Ts...>::get(i, t);
        return nullptr; // this should not happen
    }
};

template<typename TRet, typename ... Ts>
struct dynamic_get_impl<std::integral_constant<size_t, sizeof...(Ts)>, TRet, Ts...> {
    static TRet* get(size_t i, const std::tuple<Ts...>& t) {
        return nullptr;
    }
};

template<typename TRet, typename ... Ts>
TRet* dynamic_get(size_t i, std::tuple<Ts...>& t) {
    return dynamic_get_impl<std::integral_constant<size_t, 0>, TRet, Ts...>::get(i, t);
}

template<typename TRet, typename ... Ts>
TRet* dynamic_get(size_t i, const std::tuple<Ts...>& t) {
    return dynamic_get_impl<std::integral_constant<size_t, 0>, TRet, Ts...>::get(i, t);
}


template<typename TDereferenceable, typename TResult>
class simple_iterator : std::iterator<std::random_access_iterator_tag, TResult> {
    TDereferenceable *container_;
    size_t i_;
public:
    using reference = TResult;
    explicit simple_iterator(TDereferenceable& container, size_t pos) : container_(&container), i_(pos) {}
    simple_iterator& operator++() { ++i_; return *this; }
    simple_iterator operator++(int) { simple_iterator retval = *this; ++(*this); return retval; }
    bool operator==(simple_iterator other) const { return (container_ == other.container_) && (i_ == other.i_); }
    bool operator!=(simple_iterator other) const { return !(*this == other); }
    bool operator<(simple_iterator other) const { return i_ < other.i_; }
    bool operator>(simple_iterator other) const { return i_ > other.i_; }
    bool operator<=(simple_iterator other) const { return (*this < other) || (*this == other); }
    bool operator>=(simple_iterator other) const { return (*this > other) || (*this == other); }
    TResult operator*() const { return (*container_)[i_]; }
};



/**
 * @brief Extracts the argument types of a function signature and provides them
 * as a std::tuple.
 * TODO: if an STL alternative exists, use that
 */
template<typename TFunc>
struct args_of;

template<typename TRet, typename... TArgs>
struct args_of<TRet(TArgs...)> {
    using type = std::tuple<TArgs...>;
};

//template<typename TRet, typename TObj, typename... TArgs>
//struct args_of<_Mem_fn<TRet (TObj::*)(TArgs...)>> {
//    using type = std::tuple<TObj*, TArgs...>;
//};

template<typename TRet, typename TObj, typename... TArgs>
struct args_of<TRet (TObj::*)(TArgs...) const> {
    using type = std::tuple<TObj*, TArgs...>;
};

template<typename TFunc>
struct args_of<TFunc&> : public args_of<TFunc> {};

template<typename TFunc>
using args_of_t = typename args_of<TFunc>::type;

/**
 * @brief Extracts the return type of a function signature
 * 
 * This is provided because std::result_of is deprecated since C++17
 */
template<typename TFunc>
struct result_of;

template<typename TRet, typename... TArgs>
struct result_of<TRet(TArgs...)> {
    using type = TRet;
};

template<typename TRet, typename... TArgs>
struct result_of<TRet(&)(TArgs...)> {
    using type = TRet;
};

template<typename TRet, typename TObj, typename... TArgs>
struct result_of<TRet(TObj::*)(TArgs...) const> {
    using type = TRet;
};

template<typename TFunc>
using result_of_t = typename result_of<TFunc>::type;


/**
 * @brief Returns the type that results when concatenating multiple tuples
 */
template<typename... TTuples>
using tuple_cat_t = decltype(std::tuple_cat<TTuples...>(std::declval<TTuples>()...));

template<typename T, size_t I1, size_t I2, size_t... PACK1, size_t... PACK2>
constexpr std::array<T, I1+I2> array_cat_impl(std::array<T, I1> arr1, std::array<T, I2> arr2, std::index_sequence<PACK1...>, std::index_sequence<PACK2...>) {
    return { arr1[PACK1]..., arr2[PACK2]... };
}

template<typename T, size_t I1, size_t I2>
constexpr std::array<T, I1+I2> array_cat(std::array<T, I1> arr1, std::array<T, I2> arr2) {
    return array_cat_impl(arr1, arr2, std::make_index_sequence<I1>(), std::make_index_sequence<I2>());
}

/**
 * @brief Returns the type that results when concatenating multiple tuples
 */
template<typename... TTuples>
using tuple_cat_t = decltype(std::tuple_cat<TTuples...>(std::declval<TTuples>()...));


/**
 * @brief Ensures that a given type is wrapped in a tuple
 */
template<typename T = void>
struct as_tuple {
    using type = std::tuple<T>;
};

template<>
struct as_tuple<void> {
    using type = std::tuple<>;
};

template<typename... Ts>
struct as_tuple<std::tuple<Ts...>> {
    using type = std::tuple<Ts...>;
};

template<typename T>
using as_tuple_t = typename as_tuple<T>::type;

/**
 * @brief Removes a reference OR pointer from the given type.
 * 
 * This is similar to std::remove_reference, however it can also remove a
 * pointer and it does not work for types that are neither a reference or
 * a pointer.
 */
template<typename T>
struct remove_ref_or_ptr {
    static_assert(std::is_reference<T>() || std::is_pointer<T>(), "the type T is neither a reference or a pointer");
};

template<typename T>
struct remove_ref_or_ptr<T*> { using type = T; };

template<typename T>
struct remove_ref_or_ptr<T&> { using type = T; };

template<typename T>
using remove_ref_or_ptr_t = typename remove_ref_or_ptr<T>::type;

/**
 * @brief Applies remove_ref_or_ptr_t to every type of a tuple type
 */
template<typename T>
struct remove_refs_or_ptrs_from_tuple;

template<typename... Ts>
struct remove_refs_or_ptrs_from_tuple<std::tuple<Ts...>> {
    using type = std::tuple<remove_ref_or_ptr_t<Ts>...>;
};

template<typename T>
using remove_refs_or_ptrs_from_tuple_t = typename remove_refs_or_ptrs_from_tuple<T>::type;

/**
 * @brief The convert(val) function returns a reference or a pointer to val
 * depending on TTo.
 * TODO: this could be a functor
 */
template<typename TTo>
struct add_ref_or_ptr;

template<typename T>
struct add_ref_or_ptr<T&> {
    static T& convert(T& value) {
        return value;
    }
};

template<typename T>
struct add_ref_or_ptr<T*> {
    static T* convert(T& value) {
        return &value;
    }
};


/**
 * @brief The convert() function turns a given tuple of values into a tuple of
 * pointers or references based on the template argument TTo.
 */
template<typename TTo>
struct add_ref_or_ptr_to_tuple;

template<typename... TTo>
struct add_ref_or_ptr_to_tuple<std::tuple<TTo...>> {
    template<typename... TFrom, size_t... Is>
    static std::tuple<TTo...> convert_impl(std::tuple<TFrom...>&& t, std::index_sequence<Is...>) {
        using to_type = std::tuple<TTo...>;
        to_type result(add_ref_or_ptr<std::tuple_element_t<Is, to_type>>::convert(std::get<Is>(t))...);
        return result;
    }

    template<typename... TFrom>
    static std::tuple<TTo...> convert(std::tuple<TFrom...>&& t) {
        static_assert(sizeof...(TFrom) == sizeof...(TTo), "both tuples must have the same size");
        return convert_impl(std::forward<std::tuple<TFrom...>>(t), std::make_index_sequence<sizeof...(TFrom)>());
    }
};

template<typename Ts>
struct add_ptrs_to_tuple_type;

template<typename... Ts>
struct add_ptrs_to_tuple_type<std::tuple<Ts...>> {
    using type = std::tuple<Ts*...>;
};

template<typename TTuple>
using add_ptrs_to_tuple_t = typename add_ptrs_to_tuple_type<TTuple>::type;

template<typename Ts>
struct add_refs_to_tuple_type;

template<typename... Ts>
struct add_refs_to_tuple_type<std::tuple<Ts...>> {
    using type = std::tuple<Ts&...>;
};

template<typename TTuple>
using add_refs_to_tuple_t = typename add_refs_to_tuple_type<TTuple>::type;


template<typename> struct is_tuple: std::false_type {};
template<typename... T> struct is_tuple<std::tuple<T...>>: std::true_type {};


template<typename Is, typename TTuple>
struct tuple_select_type_impl;

template<size_t... Is, typename TTuple>
struct tuple_select_type_impl<std::index_sequence<Is...>, TTuple> {
    using type = std::tuple<std::tuple_element_t<Is, TTuple>...>;
};

template<size_t... Is, typename TTuple>
typename tuple_select_type_impl<std::index_sequence<Is...>, TTuple>::type
tuple_select_impl(TTuple tuple, std::index_sequence<Is...>)  {
    return typename tuple_select_type_impl<std::index_sequence<Is...>, TTuple>::type(std::get<Is>(tuple)...);
};


template<size_t I, typename TTuple>
struct tuple_take_type {
    static_assert(I <= std::tuple_size<TTuple>::value, "cannot take more elements than tuple size");
    using type = typename tuple_select_type_impl<std::make_index_sequence<I>, TTuple>::type;
};

template<size_t I, typename TTuple>
using tuple_take_t = typename tuple_take_type<I, TTuple>::type;

/**
 * @brief Returns the first I elements from the tuple as a tuple.
 * The resulting type is tuple_take_t<I, TTuple>.
 * See also: tuple_skip
 */
template<size_t I, typename TTuple>
tuple_take_t<I, TTuple> tuple_take(TTuple tuple)  {
    return tuple_select_impl(tuple, std::make_index_sequence<I>{});
};


template<size_t I, typename TTuple>
struct tuple_skip_type {
    static_assert(I <= std::tuple_size<TTuple>::value, "cannot skip more elements than tuple size");
    using type = typename tuple_select_type_impl<fibre::make_integer_sequence_from_to<std::size_t, I, std::tuple_size<TTuple>::value>, TTuple>::type;
};

template<size_t I, typename TTuple>
using tuple_skip_t = typename tuple_skip_type<I, TTuple>::type;

/**
 * @brief Returns all but the first I elements from the tuple as a tuple.
 * The resulting type is tuple_skip_t<I, TTuple>.
 * See also: tuple_take
 */
template<size_t I, typename TTuple>
tuple_skip_t<I, TTuple> tuple_skip(TTuple tuple)  {
    return tuple_select_impl(tuple, fibre::make_integer_sequence_from_to<std::size_t, I, std::tuple_size<TTuple>::value>{});
};

template<size_t I, typename T, typename... Ts>
struct repeat_type_impl {
    using type = typename repeat_type_impl<I - 1, T, T, Ts...>::type;
};

template<typename T, typename... Ts>
struct repeat_type_impl<0, T, Ts...> {
    using type = std::tuple<Ts...>;
};

template<size_t I, typename T>
using repeat_t = typename repeat_type_impl<I, T>::type;

#endif // __CPP_UTILS_HPP
