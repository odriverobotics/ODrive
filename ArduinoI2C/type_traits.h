/*
* This file is a very small part of the GCC STL because AVR-GCC ships
* without the STL.
*/

namespace std
{

  /**
   * @defgroup metaprogramming Metaprogramming
   * @ingroup utilities
   *
   * Template utilities for compile-time introspection and modification,
   * including type classification traits, type property inspection traits
   * and type transformation traits.
   *
   * @{
   */

  /// integral_constant
  template<typename _Tp, _Tp __v>
    struct integral_constant
    {
      static constexpr _Tp                  value = __v;
      typedef _Tp                           value_type;
      typedef integral_constant<_Tp, __v>   type;
      constexpr operator value_type() const noexcept { return value; }
#if __cplusplus > 201103L

#define __cpp_lib_integral_constant_callable 201304

      constexpr value_type operator()() const noexcept { return value; }
#endif
    };

  template<typename _Tp, _Tp __v>
    constexpr _Tp integral_constant<_Tp, __v>::value;

  /// The type used as a compile-time boolean with true value.
  typedef integral_constant<bool, true>     true_type;

  /// The type used as a compile-time boolean with false value.
  typedef integral_constant<bool, false>    false_type;

  template<bool __v>
    using __bool_constant = integral_constant<bool, __v>;

#if __cplusplus > 201402L
# define __cpp_lib_bool_constant 201505
  template<bool __v>
    using bool_constant = integral_constant<bool, __v>;
#endif


  // Primary type categories.

  template<typename>
    struct remove_cv;

  template<typename>
    struct __is_void_helper
    : public false_type { };

  template<>
    struct __is_void_helper<void>
    : public true_type { };

  /// is_void
  template<typename _Tp>
    struct is_void
    : public __is_void_helper<typename remove_cv<_Tp>::type>::type
    { };

  template<typename>
    struct __is_integral_helper
    : public false_type { };

  template<>
    struct __is_integral_helper<bool>
    : public true_type { };

  template<>
    struct __is_integral_helper<char>
    : public true_type { };

  template<>
    struct __is_integral_helper<signed char>
    : public true_type { };

  template<>
    struct __is_integral_helper<unsigned char>
    : public true_type { };

#ifdef _GLIBCXX_USE_WCHAR_T
  template<>
    struct __is_integral_helper<wchar_t>
    : public true_type { };
#endif

  template<>
    struct __is_integral_helper<char16_t>
    : public true_type { };

  template<>
    struct __is_integral_helper<char32_t>
    : public true_type { };

  template<>
    struct __is_integral_helper<short>
    : public true_type { };

  template<>
    struct __is_integral_helper<unsigned short>
    : public true_type { };

  template<>
    struct __is_integral_helper<int>
    : public true_type { };

  template<>
    struct __is_integral_helper<unsigned int>
    : public true_type { };

  template<>
    struct __is_integral_helper<long>
    : public true_type { };

  template<>
    struct __is_integral_helper<unsigned long>
    : public true_type { };

  template<>
    struct __is_integral_helper<long long>
    : public true_type { };

  template<>
    struct __is_integral_helper<unsigned long long>
    : public true_type { };

  // Conditionalizing on __STRICT_ANSI__ here will break any port that
  // uses one of these types for size_t.
#if defined(__GLIBCXX_TYPE_INT_N_0)
  template<>
    struct __is_integral_helper<__GLIBCXX_TYPE_INT_N_0>
    : public true_type { };

  template<>
    struct __is_integral_helper<unsigned __GLIBCXX_TYPE_INT_N_0>
    : public true_type { };
#endif
#if defined(__GLIBCXX_TYPE_INT_N_1)
  template<>
    struct __is_integral_helper<__GLIBCXX_TYPE_INT_N_1>
    : public true_type { };

  template<>
    struct __is_integral_helper<unsigned __GLIBCXX_TYPE_INT_N_1>
    : public true_type { };
#endif
#if defined(__GLIBCXX_TYPE_INT_N_2)
  template<>
    struct __is_integral_helper<__GLIBCXX_TYPE_INT_N_2>
    : public true_type { };

  template<>
    struct __is_integral_helper<unsigned __GLIBCXX_TYPE_INT_N_2>
    : public true_type { };
#endif
#if defined(__GLIBCXX_TYPE_INT_N_3)
  template<>
    struct __is_integral_helper<__GLIBCXX_TYPE_INT_N_3>
    : public true_type { };

  template<>
    struct __is_integral_helper<unsigned __GLIBCXX_TYPE_INT_N_3>
    : public true_type { };
#endif

  /// is_integral
  template<typename _Tp>
    struct is_integral
    : public __is_integral_helper<typename remove_cv<_Tp>::type>::type
    { };

  template<typename>
    struct __is_floating_point_helper
    : public false_type { };

  template<>
    struct __is_floating_point_helper<float>
    : public true_type { };

  template<>
    struct __is_floating_point_helper<double>
    : public true_type { };

  template<>
    struct __is_floating_point_helper<long double>
    : public true_type { };

#if !defined(__STRICT_ANSI__) && defined(_GLIBCXX_USE_FLOAT128)
  template<>
    struct __is_floating_point_helper<__float128>
    : public true_type { };
#endif

  /// is_floating_point
  template<typename _Tp>
    struct is_floating_point
    : public __is_floating_point_helper<typename remove_cv<_Tp>::type>::type
    { };




  // Const-volatile modifications.

  /// remove_const
  template<typename _Tp>
    struct remove_const
    { typedef _Tp     type; };

  template<typename _Tp>
    struct remove_const<_Tp const>
    { typedef _Tp     type; };

  /// remove_volatile
  template<typename _Tp>
    struct remove_volatile
    { typedef _Tp     type; };

  template<typename _Tp>
    struct remove_volatile<_Tp volatile>
    { typedef _Tp     type; };

  /// remove_cv
  template<typename _Tp>
    struct remove_cv
    {
      typedef typename
      remove_const<typename remove_volatile<_Tp>::type>::type     type;
    };


  // Primary template.
  /// Define a member typedef @c type only if a boolean constant is true.
  template<bool, typename _Tp = void>
    struct enable_if
    { };

  // Partial specialization for true.
  template<typename _Tp>
    struct enable_if<true, _Tp>
    { typedef _Tp type; };


  // Type relations.

  /// is_same
  template<typename, typename>
    struct is_same
    : public false_type { };

  template<typename _Tp>
    struct is_same<_Tp, _Tp>
    : public true_type { };
}
