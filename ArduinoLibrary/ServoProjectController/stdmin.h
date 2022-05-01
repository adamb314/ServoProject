#include <Arduino.h>
#undef max
#undef min
#undef abs
#undef round

#ifndef STD_MIN_H
#define STD_MIN_H

namespace stdmin
{

template <class T>
struct remove_reference { typedef T type; };

template <class T>
struct remove_reference<T&> { typedef T type; };

template <class T>
struct remove_reference<T&&> { typedef T type; };

template <typename T>
typename remove_reference<T>::type&& move(T&& arg)
{
  return static_cast<typename remove_reference<T>::type&&>(arg);
}

template< class T > struct remove_cv                   { typedef T type; };
template< class T > struct remove_cv<const T>          { typedef T type; };
template< class T > struct remove_cv<volatile T>       { typedef T type; };
template< class T > struct remove_cv<const volatile T> { typedef T type; };

template< class T >
struct decay {
private:
    typedef typename remove_reference<T>::type U;
public:
    typedef typename remove_cv<U>::type type;
};

/// make_signed
template <typename T> struct add_const { typedef const T type; };
template <typename T> struct add_const<const T> { typedef const T type; };

template <typename T> struct add_volatile { typedef volatile T type; };
template <typename T> struct add_volatile<volatile T> { typedef volatile T type; };
template <typename T> struct make_signed { typedef  T type; };
template <> struct make_signed<char> { typedef  signed char type; };
template <> struct make_signed<unsigned char> { typedef  signed char type; };

template <> struct make_signed<unsigned short> { typedef  short type; };
template <> struct make_signed<unsigned int> { typedef int type; };
template <> struct make_signed<unsigned long> { typedef  long type; };
template <> struct make_signed<unsigned long long> { typedef long long type; };
template <typename T> struct make_signed<const T> : add_const<typename make_signed<T>::type> {};
template <typename T> struct make_signed<volatile T> : add_volatile<typename make_signed<T>::type> {};
template <typename T> struct make_signed<const volatile T> : add_const<typename add_volatile<typename make_signed<T>::type>::type> {};

template<class T> 
const T& min(const T& a, const T& b)
{
    return (b < a) ? b : a;
}

template<class T> 
const T& max(const T& a, const T& b)
{
    return (a < b) ? b : a;
}

template<class T> 
const T abs(const T& a)
{
    return (a < 0) ? -a : a;
}

template<class T> 
T round(const T& x)
{
    return x >= 0 ? static_cast<long>(x + 0.5) : static_cast<long>(x - 0.5);
}

}

#endif
