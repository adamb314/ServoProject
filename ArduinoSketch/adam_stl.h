#ifndef ADAM_STL_H
#define ADAM_STL_H

#include <limits>
#include <algorithm>
#include <type_traits>

namespace adam_std
{

template <typename To, typename From>
To clamp_cast(const From from) noexcept
{
    constexpr From minVal = std::numeric_limits<To>::min();
    constexpr From maxVal = std::numeric_limits<To>::max();
    return static_cast<To>(std::min(std::max(from, minVal), maxVal));
}

template<typename T>
T sign(const T& i)
{
    return 1 - 2 * (i < 0);
}

template<typename T, typename T2>
T chooseOne(const T& ifTrue, const T2& ifFalse, bool b)
{
    return ifTrue * b + ifFalse * !b;
}

template<size_t N, typename T>
T wrapAround(const T& i)
{
    T out = i % N;
    return chooseOne(out, out + N, out >= 0);
}

template<size_t N, typename T>
T wrapAroundDist(const T& i)
{
    return wrapAround<N>(i + N / 2) - N / 2;
}

}

#endif
