#ifndef CLAMP_CAST_H
#define CLAMP_CAST_H

#include <limits>

template <typename To, typename From>
To clamp_cast(const From from) noexcept
{
    constexpr From minVal = std::numeric_limits<To>::min();
    constexpr From maxVal = std::numeric_limits<To>::max();
    return static_cast<To>(std::min(std::max(from, minVal), maxVal));
}

#endif
