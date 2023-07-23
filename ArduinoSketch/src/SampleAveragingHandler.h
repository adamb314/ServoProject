#include "Hardware/ThreadHandler.h"

#ifndef SAMPLE_AVERAGING_HANDLER_H
#define SAMPLE_AVERAGING_HANDLER_H

template <int32_t fixedPoint, int32_t filterConst>
class SampleMovingAveraging
{
public:
    void reset(int32_t v)
    {
        value = v * fixedPoint;
    }

    void add(int32_t v)
    {
        value = (filterConst * value) / fixedPoint + v;
    }

    int32_t get()
    {
        return value / fixedPoint;
    }

protected:
    int32_t value{0};
};

template <typename T, int maxN, bool threadSafe=true>
class SampleAveragingHandler
{
public:
    void add(T v);
    T get(bool reset = true);

protected:
    bool valueRead{false};
    T sum{};
    T halfSampleSum{};
    int n{0};
};

template <typename T, int maxN>
class SampleAveragingHandler<T, maxN, true> : public SampleAveragingHandler<T, maxN, false>
{
public:
    void add(T v);
    T get(bool reset = true);

protected:
    using unsafeImp = SampleAveragingHandler<T, maxN, false>;
};

template <typename T, int maxN, bool threadSafe>
void SampleAveragingHandler<T, maxN, threadSafe>::add(T v)
{
    if (valueRead || maxN == 1)
    {
        valueRead = false;
        n = 0;
        sum = 0;
        halfSampleSum = 0;
    }

    if (n >= maxN)
    {
        sum = halfSampleSum;
        halfSampleSum = 0;
        n = maxN / 2;
    }

    if (n >= maxN / 2)
    {
        halfSampleSum += v;
    }

    ++n;
    sum += v;
}

template <typename T, int maxN, bool threadSafe>
T SampleAveragingHandler<T, maxN, threadSafe>::get(bool reset)
{
    T tempSum;
    int tempN;
    {
        ThreadInterruptBlocker blocker;
        tempSum = sum;
        tempN = n;
        valueRead = reset;
    }
    return tempSum / tempN;
}

template <typename T, int maxN>
void SampleAveragingHandler<T, maxN, true>::add(T v)
{
    ThreadInterruptBlocker blocker;
    unsafeImp::add(v);
}

template <typename T, int maxN>
T SampleAveragingHandler<T, maxN, true>::get(bool reset)
{
    T tempSum;
    int tempN;
    {
        ThreadInterruptBlocker blocker;
        tempSum = unsafeImp::sum;
        tempN = unsafeImp::n;
        unsafeImp::valueRead = reset;
    }
    return tempSum / tempN;
}

#endif
