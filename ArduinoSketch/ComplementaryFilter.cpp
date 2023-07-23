#include "ComplementaryFilter.h"
#include "adam_stl.h"

ComplementaryFilter::ComplementaryFilter(float x0)
{
    set(x0);
}

void ComplementaryFilter::update(float lowFrqIn, float highFrqIn)
{
    long int lowFrqInFixed = (lowFrqIn - outWrapAround) * fixedPoint;
    long int highFrqInFixed = highFrqIn * fixedPoint;

    long int highFrqDiffFixed = adam_std::wrapAroundDist<wrapAroundSize * fixedPoint>(
                highFrqInFixed - lastHighFrqInFixed);
    outFixed = (aFixed * (outFixed + highFrqDiffFixed) +
            (fixedPoint - aFixed) * lowFrqInFixed) / fixedPoint;
    lastHighFrqInFixed = highFrqInFixed;

    long int outFixedWraped = adam_std::wrapAround<wrapAroundSize * fixedPoint>(outFixed);
    outWrapAround += (outFixed - outFixedWraped) / (wrapAroundSize * fixedPoint) * wrapAroundSize;
    outFixed = outFixedWraped;
}

float ComplementaryFilter::get()
{
    return outFixed * (1.0f / fixedPoint) + outWrapAround;
}

void ComplementaryFilter::set(float x0)
{
    outFixed = x0 * fixedPoint;
}

void ComplementaryFilter::setFilterConst(float a)
{
    aFixed = a * fixedPoint;
    aFixed = std::min(std::max(aFixed, (long int)0), (long int)fixedPoint - 1);
}
