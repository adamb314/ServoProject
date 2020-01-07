#include "CurrentSampler.h"

CurrentSampler::CurrentSampler(uint32_t pin) :
    value(0),
    offset(0),
    activeSample(false),
    sampler(pin)
{
}

CurrentSampler::~CurrentSampler()
{}

void CurrentSampler::init()
{
    AdcHandler::init();

    const size_t numberOfReads = 100;
    offset = 0;
    int32_t temp = 0;
    for (size_t i = 0; i < numberOfReads; i++)
    {
        sampler.triggerSample();
        temp += sampler.getValue();
    }

    offset = temp / numberOfReads;

    value = 0;
}


void CurrentSampler::triggerSample()
{
    sampler.triggerSample();

    activeSample = true;
}

bool CurrentSampler::sampleReady()
{
    return sampler.sampleReady();
}

void CurrentSampler::resetFilteredValue()
{
    filteredValue = value;
}

int32_t CurrentSampler::getValue()
{
    collectSample();

    return value;
}

int32_t CurrentSampler::getFilteredValue()
{
    collectSample();

    return (filteredValue >> 2);
}

void CurrentSampler::collectSample()
{
    if (!activeSample)
    {
        return;
    }

    // Store the value
    while (!sampler.sampleReady());   // Waiting for conversion to complete

    activeSample = false;

    value = sampler.getValue() - offset;

    filteredValue = ((3 * filteredValue) >> 2) + value;
}
