#ifndef CURRENT_SAMPLER_H
#define CURRENT_SAMPLER_H

#include <Arduino.h>
#undef max
#undef min

class CurrentSampler
{
public:
    CurrentSampler();

    ~CurrentSampler();

    void init(uint32_t pin);

    void triggerSample();

    int32_t getValue();
    int32_t getFilteredValue();

  private:
    void configureAdcPin(uint32_t pin);
    void collectSample();

    int32_t value;
    int32_t filteredValue;
    int32_t offset;
    bool activeSample;
};

#endif
