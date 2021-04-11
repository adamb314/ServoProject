#ifndef CURRENT_SAMPLER_H
#define CURRENT_SAMPLER_H

#include <Arduino.h>
#include "ArduinoC++BugFixes.h"

#include "AdcHandler.h"

class CurrentSampler
{
public:
    CurrentSampler(uint32_t pin);

    ~CurrentSampler();

    void init();

    void triggerSample();

    bool sampleReady();

    void resetFilteredValue();

    int32_t getValue();
    int32_t getFilteredValue();

  private:
    void collectSample();

    int32_t value;
    int32_t filteredValue;
    int32_t offset;
    bool activeSample;

    AverageAnalogSampler sampler;
};

#endif
