#ifndef FILTERED_ADC_H
#define FILTERED_ADC_H

#include <Arduino.h>
#undef max
#undef min

class FilteredADC
{
public:
    FilteredADC(float filterConstant);

    ~FilteredADC();

    void configureAdcPin(uint32_t pin);

    void initOffset(uint32_t pin);

    void triggerSample(uint32_t pin);
    void triggerSample();

    void collectSample();

    float getValue();

  private:
    uint32_t readSumValue;
    uint32_t numberOfReads;
    float filteredValue;
    float a;
    float offset;
    bool activeSample;
};

#endif
