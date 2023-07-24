#ifndef RESISTIVE_ENCODER_HANDLER_H
#define RESISTIVE_ENCODER_HANDLER_H

#include "AdcHandler.h"
#include "EncoderHandler.h"
#include "PwmHandler.h"
#include <array>

class ResistiveEncoderHandler : public EncoderHandlerInterface
{
  public:
    static constexpr size_t vecSize = 513;
    static constexpr size_t superSampling = 16;
    ResistiveEncoderHandler(int16_t pin, float unitsPerRev, const std::array<int16_t, vecSize>& compVec = {0});

    ~ResistiveEncoderHandler();

    virtual void init() override;

    virtual void triggerSample() override;

    virtual float getValue() override;

  protected:
    float getValue(uint16_t sensorValue);

    AnalogSampler sensor;
    float value{0.0f};
    const float scaling;
    std::array<int16_t, vecSize> compVec;
    bool newData{false};
};

#endif
