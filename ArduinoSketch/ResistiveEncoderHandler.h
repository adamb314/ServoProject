#ifndef RESISTIVE_ENCODER_HANDLER_H
#define RESISTIVE_ENCODER_HANDLER_H

#include "AdcHandler.h"
#include "EncoderHandler.h"
#include <array>

class ResistiveEncoderHandler : public EncoderHandlerInterface
{
  public:
    static constexpr int vecSize = 32;
    ResistiveEncoderHandler(int16_t pin, float unitsPerRev, const std::array<int16_t, vecSize>& compVec);

    ~ResistiveEncoderHandler();

    virtual void init() override;

    virtual void triggerSample() override;

    virtual float getValue() override;

  private:
    AnalogSampler sensor;
    float sensorValue{0};
    const float scaling;
    std::array<int16_t, vecSize> compVec;
};

#endif
