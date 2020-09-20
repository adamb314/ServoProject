#ifndef RESISTIVE_ENCODER_HANDLER_H
#define RESISTIVE_ENCODER_HANDLER_H

#include "AdcHandler.h"
#include "EncoderHandler.h"

class ResistiveEncoderHandler : public EncoderHandlerInterface
{
  public:
    ResistiveEncoderHandler(const float& scaling);

    ~ResistiveEncoderHandler();

    virtual void init() override;

    virtual void triggerSample() override;

    virtual float getValue() override;

  private:
    AnalogSampler sensor{A1};
    float sensorValue{0};
    const float scaling;
};

#endif
