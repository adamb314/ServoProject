#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <SPI.h>

class EncoderHandlerInterface
{
  public:
    virtual void init() = 0;

    virtual void triggerSample() = 0;

    virtual float getValue() = 0;
};

class EncoderHandler : public EncoderHandlerInterface
{
  public:
    EncoderHandler(int chipSelectPin);

    ~EncoderHandler();

    virtual void init() override;

    virtual void triggerSample() override;

    virtual float getValue() override;

    uint16_t getStatus();

  private:
    int chipSelectPin;
    float value;
    float wrapAroundCorretion;
    uint16_t status;
};

#endif
