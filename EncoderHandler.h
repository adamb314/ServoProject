#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <SPI.h>

class EncoderHandler
{
  public:
    EncoderHandler(int chipSelectPin);

    ~EncoderHandler();

    void init();

    void triggerSample();

    float getValue();

    uint16_t getStatus();

  private:
    int chipSelectPin;
    float value;
    float wrapAroundCorretion;
    uint16_t status;
};

#endif
