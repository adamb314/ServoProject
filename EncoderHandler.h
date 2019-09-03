#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <SPI.h>

class EncoderHandler
{
  public:
    EncoderHandler();

    ~EncoderHandler();

    void init();

    void triggerSample();

    float getValue();

    uint16_t getStatus();

  private:
    float value;
    uint16_t status;
};

#endif
