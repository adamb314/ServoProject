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

    uint16_t getValue();

    uint16_t getStatus();

  private:
    uint16_t value;
    uint16_t status;
};

#endif
