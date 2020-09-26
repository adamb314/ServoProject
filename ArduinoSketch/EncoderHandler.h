#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <SPI.h>

class EncoderHandlerInterface
{
  public:
    virtual void init() = 0;

    virtual void triggerSample() = 0;

    virtual float getValue() = 0;

    class DiagnosticData
    {
      public:
        uint16_t a{0};
        uint16_t b{0};
        uint16_t c{0};
        uint16_t d{0};
    };

    virtual DiagnosticData getDiagnosticData()
    {
        return DiagnosticData();
    }
};

class EncoderHandler : public EncoderHandlerInterface
{
  public:
    EncoderHandler(int chipSelectPin, float unitsPerRev = 4096.0);

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
    const float scaling;
};

#endif
