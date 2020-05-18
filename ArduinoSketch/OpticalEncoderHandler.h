#ifndef OPTICAL_ENCODER_HANDLER_H
#define OPTICAL_ENCODER_HANDLER_H

#include "AdcHandler.h"
#include "EncoderHandler.h"
#include <array>

class OpticalEncoderHandler : public EncoderHandlerInterface
{
  public:
    static constexpr int vecSize = 512;
    OpticalEncoderHandler(const std::array<uint16_t, vecSize>& aVec, const std::array<uint16_t, vecSize>& bVec);

    ~OpticalEncoderHandler();

    virtual void init() override;

    virtual void triggerSample() override;

    virtual float getValue() override;

    class DiagnosticData
    {
      public:
        uint16_t a;
        uint16_t b;
        uint16_t minCostIndex;
        uint16_t minCost;
    };

    DiagnosticData getDiagnosticData();

  private:
    void updatePosition();
    uint32_t calcCost(int& i, uint16_t a, uint16_t b);

    DiagnosticData diagnosticData;

    uint16_t lastMinCostIndex{0};
    uint16_t predictNextPos{0};

    std::array<uint16_t, vecSize> aVec;
    std::array<uint16_t, vecSize> bVec;

    AnalogSampler sensor1;
    AnalogSampler sensor2;
    uint16_t sensor1Value{0};
    uint16_t sensor2Value{0};
    float value;
    float wrapAroundCorretion;
    bool newData;
};

#endif
