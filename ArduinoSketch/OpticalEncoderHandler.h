#ifndef OPTICAL_ENCODER_HANDLER_H
#define OPTICAL_ENCODER_HANDLER_H

#include "AdcHandler.h"
#include "EncoderHandler.h"
#include "PwmHandler.h"
#include <array>

class OpticalEncoderHandler : public EncoderHandlerInterface
{
  public:
    static constexpr int vecSize = 512;
    OpticalEncoderHandler(const std::array<uint16_t, vecSize>& aVec, const std::array<uint16_t, vecSize>& bVec,
            int16_t sensor1Pin, int16_t sensor2Pin, float unitsPerRev,
            std::shared_ptr<SwitchAvoidingSynchronizer> synchronizer = std::shared_ptr<SwitchAvoidingSynchronizer>());

    ~OpticalEncoderHandler();

    virtual void init() override;

    virtual void triggerSample() override;

    virtual float getValue() override;

    virtual uint16_t getUnscaledRawValue() override;

    virtual DiagnosticData getDiagnosticData() override;

  private:
    void updatePosition();
    uint32_t calcCost(int& i, uint16_t a, uint16_t b);

    DiagnosticData diagnosticData;

    uint16_t lastMinCostIndex{0};
    uint16_t iAtLastOffsetUpdate{0};
    uint16_t predictNextPos{0};

    std::array<uint16_t, vecSize> aVec;
    std::array<uint16_t, vecSize> bVec;

    AnalogSampler sensor1;
    AnalogSampler sensor2;
    uint16_t sensor1Value{0};
    uint16_t sensor2Value{0};
    float sensorValueOffset{0.0f};
    float value{0.0f};
    float wrapAroundCorretion{0.0f};
    bool newData{false};

    std::shared_ptr<SwitchAvoidingSynchronizer> synchronizer;

    const float scaling;
};

#endif
