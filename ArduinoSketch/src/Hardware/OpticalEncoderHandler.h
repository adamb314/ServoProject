#ifndef OPTICAL_ENCODER_HANDLER_H
#define OPTICAL_ENCODER_HANDLER_H

#include "../adam_stl.h"
#include "AdcHandler.h"
#include "EncoderHandler.h"
#include "PwmHandler.h"
#include <array>
#include "../SampleAveragingHandler.h"

class ValueScaler
{
public:
    ValueScaler(int32_t minOut, int32_t maxOut, int32_t minIn, int32_t maxIn);

    void init(int32_t minOut, int32_t maxOut, int32_t minIn, int32_t maxIn);

    void approxInputRangeUpdate(int32_t min, int32_t max);

    int32_t getOutput(int32_t input);

protected:
    static constexpr int32_t maxInt16 = 1 << 16;
    static constexpr int32_t one14FixPoint = 1 << 14;
    static constexpr int32_t one16FixPoint = maxInt16;

    int32_t minOut;
    int32_t maxOut;
    int32_t outOverInitInDiff16FixPoint;
    int32_t initInMinMaxDiff;
    int32_t oneOverInitInDiff30FixPoint;
    int32_t minIn;
    int32_t maxIn;
    int32_t initInAndInDiffDiff;
};

class OpticalEncoderHandler : public EncoderHandlerInterface
{
public:
    static constexpr int vecSize = 2048;
    OpticalEncoderHandler(const std::array<uint16_t, vecSize>& aVec, const std::array<uint16_t, vecSize>& bVec,
            int16_t sensor1Pin, int16_t sensor2Pin, float unitsPerRev);
    OpticalEncoderHandler(const std::array<uint16_t, 512>& aVec, const std::array<uint16_t, 512>& bVec,
            int16_t sensor1Pin, int16_t sensor2Pin, float unitsPerRev);

    ~OpticalEncoderHandler();

    virtual void init() override;

    virtual void triggerSample() override;

    virtual float getValue() override;

    virtual uint16_t getUnscaledRawValue() override;

    virtual DiagnosticData getDiagnosticData() override;

    static void configurePullUpDown(int16_t portGroup, int16_t n, bool pullHigh = true);

protected:
    class OpticalSensorValueScaler
    {
    public:
        void init(const std::array<uint16_t, vecSize>& refVec);

        uint16_t get(uint16_t value);

        void update(int i, uint16_t value);

    private:
        static constexpr int indexDelta = 128;

        const std::array<uint16_t, vecSize>* pointerToRefVec{nullptr};
        int32_t minRef;
        int32_t maxRef;
        int iAtMinRef;
        int iAtMaxRef;
        int32_t minVal;
        int32_t maxVal;
        int32_t minAvgCorr;
        int32_t maxAvgCorr;
        int moveBeforeMinUpdate{indexDelta * 2};
        int moveBeforeMaxUpdate{indexDelta * 2};

        ValueScaler valueScaler{0, 1 << 16, 0, 1 << 16};
        SampleMovingAveraging<64, 63> diffAtMinRefAverager;
        SampleMovingAveraging<64, 63> diffAtMaxRefAverager;
    };

    float getValue(uint16_t sensor1Value, uint16_t sensor2Value);

    std::tuple<int32_t, uint32_t> calcPosition(uint16_t sensor1Value, uint16_t sensor2Value);

    DiagnosticData diagnosticData;

    std::array<uint16_t, vecSize> aVec;
    std::array<uint16_t, vecSize> bVec;

    AnalogSampler sensor1;
    AnalogSampler sensor1Sec;
    AnalogSampler sensor2;
    AnalogSampler sensor2Sec;
    OpticalSensorValueScaler sensor1Scaler;
    OpticalSensorValueScaler sensor2Scaler;
    float value{0.0f};
    int32_t pos{0};
    int32_t wrapAroundCorretion{0};
    bool newData{false};
    bool sampleSensor1First{false};

    const float scaling;
};

#endif
