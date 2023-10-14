#include "ResistiveEncoderHandler.h"

ResistiveEncoderHandler::ResistiveEncoderHandler(int16_t pin, float unitsPerRev, const std::array<int16_t, vecSize>& compVec) :
        EncoderHandlerInterface(unitsPerRev),
        sensor(pin, ADC_CTRLB_PRESCALER_DIV16_Val),
        scaling{unitsPerRev * (1.0f / 4096.0f) / superSampling},
        compVec(compVec)
{
}

ResistiveEncoderHandler::~ResistiveEncoderHandler()
{
}

void ResistiveEncoderHandler::init()
{
    AdcHandler::init();

    triggerSample();
    getValue();
}

void ResistiveEncoderHandler::triggerSample()
{
    sensor.triggerSample(ADC_AVGCTRL_SAMPLENUM_64_Val);
    newData = true;
}

float ResistiveEncoderHandler::getValue()
{
    if (newData)
    {
        newData = false;
        uint32_t sensorValue = sensor.getValue();

        value = getValue(sensorValue);
    }
    return value;
}

float ResistiveEncoderHandler::getValue(uint16_t sensorValue)
{
    if (scaling < 0)
    {
        sensorValue = 0xffff - sensorValue;
    }

    int32_t t = ((sensorValue / superSampling) * vecSize + 32) / 64;
    size_t i = std::min(vecSize - 2, static_cast<size_t>(t / 64));
    t -= i * 64;
    constexpr int32_t compFixedPoint = 64 / superSampling;
    int32_t comp = compVec[i] * (64 - t) + compVec[i + 1] * t;
    comp = (comp + adam_std::sign(comp) * (compFixedPoint / 2)) / compFixedPoint;

    return (sensorValue - comp) * std::abs(scaling);
}
