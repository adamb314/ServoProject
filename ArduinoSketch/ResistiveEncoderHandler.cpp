#include "ResistiveEncoderHandler.h"

ResistiveEncoderHandler::ResistiveEncoderHandler(int16_t pin, float unitsPerRev, const std::array<int16_t, vecSize>& compVec) :
        EncoderHandlerInterface(unitsPerRev),
        sensor(pin, ADC_CTRLB_PRESCALER_DIV16_Val),
        scaling{unitsPerRev * (1.0f / 4096.0f) / 1.0f},
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
        uint32_t newSensorValue = sensor.getValue() / 16;

        if (scaling < 0)
        {
            newSensorValue = 0xfff - newSensorValue;
        }

        int32_t t = (newSensorValue * vecSize + 32) / 64;
        size_t i = std::min(vecSize - 2, static_cast<size_t>(t / 64));
        t -= i * 64;
        newSensorValue -= (compVec[i] * (64 - t) + compVec[i + 1] * t + 32) / 64;

        sensorValue = newSensorValue;
    }
    return sensorValue * std::abs(scaling);
}
