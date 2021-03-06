#include "ResistiveEncoderHandler.h"

ResistiveEncoderHandler::ResistiveEncoderHandler(int16_t pin, float unitsPerRev, const std::array<int16_t, vecSize>& compVec) :
        EncoderHandlerInterface(unitsPerRev),
        sensor(pin, ADC_CTRLB_PRESCALER_DIV16_Val),
        scaling{unitsPerRev * (1.0f / 4096.0f) / 16.0f},
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
    sensor.triggerSample(ADC_AVGCTRL_SAMPLENUM_16_Val);
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

        float t = newSensorValue * (vecSize / 4096.0f);
        int i = std::min(vecSize - 2, static_cast<int>(t));
        t -= i;
        newSensorValue -= compVec[i] * (1.0f - t) + compVec[i + 1] * t;

        sensorValue = (15 * sensorValue) / 16 + newSensorValue;
    }
    return sensorValue * std::abs(scaling);
}
