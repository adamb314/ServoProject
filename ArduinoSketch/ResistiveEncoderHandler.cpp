#include "ResistiveEncoderHandler.h"

ResistiveEncoderHandler::ResistiveEncoderHandler(int16_t pin, float unitsPerRev, const std::array<int16_t, vecSize>& compVec) :
        EncoderHandlerInterface(unitsPerRev),
        sensor(pin),
        scaling{unitsPerRev * (1.0 / 4096.0) / 16.0},
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
    uint32_t newSensorValue = 0;

    const int n = 1;
    for (int i = 0; i != n; ++i)
    {
        sensor.triggerSample();
        newSensorValue += sensor.getValue();
    }

    newSensorValue /= n;

    float t = newSensorValue * (vecSize / 4096.0);
    int i = std::min(vecSize - 2, static_cast<int>(t));
    t -= i;
    newSensorValue -= compVec[i] * (1.0 - t) + compVec[i + 1] * t;

    sensorValue = (15 * sensorValue) / 16 + newSensorValue;
}

float ResistiveEncoderHandler::getValue()
{
    return sensorValue * scaling;
}
