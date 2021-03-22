#include "ResistiveEncoderHandler.h"

ResistiveEncoderHandler::ResistiveEncoderHandler(int16_t pin, float unitsPerRev, const std::array<int16_t, vecSize>& compVec) :
        sensor(pin),
        scaling{unitsPerRev * (1.0 / 4096.0)},
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
    sensorValue = 0;

    const int n = 4;
    for (int i = 0; i != n; ++i)
    {
        sensor.triggerSample();
        sensorValue += sensor.getValue();
    }

    sensorValue /= n;

    float t = sensorValue * (vecSize / 4096.0) - 0.5;
    t = std::max(0.0f, t);
    int i = std::min(vecSize - 2, static_cast<int>(t));
    t -= i;
    sensorValue += compVec[i] * (1.0 - t) + compVec[i + 1] * t;

    sensorValue *= scaling;
}

float ResistiveEncoderHandler::getValue()
{
    return sensorValue;
}
