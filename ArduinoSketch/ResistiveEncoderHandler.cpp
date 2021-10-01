#include "ResistiveEncoderHandler.h"

ResistiveEncoderHandler::ResistiveEncoderHandler(int16_t pin, float unitsPerRev, const std::array<int16_t, vecSize>& compVec,
            std::shared_ptr<SwitchAvoidingSynchronizer> synchronizer) :
        EncoderHandlerInterface(unitsPerRev),
        sensor(pin),
        scaling{unitsPerRev * (1.0f / 4096.0f) / 16.0f},
        compVec(compVec),
        synchronizer(synchronizer)
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
        if (synchronizer)
        {
            while (synchronizer->willSwitchWithIn(16))
            {
            }
        }
        sensor.triggerSample();
        newSensorValue += sensor.getValue();
    }

    newSensorValue /= n;

    float t = newSensorValue * (vecSize / 4096.0f);
    int i = std::min(vecSize - 2, static_cast<int>(t));
    t -= i;
    newSensorValue -= compVec[i] * (1.0f - t) + compVec[i + 1] * t;

    sensorValue = (15 * sensorValue) / 16 + newSensorValue;
}

float ResistiveEncoderHandler::getValue()
{
    return sensorValue * scaling;
}
