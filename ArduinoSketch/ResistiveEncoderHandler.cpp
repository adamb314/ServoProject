#include "ResistiveEncoderHandler.h"

ResistiveEncoderHandler::ResistiveEncoderHandler(int16_t pin, float unitsPerRev) :
        sensor(pin),
        scaling{unitsPerRev * (1.0 / 4096.0)}
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
    sensorValue *= scaling;
}

float ResistiveEncoderHandler::getValue()
{
    return sensorValue;
}
