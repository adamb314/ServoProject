#include "ResistiveEncoderHandler.h"

ResistiveEncoderHandler::ResistiveEncoderHandler(const float& scaling) :
        scaling{scaling}
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
