#include "EncoderHandler.h"

EncoderHandler::EncoderHandler(int chipSelectPin, float unitsPerRev) :
    EncoderHandlerInterface(unitsPerRev),
    chipSelectPin(chipSelectPin), value(0), wrapAroundCorretion(0), status(0),
    scaling(unitsPerRev * (1.0 / 4096.0))
{
}

EncoderHandler::~EncoderHandler()
{
}

void EncoderHandler::init()
{
    SPI.begin();
    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, HIGH);
    triggerSample();
}

void EncoderHandler::triggerSample()
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

    digitalWrite(chipSelectPin, LOW);

    delayMicroseconds(1);

    uint16_t send = 0xffff;
    uint16_t received;
    
    received = SPI.transfer16(send);
    SPI.endTransaction();
    digitalWrite(chipSelectPin, HIGH);
    
    received = -received;
    float newValue = (received & 0x3fff) * 0.25;
    if (newValue - value > 4096 / 2)
    {
        wrapAroundCorretion -= 4096;
    }
    else if (newValue - value < -4096 / 2)
    {
        wrapAroundCorretion += 4096;
    }
    value = newValue;
    status = 0;
}

float EncoderHandler::getValue()
{
    return (value + wrapAroundCorretion) * scaling;
}

uint16_t EncoderHandler::getStatus()
{
    return status;
}
