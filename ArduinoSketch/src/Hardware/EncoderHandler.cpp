#include "EncoderHandler.h"

EncoderHandler::EncoderHandler(int chipSelectPin, float unitsPerRev, const std::array<int16_t, vecSize>& compVec) :
    EncoderHandlerInterface(unitsPerRev),
    chipSelectPin(chipSelectPin), value(0), wrapAroundCorretion(0), status(0),
    scaling(unitsPerRev * (1.0f / 4096.0f)),
    compVec(compVec)
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
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));

    digitalWrite(chipSelectPin, LOW);

    delayMicroseconds(1);

    uint16_t send = 0xffff;
    uint16_t received;
    
    received = SPI.transfer16(send);
    SPI.endTransaction();
    digitalWrite(chipSelectPin, HIGH);
    
    received = -received;
    float newValue = (received & 0x3fff);
    if (scaling < 0)
    {
        newValue = 0x3fff - newValue;
    }
    newValue *= 0.25f;

    if (newValue - value > 4096 / 2)
    {
        wrapAroundCorretion -= 4096;
    }
    else if (newValue - value < -4096 / 2)
    {
        wrapAroundCorretion += 4096;
    }
    value = newValue;

    int32_t t = (value * vecSize + 32) / 64;
    size_t i = std::min(vecSize - 2, static_cast<size_t>(t / 64));
    t -= i * 64;
    value -= (compVec[i] * (64 - t) + compVec[i + 1] * t + 32) / 64;

    status = 0;
}

float EncoderHandler::getValue()
{
    return (value + wrapAroundCorretion) * std::abs(scaling);
}

uint16_t EncoderHandler::getStatus()
{
    return status;
}
