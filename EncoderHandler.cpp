#include "EncoderHandler.h"

EncoderHandler::EncoderHandler() :
    value(0), status(0)
{
}

EncoderHandler::~EncoderHandler()
{
}

void EncoderHandler::init()
{
    SPI.begin();
    pinMode(A5, OUTPUT);
    digitalWrite(A5, HIGH);
}

void EncoderHandler::triggerSample()
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

    digitalWrite(A5, LOW);

    delayMicroseconds(1);

    uint16_t send = 0xffff;
    uint16_t received;
    
    received = SPI.transfer16(send);
    SPI.endTransaction();
    digitalWrite(A5, HIGH);
    
    value = (received & 0x3fff) * 0.25;
    status = 0;
}

float EncoderHandler::getValue()
{
    return value;
}

uint16_t EncoderHandler::getStatus()
{
    return status;
}
