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
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));

    digitalWrite(A5, LOW);

    delayMicroseconds(1);

    uint8_t buffer[3] = {0};
    
    SPI.transfer(buffer, sizeof(buffer));
    SPI.endTransaction();
    digitalWrite(A5, HIGH);
    buffer[0] = (buffer[0] << 1) | (buffer[1] >> 7);
    buffer[1] = (buffer[1] << 1) | (buffer[2] >> 7);
    buffer[2] = (buffer[2] << 1);

    value = (buffer[0] << 4) | (buffer[1] >> 4);
    status = (buffer[1] << 4) | (buffer[2] >> 6);
}

uint16_t EncoderHandler::getValue()
{
    return value;
}

uint16_t EncoderHandler::getStatus()
{
    return status;
}
