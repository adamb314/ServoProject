#include <Arduino.h>
#include "ArduinoC++BugFixes.h"
#include <array>

#ifndef SERIAL_OPTIMIZER_H
#define SERIAL_OPTIMIZER_H

class SerialComOptimizer
{
public:
    SerialComOptimizer(Stream* serial);

    ~SerialComOptimizer();

    size_t available();

    uint8_t read();

    void write(uint8_t byte);

    void collectReadData();

    void sendWrittenData();

private:
    Stream* serial;

    std::array<uint8_t, 32> readBuffer;
    std::array<uint8_t, 32>::iterator readBufferGetIt;
    std::array<uint8_t, 32>::iterator readBufferPutIt;

    std::array<uint8_t, 32> writeBuffer;
    std::array<uint8_t, 32>::iterator writeBufferPutIt;
};

#endif
