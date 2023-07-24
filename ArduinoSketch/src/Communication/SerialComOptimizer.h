#include <Arduino.h>
#include "../ArduinoC++BugFixes.h"
#include <array>
#include <vector>
#include <algorithm>

#ifndef SERIAL_OPTIMIZER_H
#define SERIAL_OPTIMIZER_H

class SerialComOptimizer
{
public:
    SerialComOptimizer(Stream* serial, Stream* secSerial = nullptr);
    SerialComOptimizer(const SerialComOptimizer& in);

    ~SerialComOptimizer();

    void addBridge(Stream* bridge);

    size_t available();

    uint8_t read();

    bool write(uint8_t byte);

    void collectReadData();

    void sendWrittenData();

private:
    void repeatReceivedToBridges(const std::array<char, 32>& buffer, int32_t messageLength);
    void writeBridgedResponse();

    Stream* serial;
    std::vector<Stream*> serialVec;
    uint32_t comLastActiveTimestamp;
    std::vector<Stream*> bridgeSerialVec;

    std::array<uint8_t, 32> readBuffer;
    std::array<uint8_t, 32>::iterator readBufferGetIt;
    std::array<uint8_t, 32>::iterator readBufferPutIt;

    std::array<uint8_t, 32> writeBuffer;
    std::array<uint8_t, 32>::iterator writeBufferPutIt;
};

#endif
