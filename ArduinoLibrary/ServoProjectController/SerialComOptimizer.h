#include <Arduino.h>
#include "CppArray.h"
#include "FixedMemVector.h"

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

    bool read(uint8_t& c, uint32_t timeoutInMs);

    bool write(uint8_t byte);

    void collectReadData();

    void sendWrittenData();

private:
    void repeatReceivedToBridges(const CppArray<char, 32>& buffer, int32_t messageLength);
    void writeBridgedResponse();

    Stream* serial;
    FixedMemVector<Stream*, 2> serialVec;
    uint32_t comLastActiveTimestamp;
    FixedMemVector<Stream*, 2> bridgeSerialVec;

    CppArray<uint8_t, 32> readBuffer;
    CppArray<uint8_t, 32>::iterator readBufferGetIt;
    CppArray<uint8_t, 32>::iterator readBufferPutIt;

    CppArray<uint8_t, 32> writeBuffer;
    CppArray<uint8_t, 32>::iterator writeBufferPutIt;
};

#endif
