#include <Arduino.h>
#undef max
#undef min
#include <array>

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

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

class Communication
{
public:
    Communication(unsigned char nodeNr, unsigned long baud = 9600);

    bool run();

    bool blockingRun();

    int intArray[16];
    char charArray[8];

    bool intArrayChanged[16];
    bool charArrayChanged[8];

private:
    SerialComOptimizer serial;
    int intArrayBuffer[16];
    char charArrayBuffer[8];

    bool intArrayChangedBuffer[16];
    bool charArrayChangedBuffer[8];

    int communicationState;
    unsigned char waitForBytes;
    unsigned char messageLength;
    unsigned char command;
    unsigned char checksum;
    bool communicationError;
    unsigned long lastAvailableReadTimestamp;

    int sendCommunicationState;
    unsigned char numberOfSendCommands;
    unsigned char currentSendCommandIndex;
    unsigned char sendCommandBuffer[100];

    unsigned char nodeNr;
};
#endif
