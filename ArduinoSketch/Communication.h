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

    virtual void onReadyToSendEvent() = 0;
    virtual void onReceiveCompleteEvent() = 0;
    virtual void onErrorEvent() = 0;
    virtual void onComCycleEvent() = 0;
    virtual void onComIdleEvent() = 0;

    virtual void run();

protected:
    std::array<int, 16> intArray;
    std::array<char, 8> charArray;

    std::array<bool, 16> intArrayChanged;
    std::array<bool, 8> charArrayChanged;

private:
    SerialComOptimizer serial;
    std::array<int, 16> intArrayBuffer;
    std::array<char, 8> charArrayBuffer;

    std::array<bool, 16> intArrayChangedBuffer;
    std::array<bool, 8> charArrayChangedBuffer;

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
    std::array<unsigned char, 100> sendCommandBuffer;

    unsigned char lastMessageNodeNr;

    unsigned char nodeNr;
};
#endif
