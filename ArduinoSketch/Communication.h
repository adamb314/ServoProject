#include <Arduino.h>
#undef max
#undef min
#include <array>
#include "SerialComOptimizer.h"

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

class CommunicationInterface
{
public:
    virtual void onReadyToSendEvent() = 0;
    virtual void onReceiveCompleteEvent() = 0;
    virtual void onErrorEvent() = 0;
    virtual void onComCycleEvent() = 0;
    virtual void onComIdleEvent() = 0;

    virtual void run() = 0;
};

template <size_t N = 1>
class Communication : public CommunicationInterface
{
public:
    Communication(std::array<unsigned char, N> nodeNrArray, unsigned long baud = 9600);

    virtual void onReadyToSendEvent() = 0;
    virtual void onReceiveCompleteEvent() = 0;
    virtual void onErrorEvent() = 0;
    virtual void onComCycleEvent() = 0;
    virtual void onComIdleEvent() = 0;

    virtual void run();

protected:
    std::array<std::array<int, 16>, N> intArray;
    std::array<std::array<char, 8>, N> charArray;

    std::array<std::array<bool, 16>, N> intArrayChanged;
    std::array<std::array<bool, 8>, N> charArrayChanged;

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

    std::array<unsigned char, N> nodeNrArray;
    size_t nodeNrIndex;
};

#define COMMUNICATION_CPP
#include "Communication.cpp"
#undef COMMUNICATION_CPP

#endif
