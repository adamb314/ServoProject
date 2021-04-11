#include <Arduino.h>
#include "ArduinoC++BugFixes.h"
#include <array>
#include <vector>
#include "SerialComOptimizer.h"

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

class CommunicationNode
{
public:
    CommunicationNode(unsigned char nodeNr) :
        nodeNr(nodeNr)
    {
    }

    virtual void onReadyToSendEvent(){};
    virtual void onReceiveCompleteEvent(){};
    virtual void onErrorEvent(){};
    virtual void onComCycleEvent(){};
    virtual void onComIdleEvent(){};

protected:
    std::array<int, 16> intArray{{0}};
    std::array<char, 8> charArray{{0}};

    std::array<bool, 16> intArrayChanged{{false}};
    std::array<bool, 8> charArrayChanged{{false}};

private:
    unsigned char nodeNr;
    friend class Communication;
};

class Communication
{
public:
    Communication(Stream* serial, unsigned long baud = 115200);

    void addCommunicationNode(std::unique_ptr<CommunicationNode> node);

    void run();

private:
    void onReadyToSendEvent();
    void onReceiveCompleteEvent();
    void onErrorEvent();
    void onComCycleEvent();
    void onComIdleEvent();

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

    std::vector<std::unique_ptr<CommunicationNode> > nodes;
    size_t activeNodeIndex;

    friend class CommunicationNode;
};

#endif
