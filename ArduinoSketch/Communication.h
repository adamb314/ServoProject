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
    virtual void comIdleRun(){};

protected:
    std::array<int, 16> intArray{{0}};
    std::array<char, 16> charArray{{0}};

    std::array<bool, 16> intArrayChanged{{false}};
    std::array<bool, 16> charArrayChanged{{false}};

private:
    unsigned char nodeNr;
    friend class Communication;
};

class Communication
{
public:
    Communication(SerialComOptimizer serial);

    void addCommunicationNode(std::unique_ptr<CommunicationNode> node);

    void run();

private:
    void onReadyToSendEvent();
    void onReceiveCompleteEvent();
    void onErrorEvent();
    void onComCycleEvent();
    void onComIdleEvent();
    void comIdleRun();

    SerialComOptimizer serial;
    std::array<int, 16> intArrayBuffer;
    std::array<char, 16> charArrayBuffer;

    std::array<bool, 16> intArrayChangedBuffer;
    std::array<bool, 16> charArrayChangedBuffer;

    int communicationState{0};
    unsigned char waitForBytes{1};
    unsigned char messageLength{0};
    unsigned char command{0};
    unsigned char checksum{0};
    bool communicationError{false};
    unsigned long lastAvailableReadTimestamp{millis()};

    int sendCommunicationState{0};
    unsigned char numberOfSendCommands{0};
    unsigned char currentSendCommandIndex{0};
    std::array<unsigned char, 100> sendCommandBuffer;

    unsigned char lastMessageNodeNr{0};

    std::vector<std::unique_ptr<CommunicationNode> > nodes;
    size_t activeNodeIndex{0};

    friend class CommunicationNode;
};

#endif
