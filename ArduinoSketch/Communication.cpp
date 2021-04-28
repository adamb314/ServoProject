#include "Communication.h"

Communication::Communication(SerialComOptimizer serial) :
        serial(serial)
{
}

void Communication::addCommunicationNode(std::unique_ptr<CommunicationNode> node)
{
    nodes.push_back(std::move(node));
}

void Communication::onReadyToSendEvent()
{
    nodes[activeNodeIndex]->onReadyToSendEvent();
}

void Communication::onReceiveCompleteEvent()
{
    nodes[activeNodeIndex]->onReceiveCompleteEvent();
}

void Communication::onErrorEvent()
{
    nodes[activeNodeIndex]->onErrorEvent();
}

void Communication::onComCycleEvent()
{
    for (size_t i = 0; i != nodes.size(); ++i)
    {
        nodes[i]->onComCycleEvent();
    }
}

void Communication::onComIdleEvent()
{
    for (size_t i = 0; i != nodes.size(); ++i)
    {
        nodes[i]->onComIdleEvent();
    }
}

void Communication::run()
{
    bool receiveCompleate = false;
    serial.collectReadData();
    if (serial.available() >= waitForBytes)
    {
        lastAvailableReadTimestamp = millis();
        switch (communicationState)
        {
            case 0:
                {
                    unsigned char messageNodeNr = serial.read();
                    
                    if (sendCommunicationState == 0)
                    {
                        communicationError = false;
                    }

                    if (lastMessageNodeNr >= messageNodeNr)
                    {
                        onComCycleEvent();
                    }
                    lastMessageNodeNr = messageNodeNr;

                    for (size_t i = 0; i != nodes.size(); ++i)
                    {
                        if (messageNodeNr == nodes[i]->nodeNr)
                        {
                            activeNodeIndex = i;
                            break;
                        }
                    }

                    if (nodes[activeNodeIndex]->nodeNr == messageNodeNr)
                    {
                        onReadyToSendEvent();

                        intArrayBuffer = nodes[activeNodeIndex]->intArray;
                        charArrayBuffer = nodes[activeNodeIndex]->charArray;
                        intArrayChangedBuffer = nodes[activeNodeIndex]->intArrayChanged;
                        charArrayChangedBuffer = nodes[activeNodeIndex]->charArrayChanged;

                        waitForBytes = 1;
                        communicationState = 2;
                    }
                    else
                    {
                        waitForBytes = 1;
                        communicationState = 1;
                    }
                }
                break;

            case 1:
                serial.read();
                waitForBytes = 1;
                communicationState = 3;
                break;

            case 2:
                checksum = serial.read();
                checksum += nodes[activeNodeIndex]->nodeNr;

                waitForBytes = 1;
                communicationState = 4;
                break;

            case 3:
            case 4:
                messageLength = serial.read();

                checksum += messageLength;

                if (messageLength == 0)
                {
                    waitForBytes = 1;
                    communicationState = 0;
                    break;
                }

                if (communicationState == 3)
                {
                    waitForBytes = 1;
                    communicationState = 100;
                }
                else
                {
                    waitForBytes = 1;
                    communicationState = 10;
                }
                break;

            case 10:
                {
                    command = serial.read();
                    messageLength -= 1;

                    checksum += command;

                    if ((command >> 7) == 1)
                    {
                        sendCommandBuffer[numberOfSendCommands] = command - 128;
                        numberOfSendCommands++;

                        if (messageLength == 0)
                        {
                            waitForBytes = 1;
                            receiveCompleate = true;
                            communicationState = 0;
                        }
                        break;
                    }
                    else if ((command >> 6) == 1)
                    {
                        waitForBytes = 2;
                        communicationState = 30;
                    }
                    else
                    {
                        waitForBytes = 1;
                        communicationState = 20;
                    }
                    
                    if (messageLength == 0)
                    {
                        waitForBytes = 1;
                        communicationError = true;
                        communicationState = 0;
                        break;
                    }
                }
                break;

            case 20:
                if (command < charArrayBuffer.size())
                {
                    unsigned char byteValue = serial.read();

                    charArrayBuffer[command] = byteValue;
                    checksum += byteValue;

                    charArrayChangedBuffer[command] = true;
                }
                else
                {
                    serial.read();
                    communicationError = true;
                }
                messageLength -= 1;

                if (messageLength == 0)
                {
                    waitForBytes = 1;
                    receiveCompleate = true;
                    communicationState = 0;
                }
                else
                {
                    waitForBytes = 1;
                    communicationState = 10;
                }
                break;

            case 30:
                if (messageLength == 1)
                {
                    serial.read();
                    messageLength = 0;
                    waitForBytes = 1;
                    communicationError = true;
                    communicationState = 0;
                }

                if (command >= 64 &&
                        command < 64 + intArrayBuffer.size())
                {
                    unsigned char byteValue = serial.read();
                    signed short value = byteValue;
                    checksum += byteValue;

                    byteValue = serial.read();
                    value += byteValue * static_cast<unsigned short>(256);
                    checksum += byteValue;

                    intArrayBuffer[command - 64] = value;

                    intArrayChangedBuffer[command - 64] = true;
                }
                else
                {
                    serial.read();
                    serial.read();
                    communicationError = true;
                }
                messageLength -= 2;

                if (messageLength == 0)
                {
                    waitForBytes = 1;
                    receiveCompleate = true;
                    communicationState = 0;
                }
                else
                {
                    waitForBytes = 1;
                    communicationState = 10;
                }
                break;

            case 100:
                serial.read();
                messageLength -= waitForBytes;
                if (messageLength == 0)
                {
                    waitForBytes = 1;
                    communicationState = 0;
                }
                else
                {
                    waitForBytes = 1;
                }
                break;
        }
    }
    else
    {
        unsigned long timeStamp = millis();
        if (static_cast<long>(timeStamp - lastAvailableReadTimestamp) > 100)
        {
            lastAvailableReadTimestamp = timeStamp;
            waitForBytes = 1;
            communicationState = 0;
            lastMessageNodeNr = 0;

            onComIdleEvent();
        }
    }

    switch (sendCommunicationState)
    {
        case 0:
            if (communicationState == 10)
            {
                sendCommunicationState = 1;
            }
            break;

        case 1:
            if (currentSendCommandIndex == numberOfSendCommands)
            {
                if (communicationState == 0)
                {
                    currentSendCommandIndex = 0;
                    numberOfSendCommands = 0;

                    if (checksum == 0 && !communicationError)
                    {
                        serial.write(static_cast<unsigned char>(255));
                    }
                    else
                    {
                        onErrorEvent();

                        serial.write(static_cast<unsigned char>(0));
                    }

                    sendCommunicationState = 0;
                }
            }
            else
            {
                sendCommunicationState = 10;
            }
            break;

        case 10:
            {
                unsigned char sendCommand = sendCommandBuffer[currentSendCommandIndex];
                if ((sendCommand >> 6) == 1)
                {
                    int value = 0;
                    if (sendCommand >= 64 &&
                            sendCommand < 64 + nodes[activeNodeIndex]->intArray.size())
                    {
                        value = nodes[activeNodeIndex]->intArray[sendCommand - 64];
                    }
                    serial.write(sendCommand);
                    serial.write(static_cast<unsigned char>(value));
                    serial.write(static_cast<unsigned char>(value >> 8));
                }
                else
                {
                    char value = 0;
                    if (sendCommand < nodes[activeNodeIndex]->charArray.size())
                    {
                        value = nodes[activeNodeIndex]->charArray[sendCommand];
                    }
                    serial.write(sendCommand);
                    serial.write(static_cast<unsigned char>(value));
                }
                currentSendCommandIndex++;
                sendCommunicationState = 1;
            }
    }

    serial.sendWrittenData();

    if (receiveCompleate && checksum == 0)
    {
        nodes[activeNodeIndex]->intArray = intArrayBuffer;
        nodes[activeNodeIndex]->charArray = charArrayBuffer;
        nodes[activeNodeIndex]->intArrayChanged = intArrayChangedBuffer;
        nodes[activeNodeIndex]->charArrayChanged = charArrayChangedBuffer;

        onReceiveCompleteEvent();
    }
}
