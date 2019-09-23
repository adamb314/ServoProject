#include "Communication.h"

Communication::Communication(unsigned char nodeNr, unsigned long baud) :
    serial(SerialComOptimizer(&Serial1)),
    intArray{0},
    charArray{0},
    intArrayChanged{false},
    charArrayChanged{false}
{
  this->nodeNr = nodeNr;
  Serial1.begin(baud);
  waitForBytes = 1;
  communicationState = 0;
  messageLength = 0;
  checksum = 0;
  communicationError = false;
  lastAvailableReadTimestamp = millis();

  sendCommunicationState = 0;
  currentSendCommandIndex = 0;
  numberOfSendCommands = 0;

  lastMessageNodeNr = 0;
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

          if (nodeNr == messageNodeNr)
          {
            onReadyToSendEvent();

            memcpy(intArrayBuffer, intArray, sizeof(intArrayBuffer));
            memcpy(charArrayBuffer, charArray, sizeof(charArrayBuffer));
            memcpy(intArrayChangedBuffer, intArrayChanged, sizeof(intArrayChangedBuffer));
            memcpy(charArrayChangedBuffer, charArrayChanged, sizeof(charArrayChangedBuffer));

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
        checksum += nodeNr;

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
        if (command < sizeof(charArrayBuffer) / sizeof(charArrayBuffer[0]))
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
            command < 64 + sizeof(intArrayBuffer) / sizeof(intArrayBuffer[0]))
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
    if (static_cast<long>(millis() - lastAvailableReadTimestamp) > 500)
    {
      waitForBytes = 1;
      communicationState = 0;
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
              sendCommand < 64 + sizeof(intArray) / sizeof(intArray[0]))
          {
            value = intArray[sendCommand - 64];
          }
          serial.write(sendCommand);
          serial.write(static_cast<unsigned char>(value));
          serial.write(static_cast<unsigned char>(value >> 8));
        }
        else
        {
          char value = 0;
          if (sendCommand < sizeof(charArray) / sizeof(charArray[0]))
          {
            value = charArray[sendCommand];
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
    memcpy(intArray, intArrayBuffer, sizeof(intArray));
    memcpy(charArray, charArrayBuffer, sizeof(charArray));
    memcpy(intArrayChanged, intArrayChangedBuffer, sizeof(intArrayChanged));
    memcpy(charArrayChanged, charArrayChangedBuffer, sizeof(charArrayChanged));

    onReceiveCompleteEvent();
  }
}

SerialComOptimizer::SerialComOptimizer(Stream* serial) :
    serial(serial),
    readBufferGetIt(readBuffer.end() - 1),
    readBufferPutIt(readBuffer.begin()),
    writeBufferPutIt(writeBuffer.begin())
{
}

SerialComOptimizer::~SerialComOptimizer()
{
}

size_t SerialComOptimizer::available()
{
    int8_t bufferedAmount = readBufferPutIt - (readBufferGetIt + 1);
    if (bufferedAmount < 0)
    {
        bufferedAmount = readBuffer.size() + bufferedAmount;
    }
    return bufferedAmount;
}

uint8_t SerialComOptimizer::read()
{
    ++readBufferGetIt;
    if (readBufferGetIt == readBuffer.end())
    {
        readBufferGetIt = readBuffer.begin();
    }

    return *readBufferGetIt;
}

void SerialComOptimizer::write(uint8_t byte)
{
    *writeBufferPutIt = byte;
    ++writeBufferPutIt;
}

void SerialComOptimizer::collectReadData()
{
    int32_t readAmount = readBufferGetIt - readBufferPutIt;
    if (readAmount < 0)
    {
        readAmount = readBuffer.size() + readAmount;
    }

    if (readAmount == 0)
    {
        return;
    }

    int32_t availableInHardwarBuffer = serial->available();
    if (availableInHardwarBuffer < readAmount)
    {
        readAmount = availableInHardwarBuffer;
    }

    std::array<char, 32> tempReadBuffer;
    readAmount = serial->readBytes(tempReadBuffer.data(), static_cast<size_t>(readAmount));

    if (readAmount < 0)
    {
        readAmount = 0;
    }

    for (auto it = tempReadBuffer.begin(); it != tempReadBuffer.begin() + readAmount; ++it)
    {
        *readBufferPutIt = *it;

        ++readBufferPutIt;
        if (readBufferPutIt == readBuffer.end())
        {
            readBufferPutIt = readBuffer.begin();
        }
    }
}

void SerialComOptimizer::sendWrittenData()
{
    serial->write(writeBuffer.data(), writeBufferPutIt - writeBuffer.begin());
    writeBufferPutIt = writeBuffer.begin();
}
