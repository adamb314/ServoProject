#include "Communication.h"

Communication::Communication(unsigned char nodeNr, unsigned long baud)
{
  this->nodeNr = nodeNr;
  Serial.begin(baud);
  waitForBytes = 1;
  communicationState = 0;
  messageLength = 0;
  checksum = 0;
  communicationError = false;
  lastAvailableReadTimestamp = millis();

  sendCommunicationState = 0;
  currentSendCommandIndex = 0;
  numberOfSendCommands = 0;
}
  
bool Communication::run()
{
  bool receiveCompleate = false;
  if (Serial.available() >= waitForBytes)
  {
    lastAvailableReadTimestamp = millis();
    switch (communicationState)
    {
      case 0:
        {
          unsigned char messageNodeNr = Serial.read();
          
          if (sendCommunicationState == 0)
          {
            communicationError = false;
          }

          if (nodeNr == messageNodeNr)
          {
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
        Serial.read();
        waitForBytes = 1;
        communicationState = 3;
        break;

      case 2:
        checksum = Serial.read();
        checksum += nodeNr;

        waitForBytes = 1;
        communicationState = 4;
        break;

      case 3:
      case 4:
        messageLength = Serial.read();

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
          command = Serial.read();
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
          unsigned char byteValue = Serial.read();

          charArrayBuffer[command] = byteValue;
          checksum += byteValue;

          charArrayChangedBuffer[command] = true;
        }
        else
        {
          Serial.read();
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
          Serial.read();
          messageLength = 0;
          waitForBytes = 1;
          communicationError = true;
          communicationState = 0;
        }

        if (command >= 64 &&
            command < 64 + sizeof(intArrayBuffer) / sizeof(intArrayBuffer[0]))
        {
          unsigned char byteValue = Serial.read();
          signed short value = byteValue;
          checksum += byteValue;

          byteValue = Serial.read();
          value += byteValue * static_cast<unsigned short>(256);
          checksum += byteValue;

          intArrayBuffer[command - 64] = value;

          intArrayChangedBuffer[command - 64] = true;
        }
        else
        {
          Serial.read();
          Serial.read();
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
        Serial.read();
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
            Serial.write(static_cast<unsigned char>(255));
          }
          else
          {
            Serial.write(static_cast<unsigned char>(0));
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
          Serial.write(sendCommand);
          Serial.write(static_cast<unsigned char>(value));
          Serial.write(static_cast<unsigned char>(value >> 8));
        }
        else
        {
          char value = 0;
          if (sendCommand < sizeof(charArray) / sizeof(charArray[0]))
          {
            value = charArray[sendCommand];
          }
          Serial.write(sendCommand);
          Serial.write(static_cast<unsigned char>(value));
        }
        currentSendCommandIndex++;
        sendCommunicationState = 1;
      }
  }

  if (receiveCompleate && checksum == 0)
  {
    memcpy(intArray, intArrayBuffer, sizeof(intArray));
    memcpy(charArray, charArrayBuffer, sizeof(charArray));
    memcpy(intArrayChanged, intArrayChangedBuffer, sizeof(intArrayChanged));
    memcpy(charArrayChanged, charArrayChangedBuffer, sizeof(charArrayChanged));
  }

  return receiveCompleate && !communicationError;
}

bool Communication::blockingRun()
{
  bool receiveCompleate = false;
  while (Serial.available() >= waitForBytes ||
         communicationState != 0 ||
         sendCommunicationState != 0)
  {
    run();
  }

  return !communicationError;
}
