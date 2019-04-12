#include <Arduino.h>
#undef max
#undef min

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

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
