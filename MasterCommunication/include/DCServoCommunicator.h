#include "MasterCommunication.h"
#include <sstream>
#include <array>

#ifndef DC_SERVO_COMMUNICATION_H
#define DC_SERVO_COMMUNICATION_H

class DCServoCommunicator
{
  public:
    DCServoCommunicator(unsigned char nodeNr, Communication* bus);

    void setOffsetAndScaling(double scale, double offset);

    void disableBacklashControl(bool b = true);

    bool isInitComplete();

    bool isCommunicationOk();

    void setReference(const float& pos, const float& vel, const float& feedforwardU);

    void setOpenLoopControlSignal(const float& feedforwardU);

    float getPosition(bool withBacklash = true);

    float getVelocity();

    float getControlSignal();

    float getCurrent();

    float getControlError();

    int getCpuLoad();

    int getLoopTime();

    void run();

  private:
    Communication* bus;
    unsigned char nodeNr;

    bool communicationIsOk;

    int initState;
    bool backlashControlDisabled;
    bool newPositionReference;
    bool newOpenLoopControlSignal;

    std::array<bool, 10> activeIntReads;

    float backlashEncoderPos;
    float encoderPos;
    int encoderVel;
    int controlSignal;
    int current;
    int cpuLoad;
    int loopTime;

    int refPos;
    int activeRefPos[5];
    int refVel;
    int feedforwardU;

    double offset;
    double scale;
};

#endif
