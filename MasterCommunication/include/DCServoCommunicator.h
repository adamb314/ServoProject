#include "MasterCommunication.h"
#include <sstream>
#include <array>

#ifndef DC_SERVO_COMMUNICATION_H
#define DC_SERVO_COMMUNICATION_H

class DCServoCommunicator
{
  public:
    DCServoCommunicator(unsigned char nodeNr, Communication* bus);

    DCServoCommunicator(const DCServoCommunicator&) = delete;

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
    Communication* bus{nullptr};
    unsigned char nodeNr{0};

    bool communicationIsOk{false};

    int initState{0};
    bool backlashControlDisabled{false};
    bool newPositionReference{false};
    bool newOpenLoopControlSignal{false};

    std::array<bool, 10> activeIntReads{false};

    float backlashEncoderPos{0.0};
    float encoderPos{0.0};
    int encoderVel{0};
    int controlSignal{0};
    int current{0};
    int cpuLoad{0};
    int loopTime{0};

    int refPos{0};
    std::array<int, 5> activeRefPos{0};
    int refVel{0};
    int feedforwardU{0};

    double offset{0.0};
    double scale{1.0};
};

#endif
