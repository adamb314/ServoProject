#include "MasterCommunication.h"
#include <sstream>
#include <array>

#ifndef DC_SERVO_COMMUNICATION_H
#define DC_SERVO_COMMUNICATION_H

class DCServoCommunicator
{
  public:
    class OpticalEncoderChannelData
    {
    public:
        unsigned int a{0};
        unsigned int b{0};
        unsigned int minCostIndex{0};
        unsigned int minCost{0};
    };

    DCServoCommunicator(unsigned char nodeNr, Communication* bus);

    DCServoCommunicator(const DCServoCommunicator&) = delete;

    void setOffsetAndScaling(double scale, double offset);

    void disableBacklashControl(bool b = true);

    bool isInitComplete();

    bool isCommunicationOk();

    void setReference(const float& pos, const float& vel, const float& feedforwardU);

    void setOpenLoopControlSignal(const float& feedforwardU, bool pwmMode);

    float getPosition(bool withBacklash = true);

    float getVelocity();

    float getControlSignal();

    float getFeedforwardU();

    float getCurrent();

    int getPwmControlSignal();

    float getControlError(bool withBacklash = true);

    int getCpuLoad();

    int getLoopTime();

    float getBacklashCompensation();

    OpticalEncoderChannelData getOpticalEncoderChannelData();

    void run();

  private:
    Communication* bus{nullptr};
    unsigned char nodeNr{0};

    bool communicationIsOk{false};

    int initState{0};
    bool backlashControlDisabled{false};
    bool newPositionReference{false};
    bool newOpenLoopControlSignal{false};
    bool pwmOpenLoopMode{false};

    std::array<bool, 16> activeIntReads{false};
    std::array<int, 16> intReadBuffer{0};

    float backlashEncoderPos{0.0};
    float encoderPos{0.0};
    float backlashCompensation{0.0};
    int encoderVel{0};
    int controlSignal{0};
    int current{0};
    int pwmControlSignal{0};
    int cpuLoad{0};
    int loopTime{0};
    OpticalEncoderChannelData opticalEncoderChannelData;

    int refPos{0};
    std::array<int, 5> activeRefPos{0};
    int refVel{0};
    int feedforwardU{0};
    std::array<int, 5> activeFeedforwardU{0};

    double offset{0.0};
    double scale{1.0};
};

#endif
