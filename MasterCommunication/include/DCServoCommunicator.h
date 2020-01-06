#include "MasterCommunication.h"
#include <sstream>

#ifndef DC_SERVO_COMMUNICATION_H
#define DC_SERVO_COMMUNICATION_H

class DCServoCommunicator
{
  public:
    DCServoCommunicator(unsigned char nodeNr, Communication* bus);

    void setOffsetAndScaling(double scale, double offset);

    bool isInitComplete();

    bool isCommunicationOk();

    void setReference(const float& pos, const float& vel, const float& feedforwardU);

    float getPosition();

    float getVelocity();

    float getControlSignal();

    float getCurrent();

    float getControlError();

    int getCpuLoad();

    int getLoopTime();

    bool runModelIdentTest(unsigned char testSequenceNumber, unsigned int amplitude);

    std::string getRecordedModelIdentData();

    void run();

  private:
    Communication* bus;
    unsigned char nodeNr;

    bool communicationIsOk;

    int initState;
    bool newReference;

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

    class ModelIdentHandler
    {
    public:
        ModelIdentHandler();

        bool runModelIdentTest(unsigned char testSequenceNumber, unsigned int amplitude);

        std::string getRecordedData();

        bool activeRecording();

        void handleWrite(Communication* bus);

        void handleRead(Communication* bus, int encoderPos, int controlSignal, int current, int loopTime);

        unsigned int runModelIdentState;
        unsigned char testSequenceNumber;
        unsigned int amplitude;
        int lastLoopTime;
        std::stringstream dataBuilder;
    };

    ModelIdentHandler modelIdentHandler;
};

#endif
