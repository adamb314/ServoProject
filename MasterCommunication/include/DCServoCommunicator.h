#include "MasterCommunication.h"
#include <sstream>
#include <array>

#ifndef DC_SERVO_COMMUNICATION_H
#define DC_SERVO_COMMUNICATION_H

#include <type_traits>

template <typename T, typename U>
class ContinuousValueUpCaster
{
  public:
    typedef typename std::decay<T>::type ValueType;
    typedef typename std::decay<U>::type InputType;

    const ValueType& get()
    {
        return value;
    }

    void set(const ValueType& v)
    {
        value = v;
    }

    void update(const InputType& input)
    {
        typedef typename std::make_signed<InputType>::type SignedInputType;

        SignedInputType diff = input - value;

        value += diff;
    }

  protected:
    ValueType value{0};
};

class DCServoCommunicator
{
  public:
    class OpticalEncoderChannelData
    {
    public:
        unsigned short int a{0};
        unsigned short int b{0};
        unsigned short int minCostIndex{0};
        unsigned short int minCost{0};
    };

    DCServoCommunicator(unsigned char nodeNr, Communication* bus);

    DCServoCommunicator(const DCServoCommunicator&) = delete;

    void setOffsetAndScaling(double scale, double offset, double startPosition = 0);

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

    short int getPwmControlSignal();

    float getControlError(bool withBacklash = true);

    short int getCpuLoad();

    short int getLoopTime();

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
    std::array<short int, 16> intReadBuffer{0};

    ContinuousValueUpCaster<long int, short int> intReadBufferIndex3Upscaling;
    ContinuousValueUpCaster<long int, short int> intReadBufferIndex10Upscaling;
    ContinuousValueUpCaster<long int, short int> intReadBufferIndex11Upscaling;

    float backlashEncoderPos{0.0};
    float encoderPos{0.0};
    float backlashCompensation{0.0};
    short int encoderVel{0};
    short int controlSignal{0};
    short int current{0};
    short int pwmControlSignal{0};
    short int cpuLoad{0};
    short int loopTime{0};
    OpticalEncoderChannelData opticalEncoderChannelData;

    long int refPos{0};
    std::array<long int, 5> activeRefPos{0};
    short int refVel{0};
    short int feedforwardU{0};
    std::array<short int, 5> activeFeedforwardU{0};

    double offset{0.0};
    double scale{1.0};

    static constexpr int positionUpscaling = 32;
};

#endif
