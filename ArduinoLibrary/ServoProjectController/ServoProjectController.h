#include <Arduino.h>
#include "CppArray.h"
#include "FixedMemVector.h"
#include "SerialComOptimizer.h"

#ifndef SERVO_PROJECT_H
#define SERVO_PROJECT_H

class CommunicationError
{
public:
    enum ErrorCode
    {
        NO_ERROR = 0,
        COULD_NOT_SEND,
        NO_RESPONSE,
        PARTIAL_RESPONSE_TYPE_1,
        PARTIAL_RESPONSE_TYPE_2,
        PARTIAL_RESPONSE_TYPE_3,
        PARTIAL_RESPONSE_TYPE_4,
        UNEXPECTED_RESPONSE,
        CHECKSUM_ERROR
    };

    CommunicationError()
    {
    }

    CommunicationError(unsigned char nodeNr, ErrorCode code) :
            nodeNr(nodeNr), code(code)
    {
    }

    bool noError()
    {
        return code == NO_ERROR;
    }

    unsigned char nodeNr{0};
    ErrorCode code{NO_ERROR};
};

class Communication
{
public:
    Communication(){}
    ~Communication(){};

    virtual void setNodeNr(unsigned char nr) = 0;

    virtual void write(unsigned char nr, char value) = 0;

    virtual void write(unsigned char nr, short int value) = 0;

    virtual void requestReadChar(unsigned char nr) = 0;

    virtual void requestReadInt(unsigned char nr) = 0;

    virtual char getLastReadChar(unsigned char nr) = 0;

    virtual short int getLastReadInt(unsigned char nr) = 0;

    virtual CommunicationError execute() = 0;
};

class SerialCommunication : public Communication
{
public:
    SerialCommunication(SerialComOptimizer serial, uint32_t timeout = 50);

protected:
    SerialCommunication();

public:
    virtual void setNodeNr(unsigned char nr);

    virtual void write(unsigned char nr, char value);

    virtual void write(unsigned char nr, short int value);

    virtual void requestReadChar(unsigned char nr);

    virtual void requestReadInt(unsigned char nr);

    virtual char getLastReadChar(unsigned char nr);

    virtual short int getLastReadInt(unsigned char nr);

    virtual CommunicationError execute();

protected:
    FixedMemVector<unsigned char, 3 * 16 + 2 * 16> commandArray;
    FixedMemVector<unsigned char, 16 + 16> receiveArray;

    template <typename F>
    class CallAfterRetrunHandler
    {
    public:
        CallAfterRetrunHandler(F f) : f(f)
        {}
        ~CallAfterRetrunHandler()
        {
            f();
        }

        F f;
    };

    template <typename F>
    CallAfterRetrunHandler<F> createCallAfterReturnHandler(F f)
    {
        return CallAfterRetrunHandler<F>(f);
    }

    unsigned char nodeNr{1};
    CppArray<char, 16> charArray{0};
    CppArray<short int, 16> intArray{0};

    SerialComOptimizer serial;
    uint32_t timeout{0};
};

template <size_t N>
class SimulateCommunication : public SerialCommunication
{
public:
    SimulateCommunication()
    {
    }

    virtual CommunicationError execute() override;

    class ServoSim
    {
    public:
        ServoSim(unsigned char nodeNr) : nodeNr(nodeNr) {};

        void run()
        {
            intArray.at(3) = intArray.at(0);
            intArray.at(10) = intArray.at(0);
            intArray.at(4) = intArray.at(1);
            intArray.at(5) = intArray.at(2);
        }

        unsigned char nodeNr;
        CppArray<char, 16> charArray{0};
        CppArray<short int, 16> intArray{0};
    };

    FixedMemVector<ServoSim, N> servoSims;
};

template <size_t N>
CommunicationError SimulateCommunication<N>::execute()
{
    auto servoSimPtr = servoSims.end();
    for (auto& s : servoSims)
    {
        if (s.nodeNr == nodeNr)
        {
            servoSimPtr = &s;
        }
    }
    if (servoSimPtr == servoSims.end())
    {
        servoSims.push_back(ServoSim(nodeNr));
    }

    auto& servo = *servoSimPtr;
    servo.run();

    for (auto it = commandArray.begin(); it != commandArray.end(); ++it)
    {
        if (*it >= 128)
        {
            //read request, do nothing in sim
        }
        else if (*it >= 64)
        {
            const unsigned char intNr = *it - 64;
            ++it;
            short value = static_cast<unsigned char>(*it);
            ++it;
            value += static_cast<unsigned char>(*it) * static_cast<unsigned short>(256);
            servo.intArray.at(intNr) = value;
        }
        else
        {
            const unsigned char charNr = *it;
            ++it;
            servo.charArray.at(charNr) = static_cast<unsigned char>(*it);
        }
    }

    commandArray.clear();

    auto clearReceiveArrayAtExit = createCallAfterReturnHandler([this]()
        {
            this->receiveArray.clear();
        });

    for (auto it = receiveArray.begin(); it != receiveArray.end(); ++it)
    {
        if (*it >= 64)
        {
            short value = servo.intArray.at(*it - 64);
            intArray.at(*it - 64) = value;
        }
        else
        {
            char value = servo.charArray.at(*it);
            charArray.at(*it) = value;
        }
    }

    return CommunicationError(nodeNr, CommunicationError::NO_ERROR);
}

template <typename T, typename U>
class ContinuousValueUpCaster
{
  public:
    typedef typename stdmin::decay<T>::type ValueType;
    typedef typename stdmin::decay<U>::type InputType;

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
        typedef typename stdmin::make_signed<InputType>::type SignedInputType;

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

    void setOffsetAndScaling(float scale, float offset, float startPosition = 0);

    void setControlSpeed(unsigned char controlSpeed, float inertiaMarg = 1.0);
    void setControlSpeed(unsigned char controlSpeed, unsigned short int velControlSpeed,
            unsigned short int filterSpeed, float inertiaMarg = 1.0);

    void setBacklashControlSpeed(unsigned char backlashCompensationSpeed,
            float backlashCompensationCutOffSpeed, float backlashSize);

    void setFrictionCompensation(float fricComp);

    void disableBacklashControl(bool b = true);

    bool isInitComplete() const;

    bool isCommunicationOk() const;

    void setReference(const float& pos, const float& vel, const float& feedforwardU);

    void setOpenLoopControlSignal(const float& feedforwardU, bool pwmMode);

    float getPosition(bool withBacklash = true) const;

    float getVelocity() const;

    float getControlSignal() const;

    float getFeedforwardU() const;

    float getCurrent() const;

    short int getPwmControlSignal() const;

    float getControlError(bool withBacklash = true) const;

    short int getCpuLoad() const;

    short int getLoopTime() const;

    float getBacklashCompensation() const;

    OpticalEncoderChannelData getOpticalEncoderChannelData() const;

    float getLowLevelControlError() const;

    float getScaling() const;

    float getOffset() const;

    CommunicationError run();

private:
    void updateOffset();

    Communication* bus{nullptr};
    unsigned char nodeNr{0};

    bool communicationIsOk{false};

    int initState{0};
    bool backlashControlDisabled{false};
    bool newPositionReference{false};
    bool newOpenLoopControlSignal{false};
    bool pwmOpenLoopMode{false};

    unsigned char controlSpeed{50};
    unsigned short int velControlSpeed{50 * 4};
    unsigned short int filterSpeed{50 * 32};
    unsigned char inertiaMarg{0};
    unsigned char backlashCompensationSpeed{10};
    unsigned char backlashCompensationSpeedVelDecrease{0};
    unsigned char backlashSize{0};

    mutable CppArray<bool, 16> activeIntReads{false};
    CppArray<short int, 16> intReadBuffer{0};

    mutable CppArray<bool, 16> activeCharReads{false};
    CppArray<char, 16> charReadBuffer{0};

    ContinuousValueUpCaster<long int, short int> intReadBufferIndex3Upscaling;
    ContinuousValueUpCaster<long int, short int> intReadBufferIndex10Upscaling;
    ContinuousValueUpCaster<long int, short int> intReadBufferIndex11Upscaling;

    float backlashEncoderPos{0.0f};
    float encoderPos{0.0};
    float backlashCompensation{0.0};
    short int encoderVel{0};
    short int controlSignal{0};
    short int current{0};
    short int pwmControlSignal{0};
    short int cpuLoad{0};
    short int loopTime{0};
    OpticalEncoderChannelData opticalEncoderChannelData;

    float lowLevelControlError{0.0};

    long int refPos{0};
    CppArray<long int, 5> activeRefPos{0};
    short int refVel{0};
    short int feedforwardU{0};
    CppArray<short int, 5> activeFeedforwardU{0};
    float frictionCompensation{0.0f};

    float offset{0.0f};
    float startPosition{0.0f};
    float scale{1.0f};

    static constexpr int positionUpscaling = 32;
    int velocityUpscaling = 1;

    // ----------------------------------------
    // ---- Communication breaking changes ----
    // ----------------------------------------
    // 0 : version <= 4.0
    // 1 : version >= 4.1 : breaking change is velocityUpscaling = 8 > 1
    unsigned char breakingChangeNr{0};
};

template <size_t N>
class ServoManagerBase
{
protected:
    ServoManagerBase(int cycleTimeInMs, CppArray<DCServoCommunicator, N>* servos) :
        refToServos(*servos),
        cycleTimeInMs{cycleTimeInMs},
        dt(0.001f * cycleTimeInMs)
    {
    }

public:
    ~ServoManagerBase()
    {
    }

    float getDt()
    {
        return dt;
    }

    CommunicationError init()
    {
        sleepUntilTimePoint = millis();

        bool allDone = false;
        while (!allDone)
        {
            allDone = true;
            for (auto& s : refToServos)
            {
                allDone &= s.isInitComplete();
            }

            CommunicationError error = run();
            if (!error.noError())
            {
                return error;
            }
        }

        return CommunicationError();
    }

    CommunicationError run()
    {
        bool error = false;
        for (size_t i = 0; i != N; ++i)
        {

            CommunicationError error = refToServos[i].run();
            if (!error.noError())
            {
                return error;
            }

            currentPosition[i] = refToServos[i].getPosition();
        }

        sleepUntilTimePoint += cycleTimeInMs;
        while (static_cast<int32_t>(sleepUntilTimePoint - millis()) > 0)
        {
        }

        return CommunicationError();
    }

    CppArray<float, N> getPosition() const
    {
        return currentPosition;
    }

protected:
    int cycleTimeInMs;
    float dt;
    uint32_t sleepUntilTimePoint;
    CppArray<float, N> currentPosition;
    CppArray<DCServoCommunicator, N>& refToServos;
};

template <size_t N, size_t maxNrOfMove = 64>
class TrajectoryBuilder
{
public:
    TrajectoryBuilder(float dt) : dt(dt)
    {
    }

    void setStart(const CppArray<float, N>& pos)
    {
        buildFromPos = pos;
    }

    bool addLinearMove(const CppArray<float, N>& endPos, float duration)
    {
        if (pathItems.size() == pathItems.max_size())
        {
            return false;
        }

        pathItems.push_back(PathItem(buildFromPos, endPos, static_cast<size_t>(round(duration / dt)), [](float t)
            {
                return t;
            }));
        buildFromPos = endPos;

        return true;
    }

    bool addSmoothMove(const CppArray<float, N>& endPos, float duration)
    {
        if (pathItems.size() == pathItems.max_size())
        {
            return false;
        }

        pathItems.push_back(PathItem(buildFromPos, endPos, static_cast<size_t>(round(duration / dt)), [](float t)
            {
                return (1.0f - cosf(M_PI * t)) / 2.0f;
            }));
        buildFromPos = endPos;

        return true;
    }

    bool addWait(float duration)
    {
        if (pathItems.size() == pathItems.max_size())
        {
            return false;
        }

        pathItems.push_back(PathItem(buildFromPos, buildFromPos, static_cast<size_t>(round(duration / dt)), [](float t)
            {
                return t;
            }));

        return true;
    }


    CppArray<float, N> get()
    {
        return pathItems[index].get();
    }

    void clear()
    {
        pathItems.clear();
        reset();
    }

    void reset()
    {
        index = 0;
        for (auto& pathItem : pathItems)
        {
            pathItem.reset();
        }
    }

    bool step()
    {
        bool atEnd = !pathItems[index].step();
        if (atEnd)
        {
            ++index;
            if (index == pathItems.size())
            {
                index = pathItems.size() - 1;
                return false;
            }
        }
        return true;
    }

protected:
    static CppArray<float, N> interpolateVectors(
            const CppArray<float, N>& pos0,
            const CppArray<float, N>& pos1, float t)
    {
        CppArray<float, N> out;

        for (size_t i = 0; i != pos0.size(); ++i)
        {
            out[i] = (pos1[i] * t + pos0[i] * (1.0f - t));
        }
        return out;
    }

    class PathItem
    {
    public:
        using TModifierFunc = float (*)(float);

        PathItem(const CppArray<float, N>& startPos,
                const CppArray<float, N>& endPos,
                size_t nrOfSteps,
                TModifierFunc tFunc) :
            nrOfSteps(nrOfSteps),
            startPos(startPos),
            endPos(endPos),
            tFunc(tFunc)
        {
            reset();
        }

        CppArray<float, N> get()
        {
            float t = static_cast<float>(i) / nrOfSteps;
            t = tFunc(t);
            return interpolateVectors(startPos, endPos, t);
        }

        void reset()
        {
            i = 1;
        }

        bool step()
        {
            ++i;
            if (i == nrOfSteps + 1)
            {
                i = nrOfSteps;
                return false;
            }
            return true;
        }

        size_t i{1};
        size_t nrOfSteps;
        CppArray<float, N> startPos;
        CppArray<float, N> endPos;
        TModifierFunc tFunc;
    };

    float dt;
    CppArray<float, N> buildFromPos{};
    size_t index{0};
    FixedMemVector<PathItem, maxNrOfMove> pathItems;
};

template <size_t N>
class ServoReferenceSetter
{
public:
    ServoReferenceSetter(float dt) : dt(dt)
    {
    }

    void set(CppArray<DCServoCommunicator, N>& servos,
                    const CppArray<float, N>& newRefPos)
    {
        if (!initialized)
        {
            initialized = true;
            refPos = newRefPos;
            lastRefPos = newRefPos;
        }

        for (size_t i = 0; i != N; ++i)
        {
            auto& servo = servos[i];
            float vel = (newRefPos[i] - lastRefPos[i]) / 2.0f / dt;
            servo.setReference(refPos[i], vel, 0.0f);
        }

        lastRefPos = refPos;
        refPos = newRefPos;
    }

protected:
    bool initialized{false};
    float dt;
    CppArray<float, N> refPos;
    CppArray<float, N> lastRefPos;
};

#endif
