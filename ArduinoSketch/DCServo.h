#include <Arduino.h>
#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"

#include <ArduinoEigenDense.h>
#include "CurrentControlLoop.h"
#include "EncoderHandler.h"
#include "OpticalEncoderHandler.h"
#include "KalmanFilter.h"
#include "clamp_cast.h"

#ifndef DC_SERVO_H
#define DC_SERVO_H

//#define SIMULATE

template <typename T, int maxN>
class SampleAveragingHandler
{
public:
    void add(T v);
    T get();

private:
    bool valueRead{false};
    T sum{};
    T halfSampleSum{};
    int n{0};
};

class ReferenceInterpolator
{
 public:
    ReferenceInterpolator();

    void loadNew(float position, float velocity, float feedForward);

    void updateTiming();

    void resetTiming();

    void calculateNext();

    std::tuple<float, float, float> get();

    float getPositionInterpolationDist();

    void setGetTimeInterval(const uint16_t& interval);

    void setLoadTimeInterval(const uint16_t& interval);

    int16_t midPointTimeOffset{0};
 private:
    void stepAndUpdateInter();

    float pos[3]{0};
    float vel[3]{0};
    float feed[3]{0};

    float interPos;
    float interVel;
    float interFeed;

    uint16_t lastUpdateTimingTimestamp{0};
    uint16_t lastGetTimestamp{0};

    bool timingInvalid{true};
    bool refInvalid{true};
    uint16_t loadTimeInterval{12000};
    float invertedLoadInterval{1.0f / loadTimeInterval};
    float dtDiv2{loadTimeInterval * 0.000001f * 0.5f};
    uint16_t getTimeInterval{1200};
    float invertedGetInterval{1.0f / getTimeInterval};
    float getTStepSize{getTimeInterval * invertedLoadInterval};
};

class ControlConfigurationInterface
{
public:
    virtual const Eigen::Matrix3f& getA() = 0;
    virtual const Eigen::Vector3f& getB() = 0;
    virtual void limitVelocity(float& vel) = 0;
    virtual float applyForceCompensations(float u, uint16_t rawEncPos, float velRef, float vel) = 0;
    virtual float calculateFeedForward(float v1, float v0) = 0;

    virtual float getCycleTime()
    {
        auto A = getA();
        auto B = getB();

        float contineusA = (1.0f - A(1, 1)) / A(0, 1);

        return -log(A(1, 1)) / contineusA;
    }

    virtual uint32_t getCycleTimeUs()
    {
        float dt = round(getCycleTime() / 0.0002f) * 0.0002f;
        return static_cast<uint32_t>(dt * 1000000ul);
    }
};

class DefaultControlConfiguration : public ControlConfigurationInterface
{
public:
    DefaultControlConfiguration(const Eigen::Matrix3f& A, const Eigen::Vector3f& B,
        const float& maxVel, const float& frictionComp, const std::array<int16_t, 512>& posDepForceCompVec,
        const EncoderHandlerInterface* encoder) :
            A(A),
            B(B),
            b1Inv{1.0f / B[1]},
            maxVel(std::min(maxVel, encoder->unitsPerRev * (1.0f / A(0, 1) / 2.0f * 0.8f))),
            frictionComp(frictionComp),
            posDepForceCompVec(posDepForceCompVec)
    {}

    template<typename T>
    static std::unique_ptr<DefaultControlConfiguration> create(const EncoderHandlerInterface* encoder);

    virtual const Eigen::Matrix3f& getA() override
    {
        return A;
    }

    virtual const Eigen::Vector3f& getB() override
    {
        return B;
    }

    virtual void limitVelocity(float& vel) override
    {
        vel = std::min(maxVel, vel);
        vel = std::max(-maxVel, vel);
    }

    static constexpr int vecSize = 512;

    virtual float applyForceCompensations(float u, uint16_t rawEncPos, float velRef, float vel) override
    {
        float out = u;
        constexpr float eps = 1.0f;

        if (velRef > eps && vel >= -eps)
        {
            fricCompDir = 1;
        }
        else if (velRef < -eps && vel <= eps)
        {
            fricCompDir = -1;
        }

        if (fricCompDir == 1)
        {
            out += frictionComp;
        }
        else if (fricCompDir == -1)
        {
            out -= frictionComp;
        }

        size_t i = (rawEncPos * vecSize) / 4096;
        out += posDepForceCompVec[i];

        return out;
    }

    virtual float calculateFeedForward(float v1, float v0)
    {
        return b1Inv * (v1 - A(1, 1) * v0);
    }

private:
    Eigen::Matrix3f A;
    Eigen::Vector3f B;
    float b1Inv;
    float frictionComp;
    float maxVel;
    std::array<int16_t, vecSize> posDepForceCompVec;
    int fricCompDir{0};
};

template<typename T>
std::unique_ptr<DefaultControlConfiguration> DefaultControlConfiguration::create(const EncoderHandlerInterface* encoder)
{
    return std::make_unique<DefaultControlConfiguration>(
            T::getAMatrix(),
            T::getBVector(),
            T::getMaxVelocity(),
            T::getFrictionComp(),
            T::getPosDepForceCompVec(),
            encoder);
}

class DCServo
{
 public:
    DCServo(std::unique_ptr<CurrentController> currentController,
            std::unique_ptr<EncoderHandlerInterface> mainEncoderHandler,
            std::unique_ptr<EncoderHandlerInterface> outputEncoderHandler,
            std::unique_ptr<KalmanFilter> kalmanFilter,
            std::unique_ptr<ControlConfigurationInterface> controlConfig);

    bool isEnabled();

    void enable(bool b = true);

    void openLoopMode(bool enable, bool pwmMode = false);

    void onlyUseMainEncoder(bool b = true);

    void setControlSpeed(uint8_t controlSpeed);
    void setControlSpeed(uint8_t controlSpeed, uint16_t velControlSpeed, uint16_t filterSpeed, float inertiaMarg = 1.0f);

    void setBacklashControlSpeed(uint8_t backlashControlSpeed, uint8_t backlashControlSpeedVelGain = 0, uint8_t backlashSize = 0);

    void enableInternalFeedForward(bool enable = true);

    void loadNewReference(float pos, int16_t vel, int16_t feedForwardU = 0);

    void triggerReferenceTiming();

    float getPosition();

    int16_t getVelocity();
    
    int16_t getControlSignal();

    int16_t getCurrent();

    int16_t getPwmControlSignal();

    uint16_t getLoopTime();

    float getBacklashCompensation();

    float getMainEncoderPosition();

    EncoderHandlerInterface::DiagnosticData getMainEncoderDiagnosticData();

 private:
    void init();
    void calculateAndUpdateLVector();
    void controlLoop();

    void identTestLoop();

    bool controlEnabled{false};
    bool onlyUseMainEncoderControl{false};
    bool openLoopControlMode{false};
    bool pwmOpenLoopMode{false};
    bool internalFeedForwardEnabled{false};

    uint8_t controlSpeed{50};
    uint16_t velControlSpeed{50 * 4};
    uint16_t filterSpeed{50 * 4 * 8};
    uint8_t backlashControlSpeed{10};
    uint8_t backlashControlSpeedVelGain{0};
    uint8_t backlashSize{0};

    uint8_t backlashControlGainDelayCounter{0};
    float backlashControlGain{0.0f};

    float inertiaMarg{1.0f};
    bool inertiaMargDisabled{false};
    static constexpr float inertiaMargDisableRange{0.5f / 32};

    //L[0]: Proportional gain of position control loop
    //L[1]: Proportional gain of velocity control loop
    //L[2]: Integral action gain of velocity control loop
    //L[3]: Integral anti windup gain of velocity control loop
    //L[4]: Backlash compensation integral action gain
    //L[5]: Backlash compensation velocity dependent gain
    //L[6]: Backlash size
    //L[7]: inertiaMargDisabled proportional gain of velocity control loop
    //L[8]: inertiaMargDisabled integral action gain of velocity control loop
    //L[9]: inertiaMargDisabled integral anti windup gain of velocity control loop
    Eigen::Matrix<float, 10, 1> L;

    uint16_t loopTime{0};
    float rawMainPos{0.0f};
    float rawOutputPos{0.0f};
    int forceDir{0};
    float outputPosOffset{0.0f};
    float initialOutputPosOffset{0.0f};
    float posRefTimingOffset{0.0f};

    //x[0]: Estimated position
    //x[1]: Estimated velocity
    //x[2]: Estimated load disturbance
    Eigen::Vector3f x;

#ifdef SIMULATE
    Eigen::Vector3f xSim;
#endif

    int16_t current{0};
    int16_t pwmControlSignal{0};
    float kalmanControlSignal{0.0f};
    SampleAveragingHandler<int32_t, 8> currentAveraging;
    SampleAveragingHandler<int32_t, 8> controlSignalAveraging;

    ReferenceInterpolator refInterpolator;

    float Ivel{0.0f};
    float vControlRef{0.0f};
    float pwm{0.0f};
    bool pendingIntegralCalc{false};

    std::unique_ptr<CurrentController> currentController;
    std::unique_ptr<EncoderHandlerInterface> mainEncoderHandler;
    std::unique_ptr<EncoderHandlerInterface> outputEncoderHandler;
    std::unique_ptr<KalmanFilter> kalmanFilter;
    std::unique_ptr<ControlConfigurationInterface> controlConfig;

    std::vector<Thread*> threads;
};

template <typename T, int maxN>
void SampleAveragingHandler<T, maxN>::add(T v)
{
    ThreadInterruptBlocker blocker;

    if (valueRead)
    {
        valueRead = false;
        n = 0;
        sum = 0;
        halfSampleSum = 0;
    }

    if (n >= maxN)
    {
        sum = halfSampleSum;
        halfSampleSum = 0;
        n = maxN / 2;
    }

    if (n >= maxN / 2)
    {
        halfSampleSum += v;
    }

    ++n;
    sum += v;
}

template <typename T, int maxN>
T SampleAveragingHandler<T, maxN>::get()
{
    T tempSum;
    int tempN;
    {
        ThreadInterruptBlocker blocker;
        tempSum = sum;
        tempN = n;
        valueRead = true;
    }
    return tempSum / tempN;
}

#endif
