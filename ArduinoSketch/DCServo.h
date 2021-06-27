#include <Arduino.h>
#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"

#include <Eigen30.h>
#include "CurrentControlLoop.h"
#include "EncoderHandler.h"
#include "OpticalEncoderHandler.h"
#include "KalmanFilter.h"

#ifndef DC_SERVO_H
#define DC_SERVO_H

class ReferenceInterpolator
{
 public:
    ReferenceInterpolator();

    void loadNew(float position, float velocity, float feedForward);

    void updateTiming();

    void resetTiming();

    void getNext(float& position, float& velocity, float& feedForward);

    void setGetTimeInterval(const uint16_t& interval);

    void setLoadTimeInterval(const uint16_t& interval);

    int16_t midPointTimeOffset{0};
 private:
    float pos[3]{0};
    float vel[3]{0};
    float feed[3]{0};

    uint16_t lastUpdateTimingTimestamp{0};
    uint16_t lastGetTimestamp{0};

    bool timingInvalid{true};
    bool refInvalid{true};
    uint16_t loadTimeInterval{12000};
    float invertedLoadInterval{1.0 / 12000};
    float dtDiv2{12000 * 0.000001 * 0.5};
    uint16_t getTimeInterval{1200};
};

class ControlConfigurationInterface
{
public:
    virtual const Eigen::Matrix3f& getA() = 0;
    virtual const Eigen::Vector3f& getB() = 0;
    virtual void limitVelocity(float& vel) = 0;
    virtual float applyForceCompensations(float u, float rawEncPos, float velRef) = 0;

    virtual float getCycleTime()
    {
        return getA()(0, 1);
    }

    virtual uint32_t getCycleTimeUs()
    {
        return static_cast<uint32_t>(getCycleTime() * 1000000ul);
    }
};

class DefaultControlConfiguration : public ControlConfigurationInterface
{
public:
    DefaultControlConfiguration(const Eigen::Matrix3f& A, const Eigen::Vector3f& B,
        const float& maxVel, const float& frictionComp, const std::array<uint8_t, 512>& posDepForceCompVec,
        const EncoderHandlerInterface* encoder) :
            A(A),
            B(B),
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

    virtual float applyForceCompensations(float u, float rawEncPos, float velRef) override
    {
        float out = u;

        if (velRef > 0)
        {
            out += frictionComp;
        }
        else if (velRef < 0)
        {
            out -= frictionComp;
        }

        out = (out * static_cast<int32_t>(posDepForceCompVec[rawEncPos])) / 128;

        return out;
    }

private:
    Eigen::Matrix3f A;
    Eigen::Vector3f B;
    float frictionComp;
    float maxVel;
    std::array<uint8_t, 512> posDepForceCompVec;
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

    void setBacklashControlSpeed(uint8_t backlashControlSpeed, uint8_t backlashControlSpeedVelGain, uint8_t backlashSize);

    void loadNewReference(float pos, int16_t vel, int16_t feedForwardU = 0);

    void triggerReferenceTiming();

    float getPosition();

    int16_t getVelocity();
    
    int16_t getControlSignal();

    int16_t getCurrent();

    int16_t getPwmControlSignal();

    uint16_t getLoopNumber();

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

    uint8_t controlSpeed{50};
    uint8_t backlashControlSpeed{10};
    uint8_t backlashControlSpeedVelGain{0};
    uint8_t backlashSize{0};

    uint8_t backlashControlGainDelayCounter{0};
    float backlashControlGain{0.0};

    //L[0]: Proportional gain of position control loop
    //L[1]: Proportional gain of velocity control loop
    //L[2]: Integral action gain of velocity control loop
    //L[3]: Integral anti windup gain of velocity control loop
    //L[4]: Backlash compensation integral action gain
    //L[5]: Backlash compensation velocity dependent gain
    //L[6]: Backlash size
    Eigen::Matrix<float, 7, 1> L;

    uint16_t loopNumber{0};
    float rawMainPos{0.0};
    float rawOutputPos{0.0};
    int forceDir{0};
    float outputPosOffset{0.0};
    float initialOutputPosOffset{0.0};

    //x[0]: Estimated position
    //x[1]: Estimated velocity
    //x[2]: Estimated load disturbance
    Eigen::Vector3f x;

#ifdef SIMULATE
    Eigen::Vector3f xSim;
#endif

    int16_t current{0};
    int16_t pwmControlSIgnal{0};
    float controlSignal{0.0};

    ReferenceInterpolator refInterpolator;

    float Ivel{0.0};

    std::unique_ptr<CurrentController> currentController;
    std::unique_ptr<EncoderHandlerInterface> mainEncoderHandler;
    std::unique_ptr<EncoderHandlerInterface> outputEncoderHandler;
    std::unique_ptr<KalmanFilter> kalmanFilter;
    std::unique_ptr<ControlConfigurationInterface> controlConfig;

    std::vector<Thread*> threads;
};

#endif
