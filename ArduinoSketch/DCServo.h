#include <Arduino.h>
#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"

#include <ArduinoEigenDense.h>
#include "CurrentControlLoop.h"
#include "EncoderHandler.h"
#include "OpticalEncoderHandler.h"
#include "KalmanFilter.h"
#include "ComplementaryFilter.h"
#include "adam_stl.h"
#include "SampleAveragingHandler.h"
#include "ReferenceInterpolator.h"

#ifndef DC_SERVO_H
#define DC_SERVO_H

class ControlConfigurationInterface
{
public:
    virtual const Eigen::Matrix3f& getA() = 0;
    virtual const Eigen::Vector3f& getB() = 0;
    virtual void limitVelocity(float& vel) = 0;
    virtual std::tuple<int32_t, bool> applyForceCompensations(int32_t u, uint16_t rawEncPos, float velRef, float vel) = 0;
    virtual float calculateFeedForward(float v1, float v0) = 0;

    virtual float getCycleTime();

    virtual uint32_t getCycleTimeUs();
};

class DefaultControlConfiguration : public ControlConfigurationInterface
{
public:
    static constexpr int vecSize = 512;

    DefaultControlConfiguration(const Eigen::Matrix3f& A, const Eigen::Vector3f& B,
        const float& maxVel, const float& avarageFriction,
        const std::array<int16_t, vecSize>& posDepForceCompVec,
        const std::array<int16_t, vecSize>& posDepFrictionCompVec,
        const EncoderHandlerInterface* encoder);

    template<typename T>
    static std::unique_ptr<DefaultControlConfiguration> create(const EncoderHandlerInterface* encoder);

    virtual const Eigen::Matrix3f& getA() override;

    virtual const Eigen::Vector3f& getB() override;

    virtual void limitVelocity(float& vel) override;

    virtual std::tuple<int32_t, bool> applyForceCompensations(
        int32_t u, uint16_t rawEncPos, float velRef, float vel) override;

    virtual float calculateFeedForward(float v1, float v0);

private:
    Eigen::Matrix3f A;
    Eigen::Vector3f B;
    float b1Inv;
    float maxVel;
    std::array<int16_t, vecSize> posDepForceCompVec;
    std::array<int16_t, vecSize> posDepFrictionCompVec;
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
            T::getPosDepFrictionCompVec(),
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

    uint16_t getLoopNr();

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

    //L[0]: Proportional gain of position control loop
    //L[1]: Proportional gain of velocity control loop
    //L[2]: Integral action gain of velocity control loop
    //L[3]: Integral anti windup gain of velocity control loop
    //L[4]: Backlash compensation integral action gain
    //L[5]: Backlash compensation velocity dependent gain
    //L[6]: Backlash size
    Eigen::Matrix<float, 7, 1> L;

    uint32_t loopTime{0};
    uint32_t loopNr{0};
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

    int16_t current{0};
    int16_t pwmControlSignal{0};
    float kalmanControlSignal{0.0f};
    SampleAveragingHandler<int32_t, 32> currentAveraging;
    SampleAveragingHandler<int32_t, 32> controlSignalAveraging;

    ReferenceInterpolator refInterpolator;

    ComplementaryFilter outputEncoderFilter;

    float Ivel{0.0f};
    float vControlRef{0.0f};
    int32_t pwm{0};
    bool pendingIntegralCalc{false};

    std::unique_ptr<CurrentController> currentController;
    std::unique_ptr<EncoderHandlerInterface> mainEncoderHandler;
    std::unique_ptr<EncoderHandlerInterface> outputEncoderHandler;
    std::unique_ptr<KalmanFilter> kalmanFilter;
    std::unique_ptr<ControlConfigurationInterface> controlConfig;

    std::vector<Thread*> threads;
};

#endif
