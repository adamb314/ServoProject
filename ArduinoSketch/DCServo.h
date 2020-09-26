#include <Arduino.h>
#undef max
#undef min
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
    uint16_t loadTimeInterval{12000};
    float invertedLoadInterval{1.0 / 12000};
    uint16_t getTimeInterval{1200};
};

class DCServo
{
 public:
    DCServo(std::unique_ptr<CurrentController> currentController,
            std::unique_ptr<EncoderHandlerInterface> mainEncoderHandler,
            std::unique_ptr<EncoderHandlerInterface> outputEncoderHandler,
            std::unique_ptr<KalmanFilter> kalmanFilter);

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
    bool lastForceDirNotZero{false};
    float currentBacklashStepSize{0.0};
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
    float uLimitDiff{0.0};

    ReferenceInterpolator refInterpolator;

    float Ivel{0.0};

    std::unique_ptr<CurrentController> currentController;
    std::unique_ptr<EncoderHandlerInterface> mainEncoderHandler;
    std::unique_ptr<EncoderHandlerInterface> outputEncoderHandler;
    std::unique_ptr<KalmanFilter> kalmanFilter;

    std::vector<Thread*> threads;
};

#endif
