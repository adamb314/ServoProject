#ifndef CURRENT_CONTROL_LOOP_H
#define CURRENT_CONTROL_LOOP_H

#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"
#include "CurrentSampler.h"
#include "PwmHandler.h"
#include <ArduinoEigenDense.h>
#include <vector>
#include <algorithm>

class CurrentController
{
public:
    virtual void setReference(int16_t ref) = 0;

    virtual void updateVelocity(float vel) {};

    virtual void overidePwmDuty(int16_t pwm) = 0;

    virtual void activateBrake() = 0;

    virtual void applyChanges() = 0;

    virtual int16_t getLimitedRef() = 0;

    virtual int16_t getFilteredPwm() = 0;

    virtual int16_t getCurrent() = 0;
};

class CurrentControlLoop : public CurrentController
{
public:
    CurrentControlLoop(uint32_t period,  int16_t currentSensorPin = A1,
            std::unique_ptr<PwmHandler> pwmInstance = std::make_unique<HBridgeHighResPin11And12Pwm>());

    ~CurrentControlLoop();

    void setReference(int16_t ref) override;

    int16_t getLimitedRef() override;

    void overidePwmDuty(int16_t pwm) override;

    int16_t getFilteredPwm() override;

    void activateBrake() override;

    int16_t getCurrent() override;

    void applyChanges();

private:
    void run();

    std::unique_ptr<PwmHandler> pwmInstance;
    CurrentSampler* currentSampler;

    bool newPwmOverrideValue;
    bool newBrakeValue;
    int16_t newRefValue;
    int16_t newUValue;

    bool pwmOverride;
    bool brake;
    int16_t ref;
    int16_t y;
    int16_t filteredY;
    int16_t filteredPwm;
    int16_t u;
    int16_t limitedU;
    bool lastULimited;
    FunctionalWrapper<bool>* adcSampDoneCheck;
    std::vector<CodeBlocksThread*> threads;
};

class CurrentControlModel : public CurrentController
{
public:
    CurrentControlModel(float pwmToStallCurrent, float backEmfCurrent,
            std::unique_ptr<PwmHandler> pwmInstance = std::make_unique<HBridgeHighResPin11And12Pwm>());

    ~CurrentControlModel();

    void setReference(int16_t ref) override;

    void updateVelocity(float vel) override;

    void overidePwmDuty(int16_t pwm) override;

    void activateBrake() override;

    void applyChanges() override;

    int16_t getLimitedRef() override;

    int16_t getFilteredPwm() override;

    int16_t getCurrent() override;

private:
    const float pwmToStallCurrent;
    const float backEmfCurrent;

    std::unique_ptr<PwmHandler> pwmInstance;
    bool pwmOverride;
    bool brake;
    int16_t ref;
    int16_t y;
    int16_t filteredY;
    int16_t filteredPwm;
    int16_t u;
    int16_t limitedU;
    float vel;
    bool lastULimited;
};

#endif
