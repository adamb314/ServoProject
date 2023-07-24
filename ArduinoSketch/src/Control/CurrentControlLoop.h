#ifndef CURRENT_CONTROL_LOOP_H
#define CURRENT_CONTROL_LOOP_H

#include "../ArduinoC++BugFixes.h"
#include "../Hardware/ThreadHandler.h"
#include "../Hardware/CurrentSampler.h"
#include "../Hardware/PwmHandler.h"
#include <ArduinoEigenDense.h>
#include <vector>
#include <algorithm>

class CurrentController
{
public:
    CurrentController(std::unique_ptr<PwmHandler> pwmInstance):
            pwmInstance(std::move(pwmInstance)) {};

    virtual void setReference(int32_t ref) = 0;

    virtual void updateVelocity(float vel) {};

    virtual void overidePwmDuty(int32_t pwm) = 0;

    virtual void activateBrake() = 0;

    virtual void addDamping(bool b = true)
    {
        pwmInstance->addDamping(b);
    };

    virtual void applyChanges() = 0;

    virtual int32_t getLimitedRef() = 0;

    virtual int32_t getFilteredPwm() = 0;

    virtual int32_t getCurrent() = 0;

protected:
    std::unique_ptr<PwmHandler> pwmInstance;
};

class CurrentControlLoop : public CurrentController
{
public:
    CurrentControlLoop(uint32_t period,  int16_t currentSensorPin = A1,
            std::unique_ptr<PwmHandler> pwmInstance = std::make_unique<HBridgeHighResPin11And12Pwm>());

    ~CurrentControlLoop();

    void setReference(int32_t ref) override;

    int32_t getLimitedRef() override;

    void overidePwmDuty(int32_t pwm) override;

    int32_t getFilteredPwm() override;

    void activateBrake() override;

    int32_t getCurrent() override;

    void applyChanges();

private:
    void run();

    CurrentSampler* currentSampler;

    bool newPwmOverrideValue;
    bool newBrakeValue;
    int32_t newRefValue;
    int32_t newUValue;

    bool pwmOverride;
    bool brake;
    int32_t ref;
    int32_t y;
    int32_t filteredY;
    int32_t filteredPwm;
    int32_t u;
    int32_t limitedU;
    bool lastULimited;
    FunctionalWrapper<bool>* adcSampDoneCheck;
    std::vector<CodeBlocksThread*> threads;
};

class CurrentControlModel : public CurrentController
{
public:
    CurrentControlModel(
            std::unique_ptr<PwmHandler> pwmInstance = std::make_unique<HBridgeHighResPin11And12Pwm>());
    CurrentControlModel(float pwmToStallCurrent, float backEmfCurrent,
            std::unique_ptr<PwmHandler> pwmInstance = std::make_unique<HBridgeHighResPin11And12Pwm>());

    ~CurrentControlModel();

    void setReference(int32_t ref) override;

    void updateVelocity(float vel) override;

    void overidePwmDuty(int32_t pwm) override;

    void activateBrake() override;

    void applyChanges() override;

    int32_t getLimitedRef() override;

    int32_t getFilteredPwm() override;

    int32_t getCurrent() override;

private:
    constexpr static int32_t fixedPoint = 512;
    const int32_t pwmToStallCurrentF{fixedPoint};
    const int32_t backEmfCurrentFF{0};

    bool pwmOverride{true};
    bool brake{true};
    int32_t ref{0};
    int32_t y{0};
    int32_t filteredY{0};
    int32_t filteredPwm{0};
    int32_t u{0};
    int32_t limitedU{0};
    float vel{0.0f};
    bool lastULimited{false};
};

#endif
