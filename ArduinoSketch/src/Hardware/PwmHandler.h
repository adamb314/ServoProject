#ifndef PWM_HANDLER_H
#define PWM_HANDLER_H

#include <Arduino.h>
#include "../ArduinoC++BugFixes.h"
#include <sam.h>
#include <ctype.h>
#include <vector>
#include <algorithm>
#include "AdcHandler.h"

class SwitchAvoidingSynchronizer
{
public:
    class Switcher
    {
    public:
        virtual ~Switcher();
        virtual bool willSwitchWithIn(int16_t period) const = 0;
    
    private:
        void addSynchronizer(SwitchAvoidingSynchronizer* synchronizer);
        void removeSynchronizer(SwitchAvoidingSynchronizer* synchronizer);

        std::vector<SwitchAvoidingSynchronizer*> synchronizers;

        friend class SwitchAvoidingSynchronizer;
    };

    ~SwitchAvoidingSynchronizer();
    void addSwitcher(Switcher* switcher);
    bool removeSwitcher(Switcher* switcher);
    bool willSwitchWithIn(int16_t period);

private:
    std::vector<Switcher*> switchers;
};

class SwitchAvoidingSumAnalogSampler : public AdcSamplerInstance
{
public:
    SwitchAvoidingSumAnalogSampler(uint32_t pin,
            std::shared_ptr<SwitchAvoidingSynchronizer> synchronizer,
            uint16_t numberOfAdditiveSamples);

    virtual ~SwitchAvoidingSumAnalogSampler();

    void triggerSample();

    int32_t getValue();

protected:
    virtual void loadConfigAndStart() override;

    virtual bool handleResultAndCleanUp(int32_t result) override;

    int32_t value{0};
    int32_t sumOfAllSamples{0};
    const uint16_t numberOfAdditiveSamples{1};
    uint16_t switchFreeSamplesTaken{0};
    uint16_t samplesLeft{0};
    std::shared_ptr<SwitchAvoidingSynchronizer> synchronizer;
};

class PwmHandler
{
public:
    ~PwmHandler() {};
    virtual int setOutput(int output) = 0;
    virtual void activateBrake() = 0;
    virtual void addDamping(bool b = true)
    {
        damping = b;
    };

    virtual void disconnectOutput() = 0;
    virtual void connectOutput() = 0;
protected:
    bool damping{false};
};

class HBridgeHighResPin11And12Pwm : public PwmHandler, public SwitchAvoidingSynchronizer::Switcher
{
public:
    typedef uint16_t (*LinearizeFunctionType)(uint16_t);

    HBridgeHighResPin11And12Pwm(bool invert = false, LinearizeFunctionType linearizeFunction = [](uint16_t in){return in;},
            uint16_t frq = 20000);
    HBridgeHighResPin11And12Pwm(bool invert, bool dualSlope,
            LinearizeFunctionType linearizeFunction = [](uint16_t in){return in;},
            uint16_t frq = 20000);
    HBridgeHighResPin11And12Pwm(HBridgeHighResPin11And12Pwm&&);

    virtual ~HBridgeHighResPin11And12Pwm();

    HBridgeHighResPin11And12Pwm(const HBridgeHighResPin11And12Pwm&) = delete;
    HBridgeHighResPin11And12Pwm& operator=(const HBridgeHighResPin11And12Pwm&) = delete;
    HBridgeHighResPin11And12Pwm& operator=(HBridgeHighResPin11And12Pwm&&) = delete;

    virtual int setOutput(int output) override;

    virtual void activateBrake() override;

    virtual void disconnectOutput() override;

    virtual void connectOutput() override;

    virtual bool willSwitchWithIn(int16_t period) const override;

protected:
    HBridgeHighResPin11And12Pwm(Tcc* timer, bool invert, bool dualSlope, LinearizeFunctionType linearizeFunction,
          uint16_t frq);

    void configTimer();

    Tcc* const timer;

    uint16_t pin11WriteValue{0};
    uint16_t pin12WriteValue{0};

    bool outputConnected{false};
    const bool invert;
    const bool dualSlope;
    const LinearizeFunctionType linearizeFunction;

    const uint16_t tickPerUs{F_CPU / 1000000ul};
    const uint16_t switchTransientTime{(uint16_t)(4 * tickPerUs)};
    const uint8_t freqDiv{2};
};

class HBridgeHighResPin3And4Pwm : public HBridgeHighResPin11And12Pwm
{
public:
    HBridgeHighResPin3And4Pwm(bool invert = false, LinearizeFunctionType linearizeFunction = [](uint16_t in){return in;},
            uint16_t frq = 20000);
    HBridgeHighResPin3And4Pwm(bool invert, bool dualSlope,
            LinearizeFunctionType linearizeFunction = [](uint16_t in){return in;},
            uint16_t frq = 20000);
    HBridgeHighResPin3And4Pwm(HBridgeHighResPin3And4Pwm&& in);

    virtual ~HBridgeHighResPin3And4Pwm() {};

    HBridgeHighResPin3And4Pwm(const HBridgeHighResPin3And4Pwm&) = delete;
    HBridgeHighResPin3And4Pwm& operator=(const HBridgeHighResPin3And4Pwm&) = delete;
    HBridgeHighResPin3And4Pwm& operator=(HBridgeHighResPin3And4Pwm&&) = delete;

    virtual void connectOutput() override;
};

class HBridge2WirePwm : public PwmHandler
{
public:
    typedef uint16_t (*LinearizeFunctionType)(uint16_t);

    HBridge2WirePwm(int16_t pin1, int16_t pin2, LinearizeFunctionType linearizeFunction = [](uint16_t in){return in;});
    HBridge2WirePwm(HBridge2WirePwm&&);
    ~HBridge2WirePwm();

    HBridge2WirePwm(const HBridge2WirePwm&) = delete;
    HBridge2WirePwm& operator=(const HBridge2WirePwm&) = delete;
    HBridge2WirePwm& operator=(HBridge2WirePwm&&) = delete;

    virtual int setOutput(int output);

    virtual void activateBrake();

    virtual void disconnectOutput();

    virtual void connectOutput();

private:
    int16_t pin1;
    int16_t pin2;

    uint16_t pin1WriteValue{0};
    uint16_t pin2WriteValue{0};

    bool outputConnected{false};

    const LinearizeFunctionType linearizeFunction;
};

#endif
