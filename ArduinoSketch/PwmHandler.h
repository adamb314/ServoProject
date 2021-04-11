#ifndef PWM_HANDLER_H
#define PWM_HANDLER_H

#include <Arduino.h>
#include "ArduinoC++BugFixes.h"
#include <sam.h>
#include <ctype.h>

class PwmHandler
{
  public:
    ~PwmHandler() {};
    virtual int setOutput(int output) = 0;
    virtual void activateBrake() = 0;
    virtual void disconnectOutput() = 0;
    virtual void connectOutput() = 0;
};

class HBridgeHighResPin11And12Pwm : public PwmHandler
{
  public:
    typedef uint16_t (*LinearizeFunctionType)(uint16_t);

    HBridgeHighResPin11And12Pwm(bool invert = false, LinearizeFunctionType linearizeFunction = [](uint16_t in){return in;});
    HBridgeHighResPin11And12Pwm(HBridgeHighResPin11And12Pwm&&);

    virtual ~HBridgeHighResPin11And12Pwm();

    HBridgeHighResPin11And12Pwm(const HBridgeHighResPin11And12Pwm&) = delete;
    HBridgeHighResPin11And12Pwm& operator=(const HBridgeHighResPin11And12Pwm&) = delete;
    HBridgeHighResPin11And12Pwm& operator=(HBridgeHighResPin11And12Pwm&&) = delete;

    virtual int setOutput(int output) override;

    virtual void activateBrake() override;

    virtual void disconnectOutput() override;

    virtual void connectOutput() override;

protected:
    HBridgeHighResPin11And12Pwm(Tcc* timer, bool invert, LinearizeFunctionType linearizeFunction);

    void configTimer();

    Tcc* const timer;

    uint16_t pin11WriteValue{0};
    uint16_t pin12WriteValue{0};

    bool outputConnected{false};
    const bool invert;
    const LinearizeFunctionType linearizeFunction;
};

class HBridgeHighResPin3And4Pwm : public HBridgeHighResPin11And12Pwm
{
  public:
    HBridgeHighResPin3And4Pwm(bool invert = false, LinearizeFunctionType linearizeFunction = [](uint16_t in){return in;});
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
    HBridge2WirePwm(int16_t pin1, int16_t pin2);
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
};

#endif
