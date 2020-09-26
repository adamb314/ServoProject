#ifndef PWM_HANDLER_H
#define PWM_HANDLER_H

#include <Arduino.h>
#undef max
#undef min
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
    HBridgeHighResPin11And12Pwm();
    HBridgeHighResPin11And12Pwm(HBridgeHighResPin11And12Pwm&&);

    ~HBridgeHighResPin11And12Pwm();

    HBridgeHighResPin11And12Pwm(const HBridgeHighResPin11And12Pwm&) = delete;
    HBridgeHighResPin11And12Pwm& operator=(const HBridgeHighResPin11And12Pwm&) = delete;
    HBridgeHighResPin11And12Pwm& operator=(HBridgeHighResPin11And12Pwm&&) = delete;

    virtual int setOutput(int output);

    virtual void activateBrake();

    virtual void disconnectOutput();

    virtual void connectOutput();

  private:
    uint16_t pin11WriteValue{0};
    uint16_t pin12WriteValue{0};

    bool outputConnected{false};
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
