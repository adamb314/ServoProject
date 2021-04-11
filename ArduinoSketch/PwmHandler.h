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

class HBridge2WirePwm : public PwmHandler
{
  public:
    static HBridge2WirePwm* getInstance();

    ~HBridge2WirePwm();

    HBridge2WirePwm(const HBridge2WirePwm&) = delete;
    HBridge2WirePwm& operator=(const HBridge2WirePwm&) = delete;
    HBridge2WirePwm(HBridge2WirePwm&&) = delete;
    HBridge2WirePwm& operator=(HBridge2WirePwm&&) = delete;

    virtual int setOutput(int output);

    virtual void activateBrake();

    virtual void disconnectOutput();

    virtual void connectOutput();

  private:
    HBridge2WirePwm();

    unsigned int disconnectOutputCC0Backup;
    unsigned int disconnectOutputCC1Backup;
};

#endif
