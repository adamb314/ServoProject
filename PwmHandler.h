#ifndef PWM_HANDLER_H
#define PWM_HANDLER_H

#include <sam.h>
#include <ctype.h>

class PwmHandler
{
  public:
    ~PwmHandler() {};
    virtual int setOutput(int output) = 0;
    virtual void disconnectOutput() = 0;
    virtual void connectOutput() = 0;
};

class HBridge4WirePwm : public PwmHandler
{
  public:
    static HBridge4WirePwm* getInstance();

    ~HBridge4WirePwm();

    HBridge4WirePwm(const HBridge4WirePwm&) = delete;
    HBridge4WirePwm& operator=(const HBridge4WirePwm&) = delete;
    HBridge4WirePwm(HBridge4WirePwm&&) = delete;
    HBridge4WirePwm& operator=(HBridge4WirePwm&&) = delete;

    virtual int setOutput(int output);

    virtual void disconnectOutput();

    virtual void connectOutput();

  private:
    HBridge4WirePwm();

    unsigned int disconnectOutputCC0Backup;
    unsigned int disconnectOutputCC1Backup;
};

#endif
