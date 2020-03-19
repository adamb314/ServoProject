#ifndef CURRENT_CONTROL_LOOP_H
#define CURRENT_CONTROL_LOOP_H

#include "ThreadHandler.h"
#include "CurrentSampler.h"
#include "PwmHandler.h"
#include <Eigen.h>
#include <vector>

class CurrentControlLoop
{
public:
    CurrentControlLoop(uint32_t period);

    ~CurrentControlLoop();

    void setReference(int16_t ref);

    int16_t getLimitedRef();

    void overidePwmDuty(int16_t pwm);

    int16_t getFilteredPwm();

    void activateBrake();

    int16_t getCurrent();

private:
    void run();

    PwmHandler* pwmInstance;
    CurrentSampler* currentSampler;
    bool disableLoop;
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

class CurrentControlModel
{
public:
    CurrentControlModel(uint32_t period) :
        pwmInstance(HBridge4WirePwm::getInstance()),
        disableLoop(true),
        brake(true),
        ref(0),
        y(0),
        filteredY(0),
        filteredPwm(0),
        u(0),
        limitedU(0),
        lastULimited(false)
    {
    }

    ~CurrentControlModel()
    {
    }

    void setReference(int16_t ref)
    {
        disableLoop = false;
        this->ref = ref;
    }

    int16_t getLimitedRef()
    {
        if (lastULimited)
        {
            return y;
        }

        return ref;
    }

    void overidePwmDuty(int16_t pwm)
    {
        disableLoop = true;
        brake = false;
        u = pwm;
    }

    int16_t getFilteredPwm()
    {
        return filteredPwm;
    }

    void activateBrake()
    {
        disableLoop = true;
        brake = true;
    }

    int16_t getCurrent()
    {
        return filteredY;
    }

    void run(float vel)
    {
        if (disableLoop)
        {
            if (brake)
            {
                pwmInstance->activateBrake();
                limitedU = 0;
            }
            else
            {
                limitedU = pwmInstance->setOutput(u);
            }
            y = (pwmToStallCurrent + backEmfCurrent * vel) * limitedU;
            filteredY = y;
            filteredPwm = limitedU;
            return;
        }

        u = ref / (pwmToStallCurrent + backEmfCurrent * vel);

        limitedU = pwmInstance->setOutput(u);

        y = (pwmToStallCurrent + backEmfCurrent * vel) * limitedU;
        filteredY = y;
        filteredPwm = limitedU;

        if (u != limitedU)
        {
            lastULimited = true;
        }
        else
        {
            lastULimited = false;
        }
    }

private:

    static constexpr float pwmToStallCurrent = 2.61598722;
    static constexpr float backEmfCurrent = -0.70435649 * 2 * 3.1415926535897932384626433832795028841972 / 4096.0;

    PwmHandler* pwmInstance;
    bool disableLoop;
    bool brake;
    int16_t ref;
    int16_t y;
    int16_t filteredY;
    int16_t filteredPwm;
    int16_t u;
    int16_t limitedU;
    bool lastULimited;
};

#endif
