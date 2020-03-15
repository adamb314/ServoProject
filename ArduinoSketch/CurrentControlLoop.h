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
    int16_t u;
    int16_t limitedU;
    bool lastULimited;
    FunctionalWrapper<bool>* adcSampDoneCheck;
    std::vector<CodeBlocksThread*> threads;
};

#endif
