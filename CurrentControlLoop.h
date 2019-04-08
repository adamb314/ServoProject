#ifndef CURRENT_CONTROL_LOOP_H
#define CURRENT_CONTROL_LOOP_H

#include "ThreadHandler.h"
#include "FilteredADC.h"
#include "PwmHandler.h"
#include <Eigen.h>
#include <vector>

class CurrentControlLoop
{
public:
    CurrentControlLoop(uint32_t period);

    ~CurrentControlLoop();

    void setReference(float ref);

    void overidePwmDuty(int16_t pwm);

    float getCurrent();

private:
    void run();

    PwmHandler* pwmInstance;
    FilteredADC* filteredADC;
    float integral;
    float ref;
    float y;
    float controlError;
    float u;
    float limitedU;
    bool disableLoop;
    uint32_t startTime;
    int32_t loopTime;
    Eigen::Vector3f L;
    std::vector<FunctionThread*> threads;
};

#endif
