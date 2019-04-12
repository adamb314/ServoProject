#include "CurrentControlLoop.h"

CurrentControlLoop::CurrentControlLoop(uint32_t period) :
    pwmInstance(HBridge4WirePwm::getInstance()),
    currentSampler(new CurrentSampler()),
    disableLoop(true),
    ref(0),
    y(0),
    filteredY(0),
    u(0),
{
    currentSampler->init(A1);

    threads.push_back(new FunctionThread(2, period, 0,
        [&]()
        {
            currentSampler->triggerSample();
        }));

    threads.push_back(new FunctionThread(2, period, 200,
        [&]()
        {
            this->run();
        }));
}

CurrentControlLoop::~CurrentControlLoop()
{
}

void CurrentControlLoop::setReference(int16_t ref)
{
    ThreadInterruptBlocker interruptBlocker;
    disableLoop = false;
    this->ref = ref;
}

void CurrentControlLoop::overidePwmDuty(int16_t pwm)
{
    ThreadInterruptBlocker interruptBlocker;
    disableLoop = true;
    this->u = pwm;
}

int16_t CurrentControlLoop::getCurrent()
{
    ThreadInterruptBlocker interruptBlocker;
    return filteredY;
}

void CurrentControlLoop::run()
{
    y = currentSampler->getValue();
    filteredY = currentSampler->getFilteredValue();

    if (disableLoop)
    {
        pwmInstance->setOutput(u);
        return;
    }

    int16_t controlError;
    {
        ThreadInterruptBlocker interruptBlocker;
        controlError = ref - y;
    }

    u += (controlError >> 1);

    limitedU = pwmInstance->setOutput(u);

    u -= (u - limitedU);
}
