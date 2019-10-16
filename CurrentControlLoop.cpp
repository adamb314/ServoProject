#include "CurrentControlLoop.h"

CurrentControlLoop::CurrentControlLoop(uint32_t period) :
    pwmInstance(HBridge4WirePwm::getInstance()),
    currentSampler(new CurrentSampler()),
    disableLoop(true),
    ref(0),
    y(0),
    filteredY(0),
    u(0),
    limitedU(0),
    lastULimited(false)
{
    currentSampler->init(A1);

    adcSampDoneCheck = createFunctionalWrapper<bool>(
        [&]()
        {
            return currentSampler->sampleReady();
        });

    threads.push_back(createThreadWithCodeBlocks(2, period, 0,
        [&]()
        {
            currentSampler->triggerSample();
            Thread::delayNextCodeBlockUntil(adcSampDoneCheck);
        }));

    threads[0]->addCodeBlock([&]()
        {
            this->run();
        });
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

int16_t CurrentControlLoop::getLimitedCurrent()
{
    ThreadInterruptBlocker interruptBlocker;
    if (lastULimited)
    {
        return filteredY;
    }

    return ref;
}

void CurrentControlLoop::run()
{
    y = currentSampler->getValue();

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

    u += ((controlError * 3) >> 4);

    limitedU = pwmInstance->setOutput(u);

    int16_t uLimitError = u - limitedU;

    if (uLimitError != 0)
    {
        if (!lastULimited)
        {
            currentSampler->resetFilteredValue();
        }
        lastULimited = true;
    }
    else
    {
        lastULimited = false;
    }

    u -= uLimitError;

    filteredY = currentSampler->getFilteredValue();
}
