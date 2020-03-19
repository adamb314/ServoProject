#include "CurrentControlLoop.h"

CurrentControlLoop::CurrentControlLoop(uint32_t period) :
    pwmInstance(HBridge4WirePwm::getInstance()),
    currentSampler(new CurrentSampler(A1)),
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
    currentSampler->init();

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
    delete currentSampler;
}

void CurrentControlLoop::setReference(int16_t ref)
{
    ThreadInterruptBlocker interruptBlocker;
    disableLoop = false;
    this->ref = ref;
}

int16_t CurrentControlLoop::getLimitedRef()
{
    ThreadInterruptBlocker interruptBlocker;
    if (lastULimited)
    {
        return y;
    }

    return ref;
}

void CurrentControlLoop::overidePwmDuty(int16_t pwm)
{
    ThreadInterruptBlocker interruptBlocker;
    disableLoop = true;
    brake = false;
    this->u = pwm;
}

int16_t CurrentControlLoop::getFilteredPwm()
{
    ThreadInterruptBlocker interruptBlocker;
    return (filteredPwm >> 2);
}

void CurrentControlLoop::activateBrake()
{
    ThreadInterruptBlocker interruptBlocker;
    disableLoop = true;
    brake = true;
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
        if (brake)
        {
            pwmInstance->activateBrake();
            filteredPwm = 0;
            return;
        }
        limitedU = pwmInstance->setOutput(u) << 2;
        filteredPwm = limitedU;
        return;
    }

    int16_t controlError;
    {
        ThreadInterruptBlocker interruptBlocker;
        controlError = ref - y;
    }

    u += ((controlError * 3) >> 4);

    limitedU = pwmInstance->setOutput(u);

    filteredPwm = ((3 * filteredPwm) >> 2) + limitedU;

    int16_t uLimitError = u - limitedU;

    if (uLimitError != 0)
    {
        lastULimited = true;
    }
    else
    {
        lastULimited = false;
    }

    u -= uLimitError;
}
