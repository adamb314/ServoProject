#include "CurrentControlLoop.h"

CurrentControlLoop::CurrentControlLoop(uint32_t period) :
    pwmInstance(HBridge2WirePwm::getInstance()),
    currentSampler(new CurrentSampler(A1)),
    newPwmOverrideValue(true),
    newBrakeValue(true),
    newRefValue(0),
    newUValue(0),
    pwmOverride(true),
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
    newPwmOverrideValue = false;
    newBrakeValue = false;
    newRefValue = ref;
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
    newPwmOverrideValue = true;
    newBrakeValue = false;
    newUValue = pwm;
}

int16_t CurrentControlLoop::getFilteredPwm()
{
    ThreadInterruptBlocker interruptBlocker;
    return (filteredPwm >> 2);
}

void CurrentControlLoop::activateBrake()
{
    newPwmOverrideValue = true;
    newBrakeValue = true;
}

int16_t CurrentControlLoop::getCurrent()
{
    ThreadInterruptBlocker interruptBlocker;
    return filteredY;
}

void CurrentControlLoop::applyChanges()
{
    ThreadInterruptBlocker interruptBlocker;

    pwmOverride = newPwmOverrideValue;
    brake = newBrakeValue;
    ref = newRefValue;
    u = newUValue;
}


void CurrentControlLoop::run()
{
    y = currentSampler->getValue();

    filteredY = currentSampler->getFilteredValue();

    if (pwmOverride)
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

CurrentControlModel::CurrentControlModel(float pwmToStallCurrent, float backEmfCurrent) :
    pwmToStallCurrent{pwmToStallCurrent},
    backEmfCurrent{backEmfCurrent},
    pwmInstance(HBridge2WirePwm::getInstance()),
    pwmOverride(true),
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

CurrentControlModel::~CurrentControlModel()
{
}

void CurrentControlModel::setReference(int16_t ref)
{
    pwmOverride = false;
    this->ref = ref;
}

void CurrentControlModel::updateVelocity(float vel)
{
    this->vel = vel;
}

void CurrentControlModel::overidePwmDuty(int16_t pwm)
{
    pwmOverride = true;
    brake = false;
    u = pwm;
}

void CurrentControlModel::activateBrake()
{
    pwmOverride = true;
    brake = true;
}

void CurrentControlModel::applyChanges()
{
    if (pwmOverride)
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

int16_t CurrentControlModel::getLimitedRef()
{
    if (lastULimited)
    {
        return y;
    }

    return ref;
}

int16_t CurrentControlModel::getFilteredPwm()
{
    return filteredPwm;
}

int16_t CurrentControlModel::getCurrent()
{
    return filteredY;
}
