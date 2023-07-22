#include "CurrentControlLoop.h"

CurrentControlLoop::CurrentControlLoop(uint32_t period, int16_t currentSensorPin,
        std::unique_ptr<PwmHandler> pwmInstance) :
    CurrentController(std::move(pwmInstance)),
    currentSampler(new CurrentSampler(currentSensorPin)),
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

void CurrentControlLoop::setReference(int32_t ref)
{
    newPwmOverrideValue = false;
    newBrakeValue = false;
    newRefValue = ref;
}

int32_t CurrentControlLoop::getLimitedRef()
{
    ThreadInterruptBlocker interruptBlocker;
    if (lastULimited)
    {
        return y;
    }

    return ref;
}

void CurrentControlLoop::overidePwmDuty(int32_t pwm)
{
    newPwmOverrideValue = true;
    newBrakeValue = false;
    newUValue = pwm;
}

int32_t CurrentControlLoop::getFilteredPwm()
{
    ThreadInterruptBlocker interruptBlocker;
    return (filteredPwm >> 2);
}

void CurrentControlLoop::activateBrake()
{
    newPwmOverrideValue = true;
    newBrakeValue = true;
}

int32_t CurrentControlLoop::getCurrent()
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

    int32_t controlError;
    {
        ThreadInterruptBlocker interruptBlocker;
        controlError = ref - y;
    }

    u += ((controlError * 3) >> 4);

    limitedU = pwmInstance->setOutput(u);

    filteredPwm = ((3 * filteredPwm) >> 2) + limitedU;

    int32_t uLimitError = u - limitedU;

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

CurrentControlModel::CurrentControlModel(std::unique_ptr<PwmHandler> pwmInstance) :
    CurrentController(std::move(pwmInstance))
{
}

CurrentControlModel::CurrentControlModel(float pwmToStallCurrent, float backEmfCurrent,
        std::unique_ptr<PwmHandler> pwmInstance) :
    pwmToStallCurrentF{static_cast<int32_t>(pwmToStallCurrent * fixedPoint)},
    backEmfCurrentFF{static_cast<int32_t>(backEmfCurrent * fixedPoint * fixedPoint)},
    CurrentController(std::move(pwmInstance))
{
}

CurrentControlModel::~CurrentControlModel()
{
}

void CurrentControlModel::setReference(int32_t ref)
{
    pwmOverride = false;
    this->ref = ref;
}

void CurrentControlModel::updateVelocity(float vel)
{
    this->vel = vel;
}

void CurrentControlModel::overidePwmDuty(int32_t pwm)
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
    int32_t backEmfKoeffF = (backEmfCurrentFF * vel) / fixedPoint;

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
        y = (pwmToStallCurrentF * limitedU + backEmfKoeffF * abs(limitedU)) / fixedPoint;
        filteredY = y;
        filteredPwm = limitedU;
        return;
    }

    //ref = (pwmToStallCurrent * u + backEmfKoeff * abs(u) =>
    int32_t oneOverBackEnfF;
    if (ref >= 0)
    {
        oneOverBackEnfF = (fixedPoint * fixedPoint) / (pwmToStallCurrentF + backEmfKoeffF);
    }
    else
    {
        oneOverBackEnfF = (fixedPoint * fixedPoint) / (pwmToStallCurrentF - backEmfKoeffF);
    }
    u = (ref * oneOverBackEnfF) / fixedPoint;

    limitedU = pwmInstance->setOutput(u);

    y = (pwmToStallCurrentF * limitedU + backEmfKoeffF * abs(limitedU)) / fixedPoint;

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

int32_t CurrentControlModel::getLimitedRef()
{
    if (lastULimited)
    {
        return y;
    }

    return ref;
}

int32_t CurrentControlModel::getFilteredPwm()
{
    return filteredPwm;
}

int32_t CurrentControlModel::getCurrent()
{
    return filteredY;
}
