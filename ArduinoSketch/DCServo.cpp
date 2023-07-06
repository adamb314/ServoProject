#include "DCServo.h"

DCServo::DCServo(std::unique_ptr<CurrentController> currentController,
            std::unique_ptr<EncoderHandlerInterface> mainEncoderHandler,
            std::unique_ptr<EncoderHandlerInterface> outputEncoderHandler,
            std::unique_ptr<KalmanFilter> kalmanFilter,
            std::unique_ptr<ControlConfigurationInterface> controlConfig):
        currentController(std::move(currentController)),
        mainEncoderHandler(std::move(mainEncoderHandler)),
        outputEncoderHandler(std::move(outputEncoderHandler)),
        kalmanFilter(std::move(kalmanFilter)),
        controlConfig(std::move(controlConfig))
{
    init();
}

void DCServo::init()
{
    InterruptTimer::getInstance()->enableTimerSyncEvents(false);

    uint32_t cycleTime = controlConfig->getCycleTimeUs();

    refInterpolator.setGetTimeInterval(cycleTime);
    refInterpolator.setLoadTimeInterval(18000);

    mainEncoderHandler->init();
    if (outputEncoderHandler)
    {
        outputEncoderHandler->init();
    }

    mainEncoderHandler->triggerSample();
    if (outputEncoderHandler)
    {
        outputEncoderHandler->triggerSample();
    }

    rawMainPos = mainEncoderHandler->getValue();
    if (outputEncoderHandler)
    {
        rawOutputPos = outputEncoderHandler->getValue();
    }
    else
    {
        rawOutputPos = rawMainPos;
    }

    initialOutputPosOffset = rawOutputPos - rawMainPos;
    outputPosOffset = initialOutputPosOffset;

    x[0] = rawMainPos;
    x[1] = 0;
    x[2] = 0;

    kalmanFilter->reset(x);

    outputEncoderFilter.set(rawOutputPos);

    loadNewReference(rawOutputPos, 0, 0);

    threads.push_back(createThread(1, cycleTime, 0,
        [&]()
        {
            controlLoop();
        }));
}

bool DCServo::isEnabled()
{
    ThreadInterruptBlocker blocker;
    return controlEnabled;
}

void DCServo::enable(bool b)
{
    if (!isEnabled() && b)
    {
        calculateAndUpdateLVector();
    }
    ThreadInterruptBlocker blocker;

    controlEnabled = b;
}

void DCServo::openLoopMode(bool enable, bool pwmMode)
{
    ThreadInterruptBlocker blocker;
    openLoopControlMode = enable;
    pwmOpenLoopMode = pwmMode;
}

void DCServo::onlyUseMainEncoder(bool b)
{
    ThreadInterruptBlocker blocker;
    onlyUseMainEncoderControl = b;
}

void DCServo::setControlSpeed(uint8_t controlSpeed)
{
    setControlSpeed(controlSpeed, controlSpeed * 4, controlSpeed * 4 * 8);
}

void DCServo::setControlSpeed(uint8_t controlSpeed, uint16_t velControlSpeed, uint16_t filterSpeed, float inertiaMarg)
{
    this->controlSpeed = controlSpeed;
    this->velControlSpeed = velControlSpeed;
    this->filterSpeed = filterSpeed;
    this->inertiaMarg = inertiaMarg;
}

void DCServo::setBacklashControlSpeed(uint8_t backlashControlSpeed, uint8_t backlashControlSpeedVelGain, uint8_t backlashSize)
{
    this->backlashControlSpeed = backlashControlSpeed;
    this->backlashControlSpeedVelGain = backlashControlSpeedVelGain;
    this->backlashSize = backlashSize;
}

void DCServo::enableInternalFeedForward(bool enable)
{
    internalFeedForwardEnabled = enable;
}

void DCServo::loadNewReference(float pos, int16_t vel, int16_t feedForwardU)
{
    ThreadInterruptBlocker blocker;
    refInterpolator.loadNew(pos, vel, feedForwardU);
}

void DCServo::triggerReferenceTiming()
{
    ThreadInterruptBlocker blocker;

    if (controlEnabled)
    {
        refInterpolator.updateTiming();
    }
}

float DCServo::getPosition()
{
    ThreadInterruptBlocker blocker;
    return rawOutputPos - posRefTimingOffset;
}

int16_t DCServo::getVelocity()
{
    ThreadInterruptBlocker blocker;
    return adam_std::clamp_cast<int16_t>(x[1]);
}

int16_t DCServo::getControlSignal()
{    
    return controlSignalAveraging.get();
}

int16_t DCServo::getCurrent()
{
    return currentAveraging.get();
}

int16_t DCServo::getPwmControlSignal()
{
    ThreadInterruptBlocker blocker;
    return pwmControlSignal;
}

uint16_t DCServo::getLoopTime()
{
    ThreadInterruptBlocker blocker;
    auto out = adam_std::clamp_cast<uint16_t>(loopTime);
    loopTime = 0;
    return out;
}

uint16_t DCServo::getLoopNr()
{
    ThreadInterruptBlocker blocker;
    return static_cast<uint16_t>(loopNr);
}

float DCServo::getBacklashCompensation()
{
    ThreadInterruptBlocker blocker;
    return outputPosOffset - initialOutputPosOffset;
}

float DCServo::getMainEncoderPosition()
{
    ThreadInterruptBlocker blocker;
    return rawMainPos + initialOutputPosOffset - posRefTimingOffset;
}

EncoderHandlerInterface::DiagnosticData DCServo::getMainEncoderDiagnosticData()
{
    ThreadInterruptBlocker blocker;
    return mainEncoderHandler->getDiagnosticData();
}

void DCServo::controlLoop()
{
    mainEncoderHandler->triggerSample();
    if (outputEncoderHandler)
    {
        outputEncoderHandler->triggerSample();
    }

    float posRef;
    float velRef;
    float feedForwardU;
    float nextPosRef;
    float nextVelRef;
    float nextFeedForwardU;

    int32_t controlSignal = 0;

    if (controlEnabled)
    {
        if (pendingIntegralCalc)
        { 
            float limitedRefDiff = pwm - currentController->getLimitedRef();
            Ivel += L[2] * (vControlRef - x[1]);
            Ivel -= L[3] * (limitedRefDiff);
        }

        std::tie(posRef, velRef, feedForwardU) = refInterpolator.get();

        posRefTimingOffset = 0.0f;
        if (!openLoopControlMode)
        {
            posRefTimingOffset = refInterpolator.getPositionInterpolationDist();
        }

        refInterpolator.calculateNext();
        std::tie(nextPosRef, nextVelRef, nextFeedForwardU) = refInterpolator.get();
    }
    pendingIntegralCalc = false;

    rawMainPos = mainEncoderHandler->getValue();

    x = kalmanFilter->update(rawMainPos);

    if (controlEnabled)
    {
        if (!openLoopControlMode)
        {
            float posDiff = posRef - outputPosOffset - x[0];

            vControlRef = L[0] * posDiff + velRef;
            controlConfig->limitVelocity(vControlRef);

            controlSignal = adam_std::clamp_cast<int16_t>(L[1] * (vControlRef - x[1]) + Ivel);

            if (internalFeedForwardEnabled)
            {
                controlSignal += adam_std::clamp_cast<int16_t>(controlConfig->calculateFeedForward(nextVelRef, velRef));
            }

            kalmanControlSignal = controlSignal;

            controlSignal += adam_std::clamp_cast<int16_t>(feedForwardU);

            uint16_t rawEncPos = mainEncoderHandler->getUnscaledRawValue();
            bool brake;
            std::tie(pwm, brake) = controlConfig->applyForceCompensations(controlSignal, rawEncPos, vControlRef, x[1]);
            
            if (brake)
            {
                currentController->activateBrake();
            }
            else
            {
                currentController->setReference(pwm);
            }

            currentController->updateVelocity(x[1]);
            currentController->applyChanges();

            current = currentController->getCurrent();
            pwmControlSignal = currentController->getFilteredPwm();

            pendingIntegralCalc = true;
        }
        else
        {
            Ivel = 0.0f;
            outputPosOffset = rawOutputPos - rawMainPos;
            backlashControlGainDelayCounter = 0;

            if (pwmOpenLoopMode)
            {
                controlSignal = 0.0f;
                kalmanControlSignal = controlSignal;
                currentController->overidePwmDuty(feedForwardU);
            }
            else
            {
                controlSignal = feedForwardU;
                kalmanControlSignal = controlSignal;
                uint16_t rawEncPos = mainEncoderHandler->getUnscaledRawValue();
                bool temp;
                std::tie(pwm, temp) = controlConfig->applyForceCompensations(controlSignal, rawEncPos, 0.0f, x[1]);
                currentController->setReference(pwm);
            }
            currentController->applyChanges();
            current = currentController->getCurrent();
            pwmControlSignal = currentController->getFilteredPwm();
        }
    }
    else
    {
        refInterpolator.resetTiming();
        loadNewReference(rawOutputPos, 0.0f, 0.0f);
        Ivel = 0.0f;
        outputPosOffset = rawOutputPos - rawMainPos;
        backlashControlGainDelayCounter = 0.0f;
        controlSignal = 0.0f;
        kalmanControlSignal = controlSignal;
        currentController->activateBrake();
        currentController->applyChanges();
        current = currentController->getCurrent();
        pwmControlSignal = currentController->getFilteredPwm();
    }

    kalmanFilter->postUpdate(kalmanControlSignal);

    controlSignalAveraging.add(currentController->getLimitedRef() - pwm + controlSignal);

    currentAveraging.add(current);

    if (outputEncoderHandler)
    {
        outputEncoderFilter.update(outputEncoderHandler->getValue(), rawMainPos);
        rawOutputPos = outputEncoderFilter.get();
    }
    else
    {
        rawOutputPos = rawMainPos;
    }

    if (controlEnabled && !openLoopControlMode && !onlyUseMainEncoderControl)
    {
        const uint8_t backlashControlGainCycleDelay = 8;
        if (backlashControlGainDelayCounter == 0)
        {
            backlashControlGainDelayCounter = backlashControlGainCycleDelay;
            backlashControlGain = L[4] * (10 * 100 +
                    90 * std::max((int32_t)0, 100 - static_cast<int32_t>(L[5]) * std::abs(static_cast<int32_t>(velRef))));
        }
        backlashControlGainDelayCounter--;

        int newForceDir = forceDir;
        if (feedForwardU > 1.0f)
        {
            newForceDir = 1;
        }
        else if (feedForwardU < -1.0f)
        {
            newForceDir = -1;
        }

        if (newForceDir != forceDir)
        {
            outputPosOffset -= newForceDir * L[6];
        }
        forceDir = newForceDir;

        float backlashCompensationDiff = backlashControlGain * (posRef - rawOutputPos);
        outputPosOffset -= backlashCompensationDiff;
    }

    loopTime = std::max(loopTime, ThreadHandler::getInstance()->getTimingError());
    loopNr += 1;
}

void DCServo::calculateAndUpdateLVector()
{
    const Eigen::Matrix3f& A = controlConfig->getA();
    const Eigen::Vector3f& B = controlConfig->getB();

    float dt = A(0, 1);
    float a = A(1, 1);
    float b = B(1);

    // calculating discrete time poles from continues
    float posControlPole = exp(-dt * controlSpeed);
    float velControlPole = exp(-1.0f * dt * velControlSpeed);

    auto tempL = L;

    // see section 'Calculating the control parameters' of Doc/Theory.md for derivation of control equations
    tempL[0] = (1.0f - posControlPole) / dt;

    auto calculateVelContolParams = [](float a, float b, float velPole, float inertiaMarg)
        {
            velPole = std::min(std::min(velPole, a), 1.0f);

            float L1 = inertiaMarg * (a - 2.0f * velPole + 1.0f)
                        + 2.0f * sqrt(inertiaMarg * (inertiaMarg - 1.0f) * (1.0f - velPole) * (a - velPole));
            float L2 = (L1 * L1 / inertiaMarg + (a - 1.0f) * (inertiaMarg * (a - 1.0f) - 2.0f * L1)) / 4.0f;
            L1 = L1 / b;
            L2 = L2 / b;
            float L3 = std::min(10 * L2, 0.1f);

            return std::make_tuple(L1, L2, L3);
        };

    std::tie(tempL[1], tempL[2], tempL[3]) = calculateVelContolParams(a, b, velControlPole, inertiaMarg);

    float backlashControlPole = backlashControlSpeed * controlConfig->getCycleTime();
    tempL[4] = backlashControlPole / 100.0f / 100.0f;
    tempL[5] = backlashControlSpeedVelGain * (1.0f / 255) * (1.0f / 10) * 100.0f;
    tempL[6] = backlashSize;

    float velControlPoleAtInertiaMarg = 0.5f * (a - b * tempL[1] / inertiaMarg + 1.0f);
    uint16_t minFilterSpeed = std::round(-log(velControlPoleAtInertiaMarg) / dt * filterSpeed / velControlSpeed);
    auto K = kalmanFilter->calculateNewKVector(std::max(filterSpeed, minFilterSpeed));

    ThreadInterruptBlocker blocker;
    L = tempL;
    kalmanFilter->setNewKVector(K);
    if (backlashControlPole != 0.0f)
    {
        outputEncoderFilter.setFilterConst(1.0f - backlashControlPole * 1);
    }
    else
    {
        outputEncoderFilter.setFilterConst(0.0f);
    }
}

ReferenceInterpolator::ReferenceInterpolator()
{
}

void ReferenceInterpolator::loadNew(float position, float velocity, float feedForward)
{
    if (refInvalid)
    {
        if (!timingInvalid)
        {
            refInvalid = false;
        }

        pos[2] = position;
        vel[2] = velocity;
        feed[2] = feedForward;

        pos[1] = pos[2];
        vel[1] = vel[2];
        feed[1] = feed[2];

        pos[0] = pos[1];
        vel[0] = vel[1];
        feed[0] = feed[1];
    }
    else
    {
        if (midPointTimeOffset > -loadTimeInterval)
        {
            midPointTimeOffset -= loadTimeInterval;
        }

        pos[0] = pos[1];
        vel[0] = vel[1];
        feed[0] = feed[1];

        pos[1] = pos[2];
        vel[1] = vel[2];
        feed[1] = feed[2];

        pos[2] = position;
        vel[2] = velocity;
        feed[2] = feedForward;
    }
}

void ReferenceInterpolator::updateTiming()
{
    uint16_t timestamp = micros();
    if (timingInvalid)
    {
        midPointTimeOffset = 2 * getTimeInterval;

        timingInvalid = false;
    }
    else
    {
        uint16_t updatePeriod = timestamp - lastUpdateTimingTimestamp;
        uint16_t timeSinceLastGet = timestamp - lastGetTimestamp;

        int16_t timingError = 2 * getTimeInterval - (midPointTimeOffset + timeSinceLastGet);
        int16_t periodError = updatePeriod - loadTimeInterval;

        midPointTimeOffset += timingError / 8;
        loadTimeInterval += periodError / 16;

        dtDiv2 = loadTimeInterval * 0.000001f * 0.5f;
        invertedLoadInterval = 1.0f / loadTimeInterval;
        getTStepSize = getTimeInterval * invertedLoadInterval;
    }

    lastUpdateTimingTimestamp = timestamp;
}

void ReferenceInterpolator::resetTiming()
{
    timingInvalid = true;
    refInvalid = true;

    stepAndUpdateInter();
}

void ReferenceInterpolator::calculateNext()
{
    lastGetTimestamp = micros();

    stepAndUpdateInter();
}

void ReferenceInterpolator::stepAndUpdateInter()
{
    if (refInvalid)
    {
        interPos = pos[2];
        interVel = vel[2];
        interFeed = feed[2];

        return;
    }

    if (midPointTimeOffset < 2 * loadTimeInterval)
    {
        midPointTimeOffset += getTimeInterval;
    }

    float t = midPointTimeOffset * invertedLoadInterval;
    t = std::min(t, 1.2f);
    t = std::max(t, -1.0f);

    if (t < 0.0f)
    {
        t += 1.0f;

        float s = midPointTimeOffset * invertedGetInterval + 1.0f;
        s = std::max(s, 0.0f);

        interFeed = feed[0] * (1.0f - s) + feed[1] * s;
        float velDiff = vel[1] - vel[0];
        interVel = vel[0] + t * velDiff;
        interPos = pos[0] + t * (pos[1] - pos[0] + dtDiv2 * (t * velDiff - velDiff));
    }
    else
    {
        interFeed = feed[1];
        float velDiff = vel[2] - vel[1];
        interVel = vel[1] + t * velDiff;
        interPos = pos[1] + t * (pos[2] - pos[1] + dtDiv2 * (t * velDiff - velDiff));
    }
}

std::tuple<float, float, float> ReferenceInterpolator::get()
{
    return std::make_tuple(interPos, interVel, interFeed);
}

float ReferenceInterpolator::getPositionInterpolationDist()
{
    return interPos - pos[1];
}

void ReferenceInterpolator::setGetTimeInterval(const uint16_t& interval)
{
    resetTiming();

    getTimeInterval = interval;
    invertedGetInterval = 1.0f / getTimeInterval;
    getTStepSize = getTimeInterval * invertedLoadInterval;
}

void ReferenceInterpolator::setLoadTimeInterval(const uint16_t& interval)
{
    resetTiming();

    loadTimeInterval = interval;
    invertedLoadInterval = 1.0f / loadTimeInterval;
    dtDiv2 = loadTimeInterval * 0.000001f * 0.5f;
    getTStepSize = getTimeInterval * invertedLoadInterval;
}

ComplementaryFilter::ComplementaryFilter(float x0)
{
    set(x0);
}

void ComplementaryFilter::update(float lowFrqIn, float highFrqIn)
{
    int32_t lowFrqInFixed = (lowFrqIn - outWrapAround) * fixedPoint;
    int32_t highFrqInFixed = highFrqIn * fixedPoint;

    int32_t highFrqDiffFixed = adam_std::wrapAroundDist<wrapAroundSize * fixedPoint>(
                highFrqInFixed - lastHighFrqInFixed);
    outFixed = (aFixed * (outFixed + highFrqDiffFixed) +
            (fixedPoint - aFixed) * lowFrqInFixed) / fixedPoint;
    lastHighFrqInFixed = highFrqInFixed;

    int32_t outFixedWraped = adam_std::wrapAround<wrapAroundSize * fixedPoint>(outFixed);
    outWrapAround += (outFixed - outFixedWraped) / (wrapAroundSize * fixedPoint) * wrapAroundSize;
    outFixed = outFixedWraped;
}

float ComplementaryFilter::get()
{
    return outFixed * (1.0f / fixedPoint) + outWrapAround;
}

void ComplementaryFilter::set(float x0)
{
    outFixed = x0 * fixedPoint;
}

void ComplementaryFilter::setFilterConst(float a)
{
    aFixed = a * fixedPoint;
    aFixed = std::min(std::max(aFixed, (int32_t)0), (int32_t)fixedPoint - 1);
}
