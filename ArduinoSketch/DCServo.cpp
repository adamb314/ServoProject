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

#ifdef SIMULATE
    rawMainPos = 2048;
    rawOutputPos = rawMainPos * 0.99f;
#else
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
#endif

    initialOutputPosOffset = rawOutputPos - rawMainPos;
    outputPosOffset = initialOutputPosOffset;

    x[0] = rawMainPos;
    x[1] = 0;
    x[2] = 0;

#ifdef SIMULATE
    xSim = x;
#endif

    kalmanFilter->reset(x);

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

void DCServo::setControlSpeed(uint8_t controlSpeed, uint16_t velControlSpeed, uint16_t filterSpeed)
{
    this->controlSpeed = controlSpeed;
    this->velControlSpeed = velControlSpeed;
    this->filterSpeed = filterSpeed;
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
    return x[1];
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
    auto out = loopTime;
    loopTime = 0;
    return out;
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
#ifndef SIMULATE
    mainEncoderHandler->triggerSample();
    if (outputEncoderHandler)
    {
        outputEncoderHandler->triggerSample();
    }
#endif

    float posRef;
    float velRef;
    float feedForwardU;
    float nextPosRef;
    float nextVelRef;
    float nextFeedForwardU;

    float controlSignal = 0.0f;

    if (controlEnabled)
    {
        if (pendingIntegralCalc)
        {
            Ivel -= L[2] * (vControlRef - x[1]);
            Ivel += L[3] * (pwm - currentController->getLimitedRef());
        }

        std::tie(posRef, velRef, feedForwardU) = refInterpolator.get();
        refInterpolator.calculateNext();
        std::tie(nextPosRef, nextVelRef, nextFeedForwardU) = refInterpolator.get();
        posRefTimingOffset = posRef - std::get<0>(refInterpolator.getUninterpolated());
    }
    pendingIntegralCalc = false;

#ifdef SIMULATE
    float uSim = kalmanControlSignal;
    xSim[0] += controlConfig->getA()(0, 1) * xSim[1] + controlConfig->getB()[0] * uSim;
    xSim[1] = controlConfig->getA()(1, 1) * xSim[1] + controlConfig->getB()[1] * uSim;
    rawMainPos = xSim[0];
#else
    rawMainPos = mainEncoderHandler->getValue();
#endif

    x = kalmanFilter->update(rawMainPos);

    if (controlEnabled)
    {
        if (!openLoopControlMode)
        {
            float posDiff = posRef - outputPosOffset - x[0];

            vControlRef = L[0] * posDiff + velRef;
            controlConfig->limitVelocity(vControlRef);

            controlSignal = L[1] * (vControlRef - x[1]) + Ivel;

            if (internalFeedForwardEnabled)
            {
                controlSignal += controlConfig->calculateFeedForward(nextVelRef, velRef);
            }

            kalmanControlSignal = controlSignal;

            controlSignal += feedForwardU;
#ifndef SIMULATE
            uint16_t rawEncPos = mainEncoderHandler->getUnscaledRawValue();
            pwm = controlConfig->applyForceCompensations(controlSignal, rawEncPos, vControlRef, x[1]);
#else
            pwm = 0;
#endif

            currentController->updateVelocity(x[1]);
            currentController->setReference(static_cast<int16_t>(pwm));
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
                currentController->setReference(static_cast<int16_t>(controlSignal));
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

    controlSignalAveraging.add(controlSignal);
    currentAveraging.add(current);

#ifdef SIMULATE
    rawOutputPos = rawMainPos * 0.99f;
#else
    if (outputEncoderHandler)
    {
        rawOutputPos = outputEncoderHandler->getValue();
    }
    else
    {
        rawOutputPos = rawMainPos;
    }
#endif

    if (controlEnabled && !openLoopControlMode && !onlyUseMainEncoderControl)
    {
        const uint8_t backlashControlGainCycleDelay = 8;
        if (backlashControlGainDelayCounter == 0)
        {
            backlashControlGainDelayCounter = backlashControlGainCycleDelay;
            backlashControlGain = L[4] * (0.1f + 0.9f * std::max(0.0f, 1.0f - L[5] * std::abs(velRef)));
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

    loopTime = std::max(loopTime, static_cast<uint16_t>(ThreadHandler::getInstance()->getTimingError()));
}

void DCServo::calculateAndUpdateLVector()
{

    const Eigen::Matrix3f& A = controlConfig->getA();
    const Eigen::Vector3f& B = controlConfig->getB();

    float dt = A(0, 1);
    float a = A(1, 1);
    float b = B(1);

    float posControlPole = exp(-dt * controlSpeed);
    float velControlPole[] = {exp(-1.0f * dt * velControlSpeed), exp(-0.9f * dt * velControlSpeed)};

    auto tempL = L;
    tempL[0] = (1.0f - posControlPole) / dt;
    tempL[1] = (a + 1 - velControlPole[0] - velControlPole[1]) / b;
    tempL[2] = (a - b * tempL[1] - velControlPole[0] * velControlPole[1]) / b;
    tempL[3] = 10 * tempL[2];

    tempL[4] = backlashControlSpeed * controlConfig->getCycleTime();
    tempL[5] = backlashControlSpeedVelGain * (1.0f / 255) * (1.0f / 10) ;
    tempL[6] = backlashSize;

    auto K = kalmanFilter->calculateNewKVector(filterSpeed);

    ThreadInterruptBlocker blocker;
    L = tempL;
    kalmanFilter->setNewKVector(K);
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

std::tuple<float, float, float> ReferenceInterpolator::getUninterpolated()
{
    return std::make_tuple(pos[1], vel[1], feed[1]);
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
