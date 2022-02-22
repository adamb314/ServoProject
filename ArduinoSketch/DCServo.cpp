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
    rawOutputPos = rawMainPos;
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
    return rawOutputPos;
}

int16_t DCServo::getVelocity()
{
    ThreadInterruptBlocker blocker;
    return x[1];
}

int16_t DCServo::getControlSignal()
{
    ThreadInterruptBlocker blocker;
    if (controlEnabled)
    {
        return controlSignal;
    }
    return 0;
}

int16_t DCServo::getCurrent()
{
    ThreadInterruptBlocker blocker;
    return current;
}

int16_t DCServo::getPwmControlSignal()
{
    ThreadInterruptBlocker blocker;
    return pwmControlSIgnal;
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
    return rawMainPos + initialOutputPosOffset;
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

    if (controlEnabled)
    {
        if (!openLoopControlMode)
        {
            Ivel -= L[2] * (vControlRef - x[1]);
            Ivel += L[3] * (pwm - currentController->getLimitedRef());

            refInterpolator.getNext(posRef, velRef, feedForwardU);

            if (!onlyUseMainEncoderControl)
            {
                const uint8_t backlashControlGainCycleDelay = 8;
                if (backlashControlGainDelayCounter == 0)
                {
                    backlashControlGainDelayCounter = backlashControlGainCycleDelay;
                    backlashControlGain = L[4] * (0.1f + 0.9f * std::max(0.0f, 1.0f - L[5] * std::abs(velRef)));
                }
                backlashControlGainDelayCounter--;
            }
        }
    }
#ifdef SIMULATE
    xSim = controlConfig->getA() * xSim + controlConfig->getB() * controlSignal;
    rawMainPos =xSim[0];
    rawOutputPos = rawMainPos;
#else
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

    x = kalmanFilter->update(controlSignal, rawMainPos);

    if (controlEnabled)
    {
        if (!openLoopControlMode)
        {
            if (!onlyUseMainEncoderControl)
            {
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
                
                double backlashCompensationDiff = backlashControlGain * (posRef - rawOutputPos);
                outputPosOffset -= backlashCompensationDiff;
            }

            posRef -= outputPosOffset;

            float posDiff = posRef - x[0];

            vControlRef = L[0] * posDiff + velRef;
            controlConfig->limitVelocity(vControlRef);

            float u = L[1] * (vControlRef - x[1]) + Ivel + feedForwardU;

            controlSignal = u;

            uint16_t rawEncPos = mainEncoderHandler->getUnscaledRawValue();
            u = controlConfig->applyForceCompensations(u, rawEncPos, vControlRef, x[1]);
            pwm = u;

            currentController->updateVelocity(x[1]);
            currentController->setReference(static_cast<int16_t>(pwm));
            currentController->applyChanges();
            current = currentController->getCurrent();
            pwmControlSIgnal = currentController->getFilteredPwm();
        }
        else
        {
            Ivel = 0;
            outputPosOffset = rawOutputPos - rawMainPos;
            backlashControlGainDelayCounter = 0;

            float posRef;
            float velRef;
            float feedForwardU;

            refInterpolator.getNext(posRef, velRef, feedForwardU);

            if (pwmOpenLoopMode)
            {
                controlSignal = 0;
                currentController->overidePwmDuty(feedForwardU);
            }
            else
            {
                controlSignal = feedForwardU;
                currentController->setReference(static_cast<int16_t>(controlSignal));
            }
            currentController->applyChanges();
            current = currentController->getCurrent();
            pwmControlSIgnal = currentController->getFilteredPwm();
        }
    }
    else
    {
        refInterpolator.resetTiming();
        loadNewReference(rawOutputPos, 0, 0);
        Ivel = 0;
        outputPosOffset = rawOutputPos - rawMainPos;
        backlashControlGainDelayCounter = 0;
        controlSignal = 0;
        currentController->activateBrake();
        currentController->applyChanges();
        current = currentController->getCurrent();
        pwmControlSIgnal = currentController->getFilteredPwm();
    }

    int32_t newLoopTime = ThreadHandler::getInstance()->getTimingError();
    if (newLoopTime > loopTime)
    {
        loopTime = newLoopTime;
    }
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
        midPointTimeOffset = getTimeInterval;

        timingInvalid = false;
    }
    else
    {
        uint16_t updatePeriod = timestamp - lastUpdateTimingTimestamp;
        uint16_t timeSinceLastGet = timestamp - lastGetTimestamp;

        int16_t timingError = getTimeInterval - (midPointTimeOffset + timeSinceLastGet);
        int16_t periodError = updatePeriod - loadTimeInterval;

        midPointTimeOffset += timingError / 8;
        loadTimeInterval += periodError / 16;

        invertedLoadInterval = 1.0f / loadTimeInterval;
    }

    lastUpdateTimingTimestamp = timestamp;
}

void ReferenceInterpolator::resetTiming()
{
    timingInvalid = true;
    refInvalid = true;
}

void ReferenceInterpolator::getNext(float& position, float& velocity, float& feedForward)
{
    lastGetTimestamp = micros();

    if (refInvalid)
    {
        position = pos[2];
        velocity = vel[2];
        feedForward = feed[2];

        return;
    }

    if (midPointTimeOffset < 2 * loadTimeInterval)
    {
        midPointTimeOffset += getTimeInterval;
    }

    float t = midPointTimeOffset * invertedLoadInterval;

    if (t < -1.0f)
    {
        t = -1.0f;
    }
    else if (t > 1.2f)
    {
        t = 1.2f;
    }

    if (t < 0.0f)
    {
        t += 1.0f;
        feedForward = feed[0] + t * (feed[1] - feed[0]);
        float velDiff = vel[1] - vel[0];
        velocity = vel[0] + t * velDiff;
        position = pos[0] + t * (pos[1] - pos[0] + dtDiv2 * (t * velDiff - velDiff));
    }
    else
    {
        feedForward = feed[1] + t * (feed[2] - feed[1]);
        float velDiff = vel[2] - vel[1];
        velocity = vel[1] + t * velDiff;
        position = pos[1] + t * (pos[2] - pos[1] + dtDiv2 * (t * velDiff - velDiff));
    }
}

void ReferenceInterpolator::setGetTimeInterval(const uint16_t& interval)
{
    resetTiming();

    getTimeInterval = interval;
}

void ReferenceInterpolator::setLoadTimeInterval(const uint16_t& interval)
{
    resetTiming();

    loadTimeInterval = interval;
    invertedLoadInterval = 1.0f / loadTimeInterval;
    dtDiv2 = loadTimeInterval * 0.000001f * 0.5f;
}
