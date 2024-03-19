#include "DCServo.h"
#include "../Hardware/FailSafeHandler.h"

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

    cycleTime = controlConfig->getCycleTimeUs();

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
            uint32_t loopStartTime = ThreadHandler::getInstance()->getTimingError();
            controlLoop();
            uint32_t loopEndTime = ThreadHandler::getInstance()->getTimingError();

            loopTime = std::max(loopTime, loopEndTime);
            loopNr += 1;

            if (loopEndTime - loopStartTime > cycleTime)
            {
                FailSafeHandler::getInstance()->goToFailSafe();
            }
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

void DCServo::loadNewReference(float pos, float vel, int16_t feedForwardU)
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

float DCServo::getControlError()
{
    ThreadInterruptBlocker blocker;
    return posDiff;
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
    int32_t feedForwardU;
    float nextPosRef;
    float nextVelRef;
    int32_t nextFeedForwardU;

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
            posDiff = posRef - outputPosOffset - x[0];

            vControlRef = L[0] * posDiff + velRef;
            controlConfig->limitVelocity(vControlRef);

            controlSignal = adam_std::clamp_cast<int16_t>(L[1] * (vControlRef - x[1]) + Ivel);

            if (internalFeedForwardEnabled)
            {
                controlSignal += adam_std::clamp_cast<int16_t>(controlConfig->calculateFeedForward(nextVelRef, velRef));
            }

            kalmanControlSignal = controlSignal;

            controlSignal += feedForwardU;

            uint16_t rawEncPos = mainEncoderHandler->getUnscaledRawValue();
            bool brake;
            std::tie(pwm, brake) = controlConfig->applyForceCompensations(controlSignal, rawEncPos, velRef, vControlRef);
            brake |= std::abs(x[1]) <= 1;
            currentController->addDamping(brake);
            currentController->setReference(pwm);

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
                currentController->addDamping(false);
            }
            else
            {
                controlSignal = feedForwardU;
                kalmanControlSignal = controlSignal;
                uint16_t rawEncPos = mainEncoderHandler->getUnscaledRawValue();
                bool temp;
                std::tie(pwm, temp) = controlConfig->applyForceCompensations(controlSignal, rawEncPos, 0.0f, x[1]);
                currentController->addDamping(false);
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
        backlashControlGainDelayCounter = 0;
        controlSignal = 0.0f;
        kalmanControlSignal = controlSignal;
        currentController->addDamping(false);
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

            int newForceDir = forceDir;
            newForceDir = adam_std::chooseOne(1, newForceDir, feedForwardU > 1);
            newForceDir = adam_std::chooseOne(-1, newForceDir, feedForwardU < -1);
            if (newForceDir != forceDir)
            {
                outputPosOffset -= newForceDir * backlashSize;
            }
            forceDir = newForceDir;
        }
        backlashControlGainDelayCounter--;

        float backlashCompensationDiff = backlashControlGain * (posRef - rawOutputPos);
        outputPosOffset -= backlashCompensationDiff;
    }
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

    backlashControlGain = L[4] * 100 * 100;
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

float ControlConfigurationInterface::getCycleTime()
{
    auto A = getA();
    auto B = getB();

    float contineusA = (1.0f - A(1, 1)) / A(0, 1);

    if (contineusA <= 0.01f)
    {
        return A(0, 1);
    }

    return -log(A(1, 1)) / contineusA;
}

uint32_t ControlConfigurationInterface::getCycleTimeUs()
{
    float dt = round(getCycleTime() / 0.0002f) * 0.0002f;
    return static_cast<uint32_t>(dt * 1000000ul);
}

DefaultControlConfiguration::DefaultControlConfiguration(const Eigen::Matrix3f& A, const Eigen::Vector3f& B,
            const float& maxVel, const float& avarageFriction,
            const std::array<int16_t, vecSize>& posDepForceCompVec,
            const std::array<int16_t, vecSize>& posDepFrictionCompVec,
            const EncoderHandlerInterface* encoder) :
        A(A),
        B(B),
        b1Inv{1.0f / B[1]},
        maxVel(std::min(maxVel, encoder->unitsPerRev * (1.0f / A(0, 1) / 2.0f * 0.8f))),
        posDepForceCompVec(posDepForceCompVec),
        posDepFrictionCompVec(posDepFrictionCompVec)
{
    auto isAllZero = [](decltype(posDepFrictionCompVec)& vec)
        {
            return std::all_of(vec.cbegin(), vec.cend(), [](int16_t i){return i == 0;});
        };

    if (isAllZero(posDepFrictionCompVec))
    {
        this->posDepFrictionCompVec.fill(static_cast<int16_t>(std::round(avarageFriction)));
    }
}

const Eigen::Matrix3f& DefaultControlConfiguration::getA()
{
    return A;
}

const Eigen::Vector3f& DefaultControlConfiguration::getB()
{
    return B;
}

void DefaultControlConfiguration::limitVelocity(float& vel)
{
    vel = std::min(maxVel, vel);
    vel = std::max(-maxVel, vel);
}

std::tuple<int32_t, bool> DefaultControlConfiguration::applyForceCompensations(
        int32_t u, uint16_t rawEncPos, float velRef, float vel)
{
    int32_t out = u;
    constexpr int32_t eps = 1;

    if (velRef > 0)
    {
        if (vel >= 0)
        {
            fricCompDir = 1;
        }
        else if (vel < 0)
        {
            fricCompDir = 0;
        }
    }
    else if (velRef < 0)
    {
        if (vel <= 0)
        {
            fricCompDir = -1;
        }
        else if (vel > 0)
        {
            fricCompDir = 0;
        }
    }
    else
    {
        fricCompDir = 0;
    }

    size_t i = (rawEncPos * vecSize) / 4096;

    out += posDepForceCompVec[i];
    out += posDepFrictionCompVec[i] * fricCompDir;

    bool brake = std::abs(vel) <= 1 | velRef == 0.0f;

    return std::make_tuple(out, brake);
}

float DefaultControlConfiguration::calculateFeedForward(float v1, float v0)
{
    return b1Inv * (v1 - A(1, 1) * v0);
}
