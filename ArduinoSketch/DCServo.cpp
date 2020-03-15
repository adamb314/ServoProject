#include "DCServo.h"

DCServo* DCServo::getInstance()
{
    static DCServo dcServo;
    return &dcServo;
}

DCServo::DCServo() :
        controlEnabled(false),
        onlyUseMainEncoderControl(false),
        openLoopControlMode(false),
        pwmOpenLoopMode(false),
        loopNumber(0),
        current(0),
        controlSignal(0),
        uLimitDiff(0),
        Ivel(0),
        mainEncoderHandler(ConfigHolder::createMainEncoderHandler()),
        outputEncoderHandler(ConfigHolder::createOutputEncoderHandler()),
        kalmanFilter(std::make_unique<KalmanFilter>())
{
    threads.push_back(createThread(2, 1200, 0,
        [&]()
        {
            mainEncoderHandler->triggerSample();
            if (outputEncoderHandler)
            {
                outputEncoderHandler->triggerSample();
            }
        }));

    currentControl = std::make_unique<CurrentControlLoop>(400);

    L = ConfigHolder::getControlParameterVector();

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

    rawMainPos = mainEncoderHandler->getValue() * ConfigHolder::getMainEncoderGearRation();
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

    setReference(x[0], 0, 0);

    threads.push_back(createThread(1, 1200, 0,
        [&]()
        {
            controlLoop();
            loopNumber++;
        }));
}

void DCServo::enable(bool b)
{
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

void DCServo::setReference(float pos, int16_t vel, int16_t feedForwardU)
{
    ThreadInterruptBlocker blocker;
    refInterpolator.set(pos, vel, feedForwardU);
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

uint16_t DCServo::getLoopNumber()
{
    ThreadInterruptBlocker blocker;
    return loopNumber;
}

float DCServo::getMainEncoderPosition()
{
    ThreadInterruptBlocker blocker;
    return rawMainPos + initialOutputPosOffset;
}

template <class T>
OpticalEncoderHandler::DiagnosticData getMainEncoderRawDiagnosticDataDispatch(T& encoder)
{
    OpticalEncoderHandler::DiagnosticData out = {0};
    return out;
}

template <>
OpticalEncoderHandler::DiagnosticData getMainEncoderRawDiagnosticDataDispatch(std::unique_ptr<OpticalEncoderHandler>& encoder)
{
    return encoder->getDiagnosticData();
}

template <>
OpticalEncoderHandler::DiagnosticData DCServo::getMainEncoderDiagnosticData()
{
    ThreadInterruptBlocker blocker;
    return getMainEncoderRawDiagnosticDataDispatch(mainEncoderHandler);
}

void DCServo::controlLoop()
{
#ifdef SIMULATE
    xSim = kalmanFilter->getA() * xSim + kalmanFilter->getB() * controlSignal;
    rawMainPos =xSim[0];
    rawOutputPos = rawMainPos;
#else
    rawMainPos = mainEncoderHandler->getValue() * ConfigHolder::getMainEncoderGearRation();
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
            uLimitDiff = 0.99 * uLimitDiff + 0.01 * (controlSignal - currentControl->getLimitedRef());

            Ivel += L[3] * uLimitDiff;

            float posRef;
            float velRef;
            float feedForwardU;

            refInterpolator.get(posRef, velRef, feedForwardU);

            if (!onlyUseMainEncoderControl)
            {
                outputPosOffset -= L[4] * 0.0012 * (posRef - rawOutputPos);
            }

            posRef -= outputPosOffset;

            float posDiff = posRef - x[0];

            float vControlRef = L[0] * posDiff + velRef;

            float u = L[1] * (vControlRef - x[1]) + Ivel + feedForwardU;

            controlSignal = u;

            setOutput(controlSignal);
            current = currentControl->getCurrent();

            Ivel -= L[2] * (vControlRef - x[1]);
        }
        else
        {
            Ivel = 0;
            uLimitDiff = 0;
            outputPosOffset = rawOutputPos - rawMainPos;

            float posRef;
            float velRef;
            float feedForwardU;

            refInterpolator.get(posRef, velRef, feedForwardU);

            if (pwmOpenLoopMode)
            {
                controlSignal = 0;
                currentControl->overidePwmDuty(feedForwardU);
            }
            else
            {
                controlSignal = feedForwardU;
                setOutput(controlSignal);
            }
            current = currentControl->getCurrent();
        }
    }
    else
    {
        setReference(x[0], 0, 0);
        Ivel = 0;
        uLimitDiff = 0;
        outputPosOffset = rawOutputPos - rawMainPos;
        controlSignal = 0;
        currentControl->activateBrake();
        current = currentControl->getCurrent();
    }

}

int16_t DCServo::setOutput(float u)
{
    if (u > 0x7fff)
    {
        u = 0x7fff;
    }
    else if (u < -0x7fff)
    {
        u = -0x7fff;
    }
    currentControl->setReference(u);
    
    return currentControl->getLimitedRef();
}

ReferenceInterpolator::ReferenceInterpolator()
{
    set(0, 0, 0);
    set(0, 0, 0);
}

void ReferenceInterpolator::set(float position, float velocity, float feedForward)
{
    time[0] = time[1];
    time[1] = millis();

    pos[0] = pos[1];
    vel[0] = vel[1];
    feed[0] = feed[1];

    pos[1] = position;
    vel[1] = velocity;
    feed[1] = feedForward;
}

void ReferenceInterpolator::get(float& position, float& velocity, float& feedForward)
{
    uint16_t current = millis();

    uint16_t diff0 = current - time[0];
    uint16_t diff1 = time[1] - time[0];

    if (diff1 == 0)
    {
        position = pos[1];
        velocity = vel[1];
        feedForward = feed[1];

        return;
    }

    if (diff1 > 100 || diff0 > 100)
    {
        position = pos[1];
        velocity = vel[1];
        feedForward = feed[1];

        return;
    }

    float t = diff0 / diff1;

    if (t < 0)
    {
        t = 0;
    }
    else if (t > 1)
    {
        t = 1;
    }

    position = pos[0] + t * (pos[1] - pos[0]);
    velocity = vel[0] + t * (vel[1] - vel[0]);
    feedForward = feed[0] + t * (feed[1] - feed[0]);
}
