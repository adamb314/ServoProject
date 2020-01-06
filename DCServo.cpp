#include "DCServo.h"

DCServo* DCServo::getInstance()
{
    static DCServo dcServo;
    return &dcServo;
}

DCServo::DCServo() :
        controlEnabled(false),
        onlyUseMainEncoderControl(false),
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
            if (controlEnabled)
            {
                if (openLoopControlMode)
                {
                    statusLight.showOpenLoop();
                }
                else
                {
                    statusLight.showEnabled();
                }
            }
            else
            {
                statusLight.showDisabled();
            }
            controlLoop();
            loopNumber++;
        }));
}

void DCServo::enable(bool b)
{
    ThreadInterruptBlocker blocker;
    controlEnabled = b;
}

void DCServo::openLoopMode(bool b)
{
    ThreadInterruptBlocker blocker;
    openLoopControlMode = b;
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

int16_t DCServo::getMainEncoderPosition()
{
    ThreadInterruptBlocker blocker;
    return rawMainPos + initialOutputPosOffset;
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
            uLimitDiff = 0.99 * uLimitDiff + 0.01 * (controlSignal - currentControl->getLimitedCurrent());

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
            setReference(x[0], 0, 0);
            Ivel = 0;
            uLimitDiff = 0;
            outputPosOffset = rawOutputPos - rawMainPos;

            float posRef;
            float velRef;
            float feedForwardU;

            refInterpolator.get(posRef, velRef, feedForwardU);

            controlSignal = feedForwardU;

            setOutput(controlSignal);
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
        currentControl->overidePwmDuty(0);
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
    
    return currentControl->getLimitedCurrent();
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

DCServo::StatusLightHandler::StatusLightHandler() :
        dotStarLed(1, 41, 40, DOTSTAR_BGR),
        dotstarState(1),
        dotstarStateRequest(1)
{
    dotStarLed.begin();
    dotStarLed.show();

    threads.push_back(createThread(0, 150000, 0,
        [&]()
        {
            static int colorPulse = 0;
            bool changedColor = false;

            switch (dotstarState)
            {
                case 1:
                    changedColor = true;
                    if (colorPulse == 0)
                    {
                        dotStarLed.setPixelColor(0, 50, 30, 30);
                    }
                    else
                    {
                        dotStarLed.setPixelColor(0, 50, 0, 0);
                    }
                    dotstarState = 10;

                //|||||||||||||||||||||||||||||
                //VVVVVVVVVVVVVVVVVVVVVVVVVVVVV
                //INTENTIONAL FALL THROUGH HERE
                case 10:
                    colorPulse++;
                    if (colorPulse >= 5)
                    {
                        colorPulse = 0;
                    }

                    if (colorPulse == 0 || colorPulse == 1)
                    {
                        dotstarState = 1;
                    }

                    if (dotstarStateRequest != 1)
                    {
                        dotstarState = dotstarStateRequest;
                    }
                    break;

                case 2:
                    changedColor = true;
                    if (colorPulse == 0)
                    {
                        dotStarLed.setPixelColor(0, 60, 50, 60);
                    }
                    else
                    {
                        dotStarLed.setPixelColor(0, 60, 50, 0);
                    }
                    dotstarState = 20;

                //|||||||||||||||||||||||||||||
                //VVVVVVVVVVVVVVVVVVVVVVVVVVVVV
                //INTENTIONAL FALL THROUGH HERE
                case 20:
                    colorPulse++;
                    if (colorPulse >= 5)
                    {
                        colorPulse = 0;
                    }

                    if (colorPulse == 0 || colorPulse == 1)
                    {
                        dotstarState = 2;
                    }

                    if (dotstarStateRequest != 2)
                    {
                        dotstarState = dotstarStateRequest;
                    }
                    break;

                case 3:
                    changedColor = true;
                    if (colorPulse == 0)
                    {
                        dotStarLed.setPixelColor(0, 30, 30, 50);
                    }
                    else
                    {
                        dotStarLed.setPixelColor(0, 0, 0, 50);
                    }
                    dotstarState = 30;

                //|||||||||||||||||||||||||||||
                //VVVVVVVVVVVVVVVVVVVVVVVVVVVVV
                //INTENTIONAL FALL THROUGH HERE
                case 30:
                    colorPulse++;
                    if (colorPulse >= 5)
                    {
                        colorPulse = 0;
                    }

                    if (colorPulse == 0 || colorPulse == 1)
                    {
                        dotstarState = 3;
                    }

                    if (dotstarStateRequest != 3)
                    {
                        dotstarState = dotstarStateRequest;
                    }
                    break;

                default:
                    dotstarState = dotstarStateRequest;
                    break;
            }

            if (changedColor)
            {
                dotStarLed.show();
            }
        }));
}

void DCServo::StatusLightHandler::showDisabled()
{
    dotstarStateRequest = 1;
}

void DCServo::StatusLightHandler::showEnabled()
{
    dotstarStateRequest = 2;
}

void DCServo::StatusLightHandler::showOpenLoop()
{
    dotstarStateRequest = 3;
}
