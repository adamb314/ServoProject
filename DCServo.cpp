#include "DCServo.h"

DCServo* DCServo::getInstance()
{
    static DCServo dcServo;
    return &dcServo;
}

DCServo::DCServo() :
        controlEnabled(true),
        loopNumber(0),
        controlSignal(0),
        Ivel(0),
        currentControl(std::make_unique<CurrentControlLoop>(400)),
        encoderHandler(std::make_unique<EncoderHandler>()),
        kalmanFilter(std::make_unique<KalmanFilter>()),
        dotStarLed(1, 41, 40, DOTSTAR_BGR)

{
    L << 29.466422097397437, 2.1666222001611266, -0.07733338643510186;

    dotStarLed.begin();
    dotStarLed.show();

    encoderHandler->init();

#ifdef SIMULATE
    rawPos = 2048;
#else
    encoderHandler->triggerSample();
    rawPos = encoderHandler->getValue();
#endif

    x[0] = rawPos;
    x[1] = 0;
    x[2] = 0;

#ifdef SIMULATE
    xSim = x;
#endif

    kalmanFilter->reset(x);

    setReference(x[0], 0, 0);

    threads.push_back(new FunctionThread(1, 1200, 0,
        [&]()
        {
            controlLoop();
            loopNumber++;
        }));
}

void DCServo::enable(bool b)
{
    if (controlEnabled != b)
    {
        if (b)
        {
            dotStarLed.setPixelColor(0, 60, 50, 0);
        }
        else
        {
            dotStarLed.setPixelColor(0, 50, 0, 0);
        }
        dotStarLed.show();
    }

    ThreadInterruptBlocker blocker;
    controlEnabled = b;
}

void DCServo::setReference(int16_t pos, int16_t vel, int16_t feedForwardU)
{
    ThreadInterruptBlocker blocker;
    this->feedForwardU = feedForwardU;
    velRef = vel;
    posRef = pos;
}

int16_t DCServo::getPosition()
{
    ThreadInterruptBlocker blocker;
    return rawPos;
}

int16_t DCServo::getVelocity()
{
    ThreadInterruptBlocker blocker;
    return x[1];
}

int16_t DCServo::getControlSignal()
{
    ThreadInterruptBlocker blocker;
    return controlSignal;
}

uint16_t DCServo::getLoopNumber()
{
    ThreadInterruptBlocker blocker;
    return loopNumber;
}

void DCServo::controlLoop()
{
#ifdef SIMULATE
    xSim = kalmanFilter->getA() * xSim + kalmanFilter->getB() * controlSignal;
    rawPos =xSim[0];
#else
    encoderHandler->triggerSample();
    rawPos = encoderHandler->getValue();
#endif

    x = kalmanFilter->update(controlSignal, rawPos);

    if (controlEnabled)
    {
        int16_t posDiff = static_cast<int16_t>(
                        static_cast<uint16_t>(posRef * 16) -
                        static_cast<uint16_t>(x[0] * 16)) / 16;

        float vControlRef = L[0] * posDiff + velRef;

        float u = L[1] * (vControlRef - x[1]) + Ivel + feedForwardU;

        controlSignal = u;

        if (rawPos > 1410 + 800 || rawPos < 1410 - 800)
        {
            controlSignal = 0;
        }

        controlSignal = setOutput(controlSignal);

        Ivel -= L[2] * (vControlRef - x[1]);
        Ivel += 10 * L[2] * (u - controlSignal);
    }
    else
    {
        setReference(x[0], 0, 0);
        Ivel = 0;

        controlSignal = setOutput(0);
    }

}

int16_t DCServo::setOutput(int16_t u)
{
    if (u > 1023)
    {
        u = 1023;
    }
    else if (u < -1023)
    {
        u = -1023;
    }

    currentControl->setReference(u);
    
    return u;
}
