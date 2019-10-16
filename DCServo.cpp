#include "DCServo.h"

DCServo* DCServo::getInstance()
{
    static DCServo dcServo;
    return &dcServo;
}

DCServo::DCServo() :
        controlEnabled(false),
        loopNumber(0),
        current(0),
        controlSignal(0),
        Ivel(0),
        currentControl(std::make_unique<CurrentControlLoop>(400)),
        encoderHandler(std::make_unique<EncoderHandler>()),
        kalmanFilter(std::make_unique<KalmanFilter>()),
        dotStarLed(1, 41, 40, DOTSTAR_BGR),
        dotstarState(1),
        dotstarStateRequest(1),
        identTestState(NORMAL_CONTROL),
        identTestArrayIndex(0),
        identTestAmplitude(0),
        pwmOutputOnDisabled(0)

{
    L << 29.466422097397437, 2.150282330880081, -0.07733338643510186, -0.07733338643510186 * 10;

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

    threads.push_back(createThread(1, 1200, 0,
        [&]()
        {
            if (identTestState == NORMAL_CONTROL)
            {
                dotstarStateRequest = 1;
                if (controlEnabled)
                {
                    dotstarStateRequest = 2;
                }
                pwmOutputOnDisabled = 0;
            }
            else
            {
                dotstarStateRequest = 3;
                identTestLoop();
            }
            controlLoop();
            loopNumber++;
        }));

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

void DCServo::enable(bool b)
{
    ThreadInterruptBlocker blocker;
    controlEnabled = b;
}

void DCServo::setReference(float pos, int16_t vel, int16_t feedForwardU)
{
    ThreadInterruptBlocker blocker;
    refInterpolator.set(pos, vel, feedForwardU);
}

float DCServo::getPosition()
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
    if (controlEnabled)
    {
        return controlSignal;
    }
    return pwmOutputOnDisabled;
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

bool DCServo::runIdentTest1(int16_t amplitude)
{
    ThreadInterruptBlocker blocker;
    if (identTestState == IDENT_TEST_1_COMP)
    {
        identTestState = NORMAL_CONTROL;
        return true;
    }

    if (identTestState != IDENT_TEST_1)
    {
        identTestState = IDENT_TEST_1_INIT;
        identTestAmplitude = amplitude;
    }
    return false;
}

bool DCServo::runIdentTest2(int16_t amplitude)
{
    ThreadInterruptBlocker blocker;
    if (identTestState == IDENT_TEST_2_COMP)
    {
        identTestState = NORMAL_CONTROL;
        return true;
    }

    if (identTestState != IDENT_TEST_2)
    {
        identTestState = IDENT_TEST_2_INIT;
        identTestAmplitude = amplitude;
    }
    return false;
}

void DCServo::identTestLoop()
{
    controlEnabled = false;
    size_t i;
    switch (identTestState)
    {
        case IDENT_TEST_1_INIT:
            identTestArrayIndex = 0;
            identTestState = IDENT_TEST_1;
        //|||||||||||||||||||||||||||||
        //VVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        //INTENTIONAL FALL THROUGH HERE
        case IDENT_TEST_1:
            i = (identTestArrayIndex >> 4);
            if (i < sizeof(testOutputArray) / sizeof(testOutputArray[0]))
            {
                pwmOutputOnDisabled = identTestAmplitude * testOutputArray[i];
                identTestArrayIndex++;
                break;
            }

            identTestState = IDENT_TEST_1_COMP;
        //|||||||||||||||||||||||||||||
        //VVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        //INTENTIONAL FALL THROUGH HERE
        case IDENT_TEST_1_COMP:
            pwmOutputOnDisabled = 0;
            break;

        case IDENT_TEST_2_INIT:
            identTestArrayIndex = 0;
            identTestState = IDENT_TEST_2;
        //|||||||||||||||||||||||||||||
        //VVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        //INTENTIONAL FALL THROUGH HERE
        case IDENT_TEST_2:
            i = (identTestArrayIndex >> 4);
            if (i < sizeof(testOutputArray2) / sizeof(testOutputArray2[0]))
            {
                pwmOutputOnDisabled = identTestAmplitude * testOutputArray2[i];
                identTestArrayIndex++;
                break;
            }

            identTestState = IDENT_TEST_2_COMP;
        //|||||||||||||||||||||||||||||
        //VVVVVVVVVVVVVVVVVVVVVVVVVVVVV
        //INTENTIONAL FALL THROUGH HERE
        case IDENT_TEST_2_COMP:
            pwmOutputOnDisabled = 0;
            break;

        default:
            pwmOutputOnDisabled = 0;
            break;
    }
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
        float posRef;
        float velRef;
        float feedForwardU;

        refInterpolator.get(posRef, velRef, feedForwardU);


        float posDiff = posRef - x[0];

        float vControlRef = L[0] * posDiff + velRef;

        float u = L[1] * (vControlRef - x[1]) + Ivel + feedForwardU;

        controlSignal = u;

        controlSignal = setOutput(controlSignal);
        current = currentControl->getCurrent();

        Ivel -= L[2] * (vControlRef - x[1]);
        Ivel += L[3] * (u - controlSignal);
    }
    else
    {
        setReference(x[0], 0, 0);
        Ivel = 0;

        controlSignal = 0;
        currentControl->overidePwmDuty(pwmOutputOnDisabled);
        current = currentControl->getCurrent();
    }

}

int16_t DCServo::setOutput(int16_t u)
{
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

    position = pos[0] + t * (pos[1] - pos[0]);
    velocity = vel[0] + t * (vel[1] - vel[0]);
    feedForward = feed[0] + t * (feed[1] - feed[0]);
}
