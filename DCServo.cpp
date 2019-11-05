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
        uLimitDiff(0),
        Ivel(0),
        currentControl(std::make_unique<CurrentControlLoop>(400)),
        motorEncoderHandler(std::make_unique<EncoderHandler>(A4)),
        outputEncoderHandler(std::make_unique<EncoderHandler>(A5)),
        kalmanFilter(std::make_unique<KalmanFilter>()),
        dotStarLed(1, 41, 40, DOTSTAR_BGR),
        dotstarState(1),
        dotstarStateRequest(1),
        identTestState(NORMAL_CONTROL),
        identTestArrayIndex(0),
        identTestAmplitude(0),
        pwmOutputOnDisabled(0)

{
    //L << 9.940239281724569, 1.3586010780478561, -0.03237764040441623, -0.03237764040441623 * 10;
    //L << 14.865806368082696, 2.0623236695442064, -0.07122297702645312, -0.07122297702645312 * 10;
    //L << 19.76190853507559, 2.7501424347363677, -0.12380201903044662, -0.12380201903044662 * 10;
    //L << 24.628722042909875, 3.422417759025543, -0.18915403084733035, -0.18915403084733035 * 10;
    //L << 57.89092015732856, 7.721727677879117, -0.9336154818877859, -0.9336154818877859 * 10;
    L << 94.23296940236878, 11.862863259936727, -2.185085156962166, -2.185085156962166 * 10;

    dotStarLed.begin();
    dotStarLed.show();

    motorEncoderHandler->init();
    outputEncoderHandler->init();

#ifdef SIMULATE
    rawMotorPos = 2048;
    rawOutputPos = rawMotorPos;
#else
    motorEncoderHandler->triggerSample();
    outputEncoderHandler->triggerSample();
    rawMotorPos = motorEncoderHandler->getValue() * (561.0 / 189504.0);
    rawOutputPos = outputEncoderHandler->getValue();
#endif

    outputPosOffset = rawOutputPos - rawMotorPos;

    x[0] = rawMotorPos;
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

int16_t DCServo::getMotorPosition()
{
    ThreadInterruptBlocker blocker;
    return rawMotorPos * (189504.0 / 561.0);
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
    rawMotorPos =xSim[0];
    rawOutputPos = rawMotorPos;
#else
    motorEncoderHandler->triggerSample();
    outputEncoderHandler->triggerSample();
    rawMotorPos = motorEncoderHandler->getValue() * (561.0 / 189504.0);
    rawOutputPos = outputEncoderHandler->getValue();
#endif

    x = kalmanFilter->update(controlSignal, rawMotorPos);

    if (controlEnabled)
    {
        uLimitDiff = 0.99 * uLimitDiff + 0.01 * (controlSignal - currentControl->getLimitedCurrent());

        Ivel += L[3] * uLimitDiff;

        float posRef;
        float velRef;
        float feedForwardU;

        refInterpolator.get(posRef, velRef, feedForwardU);

        outputPosOffset -= 12 * 0.0012 * (posRef - rawOutputPos);

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
        outputPosOffset = rawOutputPos - rawMotorPos;
        controlSignal = 0;
        currentControl->overidePwmDuty(pwmOutputOnDisabled);
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
