#include "CurrentControlLoop.h"

CurrentControlLoop::CurrentControlLoop(uint32_t period) :
    pwmInstance(HBridge4WirePwm::getInstance()),
    filteredADC(new FilteredADC(0.9)),
    integral(0),
    ref(0),
    y(0),
    disableLoop(true)
{
    //L << 100, -100;
    //L << 120.795168, -21382.77055587;
    //L << 98.60793374, -14743.51725738 * 0.001, 0.5 * -14743.51725738 * 0.001;
    //L << 61.85140245, -6452.74163624 * 0.001, 0.5 * -6452.74163624 * 0.001;
    //L = L / 16;
    L << 4.43039669, -0.73717586287, 0.5 * -0.73717586287;

    filteredADC->configureAdcPin(A1);
    filteredADC->initOffset(A1);

    threads.push_back(new FunctionThread(2, period, 0,
        [&]()
        {
            filteredADC->triggerSample();
        }));

    threads.push_back(new FunctionThread(2, period, 200,
        [&]()
        {
            this->run();
        }));
}

CurrentControlLoop::~CurrentControlLoop()
{
}

void CurrentControlLoop::setReference(float ref)
{
    ThreadInterruptBlocker interruptBlocker;
    disableLoop = false;
    this->ref = ref;
}

void CurrentControlLoop::overidePwmDuty(int16_t pwm)
{
    ThreadInterruptBlocker interruptBlocker;
    disableLoop = true;
    pwmInstance->setOutput(pwm);
}

float CurrentControlLoop::getCurrent()
{
    ThreadInterruptBlocker interruptBlocker;
    return y;
}

void CurrentControlLoop::run()
{
    filteredADC->collectSample();
    y = filteredADC->getValue();

    if (disableLoop)
    {
        integral = 0;
        return;
    }

    float controlError;
    {
        ThreadInterruptBlocker interruptBlocker;
        controlError = ref - y;
    }


    float u = L[0] * controlError + integral;

    float limitedU = pwmInstance->setOutput(u);

    integral -= L[1] * controlError;

    integral += L[2] * (u - limitedU);
}
