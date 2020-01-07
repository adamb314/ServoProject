#include "OpticalEncoderHandler.h"

uint16_t debugValue = 0;

uint16_t OpticalEncoderHandler::getDebugValue()
{
    return debugValue;
}

OpticalEncoderHandler::OpticalEncoderHandler(const std::array<uint16_t, 512>& aVec, const std::array<uint16_t, 512>& bVec) :
    aVec(aVec), bVec(bVec), sensor1(A2), sensor2(A3), value(0), wrapAroundCorretion(0), newData(false)
{
}

OpticalEncoderHandler::~OpticalEncoderHandler()
{
}

void OpticalEncoderHandler::init()
{
    AdcHandler::init();

    triggerSample();
    getValue();
}

void OpticalEncoderHandler::triggerSample()
{
    sensor1.triggerSample();
    sensor2.triggerSample();

    newData = true;
}

float OpticalEncoderHandler::getValue()
{
    if (newData)
    {
        newData = false;
        updatePosition();
    }

    return (value + wrapAroundCorretion);
}

void OpticalEncoderHandler::updatePosition()
{
    uint16_t start = micros();

    uint16_t a = sensor1.getValue();
    uint16_t b = sensor2.getValue();

    int stepSize = static_cast<int>(vecSize / 2.0 + 1);

    int i = 0;
    uint32_t cost = calcCost(i, a, b);

    int bestI = i;
    uint32_t bestCost = cost;

    i += stepSize;
    while (i < vecSize)
    {
        cost = calcCost(i, a, b);

        if (cost < bestCost)
        {
            bestI = i;
            bestCost = cost;
        }

        i += stepSize;
    }

    int checkDir = 1;

    while (stepSize != 1)
    {
        i = bestI;

        stepSize = stepSize / 2;
        if (stepSize == 0)
        {
            stepSize = 1;
        }

        i += stepSize * checkDir;

        cost = calcCost(i, a, b);

        if (cost < bestCost)
        {
            bestI = i;
            bestCost = cost;

            checkDir *= -1;

            continue;
        }

        i -= 2 * stepSize * checkDir;

        cost = calcCost(i, a, b);

        if (cost < bestCost)
        {
            bestI = i;
            bestCost = cost;
        }
    }

    debugValue = micros() - start;

    float newValue = bestI * (4096.0 / vecSize);

    if (newValue - value > 4096 / 2)
    {
        wrapAroundCorretion -= 4096;
    }
    else if (newValue - value < -4096 / 2)
    {
        wrapAroundCorretion += 4096;
    }

    value = newValue;
}

uint32_t OpticalEncoderHandler::calcCost(int& i, uint16_t a, uint16_t b)
{
    if (i >= vecSize)
    {
        i -= vecSize;
    }
    else if (i < 0)
    {
        i += vecSize;
    }

    uint32_t tempA;
    tempA = aVec[i] - a;
    tempA = tempA * tempA;

    uint32_t tempB;
    tempB = bVec[i] - b;
    tempB = tempB * tempB;

    return tempA + tempB;
}
