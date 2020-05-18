#include "OpticalEncoderHandler.h"

OpticalEncoderHandler::OpticalEncoderHandler(const std::array<uint16_t, vecSize>& aVec, const std::array<uint16_t, vecSize>& bVec) :
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
    sensor1Value = 0;
    sensor2Value = 0;

    sensor1.triggerSample();
    sensor2.triggerSample();

    const int n = 4;
    for (int i = 0; i != n - 1; ++i)
    {
        sensor1Value += sensor1.getValue();
        sensor1.triggerSample();
        sensor2Value += sensor2.getValue();
        sensor2.triggerSample();
    }

    sensor1Value += sensor1.getValue();
    sensor2Value += sensor2.getValue();

    sensor1Value /= n;
    sensor2Value /= n;

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


OpticalEncoderHandler::DiagnosticData OpticalEncoderHandler::getDiagnosticData()
{
    return diagnosticData;
}

void OpticalEncoderHandler::updatePosition()
{
    diagnosticData.a = sensor1Value;
    diagnosticData.b = sensor2Value;

    int stepSize = static_cast<int>(vecSize / 2.0 + 1);

    int i = predictNextPos;
    uint32_t cost = calcCost(i, diagnosticData.a, diagnosticData.b);

    int bestI = i;
    uint32_t bestCost = cost;

    i += stepSize;
    while (i < vecSize)
    {
        cost = calcCost(i, diagnosticData.a, diagnosticData.b);

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

        cost = calcCost(i, diagnosticData.a, diagnosticData.b);

        if (cost < bestCost)
        {
            bestI = i;
            bestCost = cost;

            checkDir *= -1;

            continue;
        }

        i -= 2 * stepSize * checkDir;

        cost = calcCost(i, diagnosticData.a, diagnosticData.b);

        if (cost < bestCost)
        {
            bestI = i;
            bestCost = cost;
        }
    }

    int lastMinCostIndexChange = bestI - lastMinCostIndex;
    lastMinCostIndex = bestI;
    predictNextPos = bestI + lastMinCostIndexChange;
    while (predictNextPos >= vecSize)
    {
        predictNextPos -= vecSize;
    }
    while (predictNextPos < 0)
    {
        predictNextPos += vecSize;
    }

    diagnosticData.minCostIndex = bestI;
    diagnosticData.minCost = bestCost;

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

    int predictDiff = i - predictNextPos;

    if (predictDiff > vecSize / 2)
    {
        predictDiff -= vecSize;
    }
    else if (predictDiff < -vecSize / 2)
    {
        predictDiff += vecSize;
    }

    uint32_t tempA;
    tempA = aVec[i] - a;
    tempA = tempA * tempA;

    uint32_t tempB;
    tempB = bVec[i] - b;
    tempB = tempB * tempB;

    return tempA + tempB + 20 * abs(predictDiff);
}
