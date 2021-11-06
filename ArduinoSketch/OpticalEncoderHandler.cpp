#include "OpticalEncoderHandler.h"

OpticalEncoderHandler::OpticalEncoderHandler(const std::array<uint16_t, vecSize>& aVec, const std::array<uint16_t, vecSize>& bVec,
        int16_t sensor1Pin, int16_t sensor2Pin, float unitsPerRev) :
    EncoderHandlerInterface(unitsPerRev),
    aVec(aVec), bVec(bVec),
    sensor1(sensor1Pin), sensor2(sensor2Pin),
    scaling(unitsPerRev * (1.0f / 4096.0f))
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

    const int n = 1;
    for (int i = 0; i != n; ++i)
    {
        if (synchronizer)
        {
            while (synchronizer->willSwitchWithIn(16))
            {
            }
        }
        sensor1.triggerSample();
        sensor1Value += sensor1.getValue();

        if (synchronizer)
        {
            while (synchronizer->willSwitchWithIn(16))
            {
            }
        }
        sensor2.triggerSample();
        sensor2Value += sensor2.getValue();
    }

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

    return (value + wrapAroundCorretion) * scaling;
}

uint16_t OpticalEncoderHandler::getUnscaledRawValue()
{
    return diagnosticData.c;
}

EncoderHandlerInterface::DiagnosticData OpticalEncoderHandler::getDiagnosticData()
{
    auto out = diagnosticData;
    diagnosticData.d = 0;
    return out;
}

void OpticalEncoderHandler::updatePosition()
{
    int16_t offset = sensorValueOffset;
    diagnosticData.a = sensor1Value;
    diagnosticData.b = sensor2Value;
    sensor1Value -= offset;
    sensor2Value -= offset;

    int stepSize = static_cast<int>(vecSize / 2.0f + 1);

    int i = predictNextPos;
    uint32_t cost = calcCost(i, sensor1Value, sensor2Value);

    int bestI = i;
    uint32_t bestCost = cost;

    i += stepSize;
    while (i < vecSize)
    {
        cost = calcCost(i, sensor1Value, sensor2Value);

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

        cost = calcCost(i, sensor1Value, sensor2Value);

        if (cost < bestCost)
        {
            bestI = i;
            bestCost = cost;

            checkDir *= -1;

            continue;
        }

        i -= 2 * stepSize * checkDir;

        cost = calcCost(i, sensor1Value, sensor2Value);

        if (cost < bestCost)
        {
            bestI = i;
            bestCost = cost;
        }
    }

    if (std::abs(bestI - iAtLastOffsetUpdate) > (vecSize / 10))
    {
        iAtLastOffsetUpdate = bestI;

        const uint16_t& a = aVec[bestI];
        const uint16_t& b = bVec[bestI];
        int16_t currentOffset = (diagnosticData.a - a + diagnosticData.b - b) / 2;

        sensorValueOffset = 0.95f * sensorValueOffset + 0.05f * currentOffset;
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

    diagnosticData.c = bestI;
    if (bestCost > diagnosticData.d)
    {
        diagnosticData.d = bestCost;
    }

    float newValue = bestI * (4096.0f / vecSize);

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
