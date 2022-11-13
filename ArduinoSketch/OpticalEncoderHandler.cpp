#include "OpticalEncoderHandler.h"

OpticalEncoderHandler::OpticalEncoderHandler(const std::array<uint16_t, vecSize>& aVec, const std::array<uint16_t, vecSize>& bVec,
        int16_t sensor1Pin, int16_t sensor2Pin, float unitsPerRev) :
    EncoderHandlerInterface(unitsPerRev),
    aVec(aVec), bVec(bVec),
    sensor1(sensor1Pin, ADC_CTRLB_PRESCALER_DIV16_Val), sensor2(sensor2Pin, ADC_CTRLB_PRESCALER_DIV16_Val),
    scaling(unitsPerRev * (1.0f / 4096.0f))
{
}

OpticalEncoderHandler::OpticalEncoderHandler(const std::array<uint16_t, 512>& aVec, const std::array<uint16_t, 512>& bVec,
        int16_t sensor1Pin, int16_t sensor2Pin, float unitsPerRev) :
    EncoderHandlerInterface(unitsPerRev),
    sensor1(sensor1Pin, ADC_CTRLB_PRESCALER_DIV16_Val), sensor2(sensor2Pin, ADC_CTRLB_PRESCALER_DIV16_Val),
    scaling(unitsPerRev * (1.0f / 4096.0f))
{
    constexpr size_t s = static_cast<int>(vecSize / 512);
    for (size_t i = 0; i != 512; ++i)
    {
        for (size_t j = 0; j != s; ++j)
        {
            this->aVec[s * i + j] = aVec[i];
            this->bVec[s * i + j] = bVec[i];
        }
    }
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
    sensor1.triggerSample(ADC_AVGCTRL_SAMPLENUM_16_Val);
    sensor2.triggerSample(ADC_AVGCTRL_SAMPLENUM_16_Val);

    newData = true;
}

float OpticalEncoderHandler::getValue()
{
    if (newData)
    {
        sensor1Value = sensor1.getValue() / 4;
        sensor2Value = sensor2.getValue() / 4;

        newData = false;
        updatePosition();
    }

    return (value + wrapAroundCorretion) * scaling;
}

uint16_t OpticalEncoderHandler::getUnscaledRawValue()
{
    return diagnosticData.c * 2;
}

EncoderHandlerInterface::DiagnosticData OpticalEncoderHandler::getDiagnosticData()
{
    auto out = diagnosticData;
    diagnosticData.d = 0;
    return out;
}

void OpticalEncoderHandler::updatePosition()
{
    int16_t offset = sensorValueOffset / 16;
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

        sensorValueOffset = (15 * sensorValueOffset) / 16 + currentOffset;
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
    bestCost = bestCost / 16;
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
