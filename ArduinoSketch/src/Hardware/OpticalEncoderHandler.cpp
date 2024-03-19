#include "OpticalEncoderHandler.h"

OpticalEncoderHandler::OpticalEncoderHandler(const std::array<uint16_t, vecSize>& aVec, const std::array<uint16_t, vecSize>& bVec,
        int16_t sensor1Pin, int16_t sensor2Pin, float unitsPerRev) :
    EncoderHandlerInterface(unitsPerRev),
    aVec(aVec), bVec(bVec),
    sensor1(sensor1Pin, ADC_CTRLB_PRESCALER_DIV16_Val), sensor2(sensor2Pin, ADC_CTRLB_PRESCALER_DIV16_Val),
    sensor1Sec(sensor1Pin, ADC_CTRLB_PRESCALER_DIV16_Val), sensor2Sec(sensor2Pin, ADC_CTRLB_PRESCALER_DIV16_Val),
    scaling(unitsPerRev * (1.0f / vecSize))
{
    sensor1Scaler.init(this->aVec);
    sensor2Scaler.init(this->bVec);
}

OpticalEncoderHandler::OpticalEncoderHandler(const std::array<uint16_t, 512>& aVec, const std::array<uint16_t, 512>& bVec,
        int16_t sensor1Pin, int16_t sensor2Pin, float unitsPerRev) :
    EncoderHandlerInterface(unitsPerRev),
    sensor1(sensor1Pin, ADC_CTRLB_PRESCALER_DIV16_Val), sensor2(sensor2Pin, ADC_CTRLB_PRESCALER_DIV16_Val),
    sensor1Sec(sensor1Pin, ADC_CTRLB_PRESCALER_DIV16_Val), sensor2Sec(sensor2Pin, ADC_CTRLB_PRESCALER_DIV16_Val),
    scaling(unitsPerRev * (1.0f / vecSize))
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

    sensor1Scaler.init(this->aVec);
    sensor2Scaler.init(this->bVec);
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
    if (sampleSensor1First)
    {
        sensor1.triggerSample(ADC_AVGCTRL_SAMPLENUM_8_Val);
        sensor2.triggerSample(ADC_AVGCTRL_SAMPLENUM_16_Val);
        sensor1Sec.triggerSample(ADC_AVGCTRL_SAMPLENUM_8_Val);
    }
    else
    {
        sensor2.triggerSample(ADC_AVGCTRL_SAMPLENUM_8_Val);
        sensor1.triggerSample(ADC_AVGCTRL_SAMPLENUM_16_Val);
        sensor2Sec.triggerSample(ADC_AVGCTRL_SAMPLENUM_8_Val);
    }

    newData = true;
}

float OpticalEncoderHandler::getValue()
{
    if (newData)
    {
        newData = false;

        uint16_t sensor1Value;
        uint16_t sensor2Value;

        if (sampleSensor1First)
        {
            sensor1Value = sensor1.getValue();
            sensor2Value = sensor2.getValue();
            sensor1Value += sensor1Sec.getValue();
        }
        else
        {
            sensor2Value = sensor2.getValue();
            sensor1Value = sensor1.getValue();
            sensor2Value += sensor2Sec.getValue();
        }
        sampleSensor1First = !sampleSensor1First;

        sensor1Value /= 4;
        sensor2Value /= 4;

        value = getValue(sensor1Value, sensor2Value);
    }

    return value;
}

float OpticalEncoderHandler::getValue(uint16_t sensor1Value, uint16_t sensor2Value)
{
    diagnosticData.a = sensor1Value;
    diagnosticData.b = sensor2Value;
    sensor1Value = sensor1Scaler.get(sensor1Value);
    sensor2Value = sensor2Scaler.get(sensor2Value);

    int32_t newPos;
    uint32_t cost;
    std::tie(newPos, cost) = calcPosition(sensor1Value, sensor2Value);

    sensor1Scaler.update(newPos, diagnosticData.a);
    sensor2Scaler.update(newPos, diagnosticData.b);

    diagnosticData.c = newPos;
    cost = cost / 16;
    if (cost > diagnosticData.d)
    {
        diagnosticData.d = cost;
    }

    if (newPos - pos > vecSize / 2)
    {
        wrapAroundCorretion -= vecSize;
    }
    else if (newPos - pos < -vecSize / 2)
    {
        wrapAroundCorretion += vecSize;
    }

    pos = newPos;

    return (pos + wrapAroundCorretion) * scaling;
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

void OpticalEncoderHandler::configurePullUpDown(int16_t portGroup, int16_t n, bool pullHigh)
{
    PORT->Group[portGroup].PINCFG[n].bit.PULLEN = 1;
    if (pullHigh)
    {
        PORT->Group[portGroup].OUTSET.reg = (1ul << n);
    }
    else
    {
        PORT->Group[portGroup].OUTCLR.reg = (1ul << n);
    }
}

std::tuple<int32_t, uint32_t> OpticalEncoderHandler::calcPosition(
        uint16_t sensor1Value, uint16_t sensor2Value)
{
    using namespace adam_std;

    auto calcCost = [&](int i, uint16_t a, uint16_t b)
        {
            uint32_t tempA;
            tempA = aVec[i] - a;
            tempA = tempA * tempA;

            uint32_t tempB;
            tempB = bVec[i] - b;
            tempB = tempB * tempB;

            return tempA + tempB;
        };

    int stepSize = vecSize / 5;

    int bestI = 0;
    uint32_t bestCost = calcCost(bestI, sensor1Value, sensor2Value);

    int i = bestI + stepSize;
    uint32_t cost;
    while (i < vecSize)
    {
        cost = calcCost(wrapAround<vecSize>((int)i), sensor1Value, sensor2Value);

        bool temp = cost < bestCost;
        bestCost = chooseOne(cost, bestCost, temp);
        bestI = chooseOne(i, bestI, temp);

        i = i + stepSize;
    }

    bestI = wrapAround<vecSize>(bestI);

    while (stepSize != 1)
    {
        stepSize = (stepSize + 1) / 2;

        int iPos = wrapAround<vecSize>(bestI + stepSize);
        int costPos = calcCost(iPos, sensor1Value, sensor2Value);

        int iNeg = wrapAround<vecSize>(bestI - stepSize);
        int costNeg = calcCost(iNeg, sensor1Value, sensor2Value);

        bool temp = costPos < costNeg;
        iPos = chooseOne(iPos, iNeg, temp);
        costPos = chooseOne(costPos, costNeg, temp);

        temp = costPos < bestCost;
        bestCost = chooseOne(costPos, bestCost, temp);
        bestI = chooseOne(iPos, bestI, temp);
    }

    return std::make_tuple(bestI, bestCost);
}

ValueScaler::ValueScaler(int32_t minOut, int32_t maxOut, int32_t minIn, int32_t maxIn)
{
    init(minOut, maxOut, minIn, maxIn);
}

void ValueScaler::init(int32_t minOut, int32_t maxOut, int32_t minIn, int32_t maxIn)
{
    this->minOut = minOut;
    this->maxOut = maxOut;
    int32_t outMinMaxDiff = maxOut- minOut;
    initInMinMaxDiff = maxIn - minIn;
    outOverInitInDiff16FixPoint = (outMinMaxDiff * one16FixPoint) / initInMinMaxDiff;
    oneOverInitInDiff30FixPoint = (one14FixPoint * one16FixPoint) / initInMinMaxDiff;
    approxInputRangeUpdate(minIn, maxIn);
}

void ValueScaler::approxInputRangeUpdate(int32_t min, int32_t max)
{
    minIn = min;
    maxIn = max;
    int32_t valueMinMaxDiff = maxIn - minIn;
    initInAndInDiffDiff = valueMinMaxDiff - initInMinMaxDiff;
}

int32_t ValueScaler::getOutput(int32_t input)
{
    //int32_t outMinMaxDiff = maxOut- minOut;
    //int32_t valueMinMaxDiff = maxIn - minIn;
    //return (outMinMaxDiff * (input - minIn)) / valueMinMaxDiff + minOut;
    // out = (outMinMaxDiff / valueMinMaxDiff) * (input - minIn) + minOut = 
    //     = outOverInitInDiff * (initInMinMaxDiff / valueMinMaxDiff) * (input - minIn) + minOut = 
    //     = outOverInitInDiff * (initInMinMaxDiff / (initInMinMaxDiff + initInAndInDiffDiff)) * (input - minIn) + minOut
    // taylor linearization of div =>
    // (1 - (1/initInMinMaxDiff) * initInAndInDiffDiff) * (input - minIn)) + minOut
    return ((((one16FixPoint - (oneOverInitInDiff30FixPoint * initInAndInDiffDiff) / one14FixPoint)
            * (input - maxIn)) / one16FixPoint) * outOverInitInDiff16FixPoint) / one16FixPoint + maxOut;
}

void OpticalEncoderHandler::OpticalSensorValueScaler::init(const std::array<uint16_t, vecSize>& refVec)
{
    using namespace adam_std;

    pointerToRefVec = &refVec;
    minRef = (1<<16);
    maxRef = 0;
    for (int i = 0; i != vecSize; ++i)
    {
        const auto& v = refVec[i];
        if (v <= minRef)
        {
            minRef = v;
            iAtMinRef = i;
        }
        if (v >= maxRef)
        {
            maxRef = v;
            iAtMaxRef = i;
        }
    }

    minVal = minRef;
    maxVal = maxRef;

    valueScaler.init(minRef, maxRef, minVal, maxVal);
    diffAtMinRefAverager.reset(minVal);
    diffAtMaxRefAverager.reset(maxVal);

    int32_t minSum = 0;
    int32_t minSumNr = 0;
    int32_t maxSum = 0;
    int32_t maxSumNr = 0;
    for (int i = 0; i != vecSize; ++i)
    {
        const auto& v = refVec[i];
        if (std::abs(wrapAroundDist<vecSize>(i - iAtMinRef)) < indexDelta)
        {
            minSum += minRef - v;
            minSumNr += 1;
        }
        else if (std::abs(wrapAroundDist<vecSize>(i - iAtMaxRef)) < indexDelta)
        {
            maxSum += maxRef - v;
            maxSumNr += 1;
        }
    }
    minAvgCorr = ((minSum / minSumNr) * (maxVal - minVal)) / (maxRef - minRef);
    maxAvgCorr = ((maxSum / maxSumNr) * (maxVal - minVal)) / (maxRef - minRef);
}

uint16_t OpticalEncoderHandler::OpticalSensorValueScaler::get(uint16_t value)
{
    return std::max(valueScaler.getOutput(value), (int32_t)0); 
}


void OpticalEncoderHandler::OpticalSensorValueScaler::update(int i, uint16_t value)
{
    using namespace adam_std;

    int iDiffFromMin = wrapAroundDist<vecSize>(i - iAtMinRef);
    if (std::abs(iDiffFromMin) < indexDelta)
    {
        int s = sign(moveBeforeMinUpdate);
        if (abs(moveBeforeMinUpdate) > iDiffFromMin * s + indexDelta)
        {
            moveBeforeMinUpdate -= s * indexDelta / 16;
            diffAtMinRefAverager.add(value + minAvgCorr);
        } 
    }
    else
    {
        if (abs(moveBeforeMinUpdate) < indexDelta &&
                (iDiffFromMin < 0) != (moveBeforeMinUpdate < 0))
        {
            minVal = diffAtMinRefAverager.get();
        }
        moveBeforeMinUpdate = indexDelta * 2 * sign(iDiffFromMin);
        diffAtMinRefAverager.reset(minVal);
    }

    int iDiffFromMax = wrapAroundDist<vecSize>(i - iAtMaxRef);
    if (std::abs(iDiffFromMax) < indexDelta)
    {
        int s = sign(moveBeforeMaxUpdate);
        if (abs(moveBeforeMaxUpdate) > iDiffFromMax * s + indexDelta)
        {
            moveBeforeMaxUpdate -= s * indexDelta / 16;
            diffAtMaxRefAverager.add(value + maxAvgCorr);
        } 
    }
    else
    {
        if (abs(moveBeforeMaxUpdate) < indexDelta &&
                (iDiffFromMax < 0) != (moveBeforeMaxUpdate < 0))
        {
            maxVal = diffAtMaxRefAverager.get();
        }
        moveBeforeMaxUpdate = indexDelta * 2 * sign(iDiffFromMax);
        diffAtMaxRefAverager.reset(maxVal);
    }

    valueScaler.approxInputRangeUpdate(minVal, maxVal);
}
