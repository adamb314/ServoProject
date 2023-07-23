#include "ReferenceInterpolator.h"

ReferenceInterpolator::ReferenceInterpolator()
{
}

void ReferenceInterpolator::loadNew(float position, float velocity, float feedForward)
{
    if (refInvalid)
    {
        if (!timingInvalid)
        {
            refInvalid = false;
        }

        pos[2] = position;
        vel[2] = velocity;
        feed[2] = feedForward;

        pos[1] = pos[2];
        vel[1] = vel[2];
        feed[1] = feed[2];

        pos[0] = pos[1];
        vel[0] = vel[1];
        feed[0] = feed[1];
    }
    else
    {
        if (midPointTimeOffset > -loadTimeInterval)
        {
            midPointTimeOffset -= loadTimeInterval;
        }

        pos[0] = pos[1];
        vel[0] = vel[1];
        feed[0] = feed[1];

        pos[1] = pos[2];
        vel[1] = vel[2];
        feed[1] = feed[2];

        pos[2] = position;
        vel[2] = velocity;
        feed[2] = feedForward;
    }
}

void ReferenceInterpolator::updateTiming()
{
    uint16_t timestamp = micros();
    if (timingInvalid)
    {
        midPointTimeOffset = 2 * getTimeInterval;

        timingInvalid = false;
    }
    else
    {
        uint16_t updatePeriod = timestamp - lastUpdateTimingTimestamp;
        uint16_t timeSinceLastGet = timestamp - lastGetTimestamp;

        int16_t timingError = 2 * getTimeInterval - (midPointTimeOffset + timeSinceLastGet);
        int16_t periodError = updatePeriod - loadTimeInterval;

        midPointTimeOffset += timingError / 8;
        loadTimeInterval += periodError / 16;

        dtDiv2 = loadTimeInterval * 0.000001f * 0.5f;
        invertedLoadInterval = 1.0f / loadTimeInterval;
        getTStepSize = getTimeInterval * invertedLoadInterval;
    }

    lastUpdateTimingTimestamp = timestamp;
}

void ReferenceInterpolator::resetTiming()
{
    timingInvalid = true;
    refInvalid = true;

    stepAndUpdateInter();
}

void ReferenceInterpolator::calculateNext()
{
    lastGetTimestamp = micros();

    stepAndUpdateInter();
}

void ReferenceInterpolator::stepAndUpdateInter()
{
    if (refInvalid)
    {
        interPos = pos[2];
        interVel = vel[2];
        interFeed = feed[2];

        return;
    }

    if (midPointTimeOffset < 2 * loadTimeInterval)
    {
        midPointTimeOffset += getTimeInterval;
    }

    float t = midPointTimeOffset * invertedLoadInterval;
    t = std::min(t, 1.2f);
    t = std::max(t, -1.0f);

    if (t < 0.0f)
    {
        t += 1.0f;

        float s = midPointTimeOffset * invertedGetInterval + 1.0f;
        s = std::max(s, 0.0f);

        interFeed = feed[0] * (1.0f - s) + feed[1] * s;
        float velDiff = vel[1] - vel[0];
        interVel = vel[0] + t * velDiff;
        interPos = pos[0] + t * (pos[1] - pos[0] + dtDiv2 * (t * velDiff - velDiff));
    }
    else
    {
        interFeed = feed[1];
        float velDiff = vel[2] - vel[1];
        interVel = vel[1] + t * velDiff;
        interPos = pos[1] + t * (pos[2] - pos[1] + dtDiv2 * (t * velDiff - velDiff));
    }
}

std::tuple<float, float, float> ReferenceInterpolator::get()
{
    return std::make_tuple(interPos, interVel, interFeed);
}

float ReferenceInterpolator::getPositionInterpolationDist()
{
    return interPos - pos[1];
}

void ReferenceInterpolator::setGetTimeInterval(const uint16_t& interval)
{
    resetTiming();

    getTimeInterval = interval;
    invertedGetInterval = 1.0f / getTimeInterval;
    getTStepSize = getTimeInterval * invertedLoadInterval;
}

void ReferenceInterpolator::setLoadTimeInterval(const uint16_t& interval)
{
    resetTiming();

    loadTimeInterval = interval;
    invertedLoadInterval = 1.0f / loadTimeInterval;
    dtDiv2 = loadTimeInterval * 0.000001f * 0.5f;
    getTStepSize = getTimeInterval * invertedLoadInterval;
}
