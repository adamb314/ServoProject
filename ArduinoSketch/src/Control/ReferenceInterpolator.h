#include <Arduino.h>
#include "../ArduinoC++BugFixes.h"
#include <algorithm>

#ifndef REFERENCE_INTERPOLATOR_H
#define REFERENCE_INTERPOLATOR_H

class ReferenceInterpolator
{
 public:
    ReferenceInterpolator();

    void loadNew(float position, float velocity, float feedForward);

    void updateTiming();

    void resetTiming();

    void calculateNext();

    std::tuple<float, float, float> get();

    float getPositionInterpolationDist();

    void setGetTimeInterval(const uint16_t& interval);

    void setLoadTimeInterval(const uint16_t& interval);

    int16_t midPointTimeOffset{0};
 private:
    void stepAndUpdateInter();

    float pos[3]{0};
    float vel[3]{0};
    float feed[3]{0};

    float interPos;
    float interVel;
    float interFeed;

    uint16_t lastUpdateTimingTimestamp{0};
    uint16_t lastGetTimestamp{0};

    bool timingInvalid{true};
    bool refInvalid{true};
    uint16_t loadTimeInterval{12000};
    float invertedLoadInterval{1.0f / loadTimeInterval};
    float dtDiv2{loadTimeInterval * 0.000001f * 0.5f};
    uint16_t getTimeInterval{1200};
    float invertedGetInterval{1.0f / getTimeInterval};
    float getTStepSize{getTimeInterval * invertedLoadInterval};
};

#endif