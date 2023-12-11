#include <Arduino.h>
#include "../ArduinoC++BugFixes.h"
#include "ThreadHandler.h"
#include "PwmHandler.h"

#ifndef FAIL_SAFE_HANDLER
#define FAIL_SAFE_HANDLER

class FailSafeHandler
{
public:
    static constexpr uint32_t watchdogTimeout = 10000;
    static FailSafeHandler* getInstance();
    void resetWatchdogTimer();
    void goToFailSafe();

private:
    FailSafeHandler();

    void failSafeBlink();

    Thread* failSafeThread;
    bool watchdogReset{false};
    bool failSafeTriggered{false};
};

#endif
