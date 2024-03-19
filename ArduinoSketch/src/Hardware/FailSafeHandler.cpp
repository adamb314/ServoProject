#include "FailSafeHandler.h"

FailSafeHandler* FailSafeHandler::getInstance()
{
    static FailSafeHandler failSafeHandler;
    return &failSafeHandler;
}

void FailSafeHandler::resetWatchdogTimer()
{
    watchdogReset = true;
}

void FailSafeHandler::goToFailSafe()
{
    failSafeTriggered = true;
}

FailSafeHandler::FailSafeHandler()
{
    failSafeThread = createThread(127, watchdogTimeout, watchdogTimeout,
        [&]()
        {
            if (!watchdogReset)
            {
                failSafeTriggered = true;
            }
            watchdogReset = false;

            if (failSafeTriggered)
            {
                ThreadHandler::getInstance()->enableThreadExecution(false);
                PwmHandler::disconnectAllOutputs();
                failSafeBlink();
            }
        });        
}

void FailSafeHandler::failSafeBlink()
{
    pinMode(13, OUTPUT);
    while (true)
    {
        if ((millis() / 128) % 2)
        {
            digitalWrite(13, HIGH);
        }
        else
        {
            digitalWrite(13, LOW);
        }
    }
}
