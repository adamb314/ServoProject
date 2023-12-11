#include "src/ArduinoC++BugFixes.h"
#include "src/Hardware/ThreadHandler.h"
#include "src/Hardware/FailSafeHandler.h"

#include "config/config.h"

SET_THREAD_HANDLER_TICK(200);
THREAD_HANDLER_WITH_EXECUTION_ORDER_OPTIMIZED(InterruptTimer::getInstance());

ThreadHandler* threadHandler = ThreadHandler::getInstance();

std::unique_ptr<Communication> communication;

void setup()
{
    communication = ConfigHolder::getCommunicationHandler();
    threadHandler->enableThreadExecution();
}

void loop()
{
    FailSafeHandler::getInstance()->resetWatchdogTimer();
    communication->run();
}
