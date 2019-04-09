#undef max
#undef min

#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"

START_OF_THREAD_HANDLER_CONFIG
THREAD_HANDLER_WITH_EXECUTION_ORDER_OPTIMIZED(200);
END_OF_THREAD_HANDLER_CONFIG

ThreadHandler* threadHandler = ThreadHandler::getInstance();

void setup()
{
}

void loop()
{
    threadHandler->run();
}
