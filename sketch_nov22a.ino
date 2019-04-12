#undef max
#undef min

#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"
#include "DCServo.h"
#include "Communication.h"

START_OF_THREAD_HANDLER_CONFIG
THREAD_HANDLER_WITH_EXECUTION_ORDER_OPTIMIZED(200);
END_OF_THREAD_HANDLER_CONFIG

ThreadHandler* threadHandler = ThreadHandler::getInstance();

std::vector<FunctionThread*> threads;

DCServo* dcServo = nullptr;
std::unique_ptr<Communication> communication;

void setup()
{
    dcServo = DCServo::getInstance();
    communication = std::make_unique<Communication>(1, 115200);

    communication->intArray[0] = dcServo->getPosition();
    communication->intArray[1] = 0;
    communication->intArray[2] = 0;
    communication->charArray[1] = 0;

    threads.push_back(new FunctionThread(0, 600, 0,
        [&]()
        {
            static uint32_t lastPosRefTimestamp = millis();

            communication->intArray[3] = dcServo->getPosition();
            communication->intArray[4] = dcServo->getVelocity();
            communication->intArray[5] = dcServo->getControlSignal();
            communication->intArray[6] = dcServo->getCurrent();
            communication->intArray[7] = threadHandler->getCpuLoad();
            communication->intArray[8] = dcServo->getLoopNumber();

            communication->blockingRun();

            if (communication->charArray[1] == 0)
            {
                if (communication->intArrayChanged[0])
                {
                    lastPosRefTimestamp = millis();

                     dcServo->enable(true);
                }
                else if (static_cast<int32_t>(millis() - lastPosRefTimestamp) > 100)
                {
                    dcServo->enable(false);
                }

                dcServo->setReference(communication->intArray[0], communication->intArray[1], communication->intArray[2]);
                communication->intArrayChanged[0] = false;
            }
            else
            {
                int16_t amplitude = communication->intArray[2];
                if (communication->charArray[1] == 1)
                {
                    if (dcServo->runIdentTest1(amplitude))
                    {
                        communication->charArray[1] = 0;
                    }
                }
                else
                {
                    if (communication->charArray[1] == 2)
                    {
                        if (dcServo->runIdentTest2(amplitude))
                        {
                            communication->charArray[1] = 0;
                        }
                    }
                }
            }

        }));
}

void loop()
{
    threadHandler->run();
}
