#undef max
#undef min

#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"
#include "DCServo.h"
#include "Communication.h"

SET_THREAD_HANDLER_TICK(200);
THREAD_HANDLER_WITH_EXECUTION_ORDER_OPTIMIZED(InterruptTimer::getInstance());

ThreadHandler* threadHandler = ThreadHandler::getInstance();

class CommunicationHandler : public Communication
{
public:
    CommunicationHandler(DCServo* dcServo, unsigned char nodeNr, unsigned long baud) :
        Communication(nodeNr, baud),
        dcServo(dcServo)
    {
        Communication::intArray[0] = dcServo->getPosition() * 4;
        Communication::intArray[1] = 0;
        Communication::intArray[2] = 0;
        Communication::charArray[1] = 0;

        posRefTimeout = 100;

        lastPosRefTimestamp = millis() - 2 * posRefTimeout;
    }

    ~CommunicationHandler()
    {
    }

    virtual void onReadyToSendEvent()
    {
    }

    virtual void onReceiveCompleteEvent()
    {
    }

    virtual void onErrorEvent()
    {
    }

    virtual void onComCycleEvent()
    {
        Communication::intArray[3] = dcServo->getPosition() * 4;
        Communication::intArray[4] = dcServo->getVelocity();
        Communication::intArray[5] = dcServo->getControlSignal();
        Communication::intArray[6] = dcServo->getCurrent();
        Communication::intArray[7] = threadHandler->getCpuLoad();
        Communication::intArray[8] = dcServo->getLoopNumber();

        if (Communication::charArray[1] == 0)
        {
            if (Communication::intArrayChanged[0])
            {
                lastPosRefTimestamp = millis();

                 dcServo->enable(true);
            }

            dcServo->setReference(Communication::intArray[0] * 0.25, Communication::intArray[1], Communication::intArray[2]);
            Communication::intArrayChanged[0] = false;
        }
        else
        {
            int16_t amplitude = Communication::intArray[2];
            if (Communication::charArray[1] == 1)
            {
                if (dcServo->runIdentTest1(amplitude))
                {
                    Communication::charArray[1] = 0;
                }
            }
            else
            {
                if (Communication::charArray[1] == 2)
                {
                    if (dcServo->runIdentTest2(amplitude))
                    {
                        Communication::charArray[1] = 0;
                    }
                }
            }
        }
    }

    void checkCommunicationTimeout()
    {
        if (static_cast<int32_t>(millis() - lastPosRefTimestamp) > posRefTimeout)
        {
            dcServo->enable(false);
        }
    }

    void run() override
    {
        Communication::run();
        checkCommunicationTimeout();
    }

private:
    DCServo* dcServo;
    uint32_t lastPosRefTimestamp;
    uint16_t posRefTimeout;
};

std::unique_ptr<CommunicationHandler> communication;

void setup()
{
    communication = std::make_unique<CommunicationHandler>(DCServo::getInstance(), 1, 115200);

    threadHandler->enableThreadExecution();
}

void loop()
{
    communication->run();
}
