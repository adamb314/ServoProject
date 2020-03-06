#undef max
#undef min

#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"
#include "DCServo.h"
#include "StatusLightHandler.h"
#include "Communication.h"

#include "config/config.h"

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
        Communication::charArray[2] = 0;
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
        {
            ThreadInterruptBlocker blocker;

            Communication::intArray[3] = dcServo->getPosition() * 4;
            Communication::intArray[4] = dcServo->getVelocity();
            Communication::intArray[5] = dcServo->getControlSignal();
            Communication::intArray[6] = dcServo->getCurrent();
            Communication::intArray[7] = threadHandler->getCpuLoad();
            Communication::intArray[8] = dcServo->getLoopNumber();
            Communication::intArray[9] = dcServo->getMainEncoderPosition() * 4;

            auto opticalEncoderChannelData = dcServo->getMainEncoderDiagnosticData<OpticalEncoderHandler::DiagnosticData>();
            Communication::intArray[10] = opticalEncoderChannelData.a;
            Communication::intArray[11] = opticalEncoderChannelData.b;
            Communication::intArray[12] = opticalEncoderChannelData.minCostIndex;
            Communication::intArray[13] = opticalEncoderChannelData.minCost;
        }

        dcServo->onlyUseMainEncoder(Communication::charArray[2] == 1);

        dcServo->setReference(Communication::intArray[0] * 0.25, Communication::intArray[1], Communication::intArray[2]);

        if (Communication::intArrayChanged[0])
        {
            Communication::intArrayChanged[0] = false;
            dcServo->openLoopMode(false);
            dcServo->enable(true);

            statusLight.showEnabled();
        }
        else if (Communication::intArrayChanged[2])
        {
            Communication::intArrayChanged[2] = false;
            dcServo->openLoopMode(true, Communication::charArray[1] == 1);
            dcServo->enable(true);

            statusLight.showOpenLoop();
        }
        else
        {
            dcServo->enable(false);

            statusLight.showDisabled();
        }

        statusLight.showCommunicationActive();
    }

    void onComIdleEvent() override
    {
        Communication::intArrayChanged[0] = false;
        Communication::intArrayChanged[2] = false;
        dcServo->enable(false);

        statusLight.showDisabled();
        statusLight.showCommunicationInactive();
    }

private:
    DCServo* dcServo;
    StatusLightHandler statusLight;
};

std::unique_ptr<CommunicationHandler> communication;

void setup()
{
    communication = std::make_unique<CommunicationHandler>(DCServo::getInstance(), ConfigHolder::getCommunicationId(), 115200);

    threadHandler->enableThreadExecution();
}

void loop()
{
    communication->run();
}
