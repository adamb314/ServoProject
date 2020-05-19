#include "CommunicationHandlers.h"
#include "DCServo.h"

static DCServo* dcServo = nullptr;
static ThreadHandler* threadHandler = nullptr;

DCServoCommunicationHandler::DCServoCommunicationHandler(unsigned char nodeNr, unsigned long baud) :
    Communication({{nodeNr}}, baud)
{
    dcServo = DCServo::getInstance();
    threadHandler = ThreadHandler::getInstance();
    Communication::intArray[0][0] = dcServo->getPosition() * positionUpscaling;
    Communication::intArray[0][1] = 0;
    Communication::intArray[0][2] = 0;
    Communication::charArray[0][1] = 0;
    Communication::charArray[0][2] = 0;

    intArrayIndex0Upscaler.set(Communication::intArray[0][0]);
}

DCServoCommunicationHandler::~DCServoCommunicationHandler()
{
}

void DCServoCommunicationHandler::onReadyToSendEvent()
{
}

void DCServoCommunicationHandler::onReceiveCompleteEvent()
{

    dcServo->onlyUseMainEncoder(Communication::charArray[0][2] == 1);

    if (Communication::intArrayChanged[0][0])
    {
        intArrayIndex0Upscaler.update(Communication::intArray[0][0]);
        dcServo->loadNewReference(intArrayIndex0Upscaler.get() * (1.0 / positionUpscaling), Communication::intArray[0][1], Communication::intArray[0][2]);

        Communication::intArrayChanged[0][0] = false;
        dcServo->openLoopMode(false);
        dcServo->enable(true);

        statusLight.showEnabled();
    }
    else if (Communication::intArrayChanged[0][2])
    {
        intArrayIndex0Upscaler.update(Communication::intArray[0][0]);
        dcServo->loadNewReference(intArrayIndex0Upscaler.get() * (1.0 / positionUpscaling), Communication::intArray[0][1], Communication::intArray[0][2]);

        Communication::intArrayChanged[0][2] = false;
        dcServo->openLoopMode(true, Communication::charArray[0][1] == 1);
        dcServo->enable(true);

        statusLight.showOpenLoop();
    }
    else
    {
        dcServo->enable(false);

        statusLight.showDisabled();
    }
}

void DCServoCommunicationHandler::onErrorEvent()
{
}

void DCServoCommunicationHandler::onComCycleEvent()
{
    {
        ThreadInterruptBlocker blocker;

        Communication::intArray[0][3] = dcServo->getPosition() * positionUpscaling;
        Communication::intArray[0][4] = dcServo->getVelocity();
        Communication::intArray[0][5] = dcServo->getControlSignal();
        Communication::intArray[0][6] = dcServo->getCurrent();
        Communication::intArray[0][7] = dcServo->getPwmControlSignal();
        Communication::intArray[0][8] = threadHandler->getCpuLoad();
        Communication::intArray[0][9] = dcServo->getLoopNumber();
        Communication::intArray[0][10] = dcServo->getMainEncoderPosition() * positionUpscaling;
        Communication::intArray[0][11] = dcServo->getBacklashCompensation() * positionUpscaling;

        auto opticalEncoderChannelData = dcServo->getMainEncoderDiagnosticData<OpticalEncoderHandler::DiagnosticData>();
        Communication::intArray[0][12] = opticalEncoderChannelData.a;
        Communication::intArray[0][13] = opticalEncoderChannelData.b;
        Communication::intArray[0][14] = opticalEncoderChannelData.minCostIndex;
        Communication::intArray[0][15] = opticalEncoderChannelData.minCost;

        if (dcServo->isEnabled())
        {
            dcServo->triggerReferenceTiming();
        }
    }

    statusLight.showCommunicationActive();
}

void DCServoCommunicationHandler::onComIdleEvent()
{
    Communication::intArrayChanged[0][0] = false;
    Communication::intArrayChanged[0][2] = false;
    dcServo->enable(false);

    statusLight.showDisabled();
    statusLight.showCommunicationInactive();
}
