#include "CommunicationHandlers.h"
#include "DCServo.h"

static DCServo* dcServo = nullptr;
static ThreadHandler* threadHandler = nullptr;

DCServoCommunicationHandler::DCServoCommunicationHandler(unsigned char nodeNr) :
    CommunicationNode(nodeNr)
{
    dcServo = DCServo::getInstance();
    threadHandler = ThreadHandler::getInstance();
    CommunicationNode::intArray[0] = dcServo->getPosition() * positionUpscaling;
    CommunicationNode::intArray[1] = 0;
    CommunicationNode::intArray[2] = 0;
    CommunicationNode::charArray[1] = 0;
    CommunicationNode::charArray[2] = 0;
    CommunicationNode::charArray[3] = 0;
    CommunicationNode::charArray[4] = 0;

    intArrayIndex0Upscaler.set(CommunicationNode::intArray[0]);
}

DCServoCommunicationHandler::~DCServoCommunicationHandler()
{
}

void DCServoCommunicationHandler::onReadyToSendEvent()
{
}

void DCServoCommunicationHandler::onReceiveCompleteEvent()
{
    dcServo->onlyUseMainEncoder(CommunicationNode::charArray[2] == 1);

    if (CommunicationNode::charArrayChanged[3])
    {
        dcServo->setControlSpeed(CommunicationNode::charArray[3]);
    }

    if (CommunicationNode::charArrayChanged[4] ||
        CommunicationNode::charArrayChanged[5] ||
        CommunicationNode::charArrayChanged[6])
    {
        dcServo->setBacklashControlSpeed(CommunicationNode::charArray[4],
                CommunicationNode::charArray[5],
                CommunicationNode::charArray[6]);
    }

    if (CommunicationNode::intArrayChanged[0])
    {
        intArrayIndex0Upscaler.update(CommunicationNode::intArray[0]);
        dcServo->loadNewReference(intArrayIndex0Upscaler.get() * (1.0 / positionUpscaling), CommunicationNode::intArray[1], CommunicationNode::intArray[2]);

        CommunicationNode::intArrayChanged[0] = false;
        dcServo->openLoopMode(false);
        dcServo->enable(true);

        statusLight.showEnabled();
    }
    else if (CommunicationNode::intArrayChanged[2])
    {
        intArrayIndex0Upscaler.update(CommunicationNode::intArray[0]);
        dcServo->loadNewReference(intArrayIndex0Upscaler.get() * (1.0 / positionUpscaling), CommunicationNode::intArray[1], CommunicationNode::intArray[2]);

        CommunicationNode::intArrayChanged[2] = false;
        dcServo->openLoopMode(true, CommunicationNode::charArray[1] == 1);
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

        CommunicationNode::intArray[3] = dcServo->getPosition() * positionUpscaling;
        CommunicationNode::intArray[4] = dcServo->getVelocity();
        CommunicationNode::intArray[5] = dcServo->getControlSignal();
        CommunicationNode::intArray[6] = dcServo->getCurrent();
        CommunicationNode::intArray[7] = dcServo->getPwmControlSignal();
        CommunicationNode::intArray[8] = threadHandler->getCpuLoad();
        CommunicationNode::intArray[9] = dcServo->getLoopNumber();
        CommunicationNode::intArray[10] = dcServo->getMainEncoderPosition() * positionUpscaling;
        CommunicationNode::intArray[11] = dcServo->getBacklashCompensation() * positionUpscaling;

        auto opticalEncoderChannelData = dcServo->getMainEncoderDiagnosticData<OpticalEncoderHandler::DiagnosticData>();
        CommunicationNode::intArray[12] = opticalEncoderChannelData.a;
        CommunicationNode::intArray[13] = opticalEncoderChannelData.b;
        CommunicationNode::intArray[14] = opticalEncoderChannelData.minCostIndex;
        CommunicationNode::intArray[15] = opticalEncoderChannelData.minCost;

        if (dcServo->isEnabled())
        {
            dcServo->triggerReferenceTiming();
        }
    }

    statusLight.showCommunicationActive();
}

void DCServoCommunicationHandler::onComIdleEvent()
{
    CommunicationNode::intArrayChanged[0] = false;
    CommunicationNode::intArrayChanged[2] = false;
    dcServo->enable(false);

    statusLight.showDisabled();
    statusLight.showCommunicationInactive();
}
